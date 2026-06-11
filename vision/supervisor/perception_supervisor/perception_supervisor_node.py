"""
Perception Supervisor : machine a etats pour fusionner les sources de pose
du Spot et fournir une pose unifiee a l'asservissement.

Source primaire   : /spot_pose_in_summit (AprilTag, pose 6D precise)
Source secondaire : /yolo_detections + /summit_xl/front_laser/scan
                    (YOLO donne la direction, LiDAR donne la distance)

Etats :
- TAG_OK         : pose AprilTag fraiche, on la transmet telle quelle
- YOLO_TRACKING  : pas de tag, mais YOLO + LiDAR donnent une pose 3D approximative
- LOST           : ni tag ni YOLO depuis trop longtemps, on arrete

Architecture :
Utilise un MultiThreadedExecutor + ReentrantCallbackGroup. Sinon, les
callbacks YOLO (lents a cause des TF transforms) bloquent les callbacks
LaserScan, qui finissent par decrocher leur subscription DDS.

Hysteresis :
- TAG_OK -> YOLO  : si pas de pose tag depuis 1.0 s
- YOLO -> TAG_OK  : des qu'une pose tag revient (instantane)
- YOLO -> LOST    : si pas de YOLO non plus depuis 3.0 s
- LOST -> ...     : reprend des qu'une source revient
"""

import math
import numpy as np
from typing import Optional
from threading import Lock

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan, CameraInfo
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String

from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # noqa : enregistre les conversions PoseStamped


# ----------- Topics --------------
TAG_POSE_TOPIC = '/spot_pose_in_summit'
YOLO_TOPIC = '/yolo_detections'
SCAN_TOPIC = '/summit_xl/front_laser/scan'
CAM_INFO_TOPIC = '/camera/color/camera_info'

OUT_POSE_TOPIC = '/spot_target_pose'
OUT_STATUS_TOPIC = '/perception_status'

# ----------- Frames --------------
SUMMIT_FRAME = 'summit_xl_base_link'
CAM_OPTICAL_FRAME = 'camera_color_optical_frame'
LASER_FRAME = 'summit_xl_front_laser_link'

# ----------- Parametres state machine -------------
TAG_FRESH_TIMEOUT = 1.0       # s
YOLO_FRESH_TIMEOUT = 3.0      # s

# Vitesse max physique du Spot (validation des sauts)
MAX_SPOT_SPEED = 2.0          # m/s

# Confiance YOLO
CONFIDENCE_GAIN = 0.15
CONFIDENCE_DECAY = 0.85

# Stats periodiques pour monitoring
STATS_PERIOD = 2.0


class PerceptionSupervisor(Node):

    def __init__(self):
        super().__init__('perception_supervisor')

        # Callback group reentrant : permet aux callbacks de s'executer
        # en parallele dans des threads differents (avec le MultiThreadedExecutor)
        self.cb_group = ReentrantCallbackGroup()

        # Lock pour proteger les variables partagees entre threads
        self.lock = Lock()

        # tf2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Intrinseques cam
        self.fx = self.fy = self.cx = self.cy = None
        self.create_subscription(
            CameraInfo, CAM_INFO_TOPIC, self.on_cam_info, 10,
            callback_group=self.cb_group)

        # Cache des donnees
        self.last_scan: Optional[LaserScan] = None
        self.last_tag_pose: Optional[PoseStamped] = None
        self.last_tag_time: Optional[rclpy.time.Time] = None
        self.last_yolo_time: Optional[rclpy.time.Time] = None
        self.last_yolo_pose: Optional[PoseStamped] = None

        # Compteurs pour monitoring
        self.scan_count = 0
        self.yolo_count = 0

        # Subscriptions (toutes dans le meme callback group)
        self.create_subscription(
            LaserScan, SCAN_TOPIC, self.on_scan, 10,
            callback_group=self.cb_group)
        self.create_subscription(
            PoseStamped, TAG_POSE_TOPIC, self.on_tag_pose, 10,
            callback_group=self.cb_group)
        self.create_subscription(
            Detection2DArray, YOLO_TOPIC, self.on_yolo, 10,
            callback_group=self.cb_group)

        # Publications
        self.pub_pose = self.create_publisher(PoseStamped, OUT_POSE_TOPIC, 10)
        self.pub_status = self.create_publisher(String, OUT_STATUS_TOPIC, 10)

        # Etat interne
        self.state = 'LOST'
        self.yolo_confidence = 0.0

        # Timer principal a 20 Hz pour state machine
        self.create_timer(0.05, self.tick, callback_group=self.cb_group)

        # Timer stats
        self.create_timer(STATS_PERIOD, self._log_stats,
                          callback_group=self.cb_group)

        self.get_logger().info(
            "Perception supervisor pret (multi-thread). "
            f"TAG_OK timeout = {TAG_FRESH_TIMEOUT}s, "
            f"YOLO timeout = {YOLO_FRESH_TIMEOUT}s.")

    # ============== Monitoring =================

    def _log_stats(self):
        self.get_logger().info(
            f"[STATS] scans={self.scan_count}, yolo={self.yolo_count}, "
            f"state={self.state}, conf={self.yolo_confidence:.2f}, "
            f"fx={'OK' if self.fx else 'NO'}, "
            f"scan={'OK' if self.last_scan else 'NO'}")

    # ============== Callbacks =================

    def on_cam_info(self, msg: CameraInfo):
        with self.lock:
            if self.fx is None:
                self.fx = msg.k[0]
                self.fy = msg.k[4]
                self.cx = msg.k[2]
                self.cy = msg.k[5]
                self.get_logger().info(
                    f"Intrinseques cam recues : fx={self.fx:.1f}, "
                    f"cx={self.cx:.1f}, cy={self.cy:.1f}")

    def on_scan(self, msg: LaserScan):
        with self.lock:
            self.last_scan = msg
            self.scan_count += 1

    def on_tag_pose(self, msg: PoseStamped):
        with self.lock:
            self.last_tag_pose = msg
            self.last_tag_time = self.get_clock().now()

    def on_yolo(self, msg: Detection2DArray):
        # Snapshot des etats critiques sous lock, puis travail hors-lock
        with self.lock:
            self.yolo_count += 1
            if not msg.detections:
                return
            if self.fx is None:
                return
            if self.last_scan is None:
                return
            local_scan = self.last_scan
            local_fx = self.fx
            local_cx = self.cx
            local_last_yolo_pose = self.last_yolo_pose
            local_last_yolo_time = self.last_yolo_time
            local_last_tag_pose = self.last_tag_pose

        det = msg.detections[0]
        bbox = det.bbox
        score = float(det.results[0].score) if det.results else 0.0

        # 1. Bearing dans le repere optical
        bearing_optical = math.atan2(bbox.center.x - local_cx, local_fx)

        # 2. Transformation cam -> laser (TF appel, peut prendre 1-10 ms)
        v_optical = PoseStamped()
        v_optical.header.frame_id = CAM_OPTICAL_FRAME
        v_optical.header.stamp = msg.header.stamp
        v_optical.pose.position.x = math.sin(bearing_optical)
        v_optical.pose.position.y = 0.0
        v_optical.pose.position.z = math.cos(bearing_optical)
        v_optical.pose.orientation.w = 1.0

        try:
            v_laser = self.tf_buffer.transform(
                v_optical, LASER_FRAME, timeout=Duration(seconds=0.1))
        except Exception as e:
            self.get_logger().warn(
                f"TF cam->laser indisponible : {e}",
                throttle_duration_sec=2.0)
            return

        bearing_laser = math.atan2(v_laser.pose.position.y,
                                   v_laser.pose.position.x)

        # 3. Lecture LiDAR (utilise le snapshot local_scan, pas de lock)
        z_lidar = self._read_lidar_at_bearing(local_scan, bearing_laser)
        if z_lidar is None:
            self.get_logger().warn(
                f"YOLO bbox a {math.degrees(bearing_laser):.1f}° mais LiDAR vide",
                throttle_duration_sec=2.0)
            return

        # 4. Validation temporelle (utilise le snapshot)
        if local_last_yolo_pose is not None and local_last_yolo_time is not None:
            dt = (self.get_clock().now() - local_last_yolo_time).nanoseconds * 1e-9
            max_delta = MAX_SPOT_SPEED * dt + 0.5
            dist_jump = abs(z_lidar - math.hypot(
                local_last_yolo_pose.pose.position.x,
                local_last_yolo_pose.pose.position.y))
            if dist_jump > max_delta:
                self.get_logger().warn(
                    f"Saut depth incoherent : delta={dist_jump:.2f}m > max={max_delta:.2f}m",
                    throttle_duration_sec=2.0)
                return

        # 5. Construction pose dans laser puis transformation en summit
        pose_laser = PoseStamped()
        pose_laser.header.frame_id = LASER_FRAME
        pose_laser.header.stamp = msg.header.stamp
        pose_laser.pose.position.x = z_lidar * math.cos(bearing_laser)
        pose_laser.pose.position.y = z_lidar * math.sin(bearing_laser)
        pose_laser.pose.position.z = 0.0

        if local_last_tag_pose is not None:
            pose_laser.pose.orientation = local_last_tag_pose.pose.orientation
        else:
            pose_laser.pose.orientation.w = 1.0

        try:
            pose_summit = self.tf_buffer.transform(
                pose_laser, SUMMIT_FRAME, timeout=Duration(seconds=0.1))
        except Exception as e:
            self.get_logger().warn(
                f"TF laser->summit indisponible : {e}",
                throttle_duration_sec=2.0)
            return

        # Mise a jour des etats sous lock
        with self.lock:
            self.last_yolo_pose = pose_summit
            self.last_yolo_time = self.get_clock().now()
            self.yolo_confidence = min(
                1.0, self.yolo_confidence + CONFIDENCE_GAIN * score)

    # ============== State machine =================

    def tick(self):
        now = self.get_clock().now()

        with self.lock:
            tag_time = self.last_tag_time
            yolo_time = self.last_yolo_time
            tag_pose = self.last_tag_pose
            yolo_pose = self.last_yolo_pose

        tag_fresh = (tag_time is not None and
                     (now - tag_time).nanoseconds * 1e-9 < TAG_FRESH_TIMEOUT)
        yolo_fresh = (yolo_time is not None and
                      (now - yolo_time).nanoseconds * 1e-9 < YOLO_FRESH_TIMEOUT)

        with self.lock:
            if not yolo_fresh:
                self.yolo_confidence *= CONFIDENCE_DECAY

            new_state = self.state
            if tag_fresh:
                new_state = 'TAG_OK'
            elif yolo_fresh:
                new_state = 'YOLO_TRACKING'
            else:
                new_state = 'LOST'
                self.yolo_confidence = 0.0

            if new_state != self.state:
                self.get_logger().info(
                    f"Transition : {self.state} -> {new_state}")
                self.state = new_state

            current_state = self.state
            current_conf = self.yolo_confidence

        # Publication du status
        status = String()
        status.data = f"{current_state} conf={current_conf:.2f}"
        self.pub_status.publish(status)

        # Publication de la pose selon l'etat
        if current_state == 'TAG_OK' and tag_pose is not None:
            out = PoseStamped()
            out.header.stamp = now.to_msg()
            out.header.frame_id = tag_pose.header.frame_id
            out.pose = tag_pose.pose
            self.pub_pose.publish(out)

        elif current_state == 'YOLO_TRACKING' and yolo_pose is not None:
            out = PoseStamped()
            out.header.stamp = now.to_msg()
            out.header.frame_id = yolo_pose.header.frame_id
            out.pose = yolo_pose.pose
            self.pub_pose.publish(out)

    # ============== Helpers =================

    def _read_lidar_at_bearing(self, scan: LaserScan,
                               bearing_laser: float) -> Optional[float]:
        """Lit le LiDAR dans la direction bearing_laser."""
        if scan is None:
            return None

        if bearing_laser < scan.angle_min or bearing_laser > scan.angle_max:
            return None

        idx = int(round((bearing_laser - scan.angle_min) / scan.angle_increment))
        idx = max(0, min(len(scan.ranges) - 1, idx))

        i0 = max(0, idx - 4)
        i1 = min(len(scan.ranges), idx + 5)
        candidates = scan.ranges[i0:i1]
        valid = [r for r in candidates
                 if scan.range_min < r < scan.range_max and not math.isinf(r)]
        if not valid:
            return None

        z = float(np.median(valid))
        if z > scan.range_max - 0.5:
            return None

        return z


def main():
    rclpy.init()
    node = PerceptionSupervisor()

    # MultiThreadedExecutor : permet aux callbacks YOLO (lents avec TF)
    # de ne pas bloquer le callback LaserScan rapide
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
