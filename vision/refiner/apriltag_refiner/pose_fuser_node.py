"""
Pose fuser : combine les detections AprilTag raffinees avec les TF
statiques (positions des tags sur le Spot) pour publier la pose du
Spot dans le repere du Summit XL.

Pour chaque tag detecte, compose T_cam_spot = T_cam_tag . T_tag_spot
en utilisant scipy.spatial.transform.Rotation (gestion propre des
quaternions et des conventions). Si plusieurs tags sont visibles, les
positions sont fusionnees par moyenne ponderee 1/Z^2 ; l'orientation
est celle du tag le plus proche.

Sortie :
- topic /spot_pose_in_summit (PoseStamped)
- TF summit_xl_base_link -> spot_base_link
"""

import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, TransformStamped
from apriltag_msgs.msg import AprilTagDetectionArray
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
import tf2_geometry_msgs  # noqa : enregistre les conversions PoseStamped


DETECTIONS_TOPIC = '/apriltag_detections_refined'
POSE_TOPIC = '/spot_pose_in_summit'

SUMMIT_FRAME = 'summit_xl_base_link'
SPOT_FRAME = 'spot_base_link'

TAG_FRAMES = {0: 'tag_0_link', 1: 'tag_1_link', 2: 'tag_2_link'}


def quat_msg_to_scipy(q_msg) -> R:
    """Convertit un geometry_msgs/Quaternion (x,y,z,w) en scipy Rotation."""
    return R.from_quat([q_msg.x, q_msg.y, q_msg.z, q_msg.w])


def scipy_to_quat_msg(rot: R, q_msg):
    """Ecrit la rotation scipy dans un Quaternion ROS (modifie in-place)."""
    x, y, z, w = rot.as_quat()
    q_msg.x = float(x)
    q_msg.y = float(y)
    q_msg.z = float(z)
    q_msg.w = float(w)


class PoseFuserNode(Node):

    def __init__(self):
        super().__init__('pose_fuser')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(
            AprilTagDetectionArray, DETECTIONS_TOPIC,
            self.on_detections, 10)
        self.pub_pose = self.create_publisher(
            PoseStamped, POSE_TOPIC, 10)

        self.get_logger().info(
            f"Pose fuser pret. Souscrit a {DETECTIONS_TOPIC}, "
            f"publie /spot_pose_in_summit et TF {SUMMIT_FRAME} -> {SPOT_FRAME}.")

    def on_detections(self, msg: AprilTagDetectionArray):
        if not msg.detections:
            return

        camera_frame = msg.header.frame_id
        estimates = []  # liste de (translation, rotation_scipy, weight, id)

        for det in msg.detections:
            if det.id not in TAG_FRAMES:
                continue

            tag_frame = TAG_FRAMES[det.id]

            try:
                tf_spot_tag = self.tf_buffer.lookup_transform(
                    SPOT_FRAME, tag_frame,
                    rclpy.time.Time(),
                    Duration(seconds=0.1))
            except Exception as e:
                self.get_logger().warn(
                    f"TF {SPOT_FRAME} -> {tag_frame} indisponible: {e}",
                    throttle_duration_sec=2.0)
                continue

            # T_cam_tag : pose detectee du tag dans le repere optical
            p = det.pose.pose.pose.position
            t_cam_tag = np.array([p.x, p.y, p.z])
            R_cam_tag = quat_msg_to_scipy(det.pose.pose.pose.orientation)

            # T_spot_tag : calibration statique (position du tag sur le Spot)
            t_spot_tag = np.array([
                tf_spot_tag.transform.translation.x,
                tf_spot_tag.transform.translation.y,
                tf_spot_tag.transform.translation.z,
            ])
            R_spot_tag = quat_msg_to_scipy(tf_spot_tag.transform.rotation)

            # T_tag_spot = inverse de T_spot_tag
            R_tag_spot = R_spot_tag.inv()
            t_tag_spot = -R_tag_spot.apply(t_spot_tag)

            # Composition : T_cam_spot = T_cam_tag . T_tag_spot
            R_cam_spot = R_cam_tag * R_tag_spot
            t_cam_spot = R_cam_tag.apply(t_tag_spot) + t_cam_tag

            weight = 1.0 / (max(0.1, p.z) ** 2)
            estimates.append((t_cam_spot, R_cam_spot, weight, det.id))

        if not estimates:
            return

        ids_used = [e[3] for e in estimates]
        self.get_logger().info(
            f"Detections: {ids_used} | Fusionne {len(estimates)} estimations",
            throttle_duration_sec=1.0)

        # Fusion position : moyenne ponderee
        total_weight = sum(e[2] for e in estimates)
        fused_pos = sum(e[0] * e[2] for e in estimates) / total_weight

        # Fusion rotation : on prend celle du tag le plus proche
        # (moyenne de quaternions non triviale, slerp/Markley overkill ici)
        closest = max(estimates, key=lambda e: e[2])
        fused_rot = closest[1]

        pose_in_cam = PoseStamped()
        pose_in_cam.header.frame_id = camera_frame
        pose_in_cam.header.stamp = msg.header.stamp
        pose_in_cam.pose.position.x = float(fused_pos[0])
        pose_in_cam.pose.position.y = float(fused_pos[1])
        pose_in_cam.pose.position.z = float(fused_pos[2])
        scipy_to_quat_msg(fused_rot, pose_in_cam.pose.orientation)

        try:
            pose_in_summit = self.tf_buffer.transform(
                pose_in_cam, SUMMIT_FRAME,
                timeout=Duration(seconds=0.1))
        except Exception as e:
            self.get_logger().warn(
                f"Echec transform {camera_frame} -> {SUMMIT_FRAME}: {e}",
                throttle_duration_sec=2.0)
            return

        self.pub_pose.publish(pose_in_summit)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp
        tf_msg.header.frame_id = SUMMIT_FRAME
        tf_msg.child_frame_id = SPOT_FRAME
        tf_msg.transform.translation.x = pose_in_summit.pose.position.x
        tf_msg.transform.translation.y = pose_in_summit.pose.position.y
        tf_msg.transform.translation.z = pose_in_summit.pose.position.z
        tf_msg.transform.rotation = pose_in_summit.pose.orientation
        self.tf_broadcaster.sendTransform(tf_msg)


def main():
    rclpy.init()
    node = PoseFuserNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
