"""
Raffinement de pose AprilTag avec la profondeur de la D435i.

La pose 6D que publie apriltag_ros est obtenue par PnP : a partir des
4 coins du tag dans l'image et de la taille connue du tag, le solver
calcule la position 3D. Cette pose est correcte en orientation mais
bruitee en distance (~5-10 cm a 2-3 m).

Ce noeud :
1. Souscrit aux detections AprilTag et a l'image depth alignee sur RGB.
2. Pour chaque tag detecte, projette la position 3D du tag (issue de
   PnP) dans l'image via le modele pinhole forward pour obtenir le
   pixel central (u, v).
3. Lit la depth a (u, v) avec mediane sur un patch 5x5 pour gommer le
   bruit et ignorer les pixels invalides.
4. Reconstruit la position 3D du centre du tag via pinhole inverse
   (a partir des intrinseques camera_info et de la depth lue).
5. Garde l'orientation issue de PnP (fiable) mais remplace la position.
6. Republie sur /apriltag_detections_refined et envoie une TF
   "<frame>_link" visualisable dans RViz a cote de l'originale.
   Nom suivant la convention TF du tf_static (calibration extrinseque)
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from apriltag_msgs.msg import AprilTagDetectionArray
from cv_bridge import CvBridge

DETECTIONS_TOPIC = '/apriltag_detections'
DEPTH_TOPIC = '/camera/aligned_depth_to_color/image_raw'
CAMERA_INFO_TOPIC = '/camera/aligned_depth_to_color/camera_info'
REFINED_TOPIC = '/apriltag_detections_refined'

# Patch (2r+1)x(2r+1) pour la lecture depth avec mediane.
DEPTH_PATCH_RADIUS = 2  # 5x5

# Filtre depth : ignorer 0 (mesure invalide) et hors portee D435i.
DEPTH_MIN_M = 0.1
DEPTH_MAX_M = 6.0


class ApriltagRefinerNode(Node):

    def __init__(self):
        super().__init__('apriltag_refiner')

        self.bridge = CvBridge()

        self.latest_depth = None
        self.latest_depth_stamp = None
        self.fx = self.fy = self.cx = self.cy = None

        self.create_subscription(
            CameraInfo, CAMERA_INFO_TOPIC, self.on_camera_info, 10)
        self.create_subscription(
            Image, DEPTH_TOPIC, self.on_depth, 10)
        self.create_subscription(
            AprilTagDetectionArray, DETECTIONS_TOPIC,
            self.on_detections, 10)

        self.pub_refined = self.create_publisher(
            AprilTagDetectionArray, REFINED_TOPIC, 10)

        self.get_logger().info(
            f"Refiner pret. Souscrit a {DETECTIONS_TOPIC}, {DEPTH_TOPIC}, "
            f"{CAMERA_INFO_TOPIC}. Publie sur {REFINED_TOPIC} + TF.")

    def on_camera_info(self, msg: CameraInfo):
        if self.fx is None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.get_logger().info(
                f"Intrinseques recus : fx={self.fx:.1f}, fy={self.fy:.1f}, "
                f"cx={self.cx:.1f}, cy={self.cy:.1f}")

    def on_depth(self, msg: Image):
        self.latest_depth = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough')
        self.latest_depth_stamp = msg.header.stamp

    def on_detections(self, msg: AprilTagDetectionArray):
        if self.latest_depth is None or self.fx is None:
            return

        if not msg.detections:
            self.pub_refined.publish(msg)
            return

        refined_msg = AprilTagDetectionArray()
        refined_msg.header = msg.header

        for det in msg.detections:
            # Position 3D du tag dans le frame camera, issue de PnP
            pose_in = det.pose.pose.pose
            x_pnp = pose_in.position.x
            y_pnp = pose_in.position.y
            z_pnp = pose_in.position.z

            # Si z_pnp est nul ou negatif (cas pathologique), on republie tel quel
            if z_pnp <= 0:
                refined_msg.detections.append(det)
                continue

            # Projection pinhole forward : (X, Y, Z) -> (u, v)
            u = int(round(self.fx * x_pnp / z_pnp + self.cx))
            v = int(round(self.fy * y_pnp / z_pnp + self.cy))

            # Lecture depth avec patch + mediane
            z_metres = self.read_depth_at(u, v)

            if z_metres is None:
                # Depth invalide : on republie le PnP brut
                refined_msg.detections.append(det)
                continue

            # Pinhole inverse : (u, v, z_depth) -> (X, Y, Z)
            x_refined = (u - self.cx) * z_metres / self.fx
            y_refined = (v - self.cy) * z_metres / self.fy

            # Construction de la detection refined
            new_det = det
            new_det.pose.pose.pose.position.x = x_refined
            new_det.pose.pose.pose.position.y = y_refined
            new_det.pose.pose.pose.position.z = z_metres
            # Orientation conservee depuis PnP

            refined_msg.detections.append(new_det)

        self.pub_refined.publish(refined_msg)

    def read_depth_at(self, u: int, v: int):
        h, w = self.latest_depth.shape
        r = DEPTH_PATCH_RADIUS

        # Le pixel projete est-il dans l'image ? Sinon on n'a pas de depth.
        if u < 0 or u >= w or v < 0 or v >= h:
            return None

        u_min, u_max = max(0, u - r), min(w, u + r + 1)
        v_min, v_max = max(0, v - r), min(h, v + r + 1)

        patch = self.latest_depth[v_min:v_max, u_min:u_max].astype(np.float32)
        valid = (patch > DEPTH_MIN_M * 1000) & (patch < DEPTH_MAX_M * 1000)

        if not valid.any():
            return None

        z_mm = float(np.median(patch[valid]))
        return z_mm / 1000.0


def main():
    rclpy.init()
    node = ApriltagRefinerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
