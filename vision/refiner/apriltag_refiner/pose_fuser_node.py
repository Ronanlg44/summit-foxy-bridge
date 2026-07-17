#!/usr/bin/env python3
"""
pose_fuser V5 : fusion multi-tags avec ancrage sur Tag 0.

V5 : contourne le probleme de QoS incompatible avec /tf_static (Foxy)
en hardcodant les transformations statiques tag_N -> tag_0 dans le code.
Les valeurs sont les memes que dans tf_static_launch.py.

Pour chaque tag N detecte :
1. Lit la pose du tag dans le message (frame camera_color_optical_frame)
2. Applique la transformation statique tag_N -> tag_0 (hardcodee)
3. Obtient la pose estimee du Tag 0 dans le repere camera
4. Fusionne toutes les estimations (moyenne ponderee 1/Z^2)
5. Transforme la pose fusionnee vers summit_xl_base_link (via TF standard)
6. Publie sur /spot_pose_in_summit
"""


import math
import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import tf2_geometry_msgs  # noqa : enregistre les conversions PoseStamped

from geometry_msgs.msg import PoseStamped, TransformStamped
from apriltag_msgs.msg import AprilTagDetectionArray


OUTPUT_TOPIC = '/spot_pose_in_summit'
DETECTIONS_TOPIC = '/apriltag_detections_refined'
BASE_FRAME = 'summit_xl_base_link'


# ============================================================
# Transformations statiques hardcodees : tag_N -> tag_0
# ============================================================
# Valeurs identiques a celles du tf_static_launch.py.
# Format : (x, y, z, yaw, pitch, roll) pour tag_0 -> tag_N (parent -> child)
LAUNCH_VALUES = {
    1: (-0.039, -0.220, -0.036, -1.5708, 0.0, 0.0),
    2: (-0.039,  0.220, -0.036,  1.5708, 0.0, 0.0),
    3: (-0.639, -0.220, -0.037, -1.5708, 0.0, 0.0),
    4: (-0.639,  0.220, -0.037,  1.5708, 0.0, 0.0),
}


def _build_static_transforms():
    """Precalcule les matrices tag_N -> tag_0 (inverse de parent -> child)."""
    transforms = {}
    for tag_id, (x, y, z, yaw, pitch, roll) in LAUNCH_VALUES.items():
        # Rotation ZYX (yaw autour de Z, pitch autour de Y, roll autour de X)
        cy, sy = math.cos(yaw), math.sin(yaw)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cr, sr = math.cos(roll), math.sin(roll)

        R_parent_child = np.array([
            [cy * cp,  cy * sp * sr - sy * cr,  cy * sp * cr + sy * sr],
            [sy * cp,  sy * sp * sr + cy * cr,  sy * sp * cr - cy * sr],
            [-sp,      cp * sr,                 cp * cr],
        ])

        M_parent_to_child = np.eye(4)
        M_parent_to_child[:3, :3] = R_parent_child
        M_parent_to_child[:3, 3] = [x, y, z]

        # tag_N -> tag_0 = inverse de tag_0 -> tag_N
        transforms[tag_id] = np.linalg.inv(M_parent_to_child)

    return transforms


STATIC_TRANSFORMS_TAG_TO_TAG0 = _build_static_transforms()


# ============================================================
# Utilitaires quaternions et matrices
# ============================================================

def quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Convertit un quaternion (x, y, z, w) en matrice de rotation 3x3."""
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return np.array([
        [1 - 2 * (yy + zz), 2 * (xy - wz),     2 * (xz + wy)],
        [2 * (xy + wz),     1 - 2 * (xx + zz), 2 * (yz - wx)],
        [2 * (xz - wy),     2 * (yz + wx),     1 - 2 * (xx + yy)],
    ])


def matrix_to_pose(M: np.ndarray) -> tuple:
    """Convertit une matrice 4x4 en (position, quaternion)."""
    t = M[:3, 3]
    R = M[:3, :3]
    trace = np.trace(R)

    if trace > 0:
        s = 2.0 * math.sqrt(trace + 1.0)
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s

    return (t, np.array([x, y, z, w]))


def average_quaternions(quaternions: list, weights: list) -> np.ndarray:
    """Moyenne ponderee de quaternions (methode Markley : eigenvector)."""
    if len(quaternions) == 1:
        return quaternions[0]

    weights = np.array(weights, dtype=float)
    weights /= weights.sum()

    q_ref = quaternions[0]
    aligned = []
    for q in quaternions:
        if np.dot(q, q_ref) < 0:
            aligned.append(-np.array(q))
        else:
            aligned.append(np.array(q))

    M = np.zeros((4, 4))
    for q, w in zip(aligned, weights):
        M += w * np.outer(q, q)

    eigenvalues, eigenvectors = np.linalg.eigh(M)
    q_avg = eigenvectors[:, -1]
    if q_avg[3] < 0:
        q_avg = -q_avg
    return q_avg


# ============================================================
# Node principal
# ============================================================

class PoseFuserNode(Node):

    def __init__(self):
        super().__init__('pose_fuser')

        # TransformListener uniquement pour camera_color_optical_frame -> summit_xl_base_link
        # (les transformations tag_N -> tag_0 sont hardcodees)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.create_subscription(
            AprilTagDetectionArray,
            DETECTIONS_TOPIC,
            self.on_detections,
            10,
        )

        self.pub_pose = self.create_publisher(PoseStamped, OUTPUT_TOPIC, 10)

        self.get_logger().info("=" * 70)
        self.get_logger().info("pose_fuser V5 : ancrage Tag 0, transformations hardcodees")
        self.get_logger().info(f"  Topic detections  : {DETECTIONS_TOPIC}")
        self.get_logger().info(f"  Topic pose        : {OUTPUT_TOPIC}")
        self.get_logger().info(f"  Frame de sortie   : {BASE_FRAME}")
        self.get_logger().info(f"  Tags supportes    : 0, {list(LAUNCH_VALUES.keys())}")
        self.get_logger().info("=" * 70)

    def get_static_transform_tagN_to_tag0(self, tag_id: int) -> np.ndarray:
        """Retourne la matrice qui transforme un point tag_N en tag_0.
        Utilise les valeurs hardcodees (evite les problemes QoS avec /tf_static).
        """
        if tag_id == 0:
            return np.eye(4)
        if tag_id in STATIC_TRANSFORMS_TAG_TO_TAG0:
            return STATIC_TRANSFORMS_TAG_TO_TAG0[tag_id]
        return None

    def on_detections(self, msg: AprilTagDetectionArray):
        """Pour chaque tag detecte, calcule la pose estimee du Tag 0."""
        if not msg.detections:
            return

        camera_frame = msg.header.frame_id
        estimations = []

        for detection in msg.detections:
            tag_id = detection.id

            # 1. Pose du tag N dans le repere camera (lue du message)
            p = detection.pose.pose.pose.position
            q = detection.pose.pose.pose.orientation

            M_cam_to_tagN = np.eye(4)
            M_cam_to_tagN[:3, :3] = quat_to_matrix(q.x, q.y, q.z, q.w)
            M_cam_to_tagN[:3, 3] = [p.x, p.y, p.z]

            # 2. Transformation statique tag_N -> tag_0 (hardcodee)
            M_tagN_to_tag0 = self.get_static_transform_tagN_to_tag0(tag_id)
            if M_tagN_to_tag0 is None:
                self.get_logger().warn(
                    f"Tag inconnu : {tag_id}",
                    throttle_duration_sec=2.0,
                )
                continue

            # 3. Pose du Tag 0 dans le repere camera
            M_cam_to_tag0 = M_cam_to_tagN @ M_tagN_to_tag0

            pos, quat = matrix_to_pose(M_cam_to_tag0)
            distance = float(p.z) if p.z > 0.1 else 0.1

            estimations.append((pos, quat, distance, tag_id))

        if not estimations:
            return

        # Fusion 1/Z^2
        positions = np.array([e[0] for e in estimations])
        quaternions = np.array([e[1] for e in estimations])
        weights = np.array([1.0 / (e[2] ** 2) for e in estimations])
        weights_normalized = weights / weights.sum()

        pos_fused = np.average(positions, axis=0, weights=weights_normalized)
        quat_fused = average_quaternions(list(quaternions), list(weights_normalized))

        ids_used = [e[3] for e in estimations]
        self.get_logger().info(
            f"Fusion : {len(estimations)} tag(s) {ids_used}",
            throttle_duration_sec=1.0,
        )

        # PoseStamped dans le repere camera
        pose_in_cam = PoseStamped()
        pose_in_cam.header.frame_id = camera_frame
        pose_in_cam.header.stamp = msg.header.stamp
        pose_in_cam.pose.position.x = float(pos_fused[0])
        pose_in_cam.pose.position.y = float(pos_fused[1])
        pose_in_cam.pose.position.z = float(pos_fused[2])
        pose_in_cam.pose.orientation.x = float(quat_fused[0])
        pose_in_cam.pose.orientation.y = float(quat_fused[1])
        pose_in_cam.pose.orientation.z = float(quat_fused[2])
        pose_in_cam.pose.orientation.w = float(quat_fused[3])

        # Transformation vers summit_xl_base_link (via TF standard)
        try:
            pose_in_summit = self.tf_buffer.transform(
                pose_in_cam,
                BASE_FRAME,
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except Exception as e:
            self.get_logger().warn(
                f"Echec transform {camera_frame} -> {BASE_FRAME} : {e}",
                throttle_duration_sec=2.0,
            )
            return

        self.pub_pose.publish(pose_in_summit)


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
