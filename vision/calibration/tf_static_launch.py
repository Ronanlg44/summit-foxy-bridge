"""
Launch file pour publier les TF statiques de calibration extrinseque.

Tag 0 (arrière du Spot) est la reference.
Les 4 autres tags sont sur les pattes du Spot, positionnes relativement au Tag 0.
Repere du Tag 0 (convention ROS) :
  +X = normal a la face (vers l'arriere du Spot physique)
  +Y = a gauche vu depuis le Tag 0 (donc a droite du Spot physique)
  +Z = vers le haut

Publie 6 transformations :
- summit_xl_base_link -> camera_link                (D435i sur Summit)
- summit_xl_base_link -> summit_xl_front_laser_link (LiDAR Hokuyo)
- dock_frame_0 -> dock_frame_1 (patte arriere gauche)
- dock_frame_0 -> dock_frame_2 (patte arriere droite)
- dock_frame_0 -> dock_frame_3 (patte avant gauche)
- dock_frame_0 -> dock_frame_4 (patte avant droite)

Convention static_transform_publisher en ROS 2 :
  arguments = [x, y, z, yaw, pitch, roll, parent_frame, child_frame]
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # Pose de la D435i sur le Summit XL HL (mesure CAO OnShape, parallele au sol)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_summit_camera',
            arguments=[
                '0.068', '0.000', '0.190',
                '0', '0', '0',
                'summit_xl_base_link', 'camera_link',
            ],
            output='screen',
        ),

        # Pose du LiDAR Hokuyo sur le Summit XL HL (mesure CAO OnShape)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_summit_lidar',
            arguments=[
                '0.006', '0.000', '0.220',
                '0', '0', '0',
                'summit_xl_base_link', 'summit_xl_front_laser_link',
            ],
            output='screen',
        ),

        # Tag 1 - patte arriere gauche du Spot
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_tag0_tag1',
            arguments=[
                '-0.039', '-0.220', '-0.036',
                '-1.5708', '0', '0',
                'dock_frame_0', 'dock_frame_1',
            ],
            output='screen',
        ),

        # Tag 2 - patte arriere droite du Spot
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_tag0_tag2',
            arguments=[
                '-0.039', '0.220', '-0.036',
                '1.5708', '0', '0',
                'dock_frame_0', 'dock_frame_2',
            ],
            output='screen',
        ),

        # Tag 3 - patte avant gauche du Spot
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_tag0_tag3',
            arguments=[
                '-0.639', '-0.220', '-0.037',
                '-1.5708', '0', '0',
                'dock_frame_0', 'dock_frame_3',
            ],
            output='screen',
        ),

        # Tag 4 - patte avant droite du Spot
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_tag0_tag4',
            arguments=[
                '-0.639', '0.220', '-0.037',
                '1.5708', '0', '0',
                'dock_frame_0', 'dock_frame_4',
            ],
            output='screen',
        ),
    ])
