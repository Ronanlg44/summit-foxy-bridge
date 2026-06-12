"""
Launch file pour publier les TF statiques de calibration extrinseque.

Publie 5 transformations :
- summit_xl_base_link -> camera_link              (D435i sur Summit, CAO OnShape)
- summit_xl_base_link -> summit_xl_front_laser_link (LiDAR Hokuyo sur Summit, CAO)
- spot_base_link -> tag_0_link                    (tag arriere du Spot)
- spot_base_link -> tag_1_link                    (tag flanc gauche du Spot)
- spot_base_link -> tag_2_link                    (tag flanc droit du Spot)

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
                '0', '0', '0',                  # parallele au sol
                'summit_xl_base_link', 'camera_link',
            ],
            output='screen',
        ),

        # Pose du LiDAR Hokuyo sur le Summit XL HL (mesure CAO OnShape)
        # Publiee ici en TRANSIENT_LOCAL pour contourner le bug QoS du bridge.
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

        # Tag 0 - arriere du Spot (face vers -X)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_spot_tag_0',
            arguments=[
                '-0.43', '0.00', '0.01',
                '3.14159', '0', '0',         # yaw=pi, pitch=0, roll=0
                'spot_base_link', 'tag_0_link',
            ],
            output='screen',
        ),

        # Tag 1 - flanc gauche du Spot (face vers +Y)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_spot_tag_1',
            arguments=[
                '0.00', '0.12', '0.01',
                '1.5708', '0', '0',          # yaw=pi/2, pitch=0, roll=0
                'spot_base_link', 'tag_1_link',
            ],
            output='screen',
        ),

        # Tag 2 - flanc droit du Spot (face vers -Y)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_spot_tag_2',
            arguments=[
                '0.00', '-0.12', '0.01',
                '-1.5708', '0', '0',         # yaw=-pi/2, pitch=0, roll=0
                'spot_base_link', 'tag_2_link',
            ],
            output='screen',
        ),
    ])
