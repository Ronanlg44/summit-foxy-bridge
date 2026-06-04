#!/bin/bash
# Lance les TF statiques pour la calibration extrinseque.

source /opt/ros/foxy/setup.bash

# Pre-creer le dossier de logs ROS (sinon spdlog crash en race condition)
mkdir -p /root/.ros/log
chmod 755 /root/.ros/log

echo "[tf_static] Demarrage des publishers TF statiques..."

# Pose de la D435i sur le Summit
ros2 run tf2_ros static_transform_publisher \
  0.206 0.000 0.136  0 0.3986 0 \
  summit_xl_base_link camera_link &
sleep 2

# Tag 0 (arriere du Spot)
ros2 run tf2_ros static_transform_publisher \
  -0.43 0.00 0.01  0 1.5708 0 \
  spot_base_link tag_0_link &
sleep 2

# Tag 1 (flanc gauche du Spot)
ros2 run tf2_ros static_transform_publisher \
  0.00 0.12 0.01  0 -1.5708 -1.5708 \
  spot_base_link tag_1_link &
sleep 2

# Tag 2 (flanc droit du Spot)
ros2 run tf2_ros static_transform_publisher \
  0.00 -0.12 0.01  0 -1.5708 1.5708 \
  spot_base_link tag_2_link &

echo "[tf_static] 4 publishers lances. Ctrl-C pour arreter."
wait
