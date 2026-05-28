# ============================================================
#  Pont ROS 1 Kinetic (Summit XL) <-> ROS 2 Foxy (PC)
#  + perception : RealSense D435i
#  Ubuntu 20.04 : Noetic + Foxy desktop + ros1_bridge.
# ============================================================
FROM ros:noetic-ros-base-focal

SHELL ["/bin/bash", "-lc"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        curl gnupg2 lsb-release ca-certificates \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu focal main" \
        > /etc/apt/sources.list.d/ros2.list \
 && apt-get update && apt-get install -y --no-install-recommends \
        ros-foxy-desktop \
        ros-foxy-ros1-bridge \
        ros-foxy-rmw-cyclonedds-cpp \
        ros-foxy-image-transport-plugins \
        ros-foxy-compressed-image-transport \
        ros-foxy-realsense2-camera \
        ros-foxy-realsense2-camera-msgs \
        ros-foxy-realsense2-description \
        ros-foxy-apriltag \
        ros-foxy-cv-bridge \
        ros-foxy-image-geometry \
        ros-foxy-vision-msgs \
        ros-noetic-rosbash \
        ros-noetic-tf2-msgs \
        iputils-ping \
        iproute2 \
        dnsutils \
        nano \
        vim \
        less \
 && rm -rf /var/lib/apt/lists/*
 
# ------------------------------------------------------------
# Outils de build (colcon + git) pour construire apriltag_ros
# ------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        git \
 && rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------
# Build d'apriltag (lib C AprilRobotics) + apriltag_ros (wrapper
# ROS2 Foxy, fork Adlink avec launcher RealSense pret a l'emploi)
# ------------------------------------------------------------
RUN mkdir -p /opt/apriltag_ws/src \
 && cd /opt/apriltag_ws/src \
 && git clone https://github.com/AprilRobotics/apriltag.git \
 && git clone https://github.com/Adlink-ROS/apriltag_ros.git -b foxy-devel \
 && cd /opt/apriltag_ws \
 && /bin/bash -c "source /opt/ros/foxy/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release" \
 && rm -rf build log

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["dynamic"]
