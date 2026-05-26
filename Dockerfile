# ============================================================
#  Pont ROS 1 Kinetic (Summit XL) <-> ROS 2 Foxy (PC)
#  Ubuntu 20.04 : Noetic + Foxy desktop + ros1_bridge.
#  Aucune compilation source -> aucun conflit Python.
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
        ros-noetic-rosbash \
        ros-noetic-tf2-msgs \
        iputils-ping \
        iproute2 \
        dnsutils \
        nano \
        vim \
        less \
 && rm -rf /var/lib/apt/lists/*

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["dynamic"]
