#!/usr/bin/env bash
# Source ROS 1 (Noetic) + ROS 2 (Foxy), attend le master du robot
# par un test TCP pur (fiable en environnement mixte), puis lance
# le pont dynamique. Foxy <-> Foxy : pas de pont vers Jazzy.
set -e

: "${ROS_MASTER_URI:=http://192.168.0.200:11311}"
: "${ROS_IP:=}"
: "${ROS_DOMAIN_ID:=0}"

export ROS_MASTER_URI ROS_DOMAIN_ID
[ -n "$ROS_IP" ] && export ROS_IP

# Mode debug : un shell dans le conteneur, sans lancer le pont.
if [ "${1:-dynamic}" = "shell" ]; then
  echo "[pont] Mode shell (debug). ROS_MASTER_URI=$ROS_MASTER_URI ROS_IP=${ROS_IP:-auto}"
  source /opt/ros/noetic/setup.bash
  source /opt/ros/foxy/setup.bash
  [ -f /opt/apriltag_ws/install/setup.bash ] && source /opt/apriltag_ws/install/setup.bash
  exec bash
fi

MASTER_HOSTPORT="${ROS_MASTER_URI#http://}"
MASTER_HOST="${MASTER_HOSTPORT%%:*}"
MASTER_PORT="${MASTER_HOSTPORT##*:}"
MASTER_PORT="${MASTER_PORT%%/*}"

echo "[pont] ROS_MASTER_URI=$ROS_MASTER_URI"
echo "[pont] ROS_IP=${ROS_IP:-auto}  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "[pont] Attente du master ROS 1 ($MASTER_HOST:$MASTER_PORT)..."
until bash -c "exec 3<>/dev/tcp/$MASTER_HOST/$MASTER_PORT" 2>/dev/null; do
  sleep 2
done
echo "[pont] Master ROS 1 joignable. Demarrage du pont Kinetic <-> Foxy."

source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash
[ -f /opt/apriltag_ws/install/setup.bash ] && source /opt/apriltag_ws/install/setup.bash

case "${1:-dynamic}" in
  dynamic)
    exec ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
    ;;
  *)
    exec "$@"
    ;;
esac
