#!/usr/bin/env bash
# Lance le pipeline complet dans le bon ordre, dans une session tmux.
# Ordre obligatoire (bug DDS Foxy + parameter_bridge) :
#   1. realsense       (driver D435i)
#   2. tf_static       (calibration extrinseque, frames statiques)
#   3. perception_supervisor  ← AVANT bridge
#   4. bridge          (parameter_bridge selectif vers Summit XL)
#   5. apriltag, refiner, pose_fuser, yolo_detector (n'importe quel ordre)
#
# Usage :
#   ./launch_all.sh           lance tout
#   ./launch_all.sh stop      tue la session tmux
#   ./launch_all.sh attach    se connecte a la session

set -e

SESSION="summit_perception"
HERE="$(cd "$(dirname "$0")" && pwd)"
cd "$HERE"

# ----------------------------------------------------------------------
# Verification prerequis
# ----------------------------------------------------------------------

if ! command -v tmux >/dev/null 2>&1; then
  echo "[launch] tmux non installe. Installation :"
  echo "  sudo apt install tmux"
  exit 1
fi

if ! command -v docker >/dev/null 2>&1; then
  echo "[launch] docker non trouve."
  exit 1
fi

# ----------------------------------------------------------------------
# Gestion des arguments
# ----------------------------------------------------------------------

case "${1:-start}" in
  stop)
    echo "[launch] Arret de la session $SESSION..."
    tmux kill-session -t "$SESSION" 2>/dev/null || true
    echo "[launch] Nettoyage des containers..."
    docker compose down --remove-orphans
    echo "[launch] OK."
    exit 0
    ;;
  attach)
    tmux attach-session -t "$SESSION"
    exit 0
    ;;
  status)
    echo "[launch] Etat de la session $SESSION :"
    tmux list-windows -t "$SESSION" 2>/dev/null || echo "  Session non active."
    echo
    echo "[launch] Containers actifs :"
    docker ps --filter "label=com.docker.compose.project=summit_foxy" \
      --format "table {{.Names}}\t{{.Status}}"
    exit 0
    ;;
  start|"")
    ;;
  *)
    echo "Usage : $0 [start|stop|attach|status]"
    exit 1
    ;;
esac

# ----------------------------------------------------------------------
# Verification reseau (le Summit doit etre joignable)
# ----------------------------------------------------------------------

echo "[launch] Verification du reseau Summit (192.168.0.200)..."
if ! ping -c 1 -W 2 192.168.0.200 >/dev/null 2>&1; then
  echo "[launch] WARNING : Summit (192.168.0.200) ne repond pas."
  echo "[launch] Verifie que tu es bien connecte au wifi du Summit."
  read -p "[launch] Continuer quand meme ? [y/N] " confirm
  if [[ "$confirm" != "y" && "$confirm" != "Y" ]]; then
    exit 1
  fi
fi

# ----------------------------------------------------------------------
# Nettoyage des containers precedents
# ----------------------------------------------------------------------

echo "[launch] Arret des anciens containers..."
tmux kill-session -t "$SESSION" 2>/dev/null || true
docker kill $(docker ps -q) 2>/dev/null || true

# ----------------------------------------------------------------------
# Inhibe la mise en veille pendant la session
# ----------------------------------------------------------------------

if command -v systemd-inhibit >/dev/null 2>&1; then
  echo "[launch] Inhibition de la veille systeme..."
fi

# ----------------------------------------------------------------------
# Lancement orchestre via tmux
# ----------------------------------------------------------------------

echo "[launch] Creation de la session tmux '$SESSION'..."
tmux new-session -d -s "$SESSION" -n "realsense"

# Petit helper pour creer une fenetre et y lancer une commande
launch_in_window() {
  local name="$1"
  local cmd="$2"
  tmux new-window -t "$SESSION" -n "$name"
  tmux send-keys -t "$SESSION:$name" "cd '$HERE' && $cmd" C-m
}

# ----- 1. RealSense -----
echo "[launch] [1/8] realsense (driver D435i)..."
tmux send-keys -t "$SESSION:realsense" "cd '$HERE' && docker compose run --rm realsense" C-m
echo "[launch]   Attente de l'initialisation cam (15s)..."
sleep 15

# ----- 2. tf_static -----
echo "[launch] [2/8] tf_static (calibration extrinseque)..."
launch_in_window "tf_static" "docker compose run --rm tf_static"
sleep 3

# ----- 3. perception_supervisor (AVANT bridge) -----
echo "[launch] [3/8] perception_supervisor (AVANT bridge, ordre critique)..."
launch_in_window "supervisor" "docker compose run --rm perception_supervisor"
sleep 5

# ----- 4. bridge -----
echo "[launch] [4/8] bridge (parameter_bridge vers Summit)..."
launch_in_window "bridge" "docker compose run --rm bridge"
echo "[launch]   Attente de la creation des bridges (15s)..."
sleep 15

# ----- 5. apriltag -----
echo "[launch] [5/8] apriltag (detection)..."
launch_in_window "apriltag" "docker compose run --rm apriltag"
sleep 3

# ----- 6. refiner -----
echo "[launch] [6/8] refiner (raffinement pose via depth)..."
launch_in_window "refiner" "docker compose run --rm refiner"
sleep 3

# ----- 7. pose_fuser -----
echo "[launch] [7/8] pose_fuser (composition T_cam_tag . T_tag_spot)..."
launch_in_window "pose_fuser" "docker compose run --rm pose_fuser"
sleep 3

# ----- 8. yolo_detector -----
echo "[launch] [8/8] yolo_detector (YOLOv8m + ByteTrack)..."
launch_in_window "yolo" "docker compose run --rm yolo_detector"
sleep 3


