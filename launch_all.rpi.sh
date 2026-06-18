#!/usr/bin/env bash
# Lance le pipeline complet en autonomie sur la RPi4 dans tmux.
# Adapte du launch_all.sh du PC avec :
#   - Sequence sans realsense_ir/apriltag_ir (pas utile en headless)
#   - Modes "mission" (avec PID) ou "ident" (avec step_input)
#   - Pas d'inhibition de veille (RPi4 n'en a pas besoin)
#
# Ordre obligatoire (bug DDS Foxy + parameter_bridge) :
#   1. realsense              (driver D435i)
#   2. tf_static              (calibration extrinseque)
#   3. perception_supervisor  ← AVANT bridge (ordre critique)
#   4. bridge                 (parameter_bridge selectif vers Summit XL)
#   5. apriltag               (detection)
#   6. refiner                (raffinement pose via depth)
#   7. pose_fuser             (composition T_cam_tag . T_tag_spot)
#   8. yolo_detector          (fallback YOLOv8m)
#   9. pid_apriltag ou step_input (selon mode)
#
# Usage :
#   ./launch_all.rpi.sh mission   lance tout pour la mission (avec PID)
#   ./launch_all.rpi.sh ident     lance tout pour l'identification (avec step_input)
#   ./launch_all.rpi.sh stop      tue la session tmux + containers
#   ./launch_all.rpi.sh attach    se connecte a la session
#   ./launch_all.rpi.sh status    voir l'etat

set -e

SESSION="summit_rpi"
HERE="$(cd "$(dirname "$0")" && pwd)"
COMPOSE_FILE="docker-compose.rpi.yml"
cd "$HERE"

# ----------------------------------------------------------------------
# Verification prerequis
# ----------------------------------------------------------------------

if ! command -v tmux >/dev/null 2>&1; then
  echo "[launch] tmux non installe :  sudo apt install tmux"
  exit 1
fi

if ! command -v docker >/dev/null 2>&1; then
  echo "[launch] docker non trouve."
  exit 1
fi

if [ ! -f "$COMPOSE_FILE" ]; then
  echo "[launch] $COMPOSE_FILE introuvable."
  exit 1
fi

if ! docker image inspect summit-rpi:latest >/dev/null 2>&1; then
  echo "[launch] Image summit-rpi:latest absente."
  echo "[launch] Build : docker build -f Dockerfile.rpi -t summit-rpi:latest ."
  exit 1
fi

if [ ! -f .env ]; then
  echo "[launch] .env absent. Copie : cp env.rpi.example .env"
  exit 1
fi

# ----------------------------------------------------------------------
# Gestion des arguments
# ----------------------------------------------------------------------

MODE="${1:-mission}"

case "$MODE" in
  stop)
    echo "[launch] Arret de la session $SESSION..."
    tmux kill-session -t "$SESSION" 2>/dev/null || true
    echo "[launch] Nettoyage des containers..."
    docker compose -f "$COMPOSE_FILE" down --remove-orphans
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
    docker ps --format "table {{.Names}}\t{{.Status}}" | grep -E "summit|bridge|realsense|apriltag|refiner|pose_fuser|tf_static|yolo|supervisor|pid|step" || echo "  Aucun."
    exit 0
    ;;
  mission|ident)
    echo "[launch] Mode : $MODE"
    ;;
  *)
    echo "Usage : $0 [mission|ident|stop|attach|status]"
    exit 1
    ;;
esac

# ----------------------------------------------------------------------
# Verification reseau (le Summit doit etre joignable)
# ----------------------------------------------------------------------

source .env
ROBOT_IP="${ROBOT_IP:-192.168.0.200}"

echo "[launch] Verification du reseau Summit ($ROBOT_IP)..."
if ! ping -c 1 -W 2 "$ROBOT_IP" >/dev/null 2>&1; then
  echo "[launch] WARNING : Summit ($ROBOT_IP) ne repond pas."
  echo "[launch] Verifie la connexion Ethernet vers le robot."
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
docker compose -f "$COMPOSE_FILE" down --remove-orphans 2>/dev/null || true

# ----------------------------------------------------------------------
# Helper : lance une commande dans une fenetre tmux
# ----------------------------------------------------------------------

launch_in_window() {
  local name="$1"
  local cmd="$2"
  tmux new-window -t "$SESSION" -n "$name"
  tmux send-keys -t "$SESSION:$name" "cd '$HERE' && $cmd" C-m
}

# ----------------------------------------------------------------------
# Lancement orchestre dans tmux
# ----------------------------------------------------------------------

echo "[launch] Creation de la session tmux '$SESSION'..."
tmux new-session -d -s "$SESSION" -n "realsense"

# ----- 1. RealSense -----
echo "[launch] [1/9] realsense (driver D435i)..."
tmux send-keys -t "$SESSION:realsense" "cd '$HERE' && docker compose -f $COMPOSE_FILE run --rm realsense" C-m
echo "[launch]   Attente initialisation cam (15s)..."
sleep 15

# ----- 2. tf_static -----
echo "[launch] [2/9] tf_static (calibration extrinseque)..."
launch_in_window "tf_static" "docker compose -f $COMPOSE_FILE run --rm tf_static"
sleep 3

# ----- 3. perception_supervisor (AVANT bridge) -----
echo "[launch] [3/9] perception_supervisor (AVANT bridge, ordre critique)..."
launch_in_window "supervisor" "docker compose -f $COMPOSE_FILE run --rm perception_supervisor"
sleep 5

# ----- 4. bridge -----
echo "[launch] [4/9] bridge (parameter_bridge vers Summit)..."
launch_in_window "bridge" "docker compose -f $COMPOSE_FILE run --rm bridge"
echo "[launch]   Attente creation des bridges (15s)..."
sleep 15

# ----- 5. apriltag -----
echo "[launch] [5/9] apriltag (detection)..."
launch_in_window "apriltag" "docker compose -f $COMPOSE_FILE run --rm apriltag"
sleep 3

# ----- 6. refiner -----
echo "[launch] [6/9] refiner (raffinement pose via depth)..."
launch_in_window "refiner" "docker compose -f $COMPOSE_FILE run --rm refiner"
sleep 3

# ----- 7. pose_fuser -----
echo "[launch] [7/9] pose_fuser..."
launch_in_window "pose_fuser" "docker compose -f $COMPOSE_FILE run --rm pose_fuser"
sleep 3

# ----- 8. yolo_detector -----
echo "[launch] [8/9] yolo_detector (YOLOv8m + ByteTrack)..."
launch_in_window "yolo" "docker compose -f $COMPOSE_FILE run --rm yolo_detector"
sleep 3

# ----- 9. PID ou step_input selon mode -----
case "$MODE" in
  mission)
    echo "[launch] [9/9] pid_apriltag (asservissement 2-DOF)..."
    launch_in_window "pid" "docker compose -f $COMPOSE_FILE run --rm pid_apriltag"
    ;;
  ident)
    echo "[launch] [9/9] step_input (echelons cmd_vel)..."
    launch_in_window "step" "docker compose -f $COMPOSE_FILE run --rm step_input"
    ;;
esac

# ----- Shell utilitaire en bonus pour debug/bagging -----
launch_in_window "shell" "docker compose -f $COMPOSE_FILE run --rm shell"

# ----------------------------------------------------------------------
# Resume
# ----------------------------------------------------------------------

echo
echo "=============================================="
echo "[launch] Pipeline complet lance (mode: $MODE)"
echo "=============================================="
echo
echo "Commandes utiles :"
echo "  $0 attach       # voir les logs (Ctrl+B puis N/P pour naviguer)"
echo "  $0 status       # etat des fenetres et containers"
echo "  $0 stop         # tout arreter"
echo
echo "Dans tmux :"
echo "  Ctrl+B puis N/P  -> fenetre suivante/precedente"
echo "  Ctrl+B puis 0-9  -> aller a la fenetre N"
echo "  Ctrl+B puis D    -> se detacher (les containers continuent)"
echo

if [ -t 0 ]; then
  echo "[launch] Attachement automatique dans 3 secondes (Ctrl+C pour ignorer)..."
  sleep 3
  tmux attach -t "$SESSION"
fi
