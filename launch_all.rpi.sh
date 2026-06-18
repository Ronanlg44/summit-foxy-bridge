#!/bin/bash
# =====================================================================
# launch_all.rpi.sh
# Lance le pipeline complet en autonomie sur la RPi4 via tmux
#
# Usage :
#   ./launch_all.rpi.sh mission    # production : vision + bridge + pid
#   ./launch_all.rpi.sh ident      # identification : vision + bridge + step_input
#   ./launch_all.rpi.sh stop       # arrete tout proprement
# =====================================================================

set -e

cd "$(dirname "$0")"

MODE="${1:-mission}"
SESSION="summit"

case "$MODE" in
    stop)
        echo "Arret de tous les conteneurs..."
        docker compose -f docker-compose.rpi.yml down
        tmux kill-session -t "$SESSION" 2>/dev/null || true
        echo "Tout arrete."
        exit 0
        ;;
    mission|ident)
        ;;
    *)
        echo "Usage : $0 {mission|ident|stop}"
        exit 1
        ;;
esac

# Verifier que .env existe
if [ ! -f .env ]; then
    echo "ERREUR : .env absent. Copier d'abord env.rpi.example en .env"
    exit 1
fi

# Verifier que l'image existe
if ! docker image inspect summit-rpi:latest > /dev/null 2>&1; then
    echo "ERREUR : image summit-rpi:latest absente."
    echo "Build avec : docker build -f Dockerfile.rpi -t summit-rpi:latest ."
    exit 1
fi

# Tuer ancienne session tmux si existante
tmux kill-session -t "$SESSION" 2>/dev/null || true

echo "=============================================="
echo "Lancement Summit RPi4 - mode: $MODE"
echo "=============================================="

# Creer une session tmux detachee avec les services
tmux new-session -d -s "$SESSION" -n "compose" \
    "docker compose -f docker-compose.rpi.yml --profile $MODE up"

# Onglet pour shell de debug
tmux new-window -t "$SESSION:1" -n "shell" \
    "sleep 5 && docker exec -it shell bash || bash"

# Onglet pour topics ros2
tmux new-window -t "$SESSION:2" -n "topics" \
    "sleep 10 && docker exec -it shell bash -c 'source /opt/ros/foxy/setup.bash && ros2 topic list && bash' || bash"

echo ""
echo "Services lances en tmux session '$SESSION'"
echo ""
echo "Commandes utiles :"
echo "  tmux attach -t $SESSION       # se connecter aux logs"
echo "  Ctrl+B puis N/P               # naviguer entre onglets"
echo "  Ctrl+B puis D                 # se detacher (les conteneurs continuent)"
echo "  $0 stop                       # tout arreter"
echo ""

if [ -t 0 ]; then
    echo "Attachement automatique dans 3 secondes (Ctrl+C pour ignorer)..."
    sleep 3
    tmux attach -t "$SESSION"
fi
