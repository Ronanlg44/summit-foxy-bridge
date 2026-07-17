"""
Wrapper qui lance le node PID + le serveur WebSocket dans le meme container.

Le node PID tourne dans un subprocess (module python -m pid_apriltag.pid_apriltag_node)
et le WebSocket dans le processus principal.

Si l'un des deux meurt, on tue l'autre pour eviter les etats zombie.
"""

import os
import signal
import subprocess
import sys
import time


def main():
    print("[wrapper] Demarrage du wrapper PID + WebSocket")

    # Lance le node PID en subprocess
    pid_proc = subprocess.Popen(
        [sys.executable, "-m", "pid_apriltag.pid_apriltag_node"],
        stdout=None,
        stderr=None,
    )
    print(f"[wrapper] Node PID lance (pid={pid_proc.pid})")

    # Laisse le node PID demarrer et enregistrer ses services
    time.sleep(3.0)

    # Lance le WebSocket server dans le processus principal
    # (il utilise asyncio, doit etre le main thread)
    def cleanup(*args):
        print("[wrapper] Signal recu, arret du subprocess PID")
        try:
            pid_proc.terminate()
            pid_proc.wait(timeout=3.0)
        except Exception:
            pid_proc.kill()
        sys.exit(0)

    signal.signal(signal.SIGTERM, cleanup)
    signal.signal(signal.SIGINT, cleanup)

    try:
        from pid_apriltag import ws_server
        ws_server.main()
    except Exception as e:
        print(f"[wrapper] Erreur WebSocket : {e}")
        cleanup()

    # Si le WebSocket sort proprement, on cleanup aussi
    cleanup()


if __name__ == '__main__':
    main()
