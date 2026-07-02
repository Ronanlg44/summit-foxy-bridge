"""
Backend Flask de l'IHM Summit Control.

Fonctionnalites V1 :
- Boutons Start Mission / Start Ident / Stop / Status
- Boutons individuels par service (docker compose run)
- Section PID : enable/disable, publish_real_cmd
- Status systeme RPi4 : CPU, RAM, disque, temperature, tmux, containers

Toutes les commandes sont executees via SSH sur la RPi4.
"""

import os
import re
import sys
import time
import yaml
import threading
from pathlib import Path
from getpass import getpass

import paramiko
from flask import Flask, jsonify, render_template, request


# ============ Chargement de la config ============

CONFIG_PATH = Path(__file__).parent / "config.yaml"
with open(CONFIG_PATH) as f:
    CONFIG = yaml.safe_load(f)

RPI_HOST = CONFIG["rpi"]["host"]
RPI_USER = CONFIG["rpi"]["user"]
RPI_WORKDIR = CONFIG["rpi"]["workdir"]
COMPOSE_FILE = CONFIG["rpi"]["compose_file"]
LAUNCH_SCRIPT = CONFIG["rpi"]["launch_script"]
TMUX_SESSION = CONFIG["rpi"]["tmux_session"]

PID_NODE = CONFIG["pid_controller"]["node_name"]
ENABLE_TOPIC = CONFIG["pid_controller"]["enable_topic"]

WEB_HOST = CONFIG["web"]["host"]
WEB_PORT = CONFIG["web"]["port"]


# ============ Client SSH persistant ============

class SSHClient:
    """Wrapper autour de paramiko pour executer des commandes SSH."""

    def __init__(self, host, user, password):
        self.host = host
        self.user = user
        self.password = password
        self.client = paramiko.SSHClient()
        self.client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self._lock = threading.Lock()
        self._connect()

    def _connect(self):
        self.client.connect(
            hostname=self.host,
            username=self.user,
            password=self.password,
            timeout=10,
        )
        print(f"[SSH] Connecte a {self.user}@{self.host}")

    def exec(self, cmd, timeout=30):
        """Execute une commande et retourne (exit_code, stdout, stderr)."""
        with self._lock:
            try:
                transport = self.client.get_transport()
                if transport is None or not transport.is_active():
                    self._connect()
                    transport = self.client.get_transport()

                chan = transport.open_session()
                chan.settimeout(timeout)
                chan.exec_command(cmd)

                # Attendre la fin avec timeout global
                import time
                start = time.time()
                while not chan.exit_status_ready():
                    if time.time() - start > timeout:
                        chan.close()
                        return -1, "", f"Timeout global ({timeout}s)"
                    time.sleep(0.1)

                code = chan.recv_exit_status()
                out = b""
                err = b""
                while chan.recv_ready():
                    out += chan.recv(4096)
                while chan.recv_stderr_ready():
                    err += chan.recv_stderr(4096)
                chan.close()

                return code, out.decode("utf-8", errors="replace"), err.decode("utf-8", errors="replace")

            except Exception as e:
                print(f"[SSH] Erreur : {e}")
                try:
                    self._connect()
                except Exception as e2:
                    return -1, "", f"SSH reconnect failed: {e2}"
                return -1, "", f"SSH error: {e}"


ssh_client = None  # sera initialise dans main()


# ============ Helpers ============

def rpi_cmd(cmd_in_workdir):
    """Prefixe une commande avec 'cd workdir && ...'."""
    return f"cd {RPI_WORKDIR} && {cmd_in_workdir}"


def run_ros2_cmd(ros2_cmd):
    """
    Execute une commande ros2 dans un container shell temporaire.
    Prend 2-3 secondes (overhead docker run). Suffisant pour les actions ponctuelles.
    """
    full = rpi_cmd(
        f"docker compose -f {COMPOSE_FILE} run --rm shell "
        f'bash -c "{ros2_cmd}"'
    )
    return ssh_client.exec(full, timeout=30)


# ============ Flask app ============

app = Flask(__name__)


@app.route("/")
def index():
    return render_template(
        "index.html",
        services=CONFIG["services"],
        rpi_host=RPI_HOST,
        rpi_user=RPI_USER,
    )


# ============ Routes Mission ============

@app.route("/api/mission/<mode>", methods=["POST"])
def start_mission(mode):
    """Lance launch_all.rpi.sh <mode>."""
    if mode not in ("mission", "ident", "stop", "status"):
        return jsonify({"error": f"mode invalide : {mode}"}), 400

    cmd = rpi_cmd(f"{LAUNCH_SCRIPT} {mode}")
    # Detache : on ne veut pas bloquer la reponse HTTP le temps du sleep 15+15s
    if mode in ("mission", "ident"):
        # Lancement en arriere-plan
        cmd = f"nohup bash -c '{cmd}' > /tmp/launch.log 2>&1 &"
        code, out, err = ssh_client.exec(cmd, timeout=10)
        return jsonify({
            "status": "launched",
            "message": f"Pipeline demarre en arriere-plan (mode {mode})",
            "code": code,
            "stdout": out[-500:] if out else "",
            "stderr": err[-500:] if err else "",
        })
    else:
        # stop / status : bloquant mais rapide
        code, out, err = ssh_client.exec(cmd, timeout=30)
        return jsonify({
            "status": "done",
            "code": code,
            "stdout": out[-2000:] if out else "",
            "stderr": err[-500:] if err else "",
        })


# ============ Routes Services individuels ============

@app.route("/api/service/<name>/start", methods=["POST"])
def start_service(name):
    if name not in [s["name"] for s in CONFIG["services"]]:
        return jsonify({"error": f"service inconnu : {name}"}), 400

    # Verifier si la session tmux existe
    code, out, err = ssh_client.exec(
        f"tmux has-session -t {TMUX_SESSION} 2>/dev/null && echo YES || echo NO",
        timeout=5)
    tmux_exists = "YES" in out

    if tmux_exists:
        docker_cmd = (
            f"cd {RPI_WORKDIR} && "
            f"docker compose -f {COMPOSE_FILE} run --rm {name}"
        )
        # tmux new-window rend la main tout de suite
        cmd = f'tmux new-window -t {TMUX_SESSION} -n {name} "{docker_cmd}"'
        code, out, err = ssh_client.exec(cmd, timeout=5)
        method = "tmux window"
    else:
        # Detachement complet : redirection stdin/stdout/stderr + disown
        cmd = (
            f"cd {RPI_WORKDIR} && "
            f"nohup docker compose -f {COMPOSE_FILE} run --rm {name} "
            f"</dev/null >/tmp/{name}.log 2>&1 & disown"
        )
        # bash -c pour que & et disown fonctionnent
        full_cmd = f"bash -c '{cmd}'"
        code, out, err = ssh_client.exec(full_cmd, timeout=15)
        method = "background nohup"

    return jsonify({
        "status": "launched",
        "service": name,
        "method": method,
        "code": code,
    })

@app.route("/api/service/<name>/stop", methods=["POST"])
def stop_service(name):
    """Arrete un service specifique (via docker compose stop + rm)."""
    if name not in [s["name"] for s in CONFIG["services"]]:
        return jsonify({"error": f"service inconnu : {name}"}), 400

    cmd = rpi_cmd(
        f"docker compose -f {COMPOSE_FILE} stop {name} && "
        f"docker compose -f {COMPOSE_FILE} rm -f {name}"
    )
    code, out, err = ssh_client.exec(cmd, timeout=30)
    return jsonify({
        "status": "stopped",
        "service": name,
        "code": code,
        "stdout": out[-500:] if out else "",
        "stderr": err[-500:] if err else "",
    })


# ============ Routes PI Controller ============

@app.route("/api/pi/enable", methods=["POST"])
def pi_enable():
    ros_cmd = (
        f"ros2 topic pub {ENABLE_TOPIC} std_msgs/msg/Bool "
        f"\\\"data: true\\\" --once"
    )
    code, out, err = run_ros2_cmd(ros_cmd)
    return jsonify({
        "status": "ok" if code == 0 else "error",
        "code": code,
        "stdout": out[-500:], "stderr": err[-500:]
    })


@app.route("/api/pi/disable", methods=["POST"])
def pi_disable():
    ros_cmd = (
        f"ros2 topic pub {ENABLE_TOPIC} std_msgs/msg/Bool "
        f"\\\"data: false\\\" --once"
    )
    code, out, err = run_ros2_cmd(ros_cmd)
    return jsonify({
        "status": "ok" if code == 0 else "error",
        "code": code,
        "stdout": out[-500:], "stderr": err[-500:]
    })


@app.route("/api/pi/publish_real/<flag>", methods=["POST"])
def pi_publish_real(flag):
    """flag = 'true' ou 'false'."""
    if flag not in ("true", "false"):
        return jsonify({"error": "flag doit etre true ou false"}), 400

    ros_cmd = f"ros2 param set /{PID_NODE} publish_real_cmd {flag}"
    code, out, err = run_ros2_cmd(ros_cmd)
    return jsonify({
        "status": "ok" if code == 0 else "error",
        "code": code,
        "stdout": out[-500:], "stderr": err[-500:]
    })


# ============ Route Status systeme ============

@app.route("/api/status", methods=["GET"])
def status():
    """Retourne l'etat systeme de la RPi4."""
    # Une seule requete SSH pour tout recuperer (plus rapide)
    cmd = (
        "echo '--- CPU_TEMP ---' && "
        "cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null || echo N/A && "
        "echo '--- LOAD ---' && "
        "cat /proc/loadavg && "
        "echo '--- MEM ---' && "
        "free -m && "
        "echo '--- DISK ---' && "
        "df -h / && "
        "echo '--- TMUX ---' && "
        f"tmux has-session -t {TMUX_SESSION} 2>/dev/null && "
        f"echo TMUX_ACTIVE || echo TMUX_INACTIVE && "
        "echo '--- DOCKER ---' && "
        "docker ps --format '{{.Names}}|{{.Status}}'"
    )
    code, out, err = ssh_client.exec(cmd, timeout=10)

    if code != 0:
        return jsonify({"error": err or "SSH error", "code": code}), 500

    # Parse
    result = {
        "cpu_temp_c": None,
        "load_1min": None,
        "mem_used_mb": None,
        "mem_total_mb": None,
        "mem_percent": None,
        "disk_used": None,
        "disk_available": None,
        "disk_percent": None,
        "tmux_active": False,
        "containers": [],
    }

    sections = re.split(r"--- (\w+) ---\n", out)
    # sections = ["", "CPU_TEMP", "content", "LOAD", "content", ...]

    for i in range(1, len(sections), 2):
        key = sections[i].strip()
        content = sections[i + 1].strip() if i + 1 < len(sections) else ""

        if key == "CPU_TEMP":
            try:
                result["cpu_temp_c"] = int(content) / 1000.0
            except ValueError:
                pass

        elif key == "LOAD":
            parts = content.split()
            if len(parts) >= 3:
                result["load_1min"] = float(parts[0])
                result["load_5min"] = float(parts[1])
                result["load_15min"] = float(parts[2])

        elif key == "MEM":
            for line in content.splitlines():
                if line.startswith("Mem:"):
                    parts = line.split()
                    if len(parts) >= 3:
                        total = int(parts[1])
                        used = int(parts[2])
                        result["mem_total_mb"] = total
                        result["mem_used_mb"] = used
                        result["mem_percent"] = round(100 * used / total, 1)

        elif key == "DISK":
            for line in content.splitlines():
                if line.startswith("/dev/") or "overlay" in line:
                    parts = line.split()
                    if len(parts) >= 5:
                        result["disk_used"] = parts[2]
                        result["disk_available"] = parts[3]
                        result["disk_percent"] = parts[4]
                        break

        elif key == "TMUX":
            result["tmux_active"] = "TMUX_ACTIVE" in content

        elif key == "DOCKER":
            for line in content.splitlines():
                if "|" in line:
                    name, status_str = line.split("|", 1)
                    result["containers"].append({
                        "name": name.strip(),
                        "status": status_str.strip(),
                    })

    return jsonify(result)
    
# ============ Cleanup des containers ============
    
@app.route("/api/cleanup", methods=["POST"])
def cleanup():
    """Force le nettoyage : kill + rm tous les containers du projet."""
    cmd = rpi_cmd(
    f"docker ps -q --filter 'name=summit-foxy-bridge' | xargs -r docker kill; "
    f"sleep 1; "  # laisser le temps a --rm de finir
    f"docker ps -aq --filter 'name=summit-foxy-bridge' | xargs -r docker rm -f 2>/dev/null; "
    f"docker compose -f {COMPOSE_FILE} down --remove-orphans 2>/dev/null; "
    f"true"  # force exit code 0
    )
    code, out, err = ssh_client.exec(cmd, timeout=30)
    return jsonify({
        "status": "cleaned",
        "code": code,
        "stdout": out[-500:] if out else "",
        "stderr": err[-500:] if err else "",
    })


# ============ Main ============

def main():
    global ssh_client

    print("=" * 60)
    print("  SUMMIT CONTROL UI")
    print("=" * 60)
    print(f"  RPi4        : {RPI_USER}@{RPI_HOST}")
    print(f"  Workdir     : {RPI_WORKDIR}")
    print(f"  Web         : http://{WEB_HOST}:{WEB_PORT}")
    print("=" * 60)

    # Mot de passe SSH interactif au demarrage
    password = os.environ.get("SUMMIT_SSH_PASSWORD")
    if not password:
        password = getpass(f"Mot de passe SSH pour {RPI_USER}@{RPI_HOST} : ")

    try:
        ssh_client = SSHClient(RPI_HOST, RPI_USER, password)
    except Exception as e:
        print(f"[ERREUR] Connexion SSH impossible : {e}")
        sys.exit(1)

    print()
    print(f"[OK] Ouvre ton navigateur sur http://{WEB_HOST}:{WEB_PORT}")
    print("     (Ctrl+C pour arreter le serveur)")
    print()

    # Flask (pas de reloader pour eviter double-init)
    app.run(host=WEB_HOST, port=WEB_PORT, debug=False, use_reloader=False)


if __name__ == "__main__":
    main()
