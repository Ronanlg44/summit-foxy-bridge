"""
Backend Flask de l'IHM Summit Control (V4 - avec latence SSH mesuree).
"""

import os
import re
import sys
import time
import math
import yaml
import threading
import shlex
from pathlib import Path
from getpass import getpass

import paramiko
from flask import Flask, jsonify, render_template, request


CONFIG_PATH = Path(__file__).parent / "config.yaml"
with open(CONFIG_PATH) as f:
    CONFIG = yaml.safe_load(f)

RPI_HOST = CONFIG["rpi"]["host"]
RPI_USER = CONFIG["rpi"]["user"]
RPI_WORKDIR = CONFIG["rpi"]["workdir"]
COMPOSE_FILE = CONFIG["rpi"]["compose_file"]
LAUNCH_SCRIPT = CONFIG["rpi"]["launch_script"]
TMUX_SESSION = CONFIG["rpi"]["tmux_session"]

WEB_HOST = CONFIG["web"]["host"]
WEB_PORT = CONFIG["web"]["port"]


class SSHClient:

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
            timeout=30,
        )
        print(f"[SSH] Connecte a {self.user}@{self.host}")

    def exec(self, cmd, timeout=30):
        """Execute une commande et retourne (exit_code, stdout, stderr, latency_ms)."""
        with self._lock:
            t_start = time.time()
            try:
                transport = self.client.get_transport()
                if transport is None or not transport.is_active():
                    self._connect()
                    transport = self.client.get_transport()

                chan = transport.open_session()
                chan.settimeout(timeout)
                chan.exec_command(cmd)

                start = time.time()
                while not chan.exit_status_ready():
                    if time.time() - start > timeout:
                        chan.close()
                        latency = int((time.time() - t_start) * 1000)
                        return -1, "", f"Timeout global ({timeout}s)", latency

                code = chan.recv_exit_status()
                out = b""
                err = b""
                while chan.recv_ready():
                    out += chan.recv(4096)
                while chan.recv_stderr_ready():
                    err += chan.recv_stderr(4096)
                chan.close()

                latency = int((time.time() - t_start) * 1000)
                return (
                    code,
                    out.decode("utf-8", errors="replace"),
                    err.decode("utf-8", errors="replace"),
                    latency,
                )
            except Exception as e:
                print(f"[SSH] Erreur : {e}")
                latency = int((time.time() - t_start) * 1000)
                try:
                    self._connect()
                except Exception as e2:
                    return -1, "", f"SSH reconnect failed: {e2}", latency
                return -1, "", f"SSH error: {e}", latency


ssh_client = None


def rpi_cmd(cmd_in_workdir):
    return f"cd {RPI_WORKDIR} && {cmd_in_workdir}"


app = Flask(__name__)


@app.route("/")
def index():
    return render_template("index.html", rpi_host=RPI_HOST, rpi_user=RPI_USER)


@app.route("/api/mission/<mode>", methods=["POST"])
def start_mission(mode):
    if mode not in ("mission", "ident", "stop", "status"):
        return jsonify({"error": f"mode invalide : {mode}"}), 400

    if mode == "stop":
        # Combine stop + cleanup en une seule commande
        cmd = rpi_cmd(
            f"{LAUNCH_SCRIPT} stop 2>&1; "
            f"docker ps -q --filter 'name=summit-foxy-bridge' | xargs -r docker kill 2>&1; "
            f"sleep 1; "
            f"docker ps -aq --filter 'name=summit-foxy-bridge' | xargs -r docker rm -f 2>/dev/null; "
            f"docker compose -f {COMPOSE_FILE} down --remove-orphans 2>/dev/null; "
            f"true"
        )
        code, out, err, _ = ssh_client.exec(cmd, timeout=30)
        return jsonify({
            "status": "stopped_and_cleaned",
            "code": code,
            "stdout": out[-2000:] if out else "",
            "stderr": err[-500:] if err else "",
        })

    cmd = rpi_cmd(f"{LAUNCH_SCRIPT} {mode}")

    if mode in ("mission", "ident"):
        cmd = f"nohup bash -c '{cmd}' > /tmp/launch.log 2>&1 &"
        code, out, err, _ = ssh_client.exec(cmd, timeout=10)
        return jsonify({
            "status": "launched",
            "message": f"Pipeline demarre en arriere-plan (mode {mode})",
            "code": code,
            "stdout": out[-500:] if out else "",
            "stderr": err[-500:] if err else "",
        })
    else:
        # mode == "status"
        code, out, err, _ = ssh_client.exec(cmd, timeout=30)
        return jsonify({
            "status": "done",
            "code": code,
            "stdout": out[-2000:] if out else "",
            "stderr": err[-500:] if err else "",
        })


@app.route("/api/cleanup", methods=["POST"])
def cleanup():
    cmd = rpi_cmd(
        f"docker ps -q --filter 'name=summit-foxy-bridge' | xargs -r docker kill; "
        f"sleep 1; "
        f"docker ps -aq --filter 'name=summit-foxy-bridge' | xargs -r docker rm -f 2>/dev/null; "
        f"docker compose -f {COMPOSE_FILE} down --remove-orphans 2>/dev/null; "
        f"true"
    )
    code, out, err, _ = ssh_client.exec(cmd, timeout=30)
    return jsonify({
        "status": "cleaned",
        "code": code,
        "stdout": out[-500:] if out else "",
        "stderr": err[-500:] if err else "",
    })


@app.route("/api/reboot", methods=["POST"])
def reboot():
    quoted_pwd = shlex.quote(ssh_client.password)
    code, out, err, _ = ssh_client.exec(
        f"echo {quoted_pwd} | sudo -S reboot 2>&1",
        timeout=5,
    )
    return jsonify({
        "status": "reboot_sent",
        "message": "Commande reboot envoyee.",
        "code": code,
        "stdout": out[-500:] if out else "",
        "stderr": err[-500:] if err else "",
    })


@app.route("/api/status", methods=["GET"])
def status():
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
    code, out, err, latency_ms = ssh_client.exec(cmd, timeout=10)

    if code != 0:
        return jsonify({
            "error": err or "SSH error",
            "code": code,
            "ssh_latency_ms": latency_ms,
        }), 500

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
        "ssh_latency_ms": latency_ms,
    }

    sections = re.split(r"--- (\w+) ---\n", out)

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


def main():
    global ssh_client

    print("=" * 60)
    print("  SUMMIT CONTROL UI V4")
    print("=" * 60)

    password = os.environ.get("SUMMIT_SSH_PASSWORD")
    if not password:
        password = getpass(f"Mot de passe SSH pour {RPI_USER}@{RPI_HOST} : ")

    try:
        ssh_client = SSHClient(RPI_HOST, RPI_USER, password)
    except Exception as e:
        print(f"[ERREUR] Connexion SSH impossible : {e}")
        sys.exit(1)

    print(f"[OK] http://{WEB_HOST}:{WEB_PORT}")

    app.run(host=WEB_HOST, port=WEB_PORT, debug=False, use_reloader=False)


if __name__ == "__main__":
    main()
