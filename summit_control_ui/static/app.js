// Summit Control UI - JavaScript frontend V5

const consoleBox = document.getElementById("console");

const EXPECTED_SERVICES = [
    "realsense", "tf_static", "bridge", "apriltag",
    "refiner", "pose_fuser", "pid_apriltag",
];

const WS_URL = "ws://192.168.0.50:8081";

const PARAM_DEFS = [
    {name: "v_max",           label: "Vitesse max lineaire",    unit: "m/s",   step: 0.05},
    {name: "w_max",           label: "Vitesse max angulaire",   unit: "rad/s", step: 0.05},
    {name: "max_accel_lin",   label: "Accel max lineaire",      unit: "m/s^2", step: 0.1},
    {name: "max_accel_ang",   label: "Accel max angulaire",     unit: "rad/s^2", step: 0.1},
    {name: "deadband_lin",    label: "Deadband lineaire",       unit: "m",     step: 0.005},
    {name: "deadband_ang",    label: "Deadband angulaire",      unit: "rad",   step: 0.005},
    {name: "kp_lin",          label: "Kp lineaire",             unit: "",      step: 0.05},
    {name: "ki_lin",          label: "Ki lineaire",             unit: "",      step: 0.05},
    {name: "kp_ang",          label: "Kp angulaire",            unit: "",      step: 0.05},
    {name: "ki_ang",          label: "Ki angulaire",            unit: "",      step: 0.05},
    {name: "target_distance", label: "Distance cible",          unit: "m",     step: 0.1},
];

let ws = null;
let wsConnected = false;
let missionActive = false;
let currentSignalQuality = "unknown";

function log(message, type = "info") {
    const now = new Date().toLocaleTimeString();
    const line = document.createElement("div");
    line.className = "log-entry";
    const timeSpan = document.createElement("span");
    timeSpan.className = "log-time";
    timeSpan.textContent = `[${now}] `;
    line.appendChild(timeSpan);
    const msgSpan = document.createElement("span");
    if (type === "err") msgSpan.className = "log-err";
    else if (type === "ok") msgSpan.className = "log-ok";
    else if (type === "warn") msgSpan.className = "log-warn";
    msgSpan.textContent = message;
    line.appendChild(msgSpan);
    consoleBox.appendChild(line);
    consoleBox.scrollTop = consoleBox.scrollHeight;
}

async function apiCall(path, method = "POST") {
    log(`${method} ${path}`);
    try {
        const res = await fetch(`/api/${path}`, { method });
        const data = await res.json();
        if (data.error) log(`Erreur: ${data.error}`, "err");
        else if (data.status) {
            log(`OK: ${data.status}${data.message ? " - " + data.message : ""}`, "ok");
            if (data.stdout) log(`stdout: ${data.stdout}`);
            if (data.stderr) log(`stderr: ${data.stderr}`, "err");
        }
        return data;
    } catch (e) {
        log(`Exception: ${e.message}`, "err");
        return null;
    }
}

// ============ WEBSOCKET ============

function updateWsStatus(status) {
    const el = document.getElementById("ws-status");
    if (status === "connected") { el.textContent = "WS: connecte"; el.className = "ws-status ok"; }
    else if (status === "connecting") { el.textContent = "WS: connexion..."; el.className = "ws-status warn"; }
    else { el.textContent = "WS: deconnecte"; el.className = "ws-status err"; }
}

function enableParamControls(enabled) {
    document.getElementById("btn-emergency").disabled = !enabled;
    document.getElementById("btn-save-params").disabled = !enabled;
    document.getElementById("btn-reload-params").disabled = !enabled;
    document.getElementById("btn-activate-pid").disabled = !enabled;
    document.querySelectorAll(".param-input").forEach(el => el.disabled = !enabled);
    document.querySelectorAll(".param-apply").forEach(el => el.disabled = !enabled);
    const note = document.querySelector(".disabled-note");
    if (note) note.style.display = enabled ? "none" : "block";
}

function connectWs() {
    if (ws && ws.readyState !== WebSocket.CLOSED) return;
    updateWsStatus("connecting");
    ws = new WebSocket(WS_URL);

    ws.onopen = () => {
        wsConnected = true;
        updateWsStatus("connected");
        log("WebSocket connecte", "ok");
        applySignalQuality(currentSignalQuality);
        ws.send(JSON.stringify({action: "get_params"}));
    };

    ws.onmessage = (event) => {
        try {
            const data = JSON.parse(event.data);
            handleWsMessage(data);
        } catch (e) { console.error("WS parse error", e); }
    };

    ws.onclose = () => {
        wsConnected = false;
        updateWsStatus("deconnecte");
        applySignalQuality(currentSignalQuality);
        if (missionActive) setTimeout(connectWs, 3000);
    };
    ws.onerror = () => { wsConnected = false; updateWsStatus("deconnecte"); };
}

function handleWsMessage(data) {
    if (data.type === "pose") updatePoseDisplay(data);
    else if (data.type === "params") buildParamsGrid(data);
    else if (data.type === "ack") {
        if (data.error) log(`ERR (${data.action}) : ${data.error}`, "err");
        else if (data.action === "set_param") log(`Param ${data.name} = ${data.value}`, "ok");
        else if (data.action === "emergency_stop") log("URGENCE : cmd_vel = 0 envoyee, PID desactive", "warn");
        else if (data.action === "activate_pid") log("PID ACTIVE - le robot va bouger", "warn");
        else if (data.action === "save_params") log(`Parametres sauvegardes dans ${data.file}`, "ok");
        else if (data.action === "reload_params") log(`Fichier recharge : ${data.count || 0} params appliques`, "ok");
    }
    else if (data.type === "error") log(`WS erreur : ${data.message}`, "err");
}

function updatePoseDisplay(data) {
    const detectedEl = document.getElementById("spot-detected");
    const distanceEl = document.getElementById("spot-distance");
    const bearingEl = document.getElementById("spot-bearing");
    const xyEl = document.getElementById("spot-xy");
    const noteEl = document.getElementById("tracking-note");

    if (data.detected) {
        detectedEl.textContent = "OUI";
        detectedEl.className = "status-value ok";
        distanceEl.textContent = `${data.distance_m} m`;
        bearingEl.textContent = `${data.bearing_deg}°`;
        xyEl.textContent = `${data.spot_x} / ${data.spot_y} m`;
        noteEl.textContent = `Age mesure : ${data.age_s}s`;
    } else {
        detectedEl.textContent = "NON";
        detectedEl.className = "status-value warn";
        distanceEl.textContent = "—";
        bearingEl.textContent = "—";
        xyEl.textContent = "—";
        noteEl.textContent = `Pas de pose recente (age: ${data.age_s || '?'}s)`;
    }
}

function buildParamsGrid(params) {
    const grid = document.getElementById("params-grid");
    if (params.error) {
        grid.innerHTML = `<p class="disabled-note">Erreur : ${params.error}</p>`;
        return;
    }
    grid.innerHTML = "";
    for (const def of PARAM_DEFS) {
        const value = params[def.name];
        if (value === undefined || value === null) continue;

        const item = document.createElement("div");
        item.className = "param-item";
        const label = document.createElement("label");
        label.textContent = `${def.label}${def.unit ? ` (${def.unit})` : ""}`;
        item.appendChild(label);

        const controls = document.createElement("div");
        controls.className = "param-controls";
        const input = document.createElement("input");
        input.type = "number";
        input.className = "param-input";
        input.step = def.step;
        input.value = value.toFixed(4);
        input.dataset.name = def.name;
        controls.appendChild(input);

        const btn = document.createElement("button");
        btn.className = "btn btn-small btn-green param-apply";
        btn.textContent = "Appliquer";
        btn.addEventListener("click", () => {
            const val = parseFloat(input.value);
            if (isNaN(val)) { log(`Valeur invalide pour ${def.name}`, "err"); return; }
            ws.send(JSON.stringify({action: "set_param", name: def.name, value: val}));
        });
        controls.appendChild(btn);
        item.appendChild(controls);
        grid.appendChild(item);
    }
}

// ============ QUALITE SIGNAL ============

function updateSignalStatus(latencyMs) {
    const el = document.getElementById("signal-status");
    let quality, label;

    if (latencyMs === null || latencyMs === undefined || latencyMs < 0) {
        quality = "lost"; label = "Robot hors de portee";
    } else if (latencyMs < 200) {
        quality = "ok"; label = `Bon (${latencyMs}ms)`;
    } else if (latencyMs < 500) {
        quality = "medium"; label = `Moyen (${latencyMs}ms)`;
    } else if (latencyMs < 2000) {
        quality = "weak"; label = `Faible (${latencyMs}ms) - rapprochez-vous`;
    } else {
        quality = "lost"; label = "Robot hors de portee";
    }

    el.textContent = `Signal: ${label}`;
    el.className = `signal-status ${quality}`;
    currentSignalQuality = quality;
    applySignalQuality(quality);
}

function applySignalQuality(quality) {
    const isLost = (quality === "lost");
    const isWsDown = !wsConnected;

    // Boutons HTTP (mission) : grisés seulement si signal perdu
    const httpButtons = document.querySelectorAll("button[data-action]");
    httpButtons.forEach(btn => {
        if (btn.classList.contains("always-active")) {
            btn.disabled = false;
        } else if (isLost) {
            btn.disabled = true;
        }
    });

    // Boutons WebSocket (PID/params) : grisés si signal perdu OU WS deconnecte
    const wsButtons = document.querySelectorAll("#btn-activate-pid, #btn-save-params, #btn-reload-params, .param-apply");
    wsButtons.forEach(btn => {
        if (btn.classList.contains("always-active")) {
            btn.disabled = false;
        } else if (isLost || isWsDown) {
            btn.disabled = true;
        }
    });

    document.querySelectorAll(".param-input").forEach(el => {
        if (isLost || isWsDown) el.disabled = true;
    });

    if (!isLost && wsConnected) {
        enableParamControls(true);
    }
}

// ============ ACTIONS ============

document.querySelectorAll("[data-action]").forEach(btn => {
    btn.addEventListener("click", async () => {
        const action = btn.dataset.action;

        if (action === "mission/mission") {
            if (!confirm(
                "ATTENTION\n\nLe robot va se mettre a bouger des qu'il detecte un tag AprilTag.\n\n" +
                "- Espace libre ?\n- E-stop physique accessible ?\n- Personne dans le champ ?\n\nContinuer ?"
            )) return;
        }
        if (action === "mission/stop") { if (!confirm("Arreter tout le pipeline ?")) return; }
        if (action === "reboot") {
            if (!confirm("REBOOT COMPLET DE LA RPi4. SSH coupe ~1-2 min. Mission perdue. Continuer ?")) return;
        }

        btn.disabled = true;
        const result = await apiCall(action);
        btn.disabled = false;

        if (action === "mission/mission" && result && !result.error) {
            missionActive = true;
            pollMissionStartup();
        }
        if (action === "mission/stop") {
            missionActive = false;
            if (ws) ws.close();
        }
    });
});

document.getElementById("btn-emergency").addEventListener("click", () => {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({action: "emergency_stop"}));
    } else {
        log("URGENCE : WebSocket non connecte, impossible d'envoyer", "err");
    }
});

document.getElementById("btn-activate-pid").addEventListener("click", () => {
    if (!confirm(
        "ACTIVER PID \n\n" +
        "Le PID va etre reset puis active. \n" +
        "Le robot va commencer a bouger si un tag est detecte.\n\n" +
        "Continuer ?"
    )) return;
    ws.send(JSON.stringify({action: "activate_pid"}));
});

document.getElementById("btn-save-params").addEventListener("click", () => {
    if (!confirm("Sauvegarder les parametres actuels dans le fichier ?\n\nCes valeurs deviendront les nouveaux defauts au prochain demarrage.")) return;
    ws.send(JSON.stringify({action: "save_params"}));
});

document.getElementById("btn-reload-params").addEventListener("click", () => {
    if (!confirm("Recharger les parametres depuis le fichier ?\n\nLes modifications non sauvegardees seront perdues.")) return;
    ws.send(JSON.stringify({action: "reload_params"}));
});

// ============ POLLING SERVICES ============

async function pollMissionStartup() {
    log("Attente du demarrage des services (max 120s)...", "info");
    const seen = new Set();
    const startTime = Date.now();

    while (Date.now() - startTime < 120000) {
        try {
            const res = await fetch("/api/status");
            const data = await res.json();
            if (data.containers) {
                for (const c of data.containers) {
                    for (const svc of EXPECTED_SERVICES) {
                        if (c.name.includes(svc) && !seen.has(svc)) {
                            seen.add(svc);
                            log(`Service pret: ${svc}`, "ok");
                        }
                    }
                }
            }
            if (seen.size === EXPECTED_SERVICES.length) {
                log("MISSION PRETE - tous les services sont actifs", "ok");
                setTimeout(connectWs, 2000);
                return;
            }
        } catch (e) {}
        await new Promise(r => setTimeout(r, 3000));
    }

    const missing = EXPECTED_SERVICES.filter(s => !seen.has(s));
    if (missing.length > 0) log(`Timeout - services manquants: ${missing.join(", ")}`, "err");
    setTimeout(connectWs, 2000);
}

// ============ STATUS ============

function classifyPercent(p) { return p >= 90 ? "error" : p >= 75 ? "warn" : "ok"; }
function classifyTemp(t) { return t >= 75 ? "error" : t >= 65 ? "warn" : "ok"; }

async function fetchStatus() {
    const startTime = Date.now();
    try {
        const res = await fetch("/api/status");
        const data = await res.json();
        const latency = data.ssh_latency_ms !== undefined ? data.ssh_latency_ms : (Date.now() - startTime);
        updateSignalStatus(latency);

        const tempEl = document.getElementById("cpu-temp");
        if (data.cpu_temp_c !== null) {
            tempEl.textContent = `${data.cpu_temp_c.toFixed(1)} °C`;
            tempEl.className = "status-value " + classifyTemp(data.cpu_temp_c);
        } else tempEl.textContent = "N/A";

        const loadEl = document.getElementById("load");
        if (data.load_1min !== null) loadEl.textContent = data.load_1min.toFixed(2);

        const memEl = document.getElementById("mem");
        if (data.mem_percent !== null) {
            memEl.textContent = `${data.mem_used_mb} / ${data.mem_total_mb} MB (${data.mem_percent}%)`;
            memEl.className = "status-value " + classifyPercent(data.mem_percent);
        }

        const diskEl = document.getElementById("disk");
        if (data.disk_percent) {
            diskEl.textContent = `${data.disk_used} used, ${data.disk_available} free (${data.disk_percent})`;
            const pct = parseInt(data.disk_percent.replace("%", ""));
            diskEl.className = "status-value " + classifyPercent(pct);
        }

        const tmuxEl = document.getElementById("tmux");
        tmuxEl.textContent = data.tmux_active ? "ACTIVE" : "inactive";
        tmuxEl.className = "status-value " + (data.tmux_active ? "ok" : "");

        const wasMissionActive = missionActive;
        missionActive = data.tmux_active && data.containers.some(c => c.name.includes("pid_apriltag"));

        if (missionActive && !wasMissionActive && !wsConnected) setTimeout(connectWs, 2000);
        if (!missionActive && wasMissionActive && ws) ws.close();

        const contList = document.getElementById("containers");
        contList.innerHTML = "";
        if (data.containers && data.containers.length > 0) {
            data.containers.forEach(c => {
                const li = document.createElement("li");
                li.textContent = `${c.name} — ${c.status}`;
                contList.appendChild(li);
            });
        } else {
            const li = document.createElement("li");
            li.className = "empty";
            li.textContent = "Aucun container actif.";
            contList.appendChild(li);
        }
    } catch (e) {
        console.error("Status fetch error", e);
        updateSignalStatus(null);
    }
}

fetchStatus();
setInterval(fetchStatus, 5000);

console.log("Avant buildParamsGrid");
const emptyParams = {};
for (const def of PARAM_DEFS) emptyParams[def.name] = 0;
buildParamsGrid(emptyParams);
enableParamControls(false);
console.log("Apres buildParamsGrid, nb items:", document.querySelectorAll('.param-item').length);

log("Interface prete", "ok");
