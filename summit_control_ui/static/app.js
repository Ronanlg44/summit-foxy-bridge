// Summit Control UI - JavaScript frontend

const consoleBox = document.getElementById("console");

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
        if (data.error) {
            log(`Erreur: ${data.error}`, "err");
        } else if (data.status) {
            log(`OK: ${data.status}${data.message ? " - " + data.message : ""}`, "ok");
            if (data.stdout) log(`stdout: ${data.stdout.slice(0, 200)}`);
            if (data.stderr) log(`stderr: ${data.stderr.slice(0, 200)}`, "err");
        }
        return data;
    } catch (e) {
        log(`Exception: ${e.message}`, "err");
        return null;
    }
}

// ============ MISSION ============

document.querySelectorAll("[data-action]").forEach(btn => {
    btn.addEventListener("click", async () => {
        const action = btn.dataset.action;
        if (action === "mission/stop") {
            if (!confirm("Arreter tout le pipeline ?")) return;
        }
        if (action === "mission/mission" || action === "mission/ident") {
            if (!confirm(`Lancer le mode ${action.split("/")[1]} ?`)) return;
        }
        btn.disabled = true;
        await apiCall(action);
        btn.disabled = false;
    });
});

// ============ SERVICES ============

document.querySelectorAll("[data-service]").forEach(btn => {
    btn.addEventListener("click", async () => {
        const service = btn.dataset.service;
        const verb = btn.dataset.verb;
        btn.disabled = true;
        await apiCall(`service/${service}/${verb}`);
        btn.disabled = false;
    });
});

// ============ PID ============

document.querySelectorAll("[data-pi]").forEach(btn => {
    btn.addEventListener("click", async () => {
        const cmd = btn.dataset.pi;
        if (cmd === "publish_real/true") {
            if (!confirm("ATTENTION : le robot va reagir aux commandes ! Continuer ?"))
                return;
        }
        btn.disabled = true;
        await apiCall(`pi/${cmd}`);
        btn.disabled = false;
    });
});

// ============ STATUS SYSTEME ============

function classifyPercent(percent) {
    if (percent >= 90) return "error";
    if (percent >= 75) return "warn";
    return "ok";
}

function classifyTemp(temp) {
    if (temp >= 75) return "error";
    if (temp >= 65) return "warn";
    return "ok";
}

async function fetchStatus() {
    try {
        const res = await fetch("/api/status");
        const data = await res.json();

        // Temperature
        const tempEl = document.getElementById("cpu-temp");
        if (data.cpu_temp_c !== null) {
            tempEl.textContent = `${data.cpu_temp_c.toFixed(1)} °C`;
            tempEl.className = "status-value " + classifyTemp(data.cpu_temp_c);
        } else {
            tempEl.textContent = "N/A";
        }

        // Load
        const loadEl = document.getElementById("load");
        if (data.load_1min !== null) {
            loadEl.textContent = data.load_1min.toFixed(2);
        }

        // Mem
        const memEl = document.getElementById("mem");
        if (data.mem_percent !== null) {
            memEl.textContent = `${data.mem_used_mb} / ${data.mem_total_mb} MB (${data.mem_percent}%)`;
            memEl.className = "status-value " + classifyPercent(data.mem_percent);
        }

        // Disk
        const diskEl = document.getElementById("disk");
        if (data.disk_percent) {
            diskEl.textContent = `${data.disk_used} used, ${data.disk_available} free (${data.disk_percent})`;
            const pct = parseInt(data.disk_percent.replace("%", ""));
            diskEl.className = "status-value " + classifyPercent(pct);
        }

        // Tmux
        const tmuxEl = document.getElementById("tmux");
        tmuxEl.textContent = data.tmux_active ? "ACTIVE" : "inactive";
        tmuxEl.className = "status-value " + (data.tmux_active ? "ok" : "");

        // Containers
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
    }
}

// Poll toutes les 5s
fetchStatus();
setInterval(fetchStatus, 5000);

log("Interface prete", "ok");
