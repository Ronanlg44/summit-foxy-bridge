#!/usr/bin/env bash
# Lance l'IHM Summit Control.
# 1. Cree/utilise un venv Python
# 2. Installe les dependances si necessaire
# 3. Demande le mot de passe SSH
# 4. Lance Flask sur localhost:8080

set -e

HERE="$(cd "$(dirname "$0")" && pwd)"
cd "$HERE"

VENV="$HERE/.venv"

# --- venv ---
if [ ! -d "$VENV" ]; then
    echo "[start] Creation du venv Python..."
    python3 -m venv "$VENV"
fi

source "$VENV/bin/activate"

# --- Dependances ---
if ! python -c "import flask, paramiko, yaml" 2>/dev/null; then
    echo "[start] Installation des dependances..."
    pip install --upgrade pip
    pip install -r requirements.txt
fi

# --- Verification config ---
if [ ! -f config.yaml ]; then
    echo "[start] ERREUR : config.yaml introuvable."
    exit 1
fi

# --- Lancement ---
echo
echo "======================================"
echo "  Summit Control UI"
echo "======================================"
echo "  Ouvre ton navigateur sur http://127.0.0.1:8080"
echo "  Ctrl+C pour arreter"
echo "======================================"
echo

exec python app.py
