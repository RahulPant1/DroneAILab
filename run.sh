#!/usr/bin/env bash
# Start DroneLab: backend (FastAPI) + frontend (Vite), both on 127.0.0.1 only.
# Ctrl+C stops both, and only the processes this script started.
set -euo pipefail
cd "$(dirname "$0")"

# --- Read .env (KEY=VALUE lines only; nothing in it is executed) ---
env_get() {  # env_get KEY DEFAULT
  local line
  line=$(grep -E "^[[:space:]]*$1=" .env 2>/dev/null | tail -n1 || true)
  line=${line#*=}; line=${line%%#*}; line=$(echo "$line" | tr -d '[:space:]"'"'")
  echo "${line:-$2}"
}
[ -f .env ] || { echo "No .env found. Run: cp .env.example .env"; exit 1; }
export OLLAMA_URL; OLLAMA_URL=$(env_get OLLAMA_URL http://localhost:11434)
export DRONELAB_LLM_MODEL; DRONELAB_LLM_MODEL=$(env_get DRONELAB_LLM_MODEL "")
BACKEND_PORT=$(env_get BACKEND_PORT 8001)
FRONTEND_PORT=$(env_get FRONTEND_PORT 5174)

# --- Checks ---
[ -x .venv/bin/python ] || { echo "Missing .venv. See README.md (Setup)."; exit 1; }
if ! command -v node >/dev/null 2>&1; then
  # Non-interactive shells may not have NVM loaded. Load it (read-only, changes nothing).
  export NVM_DIR="${NVM_DIR:-$HOME/.nvm}"
  # shellcheck disable=SC1091
  [ -s "$NVM_DIR/nvm.sh" ] && . "$NVM_DIR/nvm.sh" >/dev/null
fi
command -v node >/dev/null 2>&1 || { echo "Node not found (expected NVM's Node >= 20)."; exit 1; }
[ -d frontend/node_modules ] || { echo "Missing frontend/node_modules. Run: (cd frontend && npm install)"; exit 1; }

port_busy() { ss -ltnH "sport = :$1" 2>/dev/null | grep -q .; }
free_port() {  # first free port at or after $1
  local p=$1
  while port_busy "$p"; do p=$((p + 1)); done
  echo "$p"
}
B=$(free_port "$BACKEND_PORT")
F=$(free_port "$FRONTEND_PORT")
[ "$F" = "$B" ] && F=$(free_port $((F + 1)))
[ "$B" != "$BACKEND_PORT" ] && echo "Port $BACKEND_PORT is busy, using $B for the backend."
[ "$F" != "$FRONTEND_PORT" ] && echo "Port $FRONTEND_PORT is busy, using $F for the frontend."

# --- Start (each in its own process group, so we can stop it and its children cleanly) ---
PIDS=()
cleanup() {  # cleanup EXIT_CODE
  trap - INT TERM EXIT
  echo; echo "Stopping DroneLab..."
  for pid in "${PIDS[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
  sleep 1
  for pid in "${PIDS[@]}"; do kill -KILL -- "-$pid" 2>/dev/null || true; done
  echo "Stopped."
  exit "$1"  # always exit, even if Ctrl+C arrived during start-up
}
trap 'cleanup 130' INT
trap 'cleanup 143' TERM
trap 'cleanup $?' EXIT

setsid .venv/bin/uvicorn backend.api:app --host 127.0.0.1 --port "$B" --log-level warning &
PIDS+=($!)
BACKEND_PORT=$B FRONTEND_PORT=$F setsid ./frontend/node_modules/.bin/vite frontend --logLevel warn &
PIDS+=($!)

# Wait until both answer (max ~20 s).
for _ in $(seq 1 40); do
  if curl -fs "http://127.0.0.1:$B/api/health" >/dev/null 2>&1 && curl -fs "http://127.0.0.1:$F/" >/dev/null 2>&1; then
    echo ""
    echo "  DroneLab is running:  http://localhost:$F"
    echo "  (backend on 127.0.0.1:$B; press Ctrl+C to stop everything)"
    echo ""
    break
  fi
  sleep 0.5
done

wait -n "${PIDS[@]}" || true
echo "A DroneLab process exited."
