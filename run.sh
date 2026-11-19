#!/usr/bin/env bash
# Start DroneLab on 127.0.0.1 only. Ctrl+C stops everything this script started (and nothing else).
#
#   ./run.sh          Development mode: backend (FastAPI) + frontend dev server (Vite, live reload).
#   ./run.sh --lite   Low-memory mode: builds the frontend once (only if its files changed), then ONE
#                     Python server serves both the page and the API. No Node runs while you use the app.
set -euo pipefail
cd "$(dirname "$0")"

LITE=0
for arg in "$@"; do
  case "$arg" in
    --lite) LITE=1 ;;
    -h|--help) sed -n '2,6p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "Unknown option: $arg (try --help)"; exit 2 ;;
  esac
done

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
[ -d frontend/node_modules ] || { echo "Missing frontend/node_modules. Run: (cd frontend && npm install)"; exit 1; }
load_node() {
  if ! command -v node >/dev/null 2>&1; then
    # Non-interactive shells may not have NVM loaded. Load it (read-only, changes nothing).
    export NVM_DIR="${NVM_DIR:-$HOME/.nvm}"
    # shellcheck disable=SC1091
    [ -s "$NVM_DIR/nvm.sh" ] && . "$NVM_DIR/nvm.sh" >/dev/null
  fi
  command -v node >/dev/null 2>&1 || { echo "Node not found (expected NVM's Node >= 20)."; exit 1; }
}

DIST=frontend/dist
needs_build() {  # true if there's no build yet, or any frontend source file is newer than it
  [ -f "$DIST/index.html" ] || return 0
  find frontend/src frontend/index.html frontend/vite.config.js frontend/package.json \
       frontend/package-lock.json frontend/tsconfig.json -newer "$DIST/index.html" 2>/dev/null | grep -q .
}
if [ "$LITE" = 1 ]; then
  if needs_build; then
    echo "Building the frontend (only needed after frontend changes)..."
    load_node
    npm --prefix frontend run build --silent >/dev/null
    echo "Build done."
  fi
else
  load_node
fi

port_busy() { ss -ltnH "sport = :$1" 2>/dev/null | grep -q .; }
free_port() {  # first free port at or after $1
  local p=$1
  while port_busy "$p"; do p=$((p + 1)); done
  echo "$p"
}
F=$(free_port "$FRONTEND_PORT")
[ "$F" != "$FRONTEND_PORT" ] && echo "Port $FRONTEND_PORT is busy, using $F for the web page."
if [ "$LITE" = 1 ]; then
  B=$F  # one server does everything
else
  B=$(free_port "$BACKEND_PORT")
  [ "$B" = "$F" ] && B=$(free_port $((B + 1)))
  [ "$B" != "$BACKEND_PORT" ] && echo "Port $BACKEND_PORT is busy, using $B for the backend."
fi

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

if [ "$LITE" = 1 ]; then
  DRONELAB_FRONTEND_DIST="$PWD/$DIST" setsid .venv/bin/uvicorn backend.api:app --host 127.0.0.1 --port "$F" --log-level warning &
  PIDS+=($!)
else
  setsid .venv/bin/uvicorn backend.api:app --host 127.0.0.1 --port "$B" --log-level warning &
  PIDS+=($!)
  BACKEND_PORT=$B FRONTEND_PORT=$F setsid ./frontend/node_modules/.bin/vite frontend --logLevel warn &
  PIDS+=($!)
fi

# Wait until it answers (max ~20 s).
for _ in $(seq 1 40); do
  if curl -fs "http://127.0.0.1:$B/api/health" >/dev/null 2>&1 && curl -fs "http://127.0.0.1:$F/" >/dev/null 2>&1; then
    echo ""
    echo "  DroneLab is running:  http://localhost:$F"
    if [ "$LITE" = 1 ]; then
      echo "  (lite mode: one Python server, no Node; press Ctrl+C to stop)"
    else
      echo "  (backend on 127.0.0.1:$B; press Ctrl+C to stop everything)"
    fi
    echo ""
    break
  fi
  sleep 0.5
done

wait -n "${PIDS[@]}" || true
echo "A DroneLab process exited."
