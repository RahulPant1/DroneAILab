# DroneLab AI: project memory for Claude Code

**Source of truth:** `docs/PRD.md`. Read it fully before any work.

## Current phase
Phase 4 done (missions: waypoints, box obstacles, straight/potential/A* planners, mission metrics) + missions, live wind and live sensor noise in the 3D Fly view (145 tests). All PRD phases complete; Roadmap items need explicit approval. Update this line at every phase change.

## Machine facts (from Phase 0)
Project at `~/DroneAILab`. Ubuntu 22.04 on WSL2 (mirrored networking). WSL has 9 GB RAM, **swap = 0**, 4 CPUs. Treat RAM as tight.
Shell starts in conda `base`: always call `.venv/bin/...` explicitly. Ports: backend **8001**, frontend **5174** (8000/5173 belong to ClipForge; don't touch it).
LLM: `gemma4:e4b-it-q4_K_M`, sent with `think: false`. Node v23.7.0 (NVM default).

## Non-negotiables (summary of PRD §1, §1a, §8)
- Work only inside `~/DroneAILab/`. Anything outside → stop and ask.
- Ollama: HTTP only. Read-only CLI (`ollama list|ps|show`). Never pull/run/create/rm/serve models.
- Python: project `.venv` only (uv, Python 3.12). Never conda base/system/other envs.
- Node: `npm install` inside `frontend/` only. No `-g`. Don't change the NVM default.
- No sudo/apt, no shell-profile or /etc edits, no Windows-side edits.
- Physics on CPU (NumPy). Never load the LLM on startup. `keep_alive: "2m"`, `num_ctx: 8192`.
- Bind to 127.0.0.1. Check ports before use. Never kill processes you didn't start.
- One phase at a time, and STOP for user approval at each phase end.

## Conventions
SI units, ENU world frame (z up), radians internally / degrees in UI, units in variable names,
plain-English docstrings on physics/control functions.

## Commands (fill in as they are created)
- Setup (Python): `uv venv --python 3.12 .venv && uv pip install --python .venv/bin/python -r backend/requirements.txt`
- Setup (frontend, Phase 1b): `cd frontend && npm install`
- Run app: `./run.sh` → http://localhost:5174 (Ctrl+C stops everything). `./run.sh --lite` = prebuilt frontend served by uvicorn on the same port, no Node at runtime (~70 MB RAM); rebuilds only when frontend files change.
- Tests: `.venv/bin/pytest -q`
- Frontend check: `cd frontend && npm run build` (tsc + vite build). Node needs NVM loaded in non-interactive shells: `. ~/.nvm/nvm.sh`
- Experiments live in `backend/experiments/` (one module each, registered in `__init__.py`); a run's plots/summary/try-next travel inside its RunResult. Saved runs: `data/experiments/` (re-run server-side on save).
- Assistant: `backend/ai/` (client, prompts, schemas, assistant). Tests use a fake Ollama (httpx.MockTransport); never load the model in pytest. Run context is rebuilt server-side from run refs. Log: `data/assistant_log.jsonl`.
- Autonomy: `backend/autonomy/` (world, planner, follower, mission, live). `live.MissionSim` = Fly-view sim with a course + mission autopilot (WS cmds `course`, `mission`; `wind` and `sensors` are on RealtimeSim, both default off). MissionFollower navigates on the estimate; crash/ground contact use truth. Lab experiment 7 = missions; map plots use `Plot(kind="map", extra=...)`. CLI: `python -m backend.cli mission --scenario box_canyon --planner potential`.
- Sensors off (`SensorConfig.enabled=False`) = controller sees truth; keep it that way for the Phase 1 tests.
- Avoid drei `<Html>` (React 19 unmount warnings) and drei `Text`/`Environment` (fetch from CDN; the app must work offline).
- Headless: `.venv/bin/python -m backend.cli hover --alt 2`
