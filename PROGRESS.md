# DroneLab progress

## Phase 0: Environment check (done 2026-11-19)

**Machine:** Ubuntu 22.04 on WSL2 3.0.5 (mirrored networking), 4 CPUs, 9 GB RAM cap, **swap 0**
(the user chose to keep this). RTX 4060 8 GB, with ~1.1 GB already used by ClipForge when it runs. 794 GB disk free.

**Tools reused:** uv 0.8.3 (Python 3.12.11 already downloaded), Node v23.7.0 via NVM 0.39.5, git 2.34.1
(global identity already set), Ollama 0.40.3 at `http://localhost:11434`.

**Decisions:**
| Topic | Choice |
|---|---|
| Project path | `~/DroneAILab` (not `~/projects/dronelab`) |
| Python env | `uv venv --python 3.12 .venv` (no conda env) |
| Ports | backend 8001, frontend 5174 (8000/5173 = ClipForge) |
| LLM | `gemma4:e4b-it-q4_K_M`, `think: false`; fallback `qwen3.5:4b` only if it doesn't fit (ask first) |
| Extra deps approved | `websockets`; frontend `react-dom` + Vite/TypeScript build tools |
| Download caches | shared `~/.cache/uv`, `~/.npm` (not removed on uninstall) |
| `.wslconfig` | unchanged (9 GB, swap 0) |

**Done:** updated the docs to match the machine; created `.env`; created `.venv` (Python 3.12.11)
with the pinned deps in `backend/requirements.txt`; `git init`.

**Verified:** all deps import; uvicorn has WebSocket support; Ollama answers over HTTP and the
model tag exists; `ollama ps` is empty (no model loaded).

**Footprint:** project 83 MB (all of it `.venv`).

## Phase 1a: Headless physics core (done 2026-11-19)

**Built:** `backend/sim/` (`config`, `frames`, `physics`, `control`, `runner`, `metrics`), `backend/cli.py`,
23 tests in `backend/tests/`, lesson `docs/learn/01-thrust-and-hover.md`, `README.md`.

**Acceptance:** all pass (`.venv/bin/pytest` → 23 passed in ~2.3 s).
| Requirement | Result |
|---|---|
| Hover at 2.0 m, ±0.05 m from 5 to 10 s | max error 0.014 m (overshoot 0.8 %, settled at 2.4 s) |
| T/W < 1 never leaves the ground + explains why | yes (rule-based summary) |
| Doubling the mass raises hover % | 0.6 → 1.2 kg: ratio 2.00 |
| Same seed gives identical telemetry | yes |
| Free fall vs ½·g·t² within 1 % | ≤ 0.1 % at 2 s, max abs error 2 cm (from t ≥ 0.25 s; see the test docstring) |
| 20 s headless run < 3 s | ~0.3 s (120 s run: 1.9 s) |

**Footprint:** peak RAM of a run ~39 MB; project 86 MB on disk.

**Design notes:**
- Motor % maps to thrust linearly (speed = max · √%), so hover % = 100 / (T/W).
- The mixer keeps attitude torques when a motor saturates by shifting the collective.
- The altitude PID uses an integral zone (0.3 m) to stop windup on take-off (the overshoot was 8 % without it).
- The controller sees true state for now (no sensors/estimator until Phase 2).

**Known limitations:** no position control yet (Phase 1b); simple ground contact (no tipping, drone is
levelled on touchdown); a 120 s run's JSON is ~1.1 MB. Floats should be rounded before saving runs in Phase 2.

## Next: Phase 1b (fly it in the browser). Waiting for "go".
