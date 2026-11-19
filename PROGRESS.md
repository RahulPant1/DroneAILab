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

## Phase 1b: Fly it in the browser (done 2026-11-19)

**Built:**
- Backend: position control (`control.py`), `pilot.py` (take-off/hover/land modes + keyboard sticks),
  `RealtimeSim` (wall-clock paced, pause, 30 Hz snapshots) in `runner.py`, `api.py` (FastAPI + `/ws/sim`).
- Frontend (React 19 + Vite 8 + TS 7 + r3f/drei, hand-written): header with Sim/AI status, Drone Studio
  (presets, 5 key params with plain-English help, live T/W, Advanced = more physics + all PID gains),
  a real-scale 3D drone, ENU axes, grid, trail, target marker, ground shadow, camera follow, on-demand rendering
  (~30 fps, none while the tab is hidden), telemetry (position/target/velocity, roll/pitch/yaw in °, motor bars),
  buttons, WASD/RF/QE keys, "fly to x/y/alt", and a lesson viewer.
- `run.sh`: reads `.env` safely, picks free ports, binds 127.0.0.1, and Ctrl+C stops everything it started.
- Lesson `docs/learn/02-roll-pitch-yaw.md`. 17 new tests (40 total).

**Acceptance:**
| Requirement | Result |
|---|---|
| First flight within 2 clicks | 1 click (Take off); the sim auto-connects on page load |
| UI smooth | renders only on new data (~30 fps); telemetry text at 10 Hz |
| Backend < 500 MB RAM | 64 MB |
| Total app < 1.5 GB | ~545 MB (Vite dev server ~480 MB of that) |
| Works with Ollama stopped | sim never calls Ollama; health shows "AI offline" (tested with an unreachable URL) |
| Ctrl+C leaves no processes | verified, both during start-up and after |

**Fixes found while testing:**
- Heavy preset crashed on a 90° turn: yaw now gets only leftover thrust, and tilt torque requests are capped
  to the motors' real authority.
- `run.sh` hung for up to 20 s if Ctrl+C came during start-up: the cleanup handler now exits.
- React 19 unmount warnings from drei `<Html>` labels: replaced with canvas sprites.

**Footprint:** project 347 MB on disk (node_modules 241 MB, .venv 83 MB).

**Not verified by me:** how the page looks and feels in a real browser (I had no browser to look at it).
I tested everything through the Vite proxy with a WebSocket client, plus the type-check and build.

**Known limitations:** the Vite dev server costs ~480 MB RAM; the trail keeps the last ~1500 points;
no live plots yet (Phase 2); a sim restarts when the page reloads or the backend restarts.

## Phase 2: Engineering Lab (done 2026-11-19)

**Built:**
- Sim: `environment.py` (steady wind + seeded gusts, drag uses airspeed, hidden payload), `sensors.py`
  (gyro/accel 250 Hz, altimeter + compass 50 Hz, GPS 10 Hz, optional gyro bias), `estimator.py`
  (complementary filters: Mahony-style tilt, alpha-beta position/altitude). With sensors off the controller sees the truth.
- `backend/experiments/`: 6 experiments, each with an objective, a 3-line concept, 2–4 validated sliders,
  a headless run, plots, a rule-based summary and one-click "try next" ideas.
- `backend/storage.py`: JSON runs + index, 100-run cap, delete-old, compare. Saves re-run on the server (genuine data).
- API: `/api/experiments`, `/api/experiments/{id}/run`, `/api/runs` (list/save/get/delete/delete-old/compare).
- Frontend: Lab tab, History tab (view, compare 2 runs with A solid / B dashed overlays + metric diff, delete),
  uPlot charts with units and a "how to read" line, CSV/JSON export.
- Lessons `docs/learn/03`–`08` (one per experiment). 24 new tests (64 total).

**Acceptance:**
| Requirement | Result |
|---|---|
| Too-high Kp visibly oscillates | Kp 25: 64 % overshoot, 6 target crossings |
| Kd = 0 overshoots more than the default | 104 % vs 0.6 % |
| Wind: measurable drift, controller recovers | 5 m/s: 0.39 m drift, back within 10 cm after 7.3 s (offset 0.5 cm) |
| Noise: visible truth vs estimate gap | yes (altitude, x, roll plots); the estimate removes ~75 % of altimeter noise |
| Save → reload → compare | verified via API, storage tests and end-to-end through run.sh |

**Design changes found while testing (documented in code):**
- Tilt-correction default 1.0 → 0.2: the accelerometer "lies" during accelerations, and at 1.0 the drone wandered 3 m.
- Altitude I-term: replaced the 0.3 m integral zone with a speed gate (|vz| < 0.5 m/s), so Ki can fix a hidden payload.
- PID I-limits are now "max output from the I-term" (independent of Ki).
- Roll/pitch motor split is reported at its peak (averaging hid it).

**Footprint:** backend 72 MB, Vite dev 533 MB, total ~605 MB RAM. Each run takes 0.3–0.7 s CPU and 47–268 KB.
Project 379 MB on disk.

**Not verified by me:** the look of the Lab/History pages and charts in a real browser.

**Known limitations:** the Vite dev server is the biggest RAM user; the estimator is a teaching-grade complementary
filter (not an EKF); live flight (Fly tab) still uses perfect sensors and no wind.

## Phase 3: Local LLM assistant (done 2026-11-19)

**Built:**
- `backend/ai/ollama_client.py`: HTTP only; per request `keep_alive 2m`, `num_ctx 8192`, `temperature 0.3`,
  `think false`; readable offline/error messages.
- `backend/ai/prompts.py`: system prompts ([Measured]/[Hypothesis], never invent numbers, "I don't know"),
  a telemetry-free compact run summary (≤ 2 runs, trimmed to ≤ 2k tokens), and the experiment catalogue for suggestions.
- `backend/ai/schemas.py`: JSON schema for Ollama `format`, plus pydantic shape check and the same range checks
  as the UI, giving readable rejections.
- `backend/ai/assistant.py`: ask/suggest, a number check that flags numbers not found in the run data,
  LaTeX tidy-up, and a JSONL log (rotates at 5 MB).
- API: `POST /api/assistant/ask`, `POST /api/assistant/suggest`. The run context is rebuilt server-side
  from saved ids or (experiment, params, seed). One LLM request at a time.
- Frontend: Assistant side panel on every tab (shows exactly which runs the AI sees), claim badges,
  unverified-number warnings, a suggestion card with a diff and "Apply & run in Lab" (nothing runs without a click).
- 26 new tests against a fake Ollama (90 total).

**Acceptance (live, real model `gemma4:e4b-it-q4_K_M`):**
| Requirement | Result |
|---|---|
| Malformed / out-of-range AI configs rejected readably | 10 rejection cases tested (bad JSON, wrong shape, unknown experiment/setting, out of range, no-op, too many changes) |
| Answers quote real metrics | "[Measured] overshoot 64.27 % and oscillations 6" (exact run values); 0 unverified numbers in 3 live answers |
| Model unloads after ~2 min idle | unloaded 113 s after the last request (`ollama ps` empty; GPU back to 1.3 GB) |
| App fully usable with Ollama off | tested with an unreachable Ollama: assistant → "AI offline" (503), the Lab still runs |

**Live performance:** first answer 58 s (includes the model load), then 2–17 s. 100 % GPU at num_ctx 8192
(no CPU split, so no need for 4096 or the fallback model). RAM available stayed ≥ 4.7 GB while loaded.

**Known limitations:** the model sometimes suggests a different experiment than the one discussed (it's still
validated and safe); the first answer after idle waits for the model to load; no streaming (the answer appears all at once).

## Next: Phase 4 (basic autonomy) only if the user asks.
