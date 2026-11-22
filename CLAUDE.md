# DroneLab AI: project memory for Claude Code

**Source of truth:** `docs/PRD.md`. Read it fully before any work.

## Current phase
Phase 4 done (missions: waypoints, box obstacles, straight/potential/A* planners, mission metrics) + missions, live wind and live sensor noise in the 3D Fly view (226 tests + 14 opt-in real-SITL tests) + rings race game + multi-drone formation. Phase 5 done (ArduCopter SITL in `external/`, MAVProxy, scripted flight, QGroundControl 4.4.5, 🛰 ArduPilot panel in the Fly view with a DroneLab-vs-ArduPilot step comparison) + showcase (public repo, CI flying ArduPilot, README) + fault-injection lab (11 scenarios) + 🩺 flight-log doctor (Logs tab, local-LLM debrief) + 📡 lidar sense-and-avoid + 🔋 battery + 🛠 Drone Builder + 🌾 farm surveys + 🎯 precision landing + 🧭 AI mission planner. Other Roadmap items need explicit approval. Update this line at every phase change.

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
- Autonomy: `backend/autonomy/` (world, planner, follower, mission, live). `live.MissionSim` = Fly-view sim with a course + mission autopilot (WS cmds `course`, `mission`; `wind` and `sensors` are on RealtimeSim, both default off). Pilot buttons and MissionFollower act on the estimate; only crash/ground contact use truth. Live view: GPS settle (5 s ground averaging) before take-off with noise on; GPS outage toggle + failsafe (level + altitude hold; W/A/S/D tilt, x/y target frozen). Lab experiment 7 = missions; map plots use `Plot(kind="map", extra=...)`. CLI: `python -m backend.cli mission --scenario box_canyon --planner potential`.
- Multi-drone: `backend/autonomy/formation.py` (wingmen = own LightweightSim + Pilot; WS `formation` cmd on MissionSim; follow the leader's broadcast estimate).
- Games: `backend/games/race.py` (rings race; WS `race` cmd on MissionSim; best times `data/race_best.json`, tests monkeypatch `BEST_TIMES_FILE`).
- Sensors off (`SensorConfig.enabled=False`) = controller sees truth; keep it that way for the Phase 1 tests.
- Avoid drei `<Html>` (React 19 unmount warnings) and drei `Text`/`Environment` (fetch from CDN; the app must work offline).
- Headless: `.venv/bin/python -m backend.cli hover --alt 2`
- Repo is PUBLIC (github.com/RahulPant1/DroneAILab). Commits use the noreply email (repo-local git config); never put the personal email in commits or files. `private/` (gitignored) holds the LinkedIn draft. CI: `.github/workflows/ci.yml` (tests, frontend, ArduCopter build cached + real SITL flights incl. all faults).
- Fault lab: `backend/sitl/faults.py` (FAULTS: safety = PASS/FAIL, limitation = CONFIRMED; fresh SITL per run, hold 20 m N, inject SIM_* params, judge vs SIMSTATE truth); `--all` writes docs/results/ (+ `--report-only`). Panel: WS `sitl` action `fault`; `GET /api/sitl/faults`. README chart: `python -m backend.sitl.showcase` (refuses to write if the recording has a > 0.3 s gap).
- Battery: `backend/sim/battery.py` (LiPo OCV table, sag V^2-Voc*V+P*R=0, physics.voltage_scale = V / (3.85*cells)); off by default (physics unchanged); RealtimeSim.check_battery = failsafe (3 s / drain_speedup below 3.6 V/cell -> home, 3.4 -> land, empty -> motors off); WS `battery`. A preset reset switches it off.
- Builder: `backend/sim/builder.py` (catalogue, momentum-theory hover power with per-prop figure of merit; torque_per_thrust set so the sim burns exactly that power); API /api/builder/parts|design; WS `design` (config + its battery). Min mass is 0.1 kg for the micro build.
- Lidar: `backend/autonomy/lidar.py` (scan = slab ray casting from the TRUE pose; OccupancyMap from the ESTIMATE, inflated by KEEP_OUT_M; optimistic A*; SenseFollower subclasses MissionFollower and swaps its route on replans). Planner "lidar" everywhere; course `hidden_walls`; live `lidar` snapshot sends the map only when its version changes; WS `lidar` toggle.
- Farm / plans: `backend/autonomy/site.py` (FARM: fields, buildings, no-fly boxes, pads, places, geofence 90 m, ceiling 60 m), `survey.py` (camera footprint, lawnmower with exact line spacing, CAM_TRIGG_DIST-style SurveyCamera, coverage grid), `plan.py` (task list -> waypoints + A* route at 0.5 m around no-fly + buildings taller than the flight; checks pass/warn/fail; PlanFollower = dwell + survey camera + precision lander; run_plan headless at 10 Hz telemetry). Live: course `farm`, WS `plan` {spec} (re-checked, refused on any fail), `precland` {mode}, `camera` {enabled}, `speed` {factor 1/2/4/8}. Lab experiments 8 (survey, 4 params) and 9 (precision landing; plot kind "images").
- Precision landing: `backend/autonomy/precland.py` (128x96 render from the TRUE pose, dark-pixel marker detection with neighbour filter, pixel -> ground offset with the ESTIMATED attitude/height; target = estimate + offset; search/hold, GPS fallback, blind below 2 m). GPS drift: `SensorConfig.gps_drift_m` (smoothed Gauss-Markov, tau 60 s, default 0 = old behaviour).
- AI mission planner: `backend/ai/mission_planner.py` (JSON schema enums = site names; not_on_map -> fail; left_out -> warn; numbers checked; log kind `mission_plan`). API `/api/site`, `/api/plan/compile`, `/api/ai/plan`. ArduPilot: `backend/sitl/plan_flight.py` (AUTO mission, `WP_SPD` in m/s on 4.7, photos from its EKF position, truth from SIMSTATE; `--write` -> docs/results/survey-compare.*), bridge action `plan`.
- Log doctor: `backend/logdoctor/` (analyze = read_log + analyze_messages; store maps ids `sitl-NNNNNNNN` / `upload-*` to files only; explain = LLM debrief on the compact summary, numbers checked, logged as kind `log_explain`). Uploads + analysis cache in `data/logs/` (gitignored: real logs contain GPS positions, often the owner's home). SITL logs pruned to 40. pymavlink is now a main dependency.
- ArduPilot SITL: `backend/sitl/` (launch, fly, ground_station), `scripts/sitl_setup.sh`, `scripts/sitl.sh`. Source+build in `external/` (gitignored), pinned Copter-4.7.3, patched to listen on 127.0.0.1 (`scripts/patches/`). Extra deps in `backend/requirements-sitl.txt`; MAVProxy installed `--no-deps` (its deps pull opencv/matplotlib/vtk). Real-flight test opt-in: `DRONELAB_SITL=1`. MAVProxy forks helpers: stop the whole tree (`ground_station.stop_tree`). `bridge.BRIDGE` = one SITL per server (WS `sitl` cmd, `sitl` in every state frame), started in the server's process group so run.sh's stop kills it; `fly.Flight` reads MAVLink in a background thread (never `recv_match(type=...)` in a wait loop: it drops other messages). QGroundControl: `backend/sitl/qgc.py` + `scripts/qgc.sh`/`qgc_setup.sh`; 4.4.5 (5.x needs glibc 2.38); HOME=external/qgc-home; auto-connect settings live in `[LinkManager]`; one TCP link to 127.0.0.1 (5762 if the panel's SITL runs, else its own SITL on 5760). The user installed libgstreamer-gl1.0-0, libgstreamer-plugins-base1.0-0, libpulse-mainloop-glib0 (2026-11-22).
