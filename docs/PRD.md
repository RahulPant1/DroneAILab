# DroneLab AI — PRD v3 (Beginner-first, laptop-safe)

> **For Claude Code:** Read this whole file before doing anything. It is the source of truth.
> The user is a **beginner in drone engineering**. The goal is **learning**, not shipping a product.
> Build **one phase at a time**, and **stop for user approval** at the end of each phase.

---

## 0. TL;DR

A local web app in WSL2 Ubuntu 22.04 where a beginner can:
1. fly a simulated quadcopter in 3D,
2. run small guided experiments (thrust, mass, PID, wind, sensor noise),
3. see real plots of what happened, and
4. ask a local LLM (Gemma, via the existing Ollama) *why* it happened, with answers grounded in actual run data.

**Priorities, in order:** readable, correct physics code > working sim > plots > AI > looks.

---

## 1. Environment and hard constraints

| Item | Value |
|---|---|
| OS | Windows 11 + WSL2 (WSL 3.0.5, mirrored networking), Ubuntu 22.04 (system Python 3.10; the project uses Python 3.12 via uv) |
| Laptop | HP Omen, RTX 4060 Laptop **8 GB VRAM** (shared with the LLM), **16 GB RAM** total. WSL is capped at **9 GB RAM, 4 CPUs, swap = 0**. This is fixed and will not be changed. |
| LLM | Gemma 4 E4B, **already installed** in Ollama as `gemma4:e4b-it-q4_K_M` (6.6 GB). Other local models (Qwen, MedGemma, DeepSeek) also exist. |
| Tooling already present | NVM/Node, Miniconda, VS Code Server. Detect these and reuse them. |
| Network | Needed only for `pip`/`npm` installs. The app must run fully offline. |
| Browser | Windows-side browser opens `http://localhost:<port>` (WSL localhost forwarding) |

**Laptop-safety rules (mandatory).** WSL has hung on this machine before.
- Physics runs on the **CPU** in Python/NumPy. The GPU is used only by the browser (WebGL) and Ollama.
- Only one heavy workload at a time. The app must never load the LLM on startup.
- App footprint: backend **< 500 MB RAM**, total app **< 1.5 GB RAM**, LLM excluded.
- Project lives at `~/DroneAILab` on the Linux filesystem, **never** under `/mnt/c/...`.
- `./run.sh` starts only DroneLab processes, and Ctrl+C stops them all. No leftover background processes.
- 3D view: cap the frame rate at about 30 fps and pause rendering when the browser tab is hidden.
- WSL memory cap (checked in Phase 0): `memory=9GB`, `swap=0`. The user has chosen to keep it. With no swap, running out of memory can freeze WSL instead of just slowing it down, so treat RAM as tight: keep the footprint targets below, and never run the sim's heavy work and the LLM at the same time. Never edit Windows files.

---

## 1a. Isolation rules: do not touch the existing environment (mandatory)

Everything DroneLab needs lives **inside `~/DroneAILab/`**. The rest of the machine is read-only for this project.

**Ollama and LLM models: use them, don't change them**
- Talk to Ollama **only over HTTP**. Allowed CLI commands: `ollama list`, `ollama ps`, `ollama show <model>` (all read-only).
- **Forbidden:** `ollama pull`, `run`, `rm`, `create`, `cp`, `push`, `serve`, `stop`; restarting or editing Ollama's systemd service; setting `OLLAMA_*` env vars globally; touching `~/.ollama` or `/usr/share/ollama`.
- Model choice comes from `.env` (`DRONELAB_LLM_MODEL`). In Phase 0, auto-detect the installed Gemma tag from `ollama list`, propose it, and let the user confirm.
- Set per-request options only: `keep_alive: "2m"`, `num_ctx: 8192`, low temperature (0.2–0.4), and `think: false` (Gemma 4 supports a "thinking" mode, which is slower and can interfere with structured output).
- VRAM is tight: the model is 6.6 GB, and other local apps (e.g. ClipForge) may already hold ~1 GB of GPU memory. In Phase 3, check `ollama ps`. If the PROCESSOR column shows a CPU/GPU split, lower `num_ctx` to 4096. If that isn't enough, propose `qwen3.5:4b` (3.3 GB) as the fallback model and ask first.
- Ollama URL comes from `.env` (`OLLAMA_URL`, default `http://localhost:11434`). If it's unreachable (e.g. Ollama runs on the Windows side), report it in Phase 0 and **ask**. Don't fix networking yourself.
- If Ollama is down at runtime, show "AI offline" and keep everything else working.

**Python: project-local environment only**
- Decided in Phase 0: `uv venv --python 3.12 .venv` inside the project (this reuses uv's existing Python 3.12.11, with no download). No conda env. The user's shell starts in conda `base`, so always call `.venv/bin/python` / `.venv/bin/pytest` explicitly, and use `uv pip install --python .venv/bin/python ...`.
- Never `pip install` into the system Python, the conda `base` env, or any existing env. No `sudo pip`, no `--break-system-packages`.
- Pin dependencies in `backend/requirements.txt`.

**Node: project-local only**
- `npm install` inside `frontend/` only. No `npm install -g`, no `npx` scaffolders that install globally.
- Use the Node already active in NVM if it's ≥ 20. Otherwise **ask** before `nvm install`, and never change `nvm alias default`.

**System and shell**
- No `sudo`, no `apt`, no edits to `~/.bashrc`, `~/.profile`, `/etc/*`, systemd units, global git config, or Windows-side files. If something system-level is needed, explain what and why, and let the user run it.

**Ports and network**
- Backend `8001`, frontend `5174` (8000/5173 are used by another local app, ClipForge; leave it alone). Ports come from `.env`. Check they're free first (`ss -ltnp`), and if not, use the next free port. Never kill a process you didn't start.
- Bind servers to `127.0.0.1` only (not `0.0.0.0`).

**Footprint (WSL is already heavy)**
- Target: the project adds **< 1 GB of disk** (venv + node_modules + data).
- Python deps for Phases 1–3 are limited to `fastapi`, `uvicorn`, `websockets` (uvicorn needs it to serve WebSockets), `numpy`, `pydantic`, `httpx`, `python-dotenv`, `pytest`. Ask before adding anything else. **Never** add `torch`, `tensorflow`, `jax`, `opencv`, `pybullet`, `mujoco` or `gymnasium` without explicit approval.
- Frontend deps: `react`, `react-dom`, `three`, `@react-three/fiber`, `@react-three/drei`, `uplot`. Dev/build only: `vite`, `@vitejs/plugin-react`, `typescript`, `@types/react`, `@types/react-dom`, `@types/three`. Write `package.json` by hand (no `npm create`). Use the NVM-active Node (v23.7.0). No UI mega-kits or state libraries.
- Saved runs: store telemetry downsampled to ≤ 50 Hz plus metrics. Default cap of 100 runs, with a "delete old runs" action.
- At each phase end, report `du -sh ~/DroneAILab` and the observed RAM use.

**Clean uninstall:** deleting `~/DroneAILab/` removes everything this project added. The only exception is the shared download caches (`~/.cache/uv`, `~/.npm`), which the user chose to keep because other projects use them too. Document this in the README.

---

## 2. Fixed tech stack (do not substitute without asking)

| Layer | Choice |
|---|---|
| Physics, control, estimation | Python 3.12, NumPy |
| Backend | FastAPI + one WebSocket for live sim |
| Frontend | React + Vite + TypeScript + `@react-three/fiber` |
| Plots | `uPlot` |
| Storage | JSON files in `data/experiments/` |
| AI | Ollama HTTP API (`/api/chat`, structured output via `format`) |
| Tests | `pytest` (backend). A frontend smoke check is enough. |
| Version control | Local `git init` in the project, commits per feature, no remote |

No Redux, Docker, databases, auth, microservices, queues or Kubernetes.

---

## 3. Architecture

```
dronelab/
  CLAUDE.md  PROGRESS.md  README.md  run.sh  .env.example  .gitignore
  backend/
    sim/
      config.py       # DroneConfig dataclass, presets, validation + limits
      frames.py       # coordinate conventions, rotation helpers
      physics.py      # 6-DOF rigid body, motors, drag, ground contact
      sensors.py      # noisy IMU/altimeter/position measurements
      estimator.py    # simple complementary / low-pass filter
      control.py      # cascaded PID (position -> attitude -> rates) + mixer
      environment.py  # wind: constant + gusts (seeded)
      runner.py       # fixed-step loop: real-time mode + fast headless mode
      metrics.py      # overshoot, settling time, RMS error, drift, energy
    experiments/      # one module per guided experiment
    ai/               # ollama_client.py, prompts.py, schemas.py
    storage.py        # save/load/list/delete runs (JSON)
    api.py            # REST + WebSocket
    cli.py            # headless runs from the terminal (learning + debugging)
    tests/
  frontend/src/       # Dashboard, Studio, Simulator, Lab, Assistant, History
  data/experiments/
  docs/PRD.md  docs/learn/
```

**Conventions (do not deviate):**
- SI units everywhere internally. World frame is **ENU** (x = east, y = north, **z = up**). Body frame: x forward, y left, z up.
- Angles are in radians internally and degrees in the UI. Attitude is a quaternion internally, with Euler roll/pitch/yaw shown in the UI.
- Variable names carry units: `mass_kg`, `thrust_n`, `alt_m`, `rate_rad_s`.

**Extensibility (interfaces only, don't build extra backends):**
- `SimBackend` protocol: `reset(config, seed)`, `step(command) -> State`, `get_state()`. The only implementation for now is `LightweightSim`, so a PX4 backend could be added later without a rewrite.
- Every run produces a `RunResult` (config, env, seed, downsampled telemetry, metrics, events), serialisable to JSON. It is the single input for plots, history, comparison and AI.

**API contract (keep it small):**
- `GET /api/health` returns `{sim: ok, llm: ok|offline, model}`
- `GET /api/presets`, `POST /api/config/validate`
- `WS /ws/sim`: client sends `{cmd: start|pause|resume|reset|takeoff|land|hover, ...}` or a manual setpoint, and the server streams state at about 30 Hz
- `POST /api/experiments/{id}/run` runs headless and returns `RunResult`
- `GET/POST/DELETE /api/runs`, `GET /api/runs/compare?a=&b=`
- `POST /api/assistant/ask`, `POST /api/assistant/suggest` (Phase 3)

---

## 4. Physics, control and estimation spec

- **Model:** X-configuration quadcopter as a 6-DOF rigid body with gravity, motor thrust `T = kT·ω²`, reaction torque `Q = kQ·ω²`, first-order motor lag (time constant about 30–50 ms), linear + quadratic drag, and simple ground contact (no falling through the floor, no bounce).
- **Integration:** fixed `dt = 0.002 s` (500 Hz) with semi-implicit Euler. Normalise the quaternion every step.
- **Rates:** physics 500 Hz, controller 250 Hz, sensors at their own rates (e.g. IMU 250 Hz, altimeter 50 Hz), UI stream about 30 Hz.
- **Modes:** *real-time* (paced to the wall clock, for the 3D view) and *headless* (as fast as possible, for experiments and tests). A 20 s headless run should finish in under 3 s.
- **Determinism:** same config + same seed gives identical results.
- **Control:** cascaded PID with output clamps and anti-windup:
  position → desired velocity → desired tilt; altitude → collective thrust; attitude → body rates → torques; mixer → 4 motor commands (clamped 0–100%).
- **Sensing:** the controller uses **estimated** state (from `sensors.py` + `estimator.py`), not ground truth. With noise disabled, the estimate equals the truth. Plots can show both.
- **Presets:** Basic learner (default, T/W ≈ 2.0), Lightweight, Heavy (T/W ≈ 1.3), High-thrust. Show T/W prominently.
- **Validation limits** apply to UI input, the API and AI-generated configs alike. Examples: mass 0.2–5 kg, arm length 0.08–0.5 m, PID gains within preset ranges, wind ≤ 15 m/s, duration ≤ 120 s.
- **Code style:** every physics/control function has a 2–4 line plain-English docstring explaining the idea, not just the code. The user will read this code to learn.

---

## 5. Phases

Every phase ends with: tests pass → app runs → **demo script the user follows** → `PROGRESS.md` updated → git commit → **STOP and wait for "go"**.

### Phase 0: Environment check (report only, no code)
Report: OS/WSL version, Python version, `uv`/conda availability, Node/NVM versions, `nvidia-smi` (in WSL), `free -h`, `df -h ~`, WSL memory cap, `ollama list`, `ollama ps`, whether `OLLAMA_URL` is reachable, and whether ports 8001/5174 are free. *(Done 2026-11-19; results are in `PROGRESS.md`.)*
Then propose: the env choice (venv vs conda), the exact install commands (project-local only), and the detected LLM model tag. **Wait for approval.**

### Phase 1a: Headless physics core (no UI)
**Deliver:** `config`, `frames`, `physics`, `control` (altitude + attitude only), `runner` (headless), `metrics`, `cli.py`, and tests.
`python -m backend.cli hover --alt 2` runs a take-off-and-hover and prints metrics plus a small ASCII altitude plot.

**Acceptance tests:**
- Default drone climbs to 2.0 m and stays within **±0.05 m** from t = 5 s to t = 10 s (no wind, no noise).
- T/W < 1.0: the drone never leaves the ground, and the result explains why.
- Doubling the mass raises hover motor % (test asserts it).
- Same seed gives identical telemetry.
- Free fall with motors off matches `z = z0 − ½·g·t²` within 1%, a physics sanity check.
- A 20 s headless run finishes in under 3 s.

**You learn:** thrust vs gravity, hover, T/W, what a control loop is. Write `docs/learn/01-thrust-and-hover.md`.

### Phase 1b: Make it fly in the browser
**Deliver:** dashboard shell, Drone Studio (presets + 5–6 key params with plain-English tooltips), 3D view (drone, ground grid, trail, simple axes), Take-off / Hover / Land / Pause / Resume / Reset buttons, keyboard nudge (WASD = horizontal, R/F = up/down, Q/E = yaw), live telemetry panel (position, velocity, roll/pitch/yaw, motor %), position control, and `run.sh`.

**Acceptance:** first flight within **2 clicks** of opening the app. The UI stays smooth. Backend < 500 MB RAM. Everything works with Ollama stopped. Ctrl+C on `run.sh` leaves no processes behind.

**You learn:** roll/pitch/yaw and how tilting creates horizontal motion. Write `docs/learn/02-roll-pitch-yaw.md`.

### Phase 2: Engineering Lab
Six guided experiments. Each has: objective → 3-line concept → 2–4 sliders → Run (headless) → plots → rule-based "what happened" text from metrics (**no AI**) → "try next" suggestions.
1. Thrust and take-off  2. Mass vs required thrust  3. Roll, pitch, yaw  4. PID tuning (step response)  5. Wind (constant + gusts)  6. Sensor noise (truth vs estimate)

Plus: save a run, history list, **compare 2 runs on overlaid plots**, and export a run as JSON/CSV.

**Acceptance:** a too-high Kp visibly oscillates; Kd = 0 gives more overshoot than the default; wind produces measurable drift that the controller recovers from; noise produces a visible gap between truth and estimate; save → reload → compare works.

**You learn:** PID intuition, disturbances, why sensors and estimation matter. Write a `docs/learn/` page per experiment.

### Phase 3: Local LLM assistant
- A side panel everywhere, with a clear "AI offline" state.
- **Grounding:** send a compact JSON summary (config, metrics, key events, max 2 runs, ≤ 2k tokens). Never send raw telemetry.
- The system prompt makes the model label claims **[Measured]** vs **[Hypothesis]**, never invent numbers, and say "I don't know" when the data doesn't show something.
- "Suggest an experiment": the model returns JSON via Ollama `format` (JSON schema) → pydantic + range validation → the change is shown as a diff → it runs **only when the user clicks Apply**.
- The LLM has no tools, shell or file access. Prompts and responses go to `data/assistant_log.jsonl` (local) for the user's review.

**Acceptance:** malformed or out-of-range AI configs are rejected with a readable reason; answers quote real metrics from the run; the model unloads after about 2 min idle (`ollama ps`); the app is fully usable with Ollama off.

### Phase 4: Basic autonomy (only when the user asks)
Waypoint missions, planned vs actual path, box obstacles, simple avoidance (potential field or A* on a 2D grid), and mission metrics.

### Phase 5: Real autopilot software in simulation (ArduPilot SITL), approved by the user 2026-11-22
Steps 1–4 approved and done (2026-11-22; for step 3 the user installed 3 GStreamer/Pulse libraries with apt themselves): build ArduCopter SITL (its built-in physics, no Gazebo) from a pinned stable tag inside `~/DroneAILab/external/ardupilot` (gitignored), and fly it from the command line (MAVProxy, plus a scripted pymavlink flight with a test). Steps 3–4 (QGroundControl, showing SITL in the DroneLab 3D view / `SimBackend`) need a separate go-ahead.
- Uses the g++ already installed. No sudo/apt: ArduPilot's `install-prereqs-ubuntu.sh` is NOT run. Its Python build/run deps go in the project `.venv`, pinned in `backend/requirements-sitl.txt`.
- Footprint exception: the ArduPilot source + build is about 2–3 GB, outside the < 1 GB target (user approved). Deleting `external/` removes it.
- SITL and MAVProxy talk MAVLink over TCP/UDP on 127.0.0.1 only.

### After Phase 5 (approved by the user 2026-11-22)
Showcase (public repo, CI that flies ArduPilot, README), fault-injection lab on ArduPilot SITL, then an AI flight-log
doctor (local Gemma explains ArduPilot logs, offline). Then (2026-11-22) lidar sense-and-avoid, a LiPo battery and
the Drone Builder; then (2026-11-22) a farm site with survey missions (also flown on ArduPilot SITL), vision-based
precision landing (a rendered down camera, numpy only, no OpenCV) and the AI mission planner (local Gemma writes a
task list from names on the map; code routes and safety-checks it; the user presses Fly). Then (2026-11-23) crop-health
(NDVI) maps stitched from the survey photos, patch detection and a local-AI farmer's report.

### Roadmap (do NOT install or scaffold)
PX4 SITL + Gazebo + MAVSDK, ROS 2, camera/vision, RL, NVIDIA Warp acceleration.

---

## 6. Learning design
- Defaults always work, and advanced parameters sit behind an "Advanced" toggle.
- Every plot has axis labels with units and a one-line "how to read this".
- `docs/learn/` pages are short (≤ 1 page), linked from the relevant UI panel, and end with 2–3 self-check questions.
- At each phase end, Claude Code gives a **10-minute code walkthrough**: which files to read, in what order, and what to look for.

---

## 7. Safety
Simulation only. No real-drone interfaces (no MAVLink to hardware, no serial/USB/Bluetooth/radio). MAVLink is allowed only to a simulated autopilot (ArduPilot SITL) on this laptop, over 127.0.0.1. AI-proposed configs are always validated and require user approval. The AI has no OS, shell or file access. No data leaves the machine.

---

## 8. Rules for Claude Code
0. Follow §1a at all times. If any step would touch anything outside `~/DroneAILab/`, **stop and ask**.
1. Inspect before installing, and reuse what exists.
2. Commit small, per feature, with clear messages (local git only).
3. Run the tests and the app after each change. Validate behaviour, not just installation.
4. Keep `CLAUDE.md` short (stack, commands, conventions, current phase) and `PROGRESS.md` current.
5. If the PRD conflicts with the reality you find, **stop and ask**. Don't silently work around it.
6. Phase-end report: what was built, how to run it, demo script, test results, RAM/disk used, known limitations, what the user learned, and the code walkthrough.
7. Never start the next phase without explicit user approval.
