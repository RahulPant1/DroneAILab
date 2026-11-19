# DroneLab AI

A local, beginner-friendly quadcopter simulator for learning drone engineering.
Spec: [docs/PRD.md](docs/PRD.md) · Progress: [PROGRESS.md](PROGRESS.md) · Lessons: [docs/learn/](docs/learn/)

## Setup (once)
```bash
cd ~/DroneAILab
uv venv --python 3.12 .venv
uv pip install --python .venv/bin/python -r backend/requirements.txt
cp .env.example .env
(cd frontend && npm install)
```

## Fly in the browser
```bash
./run.sh          # then open http://localhost:5174 in Windows. Ctrl+C stops everything.
```
Click **Take off**, then hold W/A/S/D (move), R/F (up/down), Q/E (turn).

**Courses in 3D:** under the 3D view, pick a **Course** (e.g. Box canyon) to put obstacles and waypoint rings
into the world. Fly it yourself (touching a box stops the motors), or choose a planner and press **▶ Fly mission**
to watch the autopilot fly it live. Hover / Land take over from the autopilot.

**Wind (live):** the 🌬 Wind sliders under the 3D view set a steady wind, its direction and gusts. Changes apply
mid-flight; an arrow above the drone shows the air's motion right now.

**Low on memory?** Use `./run.sh --lite`. It builds the web page once (and again only after frontend
files change), then a single Python server runs everything: about **70 MB of RAM instead of ~300–550 MB**,
same address (http://localhost:5174). Use plain `./run.sh` when you're editing the frontend and want live reload.

**Lab** tab: six guided experiments (thrust, mass, roll/pitch/yaw, PID tuning, wind, sensor noise),
each with sliders, plots, a "what happened" explanation and "try next" ideas. Save runs, then use the
**History** tab to compare two of them on overlaid plots, or export as CSV/JSON.
Saved runs live in `data/experiments/` (max 100; History has "Delete old runs").

**Missions (Lab experiment 7):** fly a waypoint mission past box obstacles and compare three ways of
avoiding them: straight lines (crash), a potential field (reactive, can get stuck), and A* path planning.
A top-down map shows the planned vs flown path. In the terminal:
`.venv/bin/python -m backend.cli mission --scenario box_canyon --planner potential`.

**💬 Assistant** (header button, any tab): ask a local model (Ollama, model from `.env`) about the run
you're looking at. It sees only a short summary (settings, metrics, events), never raw telemetry, and
labels each claim **Measured** (from your data) or **Hypothesis** (its reasoning). Numbers it writes that
aren't in your data are flagged. "Suggest an experiment" returns a checked change that runs only when
you click **Apply**. Every prompt and reply is logged to `data/assistant_log.jsonl` for you to review.
The model unloads itself after 2 idle minutes (`ollama ps`). If Ollama is off, the panel says
"AI offline" and everything else keeps working. Tip: close other GPU-heavy apps before using it.

## Terminal tools
```bash
.venv/bin/python -m backend.cli hover --alt 2     # take off and hover at 2 m
.venv/bin/python -m backend.cli presets           # list drone presets
.venv/bin/python -m backend.cli hover --help      # all options
.venv/bin/pytest                                  # run the tests
```

## Uninstall
`rm -rf ~/DroneAILab` removes everything this project added. The only things left are the shared
download caches (`~/.cache/uv`, `~/.npm`), which other projects use too.
