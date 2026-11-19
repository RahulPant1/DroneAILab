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

**Lab** tab: six guided experiments (thrust, mass, roll/pitch/yaw, PID tuning, wind, sensor noise),
each with sliders, plots, a "what happened" explanation and "try next" ideas. Save runs, then use the
**History** tab to compare two of them on overlaid plots, or export as CSV/JSON.
Saved runs live in `data/experiments/` (max 100; History has "Delete old runs").

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
