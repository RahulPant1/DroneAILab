# DroneLab AI

A local, beginner-friendly quadcopter simulator for learning drone engineering.
Spec: [docs/PRD.md](docs/PRD.md) · Progress: [PROGRESS.md](PROGRESS.md) · Lessons: [docs/learn/](docs/learn/)

## Setup (once)
```bash
cd ~/DroneAILab
uv venv --python 3.12 .venv
uv pip install --python .venv/bin/python -r backend/requirements.txt
cp .env.example .env
```

## Use (Phase 1a: terminal only)
```bash
.venv/bin/python -m backend.cli hover --alt 2     # take off and hover at 2 m
.venv/bin/python -m backend.cli presets           # list drone presets
.venv/bin/python -m backend.cli hover --help      # all options
.venv/bin/pytest                                  # run the tests
```

## Uninstall
`rm -rf ~/DroneAILab` removes everything this project added. The only things left are the shared
download caches (`~/.cache/uv`, `~/.npm`), which other projects use too.
