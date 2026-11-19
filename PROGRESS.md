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

## Next: Phase 1a (headless physics core). Waiting for "go".
