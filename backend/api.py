"""HTTP + WebSocket API for the browser UI.

  GET  /api/health           -> {sim: "ok", llm: "ok" | "offline", model}
  GET  /api/presets          -> drone presets + validation limits
  POST /api/config/validate  -> {valid, problems, derived}
  GET  /api/learn/{slug}     -> a docs/learn/ page as Markdown text
  WS   /ws/sim               -> live simulation (client sends commands, server streams state ~30 Hz)

  GET  /api/experiments                -> the six Lab experiments (sliders, concept, metrics)
  POST /api/experiments/{id}/run       -> run headless, returns a RunResult
  GET  /api/runs                       -> saved runs (newest first)
  POST /api/runs                       -> save {experiment_id, params, seed, label} (re-runs on the server)
  GET  /api/runs/compare?a=&b=         -> two saved runs + metric differences
  GET  /api/runs/{id}, DELETE /api/runs/{id}
  POST /api/runs/delete-old            -> {keep_newest: N}

  POST /api/assistant/ask      -> {question, context: [run refs], history} -> grounded answer
  POST /api/assistant/suggest  -> {context: [run refs], goal} -> validated suggestion (never auto-run)
  A run ref is {"saved_id": "..."} or {"experiment_id": "...", "params": {...}, "seed": 0, "label": "..."}.

The LLM is never loaded here: the health check only asks Ollama for its model LIST.
Run with: .venv/bin/uvicorn backend.api:app --host 127.0.0.1 --port 8001  (or ./run.sh)
Lite mode (./run.sh --lite): DRONELAB_FRONTEND_DIST points at the built frontend, and this server also
serves the web page itself, so no Node process is needed while the app runs.
"""

from __future__ import annotations

import asyncio
import os
import re
import threading
import time
from pathlib import Path

import httpx
from dotenv import load_dotenv
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles

from .ai import assistant as ai_assistant
from .ai.ollama_client import AIError, AIOffline, OllamaClient
from .experiments import EXPERIMENTS, ExperimentError, get_experiment
from .sim.config import GAIN_LIMITS, LIMITS, PRESETS, ConfigError, DroneConfig, get_preset, validate_config
from .sim.runner import RealtimeSim
from .storage import RunStore, StorageError, compare

PROJECT_ROOT = Path(__file__).resolve().parent.parent
LEARN_DIR = PROJECT_ROOT / "docs" / "learn"
load_dotenv(PROJECT_ROOT / ".env")

STREAM_HZ = 30.0
HEALTH_CACHE_S = 10.0

app = FastAPI(title="DroneLab", docs_url="/api/docs", openapi_url="/api/openapi.json")
_health_cache: dict = {"at": -1e9, "value": None}
store = RunStore()
_heavy_work = threading.Lock()  # one headless run at a time (laptop-safety rule)
_ai_lock = asyncio.Lock()  # one LLM request at a time


def settings() -> dict:
    return {
        "ollama_url": os.environ.get("OLLAMA_URL", "http://localhost:11434").rstrip("/"),
        "model": os.environ.get("DRONELAB_LLM_MODEL", ""),
    }


@app.get("/api/health")
async def health() -> dict:
    """Is the sim up, and is Ollama reachable with our model installed? (Cached for 10 s.)"""
    now = time.monotonic()
    if _health_cache["value"] is not None and now - _health_cache["at"] < HEALTH_CACHE_S:
        return _health_cache["value"]
    cfg = settings()
    llm = "offline"
    try:
        async with httpx.AsyncClient(timeout=1.5) as client:
            resp = await client.get(f"{cfg['ollama_url']}/api/tags")
        names = {m.get("name") for m in resp.json().get("models", [])}
        llm = "ok" if resp.status_code == 200 and cfg["model"] in names else "offline"
    except (httpx.HTTPError, ValueError):
        llm = "offline"
    value = {"sim": "ok", "llm": llm, "model": cfg["model"]}
    _health_cache.update(at=now, value=value)
    return value


@app.get("/api/presets")
def presets() -> dict:
    return {
        "presets": {pid: cfg.to_dict() for pid, cfg in PRESETS.items()},
        "limits": {k: list(v) for k, v in LIMITS.items()},
        "gain_limits": {k: list(v) for k, v in GAIN_LIMITS.items()},
    }


def parse_config(data) -> DroneConfig:
    if not isinstance(data, dict):
        raise ConfigError("Config must be a JSON object.")
    try:
        return DroneConfig.from_dict(data)
    except TypeError as exc:
        raise ConfigError(f"Bad config: {exc}") from None


@app.post("/api/config/validate")
def validate(body: dict) -> dict:
    try:
        cfg = parse_config(body.get("config", body))
    except ConfigError as exc:
        return {"valid": False, "problems": [str(exc)], "derived": None}
    problems = validate_config(cfg)
    return {"valid": not problems, "problems": problems, "derived": None if problems else cfg.to_dict()["derived"]}


@app.get("/api/learn/{slug}")
def learn(slug: str) -> dict:
    if not re.fullmatch(r"[0-9]{2}-[a-z0-9-]+", slug):
        raise HTTPException(404, "No such page.")
    path = LEARN_DIR / f"{slug}.md"
    if not path.is_file():
        raise HTTPException(404, "No such page.")
    return {"slug": slug, "markdown": path.read_text(encoding="utf-8")}


# ---- Lab: experiments and saved runs ----


def _run_experiment(experiment_id: str, params, seed) -> dict:
    exp = get_experiment(experiment_id)
    if params is not None and not isinstance(params, dict):
        raise ExperimentError("params must be an object.")
    if not _heavy_work.acquire(timeout=10):
        raise HTTPException(429, "Another run is still going. Try again in a moment.")
    try:
        return exp.run(params, seed if seed is not None else 0)
    finally:
        _heavy_work.release()


@app.get("/api/experiments")
def experiments() -> list[dict]:
    return [e.to_dict() for e in EXPERIMENTS.values()]


@app.post("/api/experiments/{experiment_id}/run")
def run_experiment(experiment_id: str, body: dict | None = None) -> dict:
    body = body or {}
    try:
        return _run_experiment(experiment_id, body.get("params"), body.get("seed")).to_dict()
    except ExperimentError as exc:
        raise HTTPException(400, str(exc)) from None


@app.get("/api/runs")
def list_runs() -> list[dict]:
    return store.list()


@app.post("/api/runs")
def save_run(body: dict) -> dict:
    """Save a run. The server re-runs it from (experiment, params, seed), so saved data is always genuine."""
    try:
        result = _run_experiment(str(body.get("experiment_id")), body.get("params"), body.get("seed"))
        return store.save(result, str(body.get("label") or ""))
    except (ExperimentError, StorageError) as exc:
        raise HTTPException(400, str(exc)) from None


@app.get("/api/runs/compare")
def compare_runs(a: str, b: str) -> dict:
    try:
        return compare(store.get(a), store.get(b))
    except StorageError as exc:
        raise HTTPException(404, str(exc)) from None


@app.post("/api/runs/delete-old")
def delete_old_runs(body: dict) -> dict:
    try:
        return {"deleted": store.delete_old(int(body.get("keep_newest", 20)))}
    except (StorageError, ValueError, TypeError) as exc:
        raise HTTPException(400, str(exc)) from None


@app.get("/api/runs/{run_id}")
def get_run(run_id: str) -> dict:
    try:
        return store.get(run_id)
    except StorageError as exc:
        raise HTTPException(404, str(exc)) from None


@app.delete("/api/runs/{run_id}")
def delete_run(run_id: str) -> dict:
    try:
        store.delete(run_id)
        return {"deleted": run_id}
    except StorageError as exc:
        raise HTTPException(404, str(exc)) from None


# ---- Assistant (Phase 3) ----


def get_ai_client() -> OllamaClient:
    return OllamaClient.from_env()


async def _resolve_context(refs) -> list[tuple[str | None, dict]]:
    """Turn run references into real RunResults, server-side. The client never supplies run data itself."""
    if not refs:
        return []
    if not isinstance(refs, list) or len(refs) > 2:
        raise HTTPException(400, "Attach at most 2 runs.")
    runs = []
    for ref in refs:
        if not isinstance(ref, dict):
            raise HTTPException(400, "Each attached run must be an object.")
        try:
            if ref.get("saved_id"):
                record = store.get(str(ref["saved_id"]))
                runs.append((str(ref.get("label") or "")[:100] or record["label"], record["result"]))
            elif ref.get("experiment_id"):
                result = await asyncio.to_thread(_run_experiment, str(ref["experiment_id"]), ref.get("params"), ref.get("seed"))
                label = str(ref.get("label") or "") or None
                runs.append((label, result.to_dict()))
            else:
                raise HTTPException(400, "A run reference needs 'saved_id' or 'experiment_id'.")
        except StorageError as exc:
            raise HTTPException(404, str(exc)) from None
        except ExperimentError as exc:
            raise HTTPException(400, str(exc)) from None
    return runs


async def _with_ai(call):
    if _ai_lock.locked():
        raise HTTPException(429, "The assistant is still working on the previous request.")
    async with _ai_lock:
        try:
            return await call()
        except AIOffline as exc:
            _health_cache.update(at=-1e9, value=None)  # re-check health next time
            raise HTTPException(503, f"AI offline: {exc}") from None
        except AIError as exc:
            raise HTTPException(502, str(exc)) from None
        except ValueError as exc:
            raise HTTPException(400, str(exc)) from None


@app.post("/api/assistant/ask")
async def assistant_ask(body: dict) -> dict:
    runs = await _resolve_context(body.get("context"))
    history = body.get("history") if isinstance(body.get("history"), list) else None
    return await _with_ai(lambda: ai_assistant.ask(get_ai_client(), str(body.get("question", "")), runs, history))


@app.post("/api/assistant/suggest")
async def assistant_suggest(body: dict) -> dict:
    runs = await _resolve_context(body.get("context"))
    return await _with_ai(lambda: ai_assistant.suggest(get_ai_client(), runs, str(body.get("goal") or "")))


def handle_message(session: RealtimeSim, msg) -> dict | None:
    """Apply one client message. Returns a reply for the client, or None."""
    if not isinstance(msg, dict) or not isinstance(msg.get("cmd"), str):
        return {"type": "error", "message": 'Messages look like {"cmd": "takeoff"}.'}
    cmd = msg["cmd"]
    args = {k: v for k, v in msg.items() if k != "cmd"}
    try:
        if cmd in ("reset", "start") and ("config" in args or "preset" in args):
            cfg = get_preset(args["preset"]) if "preset" in args else parse_config(args["config"])
            problems = validate_config(cfg)
            if problems:
                return {"type": "error", "message": " ".join(problems)}
            session.reset(cfg)
            return {"type": "config", "config": cfg.to_dict(), "message": f"Reset with '{cfg.name}'."}
        if cmd == "reset":
            session.reset()
            return {"type": "config", "config": session.config.to_dict(), "message": "Reset."}
        text = session.command(cmd, **args)
        return {"type": "info", "message": text} if text else None
    except (ConfigError, ValueError, TypeError) as exc:
        return {"type": "error", "message": str(exc)}


@app.websocket("/ws/sim")
async def ws_sim(ws: WebSocket) -> None:
    """One live simulation per browser tab. It stops when the tab disconnects."""
    await ws.accept()
    session = RealtimeSim(get_preset("basic"))
    outbox: list[dict] = [{"type": "config", "config": session.config.to_dict(), "message": "Connected."}]

    async def receive_loop() -> None:
        while True:
            try:
                msg = await ws.receive_json()
            except (ValueError, KeyError):
                outbox.append({"type": "error", "message": "Could not read that message (expected JSON)."})
                continue
            reply = handle_message(session, msg)
            if reply:
                outbox.append(reply)

    receiver = asyncio.create_task(receive_loop())
    loop = asyncio.get_running_loop()
    last = loop.time()
    try:
        while not receiver.done():
            await asyncio.sleep(1.0 / STREAM_HZ)
            now = loop.time()
            session.advance(now - last)
            last = now
            while outbox:
                await ws.send_json(outbox.pop(0))
            await ws.send_json(session.snapshot())
    except (WebSocketDisconnect, RuntimeError):
        pass
    finally:
        receiver.cancel()


# ---- Lite mode: serve the pre-built frontend from this server (must be added AFTER all routes) ----


def mount_frontend(target: FastAPI, dist_dir: str | Path) -> bool:
    """Serve the built web page (index.html + assets) at '/'. API and WebSocket routes keep priority
    because they were registered first. Returns False if the folder has no index.html."""
    dist = Path(dist_dir)
    if not (dist / "index.html").is_file():
        return False
    target.mount("/", StaticFiles(directory=dist, html=True), name="frontend")
    return True


if os.environ.get("DRONELAB_FRONTEND_DIST"):
    if not mount_frontend(app, os.environ["DRONELAB_FRONTEND_DIST"]):
        print(f"Warning: no built frontend in {os.environ['DRONELAB_FRONTEND_DIST']} (run: npm --prefix frontend run build)")
