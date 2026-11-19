"""HTTP + WebSocket API for the browser UI.

  GET  /api/health           -> {sim: "ok", llm: "ok" | "offline", model}
  GET  /api/presets          -> drone presets + validation limits
  POST /api/config/validate  -> {valid, problems, derived}
  GET  /api/learn/{slug}     -> a docs/learn/ page as Markdown text
  WS   /ws/sim               -> live simulation (client sends commands, server streams state ~30 Hz)

The LLM is never loaded here: the health check only asks Ollama for its model LIST.
Run with: .venv/bin/uvicorn backend.api:app --host 127.0.0.1 --port 8001  (or ./run.sh)
"""

from __future__ import annotations

import asyncio
import os
import re
import time
from pathlib import Path

import httpx
from dotenv import load_dotenv
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect

from .sim.config import GAIN_LIMITS, LIMITS, PRESETS, ConfigError, DroneConfig, get_preset, validate_config
from .sim.runner import RealtimeSim

PROJECT_ROOT = Path(__file__).resolve().parent.parent
LEARN_DIR = PROJECT_ROOT / "docs" / "learn"
load_dotenv(PROJECT_ROOT / ".env")

STREAM_HZ = 30.0
HEALTH_CACHE_S = 10.0

app = FastAPI(title="DroneLab", docs_url="/api/docs", openapi_url="/api/openapi.json")
_health_cache: dict = {"at": -1e9, "value": None}


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
