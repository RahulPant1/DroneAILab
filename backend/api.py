"""HTTP + WebSocket API for the browser UI.

  GET  /api/health           -> {sim: "ok", llm: "ok" | "offline", model}
  GET  /api/presets          -> drone presets + validation limits
  POST /api/config/validate  -> {valid, problems, derived}
  GET  /api/learn/{slug}     -> a docs/learn/ page as Markdown text
  WS   /ws/sim               -> live simulation (client sends commands, server streams state ~30 Hz)
                               incl. {"cmd": "course", "scenario": id|null} and {"cmd": "mission", "planner", "cruise_speed_m_s"}
                               and {"cmd": "sitl", "action": start|stop|takeoff|goto|hover|square|rtl|land|compare, ...}
                               (real ArduCopter SITL; its state rides along in every frame as "sitl")

  GET  /api/experiments                -> the six Lab experiments (sliders, concept, metrics)
  POST /api/experiments/{id}/run       -> run headless, returns a RunResult
  GET  /api/runs                       -> saved runs (newest first)
  POST /api/runs                       -> save {experiment_id, params, seed, label} (re-runs on the server)
  GET  /api/runs/compare?a=&b=         -> two saved runs + metric differences
  GET  /api/runs/{id}, DELETE /api/runs/{id}
  POST /api/runs/delete-old            -> {keep_newest: N}

  GET  /api/missions/scenarios -> built-in mission courses
  GET  /api/race/tracks       -> rings-race tracks + best times
  GET  /api/builder/parts      -> 🛠 Drone Builder catalogue;  POST /api/builder/design {parts} -> report
                               WS {"cmd": "design", "parts": {...}, "drain_speedup": 1} flies it on its battery;
                               WS {"cmd": "battery", "enabled", "cells", "capacity_mah", "drain_speedup"}
  GET  /api/logs               -> ArduPilot flight logs (simulator + uploaded .BIN)
  POST /api/logs/upload        -> raw .BIN body + X-Filename header -> {id}
  GET  /api/logs/{id}/analysis -> 🩺 log doctor: checks, timeline, charts
  POST /api/logs/{id}/explain  -> {question?} -> local-LLM debrief (summary only, numbers checked)
  GET  /api/sitl/faults       -> fault-injection scenarios for the ArduPilot panel ({"cmd": "sitl", "action": "fault", "fault": id})
  POST /api/missions/run       -> {mission: {waypoints, obstacles, cruise_alt_m, cruise_speed_m_s}, planner, wind_speed_m_s, seed}
  GET  /api/site               -> 🌾 the farm map (fields, buildings, no-fly zones, pads, places, geofence)
  POST /api/plan/compile       -> {spec: {tasks, altitude_m, speed_m_s, side_overlap_pct}, battery?} -> route + safety checks
  POST /api/ai/plan            -> {request: "survey the north field...", battery?} -> local-LLM task list, compiled + checked
                               WS {"cmd": "plan", "spec": {...}} flies it (re-checked; refused if any check FAILs);
                               WS {"cmd": "precland", "mode": "vision"|"gps"}, {"cmd": "camera", "enabled"}, {"cmd": "speed", "factor"}

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
import atexit
import os
import re
import threading
import time
from pathlib import Path

import httpx
from dotenv import load_dotenv
from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles

from .ai import assistant as ai_assistant
from .ai.mission_planner import plan_from_text
from .autonomy.plan import PlanError, compile_plan
from .autonomy.site import FARM
from .sim.battery import BatteryConfig
from .autonomy.mission import PLANNERS, SCENARIOS, MissionError, mission_from_dict, run_mission
from .sim.environment import EnvConfig
from .ai.ollama_client import AIError, AIOffline, OllamaClient
from .experiments import EXPERIMENTS, ExperimentError, get_experiment
from .sim import builder
from .sim.config import GAIN_LIMITS, LIMITS, PRESETS, ConfigError, DroneConfig, get_preset, validate_config
from .autonomy.live import MissionSim
from .games.race import TRACKS, load_best_times
from .logdoctor import store as logstore
from .logdoctor.analyze import analyze as analyze_log
from .logdoctor.explain import explain as explain_log
from .sitl.bridge import BRIDGE
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
atexit.register(BRIDGE.stop)  # never leave an ArduPilot SITL running after the server stops


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


# ---- Missions (Phase 4): custom courses; the built-in ones are Lab experiment 7 ----


@app.get("/api/builder/parts")
def builder_parts() -> dict:
    """🛠 Drone Builder catalogue: frames, motors, props, battery options."""
    return builder.catalog()


@app.post("/api/builder/design")
def builder_design(body: dict | None = None) -> dict:
    """Work out a design: mass, T/W, hover throttle and power, flight time, warnings."""
    try:
        d = builder.design((body or {}).get("parts") or {})
    except builder.DesignError as exc:
        raise HTTPException(400, str(exc)) from None
    return {"report": d["report"], "config": d["config"].to_dict(), "battery": d["battery"].to_dict(), "parts": d["parts"]}


@app.get("/api/logs")
def logs_list() -> dict:
    """ArduPilot flight logs: the simulator's own, and .BIN files you uploaded (newest first)."""
    return {"logs": logstore.list_logs()}


@app.post("/api/logs/upload")
async def logs_upload(request: Request) -> dict:
    """Upload an ArduPilot DataFlash log: the raw file as the body, its name in the X-Filename header."""
    size = int(request.headers.get("content-length") or 0)
    if size > logstore.MAX_UPLOAD_BYTES:
        raise HTTPException(413, f"That log is larger than {logstore.MAX_UPLOAD_BYTES // 2**20} MB.")
    data = await request.body()
    try:
        log_id = logstore.save_upload(request.headers.get("x-filename", "log.bin"), data)
    except ValueError as exc:
        raise HTTPException(400, str(exc)) from None
    return {"id": log_id}


@app.get("/api/logs/{log_id}/analysis")
def logs_analysis(log_id: str) -> dict:
    """The 🩺 log doctor's checks, timeline and charts for one log (cached)."""
    if not _heavy_work.acquire(timeout=10):
        raise HTTPException(429, "Another analysis is still going. Try again in a moment.")
    try:
        return logstore.cached_analysis(log_id, analyze_log)
    except logstore.LogNotFound as exc:
        raise HTTPException(404, str(exc)) from None
    except Exception as exc:  # noqa: BLE001  (a corrupt or unusual log)
        raise HTTPException(422, f"Couldn't read that log: {exc}") from None
    finally:
        _heavy_work.release()


@app.post("/api/logs/{log_id}/explain")
async def logs_explain(log_id: str, body: dict | None = None) -> dict:
    """The local LLM's debrief of a log (it sees the analysis summary only, never the raw log)."""
    try:
        analysis = await asyncio.to_thread(logs_analysis, log_id)
    except HTTPException:
        raise
    question = str((body or {}).get("question") or "")
    return await _with_ai(lambda: explain_log(get_ai_client(), analysis, question))


@app.get("/api/sitl/faults")
def sitl_faults() -> dict:
    """The fault-injection scenarios for the 🛰 ArduPilot panel, plus the last full report (if generated)."""
    from .sitl.faults import FAULTS

    return {"faults": [{"id": f.id, "title": f.title, "injected": f.injected, "expected": f.expected, "kind": f.kind,
                        "frame": f.frame} for f in FAULTS]}


@app.get("/api/race/tracks")
def race_tracks() -> dict:
    """The rings-race tracks, with the best time on each (None = not finished yet)."""
    best = load_best_times()
    return {"tracks": {k: {"title": v["title"], "tip": v["tip"], "rings": len(v["rings"]), "best_s": best.get(k)} for k, v in TRACKS.items()}}


@app.get("/api/missions/scenarios")
def mission_scenarios() -> dict:
    return {
        "planners": list(PLANNERS),
        "scenarios": {
            k: {"title": v["title"], "waypoints": [list(p) for p in v["waypoints"]], "obstacles": [b.to_dict() for b in v["obstacles"]]}
            for k, v in SCENARIOS.items()
        },
    }


@app.post("/api/missions/run")
def run_custom_mission(body: dict) -> dict:
    try:
        mission = mission_from_dict(body.get("mission"))
        wind = float(body.get("wind_speed_m_s") or 0.0)
        if not 0 <= wind <= 15:
            raise MissionError("wind_speed_m_s must be between 0 and 15.")
        env = EnvConfig(wind_speed_m_s=wind, wind_direction_deg=float(body.get("wind_direction_deg") or 90.0)) if wind else None
        if not _heavy_work.acquire(timeout=10):
            raise HTTPException(429, "Another run is still going. Try again in a moment.")
        try:
            result, _, _ = run_mission(mission, str(body.get("planner", "astar")), env=env, seed=int(body.get("seed") or 0))
        finally:
            _heavy_work.release()
        return result.to_dict()
    except (MissionError, ValueError, TypeError) as exc:
        raise HTTPException(400, str(exc)) from None


# ---- 🌾 Farm: flight plans and the AI mission planner ----


def _battery_from(body: dict) -> BatteryConfig | None:
    b = body.get("battery")
    if not isinstance(b, dict):
        return None
    try:
        return BatteryConfig(cells=int(b.get("cells", 3)), capacity_mah=float(b.get("capacity_mah", 3000)))
    except (TypeError, ValueError):
        raise HTTPException(400, "battery needs numbers: cells, capacity_mah.") from None


@app.get("/api/site")
def site() -> dict:
    return FARM.to_dict()


@app.post("/api/plan/compile")
def plan_compile(body: dict) -> dict:
    """Check a task list without flying it (what the 🧭 Mission planner shows before you press Fly)."""
    try:
        return compile_plan(body.get("spec") or {}, FARM, battery=_battery_from(body)).to_dict()
    except PlanError as exc:
        raise HTTPException(400, str(exc)) from None


@app.post("/api/ai/plan")
async def ai_plan(body: dict) -> dict:
    """Plain English -> task list (local LLM) -> compiled plan + safety checks. Never flies anything itself."""
    battery = _battery_from(body)
    return await _with_ai(lambda: plan_from_text(get_ai_client(), str(body.get("request") or ""), FARM, battery))


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


def handle_message(session: MissionSim, msg) -> dict | None:
    """Apply one client message. Returns a reply for the client, or None."""
    if not isinstance(msg, dict) or not isinstance(msg.get("cmd"), str):
        return {"type": "error", "message": 'Messages look like {"cmd": "takeoff"}.'}
    cmd = msg["cmd"]
    args = {k: v for k, v in msg.items() if k != "cmd"}
    try:
        if cmd == "design":  # 🛠 Drone Builder: fly this design on its own battery
            cfg, text = session.load_design(args.get("parts") or {}, float(args.get("drain_speedup", 1.0)))
            return {"type": "config", "config": cfg.to_dict(), "course": session.course_info(), "race": session.race_info(),
                    "message": text}
        if cmd == "sitl":  # the 🛰 ArduPilot panel (one SITL per server, shared by all tabs)
            return {"type": "info", "message": BRIDGE.handle(args)}
        if cmd in ("reset", "start") and ("config" in args or "preset" in args):
            cfg = get_preset(args["preset"]) if "preset" in args else parse_config(args["config"])
            problems = validate_config(cfg)
            if problems:
                return {"type": "error", "message": " ".join(problems)}
            battery_note = ""
            if session.battery_cfg is not None:  # a builder design's battery doesn't belong to another drone
                session.set_battery(False)
                battery_note = " Battery off (it belonged to the previous design)."
            session.reset(cfg)
            return {"type": "config", "config": cfg.to_dict(), "course": session.course_info(), "race": session.race_info(),
                    "message": f"Reset with '{cfg.name}'.{battery_note}"}
        if cmd == "reset":
            session.reset()
            return {"type": "config", "config": session.config.to_dict(), "course": session.course_info(),
                    "race": session.race_info(), "message": "Reset."}
        text = session.command(cmd, **args)
        if cmd in ("course", "mission", "race", "plan"):
            return {"type": "course", "course": session.course_info(), "race": session.race_info(), "message": text}
        return {"type": "info", "message": text} if text else None
    except (ConfigError, ValueError, TypeError, KeyError) as exc:
        return {"type": "error", "message": str(exc) if not isinstance(exc, KeyError) else f"Missing {exc}."}


@app.websocket("/ws/sim")
async def ws_sim(ws: WebSocket) -> None:
    """One live simulation per browser tab. It stops when the tab disconnects."""
    await ws.accept()
    session = MissionSim(get_preset("basic"))
    outbox: list[dict] = [{"type": "config", "config": session.config.to_dict(), "course": None, "race": None, "message": "Connected."}]

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
            while session.notices:
                outbox.append({"type": "info", "message": session.notices.pop(0)})
            while outbox:
                await ws.send_json(outbox.pop(0))
            snap = session.snapshot()
            snap["sitl"] = BRIDGE.status()  # None unless the ArduPilot panel has been used
            await ws.send_json(snap)
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
