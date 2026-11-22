"""AI mission planner: plain English in, a checked flight plan out.

  "Survey the north field at 25 m, then land on pad B"
        │  local LLM (Gemma via Ollama), constrained to a JSON schema whose names come from the farm map
        ▼
  task list  {"tasks": [{"type": "survey", "field": "north field"}, {"type": "land", "place": "pad B"}], "altitude_m": 25, ...}
        │  deterministic code (autonomy/plan.py): waypoints, A* routing, safety checks
        ▼
  flight plan + PASS / WARN / FAIL checks  →  the user presses Fly (never automatic)

The safety net:
  - The model only WRITES the task list. It can't produce coordinates: every place must be a name from the map
    (the JSON schema lists them), and the code looks the coordinates up.
  - Places the model couldn't find, or things it chose not to do, must be listed (not_on_map / left_out), and a
    request that mentions a place not on the map can't be flown.
  - Every number the model writes in its explanation is checked against the request, the map and the plan.
  - Altitude, speed, geofence, no-fly zones, buildings and the battery are checked by code, not by the model.
  - Each request and reply is logged to data/assistant_log.jsonl (kind "mission_plan").
"""

from __future__ import annotations

import json
import time

from backend.autonomy.plan import MAX_TASKS, PlanError, compile_plan
from backend.autonomy.site import FARM, Site
from backend.sim.battery import BatteryConfig

from .assistant import MAX_QUESTION_CHARS, log_exchange, unverified_numbers
from .ollama_client import OllamaClient

SYSTEM_PROMPT = """You are the mission planner of a farm drone in DroneLab, a simulator. Turn the user's request into a task list.

THE FARM (x = metres east of the home pad, y = metres north):
{site}

TASK TYPES (in flying order):
- {{"type": "survey", "field": "<field name>"}}: photograph the whole field with a lawnmower pattern.
- {{"type": "goto", "place": "<place name>"}}: fly to a pad, a place, or the middle of a field.
- {{"type": "hover", "place": "<place name>", "seconds": <1-60>}}: fly there and hover.
- {{"type": "land", "place": "<pad name>", "precision": true}}: land on a pad (precision = use the camera to find its marker). Last task only.
If there is no land task, the drone flies home and lands there.

RULES:
1. Use ONLY the exact names listed above. Never invent a place, never write coordinates.
2. If the request mentions a place that is not on the map, list it in "not_on_map" and leave it out of the tasks.
3. Never plan to enter a no-fly zone. If the request asks for that, leave it out and say why in "left_out".
4. altitude_m is the height for the WHOLE flight, surveys included: "survey X at 12 m", "map X from 12 m" or "fly at 12 m"
   all mean altitude_m = 12. Use the numbers the user gives, EVEN IF they look unsafe (the safety checks will judge them).
   If they didn't say: altitude_m 20, speed_m_s 3. side_overlap_pct: 70 unless they ask for something else.
5. A survey task flies to its field by itself: don't add a goto to the field before it.
6. At most {max_tasks} tasks.
7. "understood": one short sentence saying what the flight will do. "assumptions": short notes on anything you had to guess.
Reply ONLY with JSON."""


def site_description(site: Site = FARM) -> str:
    lines = []
    for f in site.fields:
        xs, ys = [p[0] for p in f.polygon], [p[1] for p in f.polygon]
        lines.append(f'- field "{f.name}" ({f.crop}): x {min(xs):.0f} to {max(xs):.0f}, y {min(ys):.0f} to {max(ys):.0f}')
    for p in site.pads:
        lines.append(f'- landing pad "{p.name}" at ({p.x_m:.0f}, {p.y_m:.0f})')
    for p in site.places:
        lines.append(f'- place "{p.name}" at ({p.x_m:.0f}, {p.y_m:.0f})')
    for b in site.buildings:
        lines.append(f'- building "{b.name}", {b.box.height_m:.0f} m tall (flown over if higher, otherwise around)')
    for n in site.no_fly:
        lines.append(f'- NO-FLY ZONE "{n.name}" ({n.why})')
    lines.append(f"- geofence: {site.geofence_radius_m:.0f} m from home; altitude ceiling {site.ceiling_m:.0f} m")
    return "\n".join(lines)


def plan_json_schema(site: Site = FARM) -> dict:
    """The JSON schema given to Ollama's `format`: the model can only use names that exist."""
    return {
        "type": "object",
        "properties": {
            "understood": {"type": "string"},
            "tasks": {
                "type": "array",
                "maxItems": MAX_TASKS,
                "items": {
                    "type": "object",
                    "properties": {
                        "type": {"type": "string", "enum": ["survey", "goto", "hover", "land"]},
                        "field": {"type": "string", "enum": [f.name for f in site.fields]},
                        "place": {"type": "string", "enum": site.destination_names()},
                        "seconds": {"type": "number"},
                        "precision": {"type": "boolean"},
                    },
                    "required": ["type"],
                },
            },
            "altitude_m": {"type": "number"},
            "speed_m_s": {"type": "number"},
            "side_overlap_pct": {"type": "number"},
            "not_on_map": {"type": "array", "items": {"type": "string"}},
            "left_out": {"type": "array", "items": {"type": "string"}},
            "assumptions": {"type": "array", "items": {"type": "string"}},
        },
        "required": ["understood", "tasks", "altitude_m", "speed_m_s", "not_on_map", "left_out", "assumptions"],
    }


def _strings(v, limit: int = 6) -> list[str]:
    return [str(s)[:200] for s in v[:limit]] if isinstance(v, list) else []


def interpret(raw: str, request: str, site: Site = FARM, battery: BatteryConfig | None = None) -> dict:
    """Check the model's reply and compile it into a plan. Raises PlanError with a plain-English reason."""
    try:
        data = json.loads(raw)
    except (TypeError, ValueError):
        raise PlanError("The AI's reply wasn't valid JSON, so it was ignored.") from None
    if not isinstance(data, dict):
        raise PlanError("The AI's reply wasn't a JSON object.")
    tasks = data.get("tasks")
    if not isinstance(tasks, list) or not tasks:
        raise PlanError("The AI didn't plan any tasks. Try saying what the drone should do, using the names on the map.")
    for t in tasks:  # the schema can't say "survey needs a field, the others a place"; tidy the obvious slip
        if isinstance(t, dict) and t.get("type") == "survey" and "field" not in t and t.get("place") in [f.name for f in site.fields]:
            t["field"] = t.pop("place")
    spec = {k: data.get(k) for k in ("tasks", "altitude_m", "speed_m_s", "side_overlap_pct") if data.get(k) is not None}
    plan = compile_plan(spec, site, battery=battery)
    ai = {"understood": str(data.get("understood", ""))[:400], "assumptions": _strings(data.get("assumptions")),
          "not_on_map": _strings(data.get("not_on_map")), "left_out": _strings(data.get("left_out"))}
    out = plan.to_dict()
    if ai["not_on_map"]:
        out["checks"].insert(0, {"id": "names", "label": "Places", "status": "fail",
                                 "detail": "Not on the map: " + ", ".join(ai["not_on_map"]) + ". Fix the request or the plan."})
        out["ok"] = False
    if ai["left_out"]:
        out["checks"].insert(0, {"id": "left_out", "label": "Left out", "status": "warn",
                                 "detail": "The AI left out: " + "; ".join(ai["left_out"])})
    text = " ".join([ai["understood"], *ai["assumptions"], *ai["left_out"]])
    known = [request, site.to_dict(), out["spec"], out["estimate"], [s for s in out["surveys"]]]
    return {"spec": out["spec"], "plan": out, "ai": ai, "unverified_numbers": unverified_numbers(text, known)}


async def plan_from_text(client: OllamaClient, request: str, site: Site = FARM, battery: BatteryConfig | None = None) -> dict:
    """Ask the local model for a task list, then check and compile it. Returns {ok: True, ...} or {ok: False, error}."""
    request = (request or "").strip()[:MAX_QUESTION_CHARS]
    if not request:
        raise ValueError("Say what the drone should do, e.g. 'survey the north field, then land on pad B'.")
    messages = [{"role": "system", "content": SYSTEM_PROMPT.format(site=site_description(site), max_tasks=MAX_TASKS)},
                {"role": "user", "content": request}]
    entry = {"kind": "mission_plan", "model": client.model, "messages": messages}
    start = time.monotonic()
    try:
        raw = await client.chat(messages, json_schema=plan_json_schema(site))
    except Exception as exc:
        log_exchange({**entry, "ok": False, "error": str(exc)})
        raise
    duration = round(time.monotonic() - start, 1)
    try:
        result = interpret(raw, request, site, battery)
    except PlanError as exc:
        log_exchange({**entry, "ok": False, "reply": raw, "rejected": str(exc), "duration_s": duration})
        return {"ok": False, "error": str(exc), "raw": raw[:2000], "duration_s": duration}
    log_exchange({**entry, "ok": True, "reply": raw, "flyable": result["plan"]["ok"], "duration_s": duration})
    return {"ok": True, **result, "model": client.model, "duration_s": duration}
