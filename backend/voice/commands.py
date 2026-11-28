"""Voice (or typed) commands -> actions on the live simulator, across the whole app.

  "take off" / "go left 3 metres" / "stop"        -> QUICK: a few fixed phrases, matched instantly, no AI.
  "race the figure eight with the autopilot"      -> AI: the local model (Gemma via Ollama) picks actions from a JSON
  "explore the building without GPS"                 schema whose names are only what exists in the app right now.
  "survey the east field and land on pad B"       -> the farm's own AI mission planner (ai/mission_planner.py).

The safety net (the same idea as the farm planner):
  - "stop" anywhere in what was heard = hover NOW, and any action waiting for a "go" is cancelled. No AI involved.
  - The model never writes coordinates or code: it picks an action and names from enums. Code checks every number
    (ranges below) and whether the action makes sense right now (e.g. the race autopilot needs a race loaded).
  - Anything that flies a route by itself, or moves the drone to another world, waits for "go" (or a click).
    Small moves (up, left, turn, land, hover) happen at once: they are what a pilot would do with the keys.
  - Every AI exchange is logged to data/assistant_log.jsonl (kind "voice").
"""

from __future__ import annotations

import json
import math
import re
import time
from dataclasses import dataclass, field

from backend.autonomy.agile import METHOD_NAMES as AGILE_NAMES
from backend.autonomy.explore import LOCAL_PLANNERS, LOCALISATION, WORLDS as EXPLORE_WORLDS
from backend.autonomy.formation import SHAPE_NAMES
from backend.autonomy.mission import PLANNERS, SCENARIOS
from backend.games.race import TRACKS

PENDING_EXPIRES_S = 60.0
MAX_ACTIONS = 3
MOVE_LIMITS_M = (0.2, 20.0)
TURN_LIMITS_DEG = (5.0, 360.0)
HEIGHT_LIMITS_M = (0.5, 50.0)  # 50 m = the pilot's ceiling (MAX_FLY_ALT_M)
WIND_LIMIT_M_S = 10.0
DIRECTIONS = ("forward", "back", "left", "right", "up", "down")
COMPASS = {"north": 90.0, "east": 0.0, "south": 270.0, "west": 180.0}  # where the wind blows FROM -> toward = +180
PLANNER_NAMES = {"straight": "straight line", "potential": "potential field", "astar": "A* (A star)",
                 "lidar": "lidar sense-and-avoid"}

# action -> (what it does, needs "go" first)
ACTIONS: dict[str, tuple[str, bool]] = {
    "hover": ("stop and hover where it is", False),
    "land": ("land where it is", False),
    "takeoff": ("take off (height_m, default 2)", False),
    "move": ("fly a short distance: direction forward/back/left/right/up/down, metres", False),
    "height": ("climb or descend to an exact height above home (height_m, up to 50)", False),
    "turn": ("turn on the spot: direction left/right, degrees", False),
    "go_home": ("fly back over the start point", False),
    "free_flight": ("empty world: no course, no race", True),
    "course": ("load an obstacle course (course)", True),
    "mission": ("fly the loaded course's mission by itself (planner)", True),
    "race": ("load a rings-race track (track)", True),
    "race_autopilot": ("let the autopilot fly the loaded race track (method)", True),
    "formation": ("wingmen following the drone (count 0-4, shape)", False),
    "explore": ("explore an unknown place by itself (world, local, localisation, movers)", True),
    "farm": ("load the farm", True),
    "farm_plan": ("on the farm: survey fields / fly to places / land on pads (request = the user's words)", True),
    "precision_land": ("on the farm: land on the nearest pad using the camera", True),
    "wind": ("set the wind (speed_m_s 0-10, from north/east/south/west)", False),
    "lidar": ("lidar on or off (on)", False),
    "sensors": ("realistic noisy sensors on or off (on)", False),
    "reset": ("put the drone back on the ground at the start", True),
}

NUMBER_WORDS = {"zero": 0, "one": 1, "a": 1, "an": 1, "two": 2, "three": 3, "four": 4, "five": 5, "six": 6,
                "seven": 7, "eight": 8, "nine": 9, "ten": 10, "eleven": 11, "twelve": 12, "fifteen": 15,
                "twenty": 20, "thirty": 30, "forty five": 45, "forty": 40, "fifty": 50, "sixty": 60, "ninety": 90,
                "half": 0.5, "a hundred and eighty": 180, "one hundred and eighty": 180, "hundred and eighty": 180,
                "three hundred and sixty": 360}


class VoiceError(ValueError):
    """Something the user should hear back (plain English)."""


@dataclass
class Pending:
    actions: list[dict]
    say: str
    at: float = field(default_factory=time.monotonic)


# ---- what exists right now -------------------------------------------------------------------------------------
def context(session) -> dict:
    """What the drone is doing and what is loaded: the model only gets to choose from this."""
    s = session.state
    flying = bool(session.pilot.sp.armed and not s.on_ground) or session.follower is not None
    course = session.course_id
    scene = ("farm" if course == "farm" else "exploration world" if course and course.startswith("explore_")
             else "obstacle course" if course else "race track" if session.race else "empty playground")
    return {
        "scene": scene,
        "course": course,
        "race": session.race.track_id if session.race else None,
        "flying": flying,
        "autopilot": session.follower is not None or session.agile_phase is not None or session.explore_phase in ("takeoff", "explore"),
        "lidar": bool(session.lidar_on),
        "sensors": bool(session.sim.estimator is not None),
        "wingmen": session.formation.count,
    }


def available(action: str, ctx: dict) -> str | None:
    """None = can do it now; otherwise why not (plain English)."""
    if action == "mission" and ctx["scene"] != "obstacle course":
        return "Load an obstacle course first (say 'load the slalom', for example)."
    if action == "race_autopilot" and not ctx["race"]:
        return "Load a race track first (say 'load the figure eight')."
    if action in ("farm_plan", "precision_land") and ctx["scene"] != "farm":
        return "That needs the farm: say 'load the farm' first."
    if action == "precision_land" and not ctx["flying"]:
        return "Take off first."
    if action in ("move", "turn", "go_home") and not ctx["flying"]:
        return "Take off first."
    return None


# ---- the quick path: fixed phrases, no AI --------------------------------------------------------------------------
def normalise(text: str) -> str:
    t = text.lower().replace("-", " ")
    t = re.sub(r"[^a-z0-9. ]+", " ", t)
    t = re.sub(r"\b(please|now|the drone|drone|can you|could you|hey|ok|okay)\b", " ", t)
    for words, n in sorted(NUMBER_WORDS.items(), key=lambda kv: -len(kv[0])):
        t = re.sub(rf"\b{words}\b", str(n), t)
    t = re.sub(r"\b(\d+)\s*(?:and a half|point 5)\b", lambda m: str(int(m.group(1)) + 0.5), t)
    return re.sub(r"\s+", " ", t).strip(" .")


STOP_RE = re.compile(r"\b(stop|freeze|halt|abort|emergency)\b")
_NUM = r"(\d+(?:\.\d+)?)"
UP_STEP_M = 3.0  # "go up" / "higher" with no number
BIG_STEP_M = 10.0  # "much higher"
_UNIT = r"(?:\s*(?:m|metre|metres|meter|meters))?"
QUICK = [
    (re.compile(r"^(?:hover|wait|stay|hold)(?: there| here| still| position)?$"), lambda m: {"action": "hover"}),
    (re.compile(r"^(?:land|come down|touch down|land here)$"), lambda m: {"action": "land"}),
    (re.compile(rf"^(?:take off|takeoff|lift off|launch|fly up)(?: to {_NUM}{_UNIT})?$"),
     lambda m: {"action": "takeoff", **({"height_m": float(m.group(1))} if m.group(1) else {})}),
    (re.compile(rf"^(?:go|fly|move)? ?(up|down|forward|forwards|ahead|back|backward|backwards|left|right)(?: by)?(?: {_NUM}{_UNIT})?$"),
     lambda m: {"action": "move", "direction": _dir(m.group(1)),
                "metres": float(m.group(2)) if m.group(2) else (UP_STEP_M if m.group(1) in ("up", "down") else 2.0)}),
    (re.compile(r"^(?:go |fly |climb |move |get )?(?:((?:a|1) bit|(?:a|1) little|slightly)|(much|way|(?:a|1) lot|loads|really)) ?(higher|lower)$|^(?:go |fly |climb |move |get )?(higher|lower)$"),
     lambda m: {"action": "move", "direction": "up" if (m.group(3) or m.group(4)) == "higher" else "down",
                "metres": 1.0 if m.group(1) else (BIG_STEP_M if m.group(2) else UP_STEP_M)}),
    (re.compile(rf"^(?:go|fly|climb|rise|descend|come down|drop|move)(?: up| down)? to {_NUM}{_UNIT}(?: high| up| height| altitude)?$"
                rf"|^(?:height|altitude) {_NUM}{_UNIT}$|^fly at {_NUM}{_UNIT}(?: high)?$"),
     lambda m: {"action": "height", "height_m": float(m.group(1) or m.group(2) or m.group(3))}),
    (re.compile(rf"^(?:turn|rotate|spin|yaw) (left|right)(?: by)?(?: {_NUM})?(?: degrees?)?$"),
     lambda m: {"action": "turn", "direction": m.group(1), "degrees": float(m.group(2)) if m.group(2) else 90.0}),
    (re.compile(r"^(?:turn|spin) (?:around|round)$"), lambda m: {"action": "turn", "direction": "left", "degrees": 180.0}),
    (re.compile(r"^(?:come back|come back home|come home|go home|go back home|return|return home|return to (?:base|home|start)|fly home|fly back home)$"),
     lambda m: {"action": "go_home"}),
    (re.compile(r"^(?:reset|start again|start over)$"), lambda m: {"action": "reset"}),
    (re.compile(r"^(?:lidar|laser) (on|off)$|^turn (on|off) (?:the )?(?:lidar|laser)$"),
     lambda m: {"action": "lidar", "on": (m.group(1) or m.group(2)) == "on"}),
    (re.compile(r"^(?:no wind|calm|wind off|stop the wind|turn off (?:the )?wind)$"), lambda m: {"action": "wind", "speed_m_s": 0.0}),
]
CONFIRM_RE = re.compile(r"^(?:yes|yeah|yep|go|go go|go ahead|do it|confirm|start|start it|fly|lets go|let s go|ok go)$")
CANCEL_RE = re.compile(r"^(?:no|nope|cancel|never mind|forget it|don t)$")


def _dir(word: str) -> str:
    return {"forwards": "forward", "ahead": "forward", "backward": "back", "backwards": "back"}.get(word, word)


def collapse_repeats(t: str) -> str:
    """'go left left' -> 'go left' (people repeat themselves, and Whisper sometimes hears a word twice)."""
    return re.sub(r"\b(\w+)(?: \1\b)+", r"\1", t)


def quick_parse(text: str) -> list[dict] | None:
    """Fixed phrases, possibly chained with 'then' / 'and'. None = not a quick command (ask the AI)."""
    t = collapse_repeats(normalise(text))
    if not t:
        return None
    parts = [p.strip() for p in re.split(r"\b(?:and then|then|and|after that)\b", t) if p.strip()]
    out = []
    for p in parts:
        for rx, make in QUICK:
            m = rx.match(p)
            if m:
                out.append(make(m))
                break
        else:
            return None
    return out[:MAX_ACTIONS] if out else None


# ---- the AI path ---------------------------------------------------------------------------------------------------
SYSTEM_PROMPT = """You turn what a person says to a toy drone in DroneLab (a simulator) into actions. Kids use it too: be
forgiving about wording, but only pick actions and names from the lists. Never invent a name, never write coordinates.

ACTIONS (use the exact action name; only the fields each needs):
{actions}

NAMES YOU MAY USE:
- course: {courses}
- track (rings races): {tracks}
- method (race autopilot): {methods}
- planner (course missions): {planners}
- shape (wingmen): {shapes}
- world (exploration): {worlds}
- local (exploration local planner): {locals}
- localisation (exploration): {locs}  ("without GPS"/"indoors" = slam; "no GPS and no lidar help" = dead_reckoning)

RULES:
0. "go left / right / forward / back / up / down" means MOVE that way (action move). Use turn ONLY when they say
   turn, rotate, spin or face. If a word is repeated ("left, left"), it is said once.
1. At most {max_actions} actions, in order. "race X with the autopilot" = race (track X) then race_autopilot.
   "fly the slalom by itself" = course (slalom) then mission. On the farm, put the whole request into one farm_plan
   action and copy the user's words into "request".
2. If they ask for something none of the actions can do, or a name that isn't listed, put it in "not_possible" in a
   few plain words, and leave it out of the actions.
3. "understood": one short friendly sentence, said back to the user, about what will happen.
Reply ONLY with JSON.

RIGHT NOW: {now}"""  # what changes goes LAST: Ollama reuses its work on the identical start of the prompt


def _names(d: dict) -> str:
    return ", ".join(f'"{k}" ({v})' for k, v in d.items())


def system_prompt(ctx: dict) -> str:
    now = (f"scene = {ctx['scene']}" + (f" ({ctx['course']})" if ctx["course"] else "")
           + (f", race track {ctx['race']} loaded" if ctx["race"] else "") + f", the drone is {'flying' if ctx['flying'] else 'on the ground'}"
           + (", an autopilot is flying it" if ctx["autopilot"] else "") + f", lidar {'on' if ctx['lidar'] else 'off'}"
           + f", {ctx['wingmen']} wingmen")
    return SYSTEM_PROMPT.format(
        now=now, actions="\n".join(f"- {k}: {v[0]}" for k, v in ACTIONS.items()),
        courses=_names({k: v["title"] for k, v in SCENARIOS.items()}),
        tracks=_names({k: v["title"] for k, v in TRACKS.items()}), methods=_names(AGILE_NAMES),
        planners=_names(PLANNER_NAMES), shapes=_names(SHAPE_NAMES), worlds=_names({k: w.title for k, w in EXPLORE_WORLDS.items()}),
        locals=_names(LOCAL_PLANNERS), locs=_names(LOCALISATION), max_actions=MAX_ACTIONS)


def json_schema() -> dict:
    """Ollama `format`: every name is an enum of what exists."""
    action = {
        "type": "object",
        "properties": {
            "action": {"type": "string", "enum": list(ACTIONS)},
            "direction": {"type": "string", "enum": list(DIRECTIONS)},
            "metres": {"type": "number"}, "degrees": {"type": "number"}, "height_m": {"type": "number"},
            "course": {"type": "string", "enum": list(SCENARIOS)},
            "planner": {"type": "string", "enum": list(PLANNERS)},
            "track": {"type": "string", "enum": list(TRACKS)},
            "method": {"type": "string", "enum": list(AGILE_NAMES)},
            "count": {"type": "integer"}, "shape": {"type": "string", "enum": list(SHAPE_NAMES)},
            "world": {"type": "string", "enum": list(EXPLORE_WORLDS)},
            "local": {"type": "string", "enum": list(LOCAL_PLANNERS)},
            "localisation": {"type": "string", "enum": list(LOCALISATION)},
            "movers": {"type": "boolean"}, "request": {"type": "string"},
            "speed_m_s": {"type": "number"}, "from": {"type": "string", "enum": list(COMPASS)},
            "on": {"type": "boolean"},
        },
        "required": ["action"],
    }
    return {"type": "object", "properties": {"understood": {"type": "string"},
                                             "actions": {"type": "array", "items": action, "maxItems": MAX_ACTIONS},
                                             "not_possible": {"type": "array", "items": {"type": "string"}}},
            "required": ["understood", "actions", "not_possible"]}


IDLE_CONTEXT = {"scene": "empty playground", "course": None, "race": None, "flying": False, "autopilot": False,
                "lidar": False, "sensors": False, "wingmen": 0}


async def warm_up(client) -> None:
    """Load the model AND let it read the (long, unchanging) instructions once, so the next command is quick.
    Called only when the user presses 🎙 Talk."""
    try:
        await client.chat([{"role": "system", "content": system_prompt(IDLE_CONTEXT)}, {"role": "user", "content": "hover"}],
                          json_schema=json_schema())
    except Exception:  # noqa: BLE001  (warming up is only a nicety)
        pass


async def ai_parse(client, text: str, ctx: dict) -> dict:
    """Ask the local model. Returns {understood, actions, not_possible, duration_s}."""
    from backend.ai.assistant import log_exchange

    messages = [{"role": "system", "content": system_prompt(ctx)}, {"role": "user", "content": text[:300]}]
    entry = {"kind": "voice", "model": client.model, "heard": text, "context": ctx}
    start = time.monotonic()
    try:
        raw = await client.chat(messages, json_schema=json_schema())
    except Exception as exc:
        log_exchange({**entry, "ok": False, "error": str(exc)})
        raise
    duration = round(time.monotonic() - start, 1)
    try:
        data = json.loads(raw)
        if not isinstance(data, dict) or not isinstance(data.get("actions"), list):
            raise ValueError
    except ValueError:
        log_exchange({**entry, "ok": False, "reply": raw[:2000], "duration_s": duration})
        raise VoiceError("Sorry, I didn't get that. Try saying it another way.") from None
    log_exchange({**entry, "ok": True, "reply": raw[:2000], "duration_s": duration})
    return {"understood": str(data.get("understood") or "")[:200],
            "actions": [a for a in data["actions"] if isinstance(a, dict)][:MAX_ACTIONS],
            "not_possible": [str(x)[:100] for x in data.get("not_possible") or []][:5], "duration_s": duration}


# ---- checking and running actions --------------------------------------------------------------------------------
def _clamp(v, lo, hi, what: str) -> float:
    try:
        x = float(v)
    except (TypeError, ValueError):
        raise VoiceError(f"I need a number for {what}.") from None
    if not math.isfinite(x) or not lo <= x <= hi:
        raise VoiceError(f"{what.capitalize()} must be between {lo:g} and {hi:g}.")
    return x


def check(action: dict, ctx: dict) -> dict:
    """Validate one action (names, numbers, whether it makes sense now). Returns a clean copy."""
    name = action.get("action")
    if name not in ACTIONS:
        raise VoiceError(f"I can't do '{name}'.")
    why = available(name, ctx)
    if why:
        raise VoiceError(why)
    a: dict = {"action": name}
    if name == "move":
        d = action.get("direction")
        if d not in DIRECTIONS:
            raise VoiceError("Which way: forward, back, left, right, up or down?")
        a.update(direction=d, metres=_clamp(action.get("metres", 2.0), *MOVE_LIMITS_M, "the distance in metres"))
    elif name == "turn":
        d = action.get("direction")
        if d not in ("left", "right"):
            raise VoiceError("Turn left or right?")
        a.update(direction=d, degrees=_clamp(action.get("degrees", 90.0), *TURN_LIMITS_DEG, "the angle in degrees"))
    elif name == "height":
        h = _clamp(action.get("height_m"), *HEIGHT_LIMITS_M, "the height in metres")
        a = {"action": "takeoff", "height_m": h} if not ctx["flying"] else {"action": "height", "height_m": h}
    elif name == "takeoff":
        a["height_m"] = _clamp(action.get("height_m", 2.0), *HEIGHT_LIMITS_M, "the height in metres")
    elif name == "course":
        if action.get("course") not in SCENARIOS:
            raise VoiceError(f"I know these courses: {', '.join(v['title'] for v in SCENARIOS.values())}.")
        a["course"] = action["course"]
    elif name == "mission":
        a["planner"] = action.get("planner") if action.get("planner") in PLANNERS else "astar"
    elif name == "race":
        if action.get("track") not in TRACKS:
            raise VoiceError(f"I know these race tracks: {', '.join(v['title'] for v in TRACKS.values())}.")
        a["track"] = action["track"]
    elif name == "race_autopilot":
        a["method"] = action.get("method") if action.get("method") in AGILE_NAMES else "geometric"
    elif name == "formation":
        a["count"] = int(_clamp(action.get("count", 2), 0, 4, "the number of wingmen"))
        a["shape"] = action.get("shape") if action.get("shape") in SHAPE_NAMES else "v"
    elif name == "explore":
        if action.get("world") not in EXPLORE_WORLDS:
            raise VoiceError(f"I can explore: {', '.join(w.title for w in EXPLORE_WORLDS.values())}.")
        a.update(world=action["world"], local=action.get("local") if action.get("local") in LOCAL_PLANNERS else "dwa",
                 localisation=action.get("localisation") if action.get("localisation") in LOCALISATION else "gps",
                 movers=bool(action.get("movers", False)))
    elif name == "farm_plan":
        a["request"] = str(action.get("request") or "").strip()[:300]
        if not a["request"]:
            raise VoiceError("What should it do on the farm?")
    elif name == "wind":
        a["speed_m_s"] = _clamp(action.get("speed_m_s", 3.0), 0.0, WIND_LIMIT_M_S, "the wind speed in m/s")
        a["from"] = action.get("from") if action.get("from") in COMPASS else "west"
    elif name in ("lidar", "sensors"):
        a["on"] = bool(action.get("on", True))
    return a


def needs_go(action: dict, ctx: dict) -> bool:
    if action["action"] in ("free_flight", "reset", "race", "course", "farm"):
        return ctx["flying"]  # on the ground, loading a world is harmless; in the air it moves the drone
    return ACTIONS[action["action"]][1]


def describe(a: dict) -> str:
    n = a["action"]
    if n == "move":
        return f"go {a['direction']} {a['metres']:g} m"
    if n == "turn":
        return f"turn {a['direction']} {a['degrees']:g}° on the spot"
    if n == "takeoff":
        return f"take off to {a['height_m']:g} m"
    if n == "height":
        return f"fly at {a['height_m']:g} m"
    if n == "course":
        return f"load the course '{SCENARIOS[a['course']]['title']}'"
    if n == "mission":
        return f"fly the mission by itself ({PLANNER_NAMES[a['planner']]})"
    if n == "race":
        return f"load the race '{TRACKS[a['track']]['title']}'"
    if n == "race_autopilot":
        return f"fly the race on autopilot ({AGILE_NAMES[a['method']]})"
    if n == "formation":
        return f"{a['count']} wingmen in a {SHAPE_NAMES[a['shape']]}" if a["count"] else "send the wingmen home"
    if n == "explore":
        return (f"explore {EXPLORE_WORLDS[a['world']].title}, {LOCALISATION[a['localisation']]}"
                + (", with someone walking around" if a["movers"] else ""))
    if n == "farm_plan":
        return f"plan on the farm: “{a['request']}”"
    if n == "wind":
        return "calm air" if a["speed_m_s"] == 0 else f"wind {a['speed_m_s']:g} m/s from the {a['from']}"
    if n in ("lidar", "sensors"):
        return f"{n} {'on' if a['on'] else 'off'}"
    return ACTIONS[n][0]


def to_messages(a: dict, session) -> list[dict]:
    """One checked action -> the WebSocket commands the buttons would send."""
    n = a["action"]
    if n in ("hover", "land"):
        return [{"cmd": n}]
    if n == "takeoff":
        return [{"cmd": "takeoff", "alt_m": a["height_m"]}]
    if n in ("move", "turn", "go_home", "height"):
        sp = session.pilot.sp
        est = session.sim.estimate.pos_m
        x, y, alt, yaw = float(est[0]), float(est[1]), float(sp.alt_m), float(sp.yaw_rad)
        if n == "move":
            d, m = a["direction"], a["metres"]
            if d == "up":
                alt += m
            elif d == "down":
                alt -= m
            else:
                ang = yaw + {"forward": 0.0, "left": math.pi / 2, "back": math.pi, "right": -math.pi / 2}[d]
                x, y = x + m * math.cos(ang), y + m * math.sin(ang)
        elif n == "height":
            alt = a["height_m"]
        elif n == "turn":
            yaw += math.radians(a["degrees"]) * (1 if a["direction"] == "left" else -1)
        else:
            start = session.start_state()
            x, y = (float(start.pos_m[0]), float(start.pos_m[1])) if start is not None else (0.0, 0.0)
        return [{"cmd": "target", "x_m": round(x, 3), "y_m": round(y, 3), "alt_m": round(alt, 3), "yaw_rad": round(yaw, 4)}]
    if n == "free_flight":
        return [{"cmd": "race", "track": None}, {"cmd": "course", "scenario": None}]
    if n == "course":
        return [{"cmd": "course", "scenario": a["course"]}]
    if n == "farm":
        return [{"cmd": "course", "scenario": "farm"}]
    if n == "mission":
        return [{"cmd": "mission", "planner": a["planner"]}]
    if n == "race":
        return [{"cmd": "race", "track": a["track"]}]
    if n == "race_autopilot":
        return [{"cmd": "agile", "method": a["method"], "tilt_deg": 30.0}]
    if n == "formation":
        return [{"cmd": "formation", "count": a["count"], "shape": a["shape"]}]
    if n == "explore":
        return [{"cmd": "explore", **{k: a[k] for k in ("world", "local", "localisation", "movers")}}]
    if n == "precision_land":
        return [{"cmd": "precland", "mode": "vision"}]
    if n == "wind":
        return [{"cmd": "wind", "speed_m_s": a["speed_m_s"], "direction_deg": (COMPASS[a["from"]] + 180.0) % 360.0,
                 "gust_m_s": round(0.25 * a["speed_m_s"], 2)}]
    if n == "lidar":
        return [{"cmd": "lidar", "enabled": a["on"]}]
    if n == "sensors":
        return [{"cmd": "sensors", "enabled": a["on"], "noise_scale": 1.0}]
    if n == "reset":
        return [{"cmd": "reset"}]
    if n == "farm_plan":
        return [{"cmd": "plan", "spec": a["spec"]}]  # the spec is added after the farm planner ran
    raise VoiceError(f"I can't do '{n}' yet.")


def short_word(a: dict) -> str:
    """A one-or-two-word spoken answer for the instant commands (a long sentence after 'left' feels slow)."""
    n = a["action"]
    if n == "move":
        return {"forward": "Forward!", "back": "Back!", "left": "Left!", "right": "Right!", "up": "Up!", "down": "Down!"}[a["direction"]]
    if n == "height":
        return f"{a['height_m']:g} metres!"
    if n == "turn":
        return "Turning around!" if a["degrees"] >= 170 else f"Turning {a['direction']}!"
    return {"takeoff": "Taking off!", "land": "Landing!", "hover": "Stopping!", "go_home": "Going home!",
            "reset": "Reset!", "lidar": "OK!", "wind": "OK!"}.get(n, "OK!")


def join_words(parts: list[str]) -> str:
    return parts[0] if len(parts) == 1 else ", ".join(parts[:-1]) + " and " + parts[-1]


def is_quick(text: str) -> bool:
    """True if this needs no AI (stop, go, cancel, or a fixed phrase): it can run at once, even while the AI thinks."""
    t = normalise(text or "")
    return bool(not t or STOP_RE.search(t) or CONFIRM_RE.match(t) or CANCEL_RE.match(t) or quick_parse(text) is not None)


# ---- one heard sentence, start to finish ---------------------------------------------------------------------------
def _after(a: dict, ctx: dict) -> dict:
    """What the world will look like after this action (so 'load the race, then autopilot' checks out)."""
    c = dict(ctx)
    n = a["action"]
    if n == "takeoff":
        c["flying"] = True
    elif n == "land":
        c["flying"] = False
    elif n == "race":
        c.update(race=a["track"], scene="race track", course=None)
    elif n == "course":
        c.update(scene="obstacle course", course=a["course"], race=None)
    elif n == "farm":
        c.update(scene="farm", course="farm", race=None)
    elif n in ("free_flight", "reset"):
        c.update(scene="empty playground" if n == "free_flight" else c["scene"], course=None if n == "free_flight" else c["course"],
                 race=None if n == "free_flight" else c["race"], flying=False)
    elif n == "explore":
        c.update(scene="exploration world", course=f"explore_{a['world']}", race=None, flying=True)
    elif n in ("mission", "race_autopilot", "farm_plan"):
        c["flying"] = True
    return c


def apply_hints(action: dict, heard: str) -> dict:
    """Plain word rules on top of the model's choice, for things kids say that a small model gets wrong.
    They only fill in details of an action the model already chose, never add one."""
    t = normalise(heard)
    a = dict(action)
    if a.get("action") == "explore":
        if re.search(r"\b(forest|trees?|woods)\b", t):
            a["world"] = "forest"
        elif re.search(r"\b(building|house|rooms?|indoors|inside)\b", t):
            a["world"] = "rooms"
        if re.search(r"\b(dead reckoning|motion sensors|no help|on its own sensors)\b", t):
            a["localisation"] = "dead_reckoning"
        elif re.search(r"\b(without|no) gps\b|\bgps off\b|\bindoors\b|\bslam\b", t):
            a["localisation"] = "slam"
        if re.search(r"\b(someone|somebody|person|people|man|kid|walking|walker|moving obstacle)\b", t):
            a["movers"] = True
        if re.search(r"\b(chase|pursuit|carrot)\b", t):
            a["local"] = "pursuit"
    if a.get("action") == "turn" and re.search(r"\b(go|move|fly|slide|step)\b", t) and not re.search(r"\b(turn|rotate|spin|face|yaw)\b", t):
        a = {"action": "move", "direction": a.get("direction", "left"), "metres": 2.0}  # "go left" slides, it doesn't turn
    if a.get("action") == "farm_plan":
        a["request"] = heard  # the farm planner reads the user's own words, not the model's summary
    return a


def _run(actions: list[dict], session, run_msg) -> tuple[list[dict], str | None]:
    """Send each action's commands, in order. Stops at the first refusal and says why."""
    replies: list[dict] = []
    for a in actions:
        for msg in to_messages(a, session):
            r = run_msg(msg)
            if r:
                replies.append(r)
                if r.get("type") == "error":
                    return replies, r.get("message")
    return replies, None


async def handle(session, text: str, run_msg, get_client, ai_busy) -> list[dict]:
    """What to send back for one heard (or typed) sentence: the simulator's own replies, then a 'voice' reply that
    says what happened (the browser speaks it). `run_msg(msg)` applies a WebSocket command; `get_client()` gives the
    Ollama client; `ai_busy()` is True while another AI request is running."""
    heard = (text or "").strip()[:300]

    def reply(say: str, source: str, actions: list[dict] | None = None, pending: bool = False, extra=None,
              short: str | None = None) -> list[dict]:
        return [*(extra or []), {"type": "voice", "heard": heard, "say": say, "source": source, "pending": pending,
                                 "actions": [describe(a) for a in actions or []], "short": short}]

    if not heard:
        return reply("I didn't hear anything. Try again a bit closer to the microphone.", "quick")
    t = normalise(heard)
    pending: Pending | None = getattr(session, "voice_pending", None)
    if pending and time.monotonic() - pending.at > PENDING_EXPIRES_S:
        pending = session.voice_pending = None

    if STOP_RE.search(t):  # stop always wins, whatever else was said
        session.voice_pending = None
        out = run_msg({"cmd": "hover"}) if context(session)["flying"] else None
        return reply("Stopping. Hovering here." if out is not None or context(session)["flying"] else "Stopped.", "quick",
                     [{"action": "hover"}], extra=[out] if out else None, short="Stop!")
    if CONFIRM_RE.match(t):
        if not pending:
            return reply("Nothing is waiting for a go. Tell me what to do first.", "quick")
        session.voice_pending = None
        replies, err = _run(pending.actions, session, run_msg)
        if err:
            return reply(f"I couldn't: {err}", "quick", pending.actions, extra=replies)
        return reply(f"Going: {join_words([describe(a) for a in pending.actions])}.", "quick", pending.actions, extra=replies)
    if CANCEL_RE.match(t):
        session.voice_pending = None
        return reply("Cancelled." if pending else "OK.", "quick")

    ctx = context(session)
    actions = quick_parse(heard)
    source, understood, not_possible = "quick", "", []
    if actions is None:
        if ai_busy():
            return reply("I'm still thinking about the last one. Say it again in a moment.", "ai")
        from backend.ai.ollama_client import AIError, AIOffline

        try:
            res = await ai_parse(get_client(), heard, ctx)
        except AIOffline:
            return reply("The AI helper is offline, so I only understand simple commands: take off, land, stop, "
                         "go left 2 metres, turn right, come home.", "ai")
        except AIError as exc:
            return reply(f"The AI helper had a problem: {exc}", "ai")
        source, understood, not_possible = "ai", res["understood"], res["not_possible"]
        actions = [apply_hints(a, heard) for a in res["actions"]]
        if not re.search(r"\b(then|and|again|twice|two times|times)\b", normalise(heard)):
            # The same action twice in a row, but they didn't ask for it twice: they just repeated themselves.
            actions = [x for i, x in enumerate(actions) if i == 0 or x != actions[i - 1]]
        if any(h != r for h, r in zip(actions, res["actions"]) if h.get("action") != "farm_plan"):
            understood = ""  # a word rule corrected the model: say what will REALLY happen, not its sentence
    if not actions:
        msg = f"Sorry, I can't do that ({join_words(not_possible)})." if not_possible else "Sorry, I didn't understand what to do."
        return reply(msg, source)

    checked, c = [], ctx
    for raw in actions:
        try:
            a = check(raw, c)
        except VoiceError as exc:
            return reply(str(exc), source)
        if a["action"] == "farm_plan":
            from backend.ai.mission_planner import plan_from_text
            from backend.autonomy.site import FARM

            res = await plan_from_text(get_client(), a["request"], FARM, getattr(session, "battery_cfg", None))
            if not res.get("ok"):
                return reply(f"The farm planner couldn't plan that: {res.get('error')}", source)
            fails = [f"{ch['label']}: {ch['detail']}" for ch in res["plan"]["checks"] if ch["status"] == "fail"]
            if fails:
                return reply(f"That plan fails a safety check: {fails[0]}", source)
            a["spec"] = res["spec"]
            understood = res["ai"].get("understood") or understood
        checked.append(a)
        c = _after(a, c)

    plain = join_words([describe(a) for a in checked])
    words = understood or (plain[:1].upper() + plain[1:] + ".")  # (not .capitalize(): it lowercases "GPS")
    if not_possible:
        words += f" (Not possible: {join_words(not_possible)}.)"
    if any(needs_go(a, ctx) for a in checked):
        session.voice_pending = Pending(checked, words)
        return reply(f"{words} Say “go” to start, or “cancel”.", source, checked, pending=True)
    replies, err = _run(checked, session, run_msg)
    if err:
        return reply(f"I couldn't: {err}", source, checked, extra=replies)
    short = " ".join(short_word(a) for a in checked) if source == "quick" else None
    return reply(words, source, checked, extra=replies, short=short)
