"""Run a waypoint mission in the simulator and measure how it went.

Mission metrics:
  completed        : reached every waypoint and landed, without hitting anything
  mission_time_s   : take-off to final waypoint
  planned / flown length, and path efficiency (planned ÷ flown)
  cross-track error: how far the drone strayed from the planned route
  min clearance    : the closest the drone's edge came to an obstacle
"""

from __future__ import annotations

import math

import numpy as np

from backend.sim.config import DroneConfig, get_preset
from backend.sim.environment import EnvConfig
from backend.sim.physics import State
from backend.sim.runner import RunResult, run_headless

from .follower import Mission, MissionFollower
from .planner import PlanInfo, PlanningError, path_length, plan_route
from .world import DRONE_RADIUS_M, SAFETY_MARGIN_M, Box, clearance_series

PLANNERS = ("straight", "potential", "astar")
MAX_WAYPOINTS = 20
MAX_OBSTACLES = 20
COORD_LIMIT_M = 100.0
MAX_MISSION_S = 120.0


class MissionError(ValueError):
    """A mission that can't be flown. The message is plain English."""


SCENARIOS: dict[str, dict] = {
    "around_block": {
        "title": "Around the block",
        "waypoints": [(0.0, 0.0), (10.0, -2.0)],
        "obstacles": [Box(4.0, 6.0, -1.0, 3.0)],
    },
    "slalom": {
        "title": "Slalom",
        "waypoints": [(0.0, 0.0), (12.0, 0.0)],
        "obstacles": [Box(3.0, 4.0, -4.0, 0.6), Box(7.5, 8.5, -0.6, 4.0)],
    },
    "box_canyon": {
        "title": "Box canyon (U-shaped trap)",
        "waypoints": [(0.0, 0.0), (10.0, 0.0)],
        "obstacles": [Box(6.0, 6.5, -2.5, 2.5), Box(3.5, 6.5, 2.0, 2.5), Box(3.5, 6.5, -2.5, -2.0)],
    },
    "patrol": {
        "title": "Square patrol",
        "waypoints": [(0.0, 0.0), (6.0, 0.0), (6.0, 6.0), (0.0, 6.0), (0.0, 0.5)],
        "obstacles": [Box(2.5, 3.5, 2.5, 3.5)],
    },
}


def scenario_mission(scenario_id: str, cruise_speed_m_s: float = 1.5, cruise_alt_m: float = 2.0) -> Mission:
    sc = SCENARIOS[scenario_id]
    return Mission(list(sc["waypoints"]), list(sc["obstacles"]), cruise_alt_m, cruise_speed_m_s)


def mission_from_dict(d: dict) -> Mission:
    """Build and check a custom mission (e.g. from the API)."""
    if not isinstance(d, dict):
        raise MissionError("A mission must be a JSON object.")
    try:
        wps = [(float(p[0]), float(p[1])) for p in d.get("waypoints", [])]
        obs = [Box(float(o["x_min"]), float(o["x_max"]), float(o["y_min"]), float(o["y_max"]), float(o.get("height_m", 4.0)))
               for o in d.get("obstacles", [])]  # fmt: skip
        m = Mission(wps, obs, float(d.get("cruise_alt_m", 2.0)), float(d.get("cruise_speed_m_s", 1.5)))
    except (TypeError, ValueError, KeyError, IndexError):
        raise MissionError("Waypoints must be [x, y] pairs and obstacles {x_min, x_max, y_min, y_max, height_m}.") from None
    validate_mission(m)
    return m


def validate_mission(m: Mission) -> None:
    if not 2 <= len(m.waypoints) <= MAX_WAYPOINTS:
        raise MissionError(f"A mission needs 2 to {MAX_WAYPOINTS} waypoints (the first is the take-off point).")
    if len(m.obstacles) > MAX_OBSTACLES:
        raise MissionError(f"At most {MAX_OBSTACLES} obstacles.")
    if not 0.5 <= m.cruise_alt_m <= 20.0:
        raise MissionError("Cruise altitude must be between 0.5 and 20 m.")
    if not 0.3 <= m.cruise_speed_m_s <= 3.0:
        raise MissionError("Cruise speed must be between 0.3 and 3 m/s.")
    values = [v for p in m.waypoints for v in p] + [v for b in m.obstacles for v in (b.x_min, b.x_max, b.y_min, b.y_max)]
    if any(not math.isfinite(v) or abs(v) > COORD_LIMIT_M for v in values):
        raise MissionError(f"All coordinates must be within ±{COORD_LIMIT_M:.0f} m.")
    for i, b in enumerate(m.obstacles):
        if b.x_max <= b.x_min or b.y_max <= b.y_min or not 0.1 <= b.height_m <= 50:
            raise MissionError(f"Obstacle {i + 1} has a zero or negative size (or a height outside 0.1–50 m).")
    for i, (x, y) in enumerate(m.waypoints):
        for b in m.obstacles:
            if b.blocks_altitude(m.cruise_alt_m) and b.distance_2d(x, y) < DRONE_RADIUS_M:
                raise MissionError(f"Waypoint {i} ({x:.1f}, {y:.1f}) is inside or touching an obstacle.")


def _cross_track(x: np.ndarray, y: np.ndarray, route: list[tuple[float, float]]) -> np.ndarray:
    """Distance from each (x, y) to the nearest point of the planned route (a polyline)."""
    best = np.full(len(x), np.inf)
    for (ax, ay), (bx, by) in zip(route, route[1:]):
        vx, vy = bx - ax, by - ay
        L2 = vx * vx + vy * vy
        f = np.clip(((x - ax) * vx + (y - ay) * vy) / L2, 0, 1) if L2 > 0 else np.zeros_like(x)
        best = np.minimum(best, np.hypot(x - (ax + f * vx), y - (ay + f * vy)))
    return best


def run_mission(
    mission: Mission,
    planner: str,
    config: DroneConfig | None = None,
    env: EnvConfig | None = None,
    seed: int = 0,
) -> tuple[RunResult, PlanInfo, list[tuple[float, float]]]:
    """Plan, fly and measure a mission. Returns (result, planning info, planned route)."""
    if planner not in PLANNERS:
        raise MissionError(f"Planner must be one of: {', '.join(PLANNERS)}.")
    validate_mission(mission)
    cfg = config or get_preset("basic")
    try:
        route, info = plan_route(mission.waypoints, mission.obstacles, planner, mission.cruise_alt_m)
    except PlanningError as exc:
        raise MissionError(str(exc)) from None

    follower = MissionFollower(mission, route, planner)
    start = State.at(0.0)
    start.pos_m[:2] = mission.waypoints[0]
    r = run_headless(cfg, follower, MAX_MISSION_S, seed, initial_state=start, env=env,
                     stop_when=lambda: follower.done, scenario={"duration_s": MAX_MISSION_S})  # fmt: skip
    r.events = sorted(r.events + follower.events, key=lambda e: e["t_s"])

    t = np.asarray(r.telemetry["t_s"])
    x, y, z = (np.asarray(r.telemetry[k]) for k in ("x_m", "y_m", "z_m"))
    xt = _cross_track(x, y, route)
    clr = clearance_series(mission.obstacles, x, y, z)
    airborne = z > 0.05
    cruising = (t >= (follower.cruise_started_s or math.inf)) & (t <= (follower.finished_s or t[-1]))
    speed = np.hypot(np.asarray(r.telemetry["vx_m_s"]), np.asarray(r.telemetry["vy_m_s"]))
    r.telemetry["cross_track_m"] = [round(float(v), 4) for v in xt]
    r.telemetry["clearance_m"] = [round(float(min(v, 5.0)), 4) for v in clr]  # capped at 5 m for plotting
    r.telemetry["ground_speed_m_s"] = [round(float(v), 4) for v in speed]
    r.telemetry["cruise_speed_m_s"] = [mission.cruise_speed_m_s] * len(t)

    kinds = [e["type"] for e in r.events]
    flown = float(np.sum(np.hypot(np.diff(x), np.diff(y))))
    planned = path_length(route)
    min_clear = float(np.min(clr[airborne])) if airborne.any() else math.inf
    r.metrics.update(
        {
            "planner": planner,
            "completed": "landed" in kinds and "collision" not in kinds,
            "collisions": kinds.count("collision"),
            "got_stuck": "stuck" in kinds,
            "waypoints_reached": kinds.count("waypoint"),
            "waypoints_total": len(mission.waypoints) - 1,
            "mission_time_s": (follower.finished_s - follower.cruise_started_s) if follower.finished_s and follower.cruise_started_s else None,
            "planned_length_m": planned,
            "flown_length_m": flown,
            "straight_line_m": path_length(list(mission.waypoints)),
            "cross_track_rms_m": float(np.sqrt(np.mean(xt[cruising] ** 2))) if cruising.any() else None,
            "cross_track_max_m": float(np.max(xt[cruising])) if cruising.any() else None,
            "min_clearance_m": min_clear if math.isfinite(min_clear) else None,
            # Closest the flown path came to each waypoint (after the start), worst case.
            "max_waypoint_miss_m": float(max(np.min(np.hypot(x - wx, y - wy)) for wx, wy in mission.waypoints[1:])),
            "safety_margin_m": SAFETY_MARGIN_M,
            "cells_expanded": info.cells_expanded,
            "planning_time_ms": round(info.planning_time_ms, 1),
        }
    )
    r.scenario = {
        "duration_s": float(t[-1]),
        "mission": {
            "waypoints": [list(p) for p in mission.waypoints],
            "obstacles": [b.to_dict() for b in mission.obstacles],
            "cruise_alt_m": mission.cruise_alt_m,
            "cruise_speed_m_s": mission.cruise_speed_m_s,
        },
        "planned_route": [[round(a, 3), round(b, 3)] for a, b in route],
    }
    return r, info, route
