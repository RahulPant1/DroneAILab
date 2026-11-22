"""Task plans: "survey the north field, hover over the pond, land on pad B", turned into a checked route.

A plan is a short list of TASKS that refer to named things on the site map (site.py):
    {"type": "survey", "field": "north field"}
    {"type": "goto",   "place": "pond"}
    {"type": "hover",  "place": "gate", "seconds": 10}
    {"type": "land",   "place": "pad B", "precision": true}     (last; without it: fly home and land)
plus altitude_m, speed_m_s and side_overlap_pct for the whole flight.

compile_plan() turns that into waypoints and a route, and runs the SAFETY CHECKS a real ground station or
flight-approval system would: altitude ceiling, speed, geofence, no-fly zones, buildings (routed around with A*
when taller than the flight), battery (estimated energy vs what the pack holds, with a reserve), and where it
lands. A plan with any FAIL is not flyable. This code is deterministic: the AI planner only writes the task
list; every number and every check comes from here.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from backend.sim.battery import LOW_SOC_ESTIMATE, BatteryConfig
from backend.sim.config import DroneConfig, get_preset
from backend.sim.control import Setpoint
from backend.sim.environment import EnvConfig
from backend.sim.physics import State
from backend.sim.runner import PHYSICS_DT_S, LightweightSim, RunResult, run_headless
from backend.sim.sensors import SensorConfig

from .follower import Mission, MissionFollower
from .planner import PlanningError, path_length, plan_route
from .precland import PrecisionLander
from .site import FARM, Site
from .survey import CAMERA, SurveyCamera, SurveyError, coverage, plan_lawnmower
from .world import DRONE_RADIUS_M, SAFETY_MARGIN_M, Box

TASK_TYPES = ("survey", "goto", "hover", "land")
MAX_TASKS = 8
MIN_ALT_M = 5.0
MIN_SPEED_M_S, MAX_SPEED_M_S = 1.0, 3.0  # 3 m/s = the simulator's position-controller speed limit
MAX_HOVER_S = 60.0
ROUTE_GRID_M = 0.5
RESERVE_FAIL_PCT = 30.0  # land with less than this left -> the low-battery failsafe would cut the mission short
RESERVE_WARN_PCT = 40.0
CRUISE_POWER_FACTOR = 1.08  # flying at a few m/s takes a little more power than hovering (measured: ~5-8 %)
CLIMB_RATE_M_S = 2.5  # measured average take-off climb of the default drone
TELEMETRY_EVERY = 50  # 10 Hz telemetry for these long flights (keeps a 3-minute run's record small)


class PlanError(ValueError):
    """A plan that doesn't make sense (unknown task, unknown place). The message is plain English."""


@dataclass
class CompiledPlan:
    spec: dict
    waypoints: list[tuple[float, float]]
    labels: list[str]  # what each waypoint is ("north field line 1 start", "pond", ...)
    route: list[tuple[float, float]]
    obstacles: list[Box]  # what the route keeps clear of: no-fly zones + buildings taller than the flight
    buildings: list[Box]  # what the drone can physically hit
    camera_lines: dict[int, float]  # waypoint index of each survey line's END -> line direction (rad)
    photo_spacing_m: float | None
    dwell: dict[int, float]
    land: dict  # {"place", "x_m", "y_m", "precision", "pad_size_m"}
    surveys: list[dict]  # per surveyed field: its plan
    checks: list[dict]
    estimate: dict

    @property
    def ok(self) -> bool:
        return all(c["status"] != "fail" for c in self.checks)

    def to_dict(self) -> dict:
        return {
            "ok": self.ok,
            "spec": self.spec,
            "waypoints": [[round(x, 2), round(y, 2)] for x, y in self.waypoints],
            "labels": self.labels,
            "route": [[round(x, 2), round(y, 2)] for x, y in self.route],
            "land": self.land,
            "surveys": self.surveys,
            "checks": self.checks,
            "estimate": self.estimate,
        }


def _check(checks: list, cid: str, label: str, status: str, detail: str) -> None:
    checks.append({"id": cid, "label": label, "status": status, "detail": detail})


def _num(spec: dict, key: str, default: float) -> float:
    v = spec.get(key, default)
    if v is None:
        return default
    try:
        v = float(v)
    except (TypeError, ValueError):
        raise PlanError(f"'{key}' must be a number.") from None
    if not math.isfinite(v):
        raise PlanError(f"'{key}' must be a finite number.")
    return v


def normalise_spec(spec: dict, site: Site = FARM) -> dict:
    """Check the plan's STRUCTURE (task types, names). Values are checked later as safety checks."""
    if not isinstance(spec, dict):
        raise PlanError("A plan is a JSON object with a list of tasks.")
    tasks = spec.get("tasks")
    if not isinstance(tasks, list) or not 1 <= len(tasks) <= MAX_TASKS:
        raise PlanError(f"A plan needs 1 to {MAX_TASKS} tasks.")
    # "goto <field>" right before "survey <same field>" adds a pointless detour to the field's middle: drop it.
    tasks = [t for i, t in enumerate(tasks) if not (
        isinstance(t, dict) and t.get("type") == "goto" and i + 1 < len(tasks) and isinstance(tasks[i + 1], dict)
        and tasks[i + 1].get("type") == "survey" and tasks[i + 1].get("field") == t.get("place"))]
    out = []
    destinations = site.destination_names()
    for i, t in enumerate(tasks):
        if not isinstance(t, dict) or t.get("type") not in TASK_TYPES:
            raise PlanError(f"Task {i + 1}: type must be one of {', '.join(TASK_TYPES)}.")
        kind = t["type"]
        if kind == "survey":
            name = t.get("field")
            if not site.field(name):
                raise PlanError(f"Task {i + 1}: there's no field called '{name}'. Fields: {', '.join(f.name for f in site.fields)}.")
            out.append({"type": "survey", "field": name})
        else:
            name = t.get("place")
            if name not in destinations:
                raise PlanError(f"Task {i + 1}: there's no place called '{name}'. Places: {', '.join(destinations)}.")
            task = {"type": kind, "place": name}
            if kind == "hover":
                task["seconds"] = _num(t, "seconds", 10.0)
            if kind == "land":
                if i != len(tasks) - 1:
                    raise PlanError("Landing must be the last task.")
                task["precision"] = bool(t.get("precision", True))
            out.append(task)
    return {
        "tasks": out,
        "altitude_m": _num(spec, "altitude_m", 20.0),
        "speed_m_s": _num(spec, "speed_m_s", 3.0),
        "side_overlap_pct": _num(spec, "side_overlap_pct", 70.0),
        "front_overlap_pct": _num(spec, "front_overlap_pct", 75.0),
    }


_HOVER_POWER_CACHE: dict[tuple, float] = {}


def hover_power_w(config: DroneConfig, battery: BatteryConfig) -> float:
    """Electrical power to hover, measured with a 3-second simulated hover (cached)."""
    key = (repr(config.to_dict()), battery.cells, battery.capacity_mah, battery.mass_kg)
    if key not in _HOVER_POWER_CACHE:
        sim = LightweightSim()
        sim.reset(config, 0, State.hovering(config, 3.0), battery=BatteryConfig(**{**battery.__dict__, "drain_speedup": 1.0}))
        sp = Setpoint(alt_m=3.0, position_control=True)
        for _ in range(int(3.0 / PHYSICS_DT_S)):
            sim.step(sp)
        _HOVER_POWER_CACHE[key] = float(sim.battery.power_w)
    return _HOVER_POWER_CACHE[key]


def _segment_hits_box(a, b, box: Box, margin: float) -> bool:
    n = max(2, int(math.dist(a, b) / 0.25) + 1)
    return any(box.distance_2d(a[0] + (b[0] - a[0]) * k / (n - 1), a[1] + (b[1] - a[1]) * k / (n - 1)) < margin for k in range(n))


def compile_plan(spec: dict, site: Site = FARM, config: DroneConfig | None = None,
                 battery: BatteryConfig | None = None) -> CompiledPlan:
    """Turn a task list into waypoints + a safe route, and run the safety checks."""
    s = normalise_spec(spec, site)
    cfg = config or get_preset("basic")
    bat = battery or BatteryConfig()
    alt, speed = s["altitude_m"], s["speed_m_s"]
    checks: list[dict] = []

    # ---- values ----
    if MIN_ALT_M <= alt <= site.ceiling_m:
        _check(checks, "altitude", "Altitude", "pass", f"{alt:.0f} m (allowed {MIN_ALT_M:.0f}-{site.ceiling_m:.0f} m on this farm).")
    else:
        why = "above this farm's ceiling" if alt > site.ceiling_m else "too low to clear trees and people safely"
        _check(checks, "altitude", "Altitude", "fail", f"{alt:.0f} m is {why} (allowed {MIN_ALT_M:.0f}-{site.ceiling_m:.0f} m; "
                                                         "the legal limit for hobby drones is usually 120 m).")
    if MIN_SPEED_M_S <= speed <= MAX_SPEED_M_S:
        _check(checks, "speed", "Speed", "pass", f"{speed:.1f} m/s.")
    else:
        _check(checks, "speed", "Speed", "fail", f"{speed:.1f} m/s is outside {MIN_SPEED_M_S:.0f}-{MAX_SPEED_M_S:.0f} m/s "
                                                   "(this drone's position controller can't go faster than 3 m/s).")
    side, front = s["side_overlap_pct"], s["front_overlap_pct"]
    survey_alt = min(max(alt, MIN_ALT_M), site.ceiling_m)

    # ---- waypoints ----
    home = site.pad("home")
    wps: list[tuple[float, float]] = [(home.x_m, home.y_m)]
    labels = ["take-off (home)"]
    camera_lines: dict[int, float] = {}
    dwell: dict[int, float] = {}
    surveys: list[dict] = []
    photo_spacing = None
    for t in s["tasks"]:
        if t["type"] == "survey":
            f = site.field(t["field"])
            try:
                sp = plan_lawnmower(f.polygon, survey_alt, side / 100.0, front / 100.0, start=wps[-1])
            except SurveyError as exc:
                _check(checks, "survey", f"Survey of {f.name}", "fail", str(exc))
                continue
            photo_spacing = sp.photo_spacing_m
            for k, (a, b) in enumerate(sp.lines):
                wps += [a, b]
                labels += [f"{f.name} line {k + 1} start", f"{f.name} line {k + 1} end"]
                camera_lines[len(wps) - 1] = sp.angle_rad
            surveys.append({"field": f.name, "polygon": [list(p) for p in f.polygon], **sp.to_dict()})
        elif t["type"] in ("goto", "hover"):
            p = site.point(t["place"])
            if math.dist(p, wps[-1]) > 0.5:
                wps.append(p)
                labels.append(t["place"])
            if t["type"] == "hover":
                sec = t["seconds"]
                if 1 <= sec <= MAX_HOVER_S:
                    dwell[len(wps) - 1] = sec
                else:
                    _check(checks, "hover", f"Hover at {t['place']}", "fail", f"{sec:.0f} s: hover between 1 and {MAX_HOVER_S:.0f} s.")
    last = s["tasks"][-1]
    if last["type"] == "land":
        pad = site.pad(last["place"])
        if pad is None:
            _check(checks, "landing", "Landing spot", "fail",
                   f"'{last['place']}' isn't a landing pad. Land on: {', '.join(p.name for p in site.pads)}.")
            pad, precision = home, False
        else:
            precision = last["precision"]
    else:
        pad, precision = home, True
    if math.dist((pad.x_m, pad.y_m), wps[-1]) > 0.5 or len(wps) == 1:
        wps.append((pad.x_m, pad.y_m))
        labels.append(f"land on {pad.name}")
    else:
        labels[-1] += f" (land on {pad.name})"
    land = {"place": pad.name, "x_m": pad.x_m, "y_m": pad.y_m, "precision": precision, "pad_size_m": pad.size_m}
    if not any(c["id"] == "landing" for c in checks):
        how = ("precision landing on its marker (down camera)" if precision else "landing on its GPS position")
        _check(checks, "landing", "Landing spot", "pass", f"{pad.name}: {how}.")

    # ---- geofence + no-fly (on the waypoints) ----
    far = [(labels[i], math.hypot(x - home.x_m, y - home.y_m)) for i, (x, y) in enumerate(wps)]
    outside = [(n, d) for n, d in far if d > site.geofence_radius_m]
    if outside:
        _check(checks, "geofence", "Geofence", "fail", "Outside the " + f"{site.geofence_radius_m:.0f} m geofence: "
               + ", ".join(f"{n} ({d:.0f} m)" for n, d in outside[:3]) + ".")
    inside_nfz = [(labels[i], z.name) for i, (x, y) in enumerate(wps) for z in site.no_fly if z.box.distance_2d(x, y) == 0.0]
    if inside_nfz:
        _check(checks, "no_fly", "No-fly zones", "fail", "; ".join(f"{n} is inside the no-fly zone '{z}'" for n, z in inside_nfz[:3]) + ".")

    # ---- route around no-fly zones and tall buildings (A*) ----
    obstacles = site.obstacles(survey_alt)
    route = list(wps)
    if not outside and not inside_nfz:
        try:
            route, _ = plan_route(wps, obstacles, "astar", survey_alt, ROUTE_GRID_M)
        except PlanningError as exc:
            _check(checks, "route", "Route", "fail", str(exc))
        else:
            keep = DRONE_RADIUS_M + SAFETY_MARGIN_M
            avoided_nfz = [z.name for z in site.no_fly if any(_segment_hits_box(a, b, z.box, keep) for a, b in zip(wps, wps[1:]))]
            avoided_b = [f"{b.name} ({b.box.height_m:.0f} m tall)" for b in site.buildings if b.box.blocks_altitude(survey_alt)
                         and any(_segment_hits_box(a, b2, b.box, keep) for a, b2 in zip(wps, wps[1:]))]
            over = [b.name for b in site.buildings if not b.box.blocks_altitude(survey_alt)]
            _check(checks, "no_fly", "No-fly zones", "pass",
                   (f"Route goes around: {', '.join(avoided_nfz)}." if avoided_nfz else "The route never enters one.")
                   + f" ({len(site.no_fly)} zones on this farm.)")
            txt = f"Routed around: {', '.join(avoided_b)}." if avoided_b else "Nothing tall in the way."
            if over:
                txt += f" At {survey_alt:.0f} m it flies over the {', '.join(over)}."
            _check(checks, "obstacles", "Buildings", "pass", txt)
            worst = max(math.hypot(x - home.x_m, y - home.y_m) for x, y in route)
            if worst > site.geofence_radius_m:
                _check(checks, "geofence", "Geofence", "fail", f"The detour goes {worst:.0f} m from home (limit {site.geofence_radius_m:.0f} m).")
            else:
                _check(checks, "geofence", "Geofence", "pass", f"Furthest point {worst:.0f} m from home (limit {site.geofence_radius_m:.0f} m).")

    # ---- time + battery ----
    dist = path_length(route)
    climb_s = survey_alt / CLIMB_RATE_M_S
    cruise_s = dist / max(speed, 0.1) * 1.12 + 2.5 * max(0, len(route) - 2)  # lag + turning at corners
    if precision:
        land_s = max(0.0, survey_alt - 4.0) / 0.8 + 2.0 / 0.5 + 2.0 / 0.5
    else:
        land_s = max(0.0, survey_alt - 5.0) / 1.5 + 5.0 / 0.6
    hover_s = sum(dwell.values())
    total_s = climb_s + cruise_s + land_s + hover_s + 2.0
    p_w = hover_power_w(cfg, bat)
    energy_wh = p_w * CRUISE_POWER_FACTOR * total_s / 3600.0
    used_pct = 100.0 * energy_wh / bat.energy_wh
    left_pct = 100.0 - used_pct
    budget = f"{energy_wh:.1f} Wh of the {bat.energy_wh:.0f} Wh pack ({bat.cells}S {bat.capacity_mah:.0f} mAh): lands with ~{left_pct:.0f} % left"
    if left_pct < RESERVE_FAIL_PCT:
        _check(checks, "battery", "Battery", "fail", f"{budget}. Below {RESERVE_FAIL_PCT:.0f} % the low-battery failsafe "
               f"(~{LOW_SOC_ESTIMATE * 100:.0f} %) could cut the mission short.")
    elif left_pct < RESERVE_WARN_PCT:
        _check(checks, "battery", "Battery", "warn", f"{budget}: a thin reserve, no room for wind or a second try.")
    else:
        _check(checks, "battery", "Battery", "pass", f"{budget}.")
    order = {"altitude": 0, "speed": 1, "geofence": 2, "no_fly": 3, "obstacles": 4, "route": 5, "survey": 6, "hover": 7,
             "battery": 8, "landing": 9}
    checks.sort(key=lambda c: order.get(c["id"], 99))
    estimate = {
        "distance_m": round(dist, 1),
        "time_s": round(total_s, 1),
        "hover_power_w": round(p_w, 1),
        "energy_wh": round(energy_wh, 2),
        "battery_used_pct": round(used_pct, 1),
        "battery_left_pct": round(left_pct, 1),
        "photos": sum(sv["photos_estimate"] for sv in surveys),
    }
    return CompiledPlan(s, wps, labels, route, obstacles, [b.box for b in site.buildings], camera_lines, photo_spacing, dwell, land, surveys, checks, estimate)


class PlanFollower(MissionFollower):
    """MissionFollower + three extras: hover (dwell) at chosen waypoints, the survey camera on survey lines,
    and an optional precision lander for the final landing."""

    def __init__(self, plan: CompiledPlan, lander: PrecisionLander | None = None):
        # Collisions are checked against real buildings only; no-fly zones are kept clear by the planned route.
        mission = Mission(list(plan.waypoints), list(plan.buildings), plan.spec["altitude_m"], plan.spec["speed_m_s"])
        super().__init__(mission, list(plan.route), "astar")
        self.plan = plan
        self.camera = SurveyCamera(plan.photo_spacing_m, plan.camera_lines) if plan.camera_lines else None
        self.lander = lander
        self._hold_until: float | None = None
        self._hold_s = 0.0
        self._lander_events = 0

    def setpoint_for(self, state: State, t: float, truth: State | None = None) -> Setpoint:
        truth = truth or state
        before = self.next_wp
        sp = super().setpoint_for(state, t, truth)
        if self.phase == "cruise":
            if self.next_wp != before and before in self.plan.dwell:
                self._hold_until = t + self.plan.dwell[before]
                self._hold_s = self.wp_s[before - 1]
                self._event(t, "hover", f"Hovering {self.plan.dwell[before]:.0f} s at {self.plan.labels[before]}.",
                            float(state.pos_m[0]), float(state.pos_m[1]))
            if self._hold_until is not None:
                if t < self._hold_until:
                    self.s = self._hold_s
                    sp.x_m, sp.y_m = self._point_at(self.s)
                else:
                    self._hold_until = None
        if self.camera:
            line = self.next_wp if self.phase == "cruise" and self.next_wp in self.plan.camera_lines else None
            self.camera.update(t, line, state.pos_m[:2], truth.pos_m)
        if self.phase == "land" and self.lander:
            sp.x_m, sp.y_m, sp.alt_m = self.lander.update(t, state, truth)
            for tt, kind, msg in self.lander.events[self._lander_events:]:
                self._event(tt, kind, msg, float(state.pos_m[0]), float(state.pos_m[1]))
            self._lander_events = len(self.lander.events)
        return sp


def make_lander(plan: CompiledPlan, site: Site, seed: int, mode: str | None = None) -> PrecisionLander | None:
    """The precision lander for this plan's landing pad (None = plain GPS landing)."""
    land = plan.land
    if not land["precision"] and mode is None:
        return None
    pads = [(p.x_m, p.y_m, p.size_m) for p in site.pads]
    return PrecisionLander((land["x_m"], land["y_m"], land["pad_size_m"]), mode or "vision", seed, pads)


def run_plan(plan: CompiledPlan, site: Site = FARM, config: DroneConfig | None = None, env: EnvConfig | None = None,
             sensors: SensorConfig | None = None, seed: int = 0, battery: BatteryConfig | None = None,
             lander_mode: str | None = None) -> RunResult:
    """Fly a compiled plan headless and measure it: coverage per field, where it really landed, battery used."""
    cfg = config or get_preset("basic")
    lander = make_lander(plan, site, seed, lander_mode)
    follower = PlanFollower(plan, lander)
    start = State.at(0.0)
    start.pos_m[:2] = plan.waypoints[0]
    budget_s = min(600.0, plan.estimate["time_s"] * 1.8 + 30.0)
    r = run_headless(cfg, follower, budget_s, seed, initial_state=start, env=env, sensors=sensors,
                     stop_when=lambda: follower.done, scenario={"duration_s": budget_s}, battery=battery,
                     telemetry_every_n_steps=TELEMETRY_EVERY)
    r.events = sorted(r.events + follower.events, key=lambda e: e["t_s"])
    x, y, z = (np.asarray(r.telemetry[k]) for k in ("x_m", "y_m", "z_m"))
    kinds = [e["type"] for e in r.events]
    photos = follower.camera.photos if follower.camera else []
    per_field = []
    for sv in plan.surveys:
        shots = [p for p in photos if p["line"] is not None and plan.labels[p["line"]].startswith(sv["field"] + " ")]
        cov = coverage(sv["polygon"], shots, CAMERA)
        per_field.append({"field": sv["field"], "photos": len(shots), **cov})
    nfz_hits = sum(1 for zz in site.no_fly if np.any([zz.box.distance_2d(a, b) == 0.0 for a, b in zip(x[z > 0.3], y[z > 0.3])]))
    landed = "landed" in kinds
    r.metrics.update({
        "completed": landed and "collision" not in kinds,
        "collisions": kinds.count("collision"),
        "no_fly_entered": nfz_hits,
        "max_distance_from_home_m": float(np.max(np.hypot(x, y))),
        "flight_time_s": float(r.telemetry["t_s"][-1]),
        "estimated_time_s": plan.estimate["time_s"],
        "flown_length_m": float(np.sum(np.hypot(np.diff(x), np.diff(y)))),
        "photos": len(photos),
        "coverage_pct": min((f["coverage_pct"] for f in per_field), default=None),
        "landing_error_m": float(math.hypot(x[-1] - plan.land["x_m"], y[-1] - plan.land["y_m"])) if landed else None,
        "marker_sightings": lander.sightings if lander else None,
        "first_correction_m": lander.first_correction_m if lander else None,
    })
    r.telemetry["ground_speed_m_s"] = [round(float(v), 3) for v in np.hypot(r.telemetry["vx_m_s"], r.telemetry["vy_m_s"])]
    r.scenario = {"duration_s": float(r.telemetry["t_s"][-1]), "plan": plan.to_dict(), "site": site.to_dict(),
                  "photos": photos, "coverage": per_field}
    if lander:
        r.scenario.update({"camera_frames": lander.keyframes, "first_fix_alt_m": lander.first_fix_alt_m})
    return r
