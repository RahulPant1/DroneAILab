"""Experiment 7: Waypoint missions and obstacle avoidance. Compare three ways of getting from A to B."""

from __future__ import annotations

from backend.autonomy.mission import SCENARIOS, MissionError, run_mission, scenario_mission
from backend.sim.environment import EnvConfig
from backend.sim.runner import RunResult

from .base import Experiment, ExperimentError, Metric, Param, Plot, Series, fmt, plots_to_dicts, try_next

PLANNER_NAMES = {"straight": "Straight lines", "potential": "Potential field", "astar": "A* path planning",
                 "lidar": "Lidar: sense and replan"}
WIND_DIRECTION_DEG = 90.0  # wind blows toward north: a crosswind for these mostly east-going routes


def run(params: dict, seed: int) -> RunResult:
    planner = params["planner"]
    mission = scenario_mission(params["scenario"], cruise_speed_m_s=params["cruise_speed_m_s"])
    env = EnvConfig(wind_speed_m_s=params["wind_speed_m_s"], wind_direction_deg=WIND_DIRECTION_DEG) if params["wind_speed_m_s"] > 0 else None
    try:
        r, info, route = run_mission(mission, planner, env=env, seed=seed)
    except MissionError as exc:
        raise ExperimentError(str(exc)) from None

    r.summary = summarise(params, r)
    r.try_next = suggestions(params, r)
    r.plots = plots_to_dicts(
        Plot("map", "Top-down map: planned vs flown", "north y (m)",
             "Dark boxes = obstacles (faint outline = the safety zone the planner keeps clear). Dashed yellow = the plan, "
             "solid = where the drone really flew. Numbers = waypoints, ▲ = start.",
             (Series("x_m", "flown path"), Series("y_m", "flown path")), kind="map",
             extra={"obstacles": r.scenario["mission"]["obstacles"], "waypoints": r.scenario["mission"]["waypoints"],
                    "planned_route": r.scenario["planned_route"], "safety_zone_m": 0.3 + r.metrics["safety_margin_m"],
                    **({"discovered": r.scenario["discovered"], "plans": r.scenario["plans"]} if planner == "lidar" else {})}),
        Plot("clearance", "Distance to obstacles and to the plan", "distance (m)",
             "Clearance = gap between the drone's edge and the nearest obstacle (0 = touching; shown up to 5 m). "
             "Cross-track = how far it is from the planned path.",
             (Series("clearance_m", "obstacle clearance"), Series("cross_track_m", "off the planned path", style="dashed"))),
        Plot("speed", "Ground speed", "speed (m/s)",
             "Horizontal speed vs the requested cruise speed (dashed). It slows at corners and when the carrot waits for it.",
             (Series("ground_speed_m_s", "ground speed"), Series("cruise_speed_m_s", "cruise speed", style="dashed"))),
        Plot("alt", "Altitude", "altitude (m)", "Take-off, cruise at constant height, then landing.",
             (Series("z_m", "altitude"), Series("alt_setpoint_m", "target", style="dashed"))),
    )  # fmt: skip
    return r


def _event(r: RunResult, kind: str) -> dict | None:
    return next((e for e in r.events if e["type"] == kind), None)


def summarise(p: dict, r: RunResult) -> list[str]:
    m = r.metrics
    name = PLANNER_NAMES[p["planner"]]
    lines = []
    if m["completed"]:
        lines.append(
            f"{name}: mission complete. {m['waypoints_reached']}/{m['waypoints_total']} waypoints in {fmt(m['mission_time_s'], 1)} s, "
            f"flying {m['flown_length_m']:.1f} m (the straight-line route is {m['straight_line_m']:.1f} m)."
        )
    crash, stuck = _event(r, "collision"), _event(r, "stuck")
    if crash:
        lines.append(
            f"{name}: CRASHED into an obstacle at ({crash['x_m']:.1f}, {crash['y_m']:.1f}) after {crash['t_s']:.1f} s. "
            + ("Straight lines ignore obstacles completely, so the drone flew right into one." if p["planner"] == "straight"
               else "Something pushed it off its safe path.")
        )  # fmt: skip
    if stuck:
        lines.append(
            f"{name}: STUCK at ({stuck['x_m']:.1f}, {stuck['y_m']:.1f}). The goal's pull and the obstacles' push cancelled out, "
            "a 'local minimum'. A potential field only reacts to what's close by, so it can't see the way out. "
            "A planner that searches the whole map (A*) can."
        )
    if p["planner"] == "lidar":
        lines.append(
            f"It knew only the waypoints. Its lidar took {m['lidar_scans']} scans and mapped {m['map_cells']} cells of obstacle "
            f"(red dots); it replanned {m['replans']} times as new obstacles came into view"
            + (f" and braked {m['brakes']} times" if m["brakes"] else "") + ". Faint dashed lines = the plans it dropped."
        )
        lines.append("A* knows the whole map before take-off; the lidar planner has to discover it, so its route is usually "
                     "a little longer. That's the price of not having a map.")
    if p["planner"] == "astar":
        lines.append(
            f"A* searched {m['cells_expanded']} grid cells in {m['planning_time_ms']:.0f} ms before take-off and planned a "
            f"{m['planned_length_m']:.1f} m route that keeps {m['safety_margin_m']:.1f} m of extra space around every obstacle."
        )
    elif p["planner"] == "potential" and m["completed"]:
        lines.append(
            "The potential field steered on the fly: the goal pulls, obstacles push only when close. That's why it "
            "passed nearer to obstacles than a planned route would."
        )
    if m["min_clearance_m"] is not None and not crash:
        lines.append(f"Closest approach to an obstacle: {m['min_clearance_m']:.2f} m between the drone's edge and the obstacle.")
    if m["completed"] and m["cross_track_max_m"] is not None:
        lines.append(
            f"It stayed within {m['cross_track_max_m']:.2f} m of the planned path (RMS {m['cross_track_rms_m']:.2f} m) and "
            f"passed every waypoint within {m['max_waypoint_miss_m']:.2f} m."
            + (" At this speed it cuts corners: it can't turn instantly, so it swings wide or short." if m["cross_track_max_m"] > 0.3 else "")
        )
    if p["wind_speed_m_s"] > 0:
        lines.append(f"A {p['wind_speed_m_s']:.0f} m/s crosswind (blowing north) pushed it sideways, so it had to lean into the wind the whole way.")
    return lines


def suggestions(p: dict, r: RunResult) -> list[dict]:
    out = []
    if p["planner"] != "astar":
        out.append(try_next("Same mission with A* path planning.", planner="astar"))
    if p["planner"] != "potential":
        out.append(try_next("Try the potential field: does it find its own way around?", planner="potential"))
    if p["planner"] != "lidar":
        out.append(try_next("Hidden walls with the lidar planner: it has to discover the second wall.", scenario="hidden_walls", planner="lidar"))
    if p["scenario"] != "box_canyon":
        out.append(try_next("The box canyon: a U-shaped trap. Compare the potential field with A*.", scenario="box_canyon", planner="potential"))
    out.append(try_next("Fly faster (3 m/s) and watch the corners in the square patrol.", scenario="patrol", cruise_speed_m_s=3.0, planner="astar"))
    if p["wind_speed_m_s"] == 0:
        out.append(try_next("Add a 6 m/s crosswind: how close does it get to the slalom walls now?", scenario="slalom", planner="astar", wind_speed_m_s=6.0))
    return out[:4]


EXPERIMENT = Experiment(
    id="missions",
    number=7,
    title="Missions and obstacle avoidance",
    objective="Fly a waypoint mission past obstacles, and compare three ways of finding the way.",
    concept=(
        "A mission is a list of waypoints: take off, visit each in order, land.",
        "Straight lines ignore obstacles; a potential field reacts to nearby ones; A* plans around all of them before take-off; "
        "the lidar planner knows none of them: it senses them with a laser scanner and replans as it goes.",
        "Planned vs flown path shows how well the controller tracks the plan (corners, speed, wind).",
    ),
    params=(
        Param("scenario", "Scenario", "", "around_block", "Which course to fly.", options=tuple((k, v["title"]) for k, v in SCENARIOS.items())),
        Param("planner", "Planner", "", "astar", "How to get past obstacles.", options=tuple(PLANNER_NAMES.items())),
        Param("cruise_speed_m_s", "Cruise speed", "m/s", 1.5, "How fast to fly along the route.", 0.5, 3.0, 0.1),
        Param("wind_speed_m_s", "Crosswind", "m/s", 0.0, "Steady wind blowing toward north.", 0.0, 8.0, 0.5),
    ),
    key_metrics=(
        Metric("completed", "Mission complete"),
        Metric("collisions", "Collisions", "", 0),
        Metric("mission_time_s", "Mission time", "s", 1),
        Metric("flown_length_m", "Distance flown", "m", 1),
        Metric("min_clearance_m", "Closest to an obstacle", "m"),
        Metric("cross_track_max_m", "Max distance from the plan", "m"),
    ),
    learn_slug="09-exp-missions",
    runner=run,
)
