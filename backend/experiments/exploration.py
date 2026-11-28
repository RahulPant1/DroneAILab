"""Experiment 13: autonomous exploration. Map an unknown place with a lidar: frontiers, local planners, SLAM."""

from __future__ import annotations

import numpy as np

from backend.autonomy.explore import LOCAL_PLANNERS, LOCALISATION, WORLDS, explore_metrics, run_explore
from backend.sim.runner import RunResult

from .base import Experiment, Metric, Param, Plot, Series, plots_to_dicts, try_next

MOVERS = {"off": "Nobody else", "on": "Someone walking through"}


def run(params: dict, seed: int) -> RunResult:
    world, local, loc, movers = params["world"], params["local_planner"], params["localisation"], params["movers"] == "on"
    r, ex = run_explore(world, local, loc, movers, seed=seed)
    m = explore_metrics(r, ex)
    r.metrics.update(m)
    w = WORLDS[world]
    g = ex.grid
    occ = np.argwhere(g.occupied())
    r.scenario = {"world": world, "local_planner": local, "localisation": loc, "movers": movers,
                  "coverage_log": ex.coverage_log}
    r.summary = summarise(params, m)
    r.try_next = suggestions(params, m)
    r.plots = plots_to_dicts(
        Plot("map", "Top-down: the real walls, the map it built, and the flight", "north y (m)",
             "Grey boxes = the real walls (the drone never sees this list). Red dots = the walls in the map it built "
             "from its lidar, drawn where it THOUGHT it was. Solid line = where it really flew; dashed = where it "
             "thought it was. Numbered points = the frontiers it chose to fly to, in order.",
             (Series("x_m", "flown path"), Series("y_m", "flown path")), kind="map",
             extra={"obstacles": [b.to_dict() for b in w.walls], "waypoints": [list(p) for p in ex.goals],
                    "safety_zone_m": 0.0, "planned_route": [list(p) for p in ex.path],
                    "discovered": [[round(x, 2), round(y, 2)] for x, y in (g.center(i, j) for i, j in occ)],
                    "estimate": [r.telemetry["x_est_m"], r.telemetry["y_est_m"]]}),
        Plot("coverage", "How much of the floor it has mapped", "mapped (%)",
             "Share of the real floor area whose cells the map knows (free or wall). It climbs fast in open space and "
             "slows down as the last corners need a detour.", (Series("coverage", "mapped", 100.0),)),
        Plot("clear", "Distance to the nearest obstacle (truth)", "gap (m)",
             "From the drone's edge to the nearest wall, tree or person, measured on the TRUE position. Below 0 = it hit "
             "something and the flight ends.", (Series("clear_m", "gap"),)),
        Plot("err", "How wrong is its position?", "error (m)",
             "Estimated minus true position. With GPS it stays small; without GPS the motion sensors drift, unless scan "
             "matching against the map pins it down.", (Series("est_err_m", "position error"),)),
        Plot("speed", "Speed the local planner chose", "speed (m/s)",
             "The velocity the local planner asked for, 10 times a second. 0 = it stopped (braked).",
             (Series("speed_cmd_m_s", "commanded speed"),)),
    )  # fmt: skip
    return r


def summarise(p: dict, m: dict) -> list[str]:
    lines = []
    if m["outcome"] == "crashed":
        lines.append(f"It crashed after {m['flight_time_s']:.0f} s, with {m['coverage_pct']:.0f} % of the floor mapped.")
    elif m["outcome"] == "done":
        t90 = f" (90 % after {m['time_to_90_s']:.0f} s)" if m["time_to_90_s"] is not None else ""
        lines.append(f"Explored on its own: {m['coverage_pct']:.0f} % of the floor mapped in {m['flight_time_s']:.0f} s{t90}, "
                     f"{m['path_length_m']:.0f} m flown, {m['frontier_goals']} frontiers visited. Then no frontier was "
                     "left that it could reach.")
    else:
        lines.append(f"Ran out of time with {m['coverage_pct']:.0f} % mapped.")
    if p["local_planner"] == "dwa":
        lines.append(f"DWA picked a velocity 10 times a second from what the lidar saw right then, keeping at least "
                     f"{m['min_clearance_m']:.2f} m between the drone's edge and anything (truth).")
    else:
        lines.append(f"Chasing a point 1.5 m along the path cuts corners: the closest it came was {m['min_clearance_m']:.2f} m "
                     f"(below 0 = contact), and it braked {m['brakes']} times. A local planner that looks at the lidar "
                     "before choosing a velocity (DWA) fixes that.")
    if p["localisation"] == "dead_reckoning":
        lines.append(f"No GPS and nothing to replace it: the EKF could only dead-reckon. Its position drifted up to "
                     f"{m['est_err_max_m']:.1f} m, so the map it drew was smeared and the drone flew into walls it "
                     "thought were elsewhere.")
    elif p["localisation"] == "slam":
        lines.append(f"No GPS, but {m['scan_matches']} scan matches against its own map kept the position within "
                     f"{m['est_err_max_m']:.2f} m (mean {m['est_err_mean_m']:.2f} m): SLAM, building the map and finding "
                     "itself in it at the same time.")
    else:
        lines.append(f"GPS + EKF: position good to {m['est_err_mean_m']:.2f} m on average.")
    if p["movers"] == "on":
        lines.append("Someone walked through. The map only remembers what the lidar keeps seeing: their trail fades as "
                     "beams pass through the empty space again.")
    return lines


def suggestions(p: dict, m: dict) -> list[dict]:
    out = []
    if p["local_planner"] == "pursuit":
        out.append(try_next("The Dynamic Window Approach: let the lidar choose the velocity.", local_planner="dwa"))
    if p["localisation"] == "gps":
        out.append(try_next("Indoors there is no GPS: what happens on motion sensors alone?", localisation="dead_reckoning"))
    elif p["localisation"] == "dead_reckoning":
        out.append(try_next("Find its position from the walls instead: lidar scan matching (SLAM).", localisation="slam"))
    if p["movers"] == "off":
        out.append(try_next("Someone walks through while it explores.", movers="on"))
    out.append(try_next("The forest: trees instead of walls.", world="forest") if p["world"] == "rooms"
               else try_next("The building: rooms and doors.", world="rooms"))
    return out[:4]


EXPERIMENT = Experiment(
    id="exploration",
    number=13,
    title="Autonomous exploration: map an unknown place",
    objective="Drop the drone into a place it has never seen and let it map it alone, with only its lidar.",
    concept=(
        "Frontier exploration: fly to the edge between what is known to be free and what is still unknown.",
        "A local planner (DWA) picks a safe velocity from the live scan, 10 times a second.",
        "Without GPS, matching each scan against the map so far gives the position: SLAM.",
    ),
    params=(
        Param("world", "World", "", "rooms", "Where to drop it.", options=tuple((k, w.title) for k, w in WORLDS.items())),
        Param("local_planner", "Local planner", "", "dwa", "How it chooses its velocity moment to moment.",
              options=tuple(LOCAL_PLANNERS.items())),
        Param("localisation", "Localisation", "", "gps", "How it knows where it is.", options=tuple(LOCALISATION.items())),
        Param("movers", "Moving obstacle", "", "off", "Someone walking through the place.", options=tuple(MOVERS.items())),
    ),
    key_metrics=(
        Metric("coverage_pct", "Floor mapped", "%", 1),
        Metric("time_to_90_s", "Time to map 90 %", "s", 1),
        Metric("min_clearance_m", "Closest to an obstacle (truth)", "m", 2),
        Metric("est_err_mean_m", "Position error (mean)", "m", 3),
        Metric("est_err_max_m", "Position error (worst)", "m", 2),
        Metric("path_length_m", "Distance flown", "m", 1),
    ),
    learn_slug="26-exploration",
    runner=run,
)
