"""Experiment 11: Agile flight. A smooth minimum-snap trajectory through race rings, flown three ways."""

from __future__ import annotations

from backend.autonomy.agile import METHOD_NAMES, METHODS, run_track
from backend.games.race import TRACKS
from backend.sim.environment import EnvConfig
from backend.sim.runner import RunResult
from backend.sim.sensors import SensorConfig

from .base import Experiment, Metric, Param, Plot, Series, motor_series, plots_to_dicts, try_next

CONDITIONS = {
    "calm": ("Calm air, perfect sensors", None, None),
    "windy": ("Gusty 5 m/s crosswind", EnvConfig(wind_speed_m_s=5.0, wind_direction_deg=90.0, gust_std_m_s=1.0), None),
    "sensors": ("Real sensors (our simple filter)", None, SensorConfig(enabled=True)),
    "sensors_gyro": ("Real sensors, gyro-only tilt", None, SensorConfig(enabled=True, tilt_correction=0.0)),
    "sensors_ekf": ("Real sensors, Kalman filter (EKF)", None, SensorConfig(enabled=True, estimator="ekf")),
}


def run(params: dict, seed: int) -> RunResult:
    label, env, sensors = CONDITIONS[params["conditions"]]
    r = run_track(params["track"], params["method"], 12.0, env=env, sensors=sensors, seed=seed,
                  tilt_deg=params["max_tilt_deg"])
    plan = r.scenario["plan"]
    gates = [[g["x_m"], g["y_m"], g["heading_deg"], g["radius_m"], 1 if t is not None else 0]
             for g, t in zip(plan["rings"], r.scenario["ring_times_s"])]
    r.summary = summarise(params, r)
    r.try_next = suggestions(params, r)
    r.plots = plots_to_dicts(
        Plot("map", "Top-down: the plan and the flight", "north y (m)",
             "Dashed yellow = the plan (straight lines for waypoints, the smooth trajectory otherwise); solid = where the "
             "drone really flew. Each ring is a short bar with an arrow: the way to fly through. Green = flown through, "
             "red = missed.", (Series("x_m", "flown path"), Series("y_m", "flown path")), kind="map",
             extra={"obstacles": [], "waypoints": [plan["points"][0][:2]], "safety_zone_m": 0.0,
                    "planned_route": [p[:2] for p in plan["path"]], "gates": gates}),
        Plot("err", "How far from where the plan said to be", "distance (m)",
             "Distance from the reference at that moment: the trajectory sample (or the carrot, for waypoints). "
             "Feedforward keeps this small; without it the drone has to fall behind before it reacts.",
             (Series("track_err_m", "tracking error"),)),
        Plot("speed", "Speed", "speed (m/s)",
             "3D speed. The plan speeds up on straights and slows before tight rings, as fast as the tilt limit allows.",
             (Series("speed_m_s", "speed"),)),
        Plot("alt", "Height vs plan", "altitude (m)", "The rings sit at different heights; the trajectory climbs and "
             "descends smoothly between them.", (Series("z_m", "altitude"), Series("ref_z_m", "plan", style="dashed"))),
        Plot("tilt", "Tilt (how hard it leans)", "angle (deg)",
             "Roll and pitch. Leaning is how a quadcopter accelerates sideways: the max-tilt setting caps it.",
             (Series("roll_rad", "roll", 57.2958), Series("pitch_rad", "pitch", 57.2958))),
        Plot("motors", "Motor commands", "motor (%)",
             "Hitting 100 % (or 0 %) = saturation: the controller asked for more than the motors can give.", motor_series()),
    )  # fmt: skip
    return r


def summarise(p: dict, r: RunResult) -> list[str]:
    m = r.metrics
    name = METHOD_NAMES[p["method"]]
    cond = CONDITIONS[p["conditions"]][0]
    lines = []
    if m["lap_time_s"] is not None:
        lines.append(f"{name} ({cond}): all {m['rings_total']} rings in {m['lap_time_s']:.1f} s, top speed "
                     f"{m['max_speed_m_s']:.1f} m/s, leaning up to {m['max_tilt_deg']:.0f}°.")
    else:
        lines.append(f"{name} ({cond}): flew through {m['rings_passed']} of {m['rings_total']} rings. A ring has a "
                     f"0.9 m radius, so being more than about half a metre off at the wrong moment misses it.")
    lines.append(f"Distance from the planned path: {m['path_err_mean_m']:.2f} m on average, {m['path_err_max_m']:.2f} m at "
                 "worst.")
    if p["method"] == "waypoints":
        lines.append(
            "Waypoints: a carrot slides along straight lines and the drone chases it. It only reacts once it has fallen "
            f"behind (it trails the carrot by about {m['track_err_mean_m']:.1f} m), and the position loop caps it at 3 m/s "
            "without a plan. Safe and simple: this is how our missions fly.")
    elif p["method"] == "pid_ff":
        lines.append(
            "Trajectory + PID feedforward: the planner computed one smooth curve, and the controller is told the speed and "
            "acceleration it needs at every moment (plus the drag it will meet), so it doesn't wait for an error. The "
            "cascade still turns acceleration into roll and pitch with a small-angle shortcut, one axis at a time.")
    else:
        lines.append(
            "Geometric controller: works out the exact thrust direction for the wanted 3D acceleration (no small-angle "
            "shortcut, height included), and feeds the inner loop the spin rate the plan's jerk needs. Same gains as the "
            "PID: the difference is structure.")
    if p["conditions"] == "windy":
        lines.append("The wind is NOT in the plan: feedforward only helps with what you know. The gusts have to be fought "
                     "by feedback (P, D and the slow integral), so every method drifts more.")
    if p["conditions"] == "sensors":
        lines.append(
            "Real sensors: the estimate trusts the accelerometer as 'which way is down'. In a long banked turn the "
            "accelerometer feels the thrust, not gravity, so the estimate slowly levels out while the drone is really "
            "tilted. The controller then flies the wrong acceleration. Try 'gyro-only tilt' to see the difference.")
    if p["conditions"] == "sensors_gyro":
        lines.append(
            "Gyro-only tilt: no accelerometer correction, so turns don't fool the estimate. It works here because this "
            "gyro has no bias and the lap is short. A real autopilot (ArduPilot's EKF3) fuses GPS velocity to keep the "
            "tilt right during aggressive flight.")
    if p["conditions"] == "sensors_ekf":
        lines.append(
            "Kalman filter: it fuses GPS velocity, so a tilt error shows up as a velocity disagreement and gets "
            "corrected, even in long banked turns. Experiment 12 compares it with the simple filter side by side.")
    return lines


def suggestions(p: dict, r: RunResult) -> list[dict]:
    out = []
    if p["method"] == "waypoints":
        out.append(try_next("Fly the same rings on a smooth trajectory (PID + feedforward).", method="pid_ff"))
    elif p["method"] == "pid_ff":
        out.append(try_next("Same trajectory, geometric controller: how much closer to the line?", method="geometric"))
    else:
        out.append(try_next("Back to plain waypoints: compare the lap time.", method="waypoints"))
    if p["max_tilt_deg"] < 40:
        out.append(try_next("Let it lean 45°: a faster plan. Does the controller keep up?", max_tilt_deg=45.0))
    else:
        out.append(try_next("A gentle 15° limit: slower, but how much more accurate?", max_tilt_deg=15.0))
    if p["conditions"] == "calm":
        out.append(try_next("Real sensors: the accelerometer is fooled in turns.", conditions="sensors"))
    elif p["conditions"] == "sensors":
        out.append(try_next("The Kalman filter (EKF): fuse GPS velocity instead.", conditions="sensors_ekf"))
    else:
        out.append(try_next("A gusty crosswind the plan doesn't know about.", conditions="windy"))
    if p["track"] != "figure_eight":
        out.append(try_next("The figure eight: 8 rings, two heights in the middle.", track="figure_eight"))
    return out[:4]


EXPERIMENT = Experiment(
    id="agile_flight",
    number=11,
    title="Agile flight: trajectories and tracking",
    objective="Fly race rings on one smooth planned curve, and compare how three controllers follow it.",
    concept=(
        "Minimum-snap trajectory: one smooth curve through every ring, as fast as the tilt limit allows.",
        "Feedforward: tell the controller the planned speed and acceleration, so it doesn't wait for an error.",
        "Geometric (SE(3)) control: compute the exact thrust direction, with no small-angle shortcut.",
    ),
    params=(
        Param("track", "Track", "", "figure_eight", "Which rings to fly.",
              options=tuple((k, v["title"]) for k, v in TRACKS.items())),
        Param("method", "How to fly it", "", "geometric", "Waypoints (no plan) or a trajectory with one of two controllers.",
              options=tuple((m, METHOD_NAMES[m]) for m in METHODS)),
        Param("max_tilt_deg", "Max tilt", "deg", 30.0, "How hard it may lean: more tilt = more acceleration = a faster plan.",
              10.0, 45.0, 1.0),
        Param("conditions", "Conditions", "", "calm", "Perfect world, wind, or real sensors.",
              options=tuple((k, v[0]) for k, v in CONDITIONS.items())),
    ),
    key_metrics=(
        Metric("lap_time_s", "Lap time", "s", 2),
        Metric("rings_passed", "Rings flown through", "", 0),
        Metric("path_err_mean_m", "Off the planned path (mean)", "m", 3),
        Metric("track_err_mean_m", "Behind the reference (mean)", "m", 3),
        Metric("max_speed_m_s", "Top speed", "m/s", 1),
        Metric("max_tilt_deg", "Max tilt", "deg", 0),
    ),
    learn_slug="21-trajectories",
    runner=run,
)
