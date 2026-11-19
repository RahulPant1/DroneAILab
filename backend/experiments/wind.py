"""Experiment 5: Wind. A disturbance pushes the drone off its spot, and the controller fights back."""

from __future__ import annotations

import numpy as np

from backend.sim.config import get_preset
from backend.sim.control import Setpoint
from backend.sim.environment import EnvConfig
from backend.sim.physics import State
from backend.sim.runner import RunResult, run_headless

from .base import Experiment, Metric, Param, Plot, Series, add_channel, ch, fmt, motor_series, plots_to_dicts, try_next

ALT_M = 2.0
WIND_START_S = 3.0
DURATION_S = 25.0
TAIL_S = 5.0  # "steady state" = the last 5 s
RECOVERED_M = 0.1


def run(params: dict, seed: int) -> RunResult:
    cfg = get_preset("basic").with_changes(gains={"vel_ki": params["vel_ki"]})
    env = EnvConfig(
        wind_speed_m_s=params["wind_speed_m_s"],
        wind_direction_deg=params["wind_direction_deg"],
        gust_std_m_s=params["gust_std_m_s"],
        wind_start_s=WIND_START_S,
    )
    r = run_headless(cfg, Setpoint(alt_m=ALT_M, position_control=True), DURATION_S, seed,
                     initial_state=State.hovering(cfg, ALT_M), env=env, scenario={"duration_s": DURATION_S})  # fmt: skip

    t = ch(r, "t_s")
    err = np.hypot(ch(r, "x_m") - ch(r, "x_setpoint_m"), ch(r, "y_m") - ch(r, "y_setpoint_m"))
    tilt_deg = np.degrees(np.arccos(np.clip(np.cos(ch(r, "roll_rad")) * np.cos(ch(r, "pitch_rad")), -1, 1)))
    if "wind_x_m_s" in r.telemetry:
        wind_speed = np.hypot(ch(r, "wind_x_m_s"), ch(r, "wind_y_m_s"))
    else:
        wind_speed = np.zeros_like(t)
    add_channel(r, "position_error_m", err)
    add_channel(r, "tilt_deg", tilt_deg)
    add_channel(r, "wind_speed_m_s", wind_speed)

    after = t >= WIND_START_S
    tail = t >= DURATION_S - TAIL_S
    i_peak = int(np.argmax(np.where(after, err, -1)))
    below = np.flatnonzero((t > t[i_peak]) & (err < RECOVERED_M))
    r.metrics.update(
        {
            "max_drift_m": float(err[after].max()),
            "time_of_max_drift_s": float(t[i_peak] - WIND_START_S),
            "recovery_time_s": float(t[below[0]] - WIND_START_S) if below.size else None,
            "steady_offset_m": float(err[tail].mean()),
            "position_wander_rms_m": float(np.sqrt(np.mean(err[tail] ** 2))),
            "lean_angle_deg": float(tilt_deg[tail].mean()),
            "mean_wind_speed_m_s": float(wind_speed[after].mean()),
            "max_motor_pct": float(max(ch(r, f"motor{i}_pct").max() for i in range(1, 5))),
        }
    )
    r.summary = summarise(params, r)
    r.try_next = [
        try_next("Set the speed I-gain to 0: does the drone still get back to its spot?", vel_ki=0.0),
        try_next("Add gusts of 2 m/s: watch the drone wander and the motors work.", gust_std_m_s=2.0),
        try_next("Strong wind, 11 m/s: how far does it lean?", wind_speed_m_s=11.0),
        try_next("Blow from the north-east (direction 225°) and see the drift direction change.", wind_direction_deg=225.0),
    ]
    r.plots = plots_to_dicts(
        Plot("err", "Distance from the hover spot", "position error (m)",
             f"Wind switches on at t = {WIND_START_S:.0f} s. The peak is how far it got pushed; the tail shows whether it fought back to 0.",
             (Series("position_error_m", "distance from target"),)),
        Plot("pos", "Position", "position (m)", "East (x) and north (y) position. The target is (0, 0).",
             (Series("x_m", "east (x)"), Series("y_m", "north (y)"))),
        Plot("tilt", "Lean angle and wind", "tilt (°) / wind (m/s)",
             "To hold position in wind, the drone must lean INTO it, using part of its thrust to push back.",
             (Series("tilt_deg", "tilt (°)"), Series("wind_speed_m_s", "wind speed (m/s)", style="dashed"))),
        Plot("motors", "Motor commands", "motor command (%)",
             "Leaning needs extra total thrust, and gusts make the motors work constantly.",
             motor_series()),
    )  # fmt: skip
    return r


def summarise(p: dict, r: RunResult) -> list[str]:
    m = r.metrics
    if p["wind_speed_m_s"] == 0 and p["gust_std_m_s"] == 0:
        return ["No wind, so nothing pushed the drone. Set a wind speed and run again."]
    lines = [
        f"The wind (average {m['mean_wind_speed_m_s']:.1f} m/s) pushed the drone up to {m['max_drift_m']:.2f} m off its spot, "
        f"{m['time_of_max_drift_s']:.1f} s after it started. Drag from the moving air is the push.",
        f"To fight it, the drone leaned about {m['lean_angle_deg']:.1f}° into the wind, using part of its thrust to push back.",
    ]
    if p["vel_ki"] == 0:
        lines.append(
            f"With the speed I-gain at 0, it settled {m['steady_offset_m']:.2f} m downwind and stayed there. "
            "The P terms only push back when there's an error, so some error has to remain. The I term removes it."
        )
    elif m["steady_offset_m"] < 0.15:
        rec = f" after {m['recovery_time_s']:.1f} s" if m["recovery_time_s"] is not None else ""
        lines.append(
            f"It recovered{rec}: the I term gradually 'learned' the wind and brought the drone back "
            f"(average offset over the last {TAIL_S:.0f} s: {m['steady_offset_m'] * 100:.0f} cm)."
        )
    else:
        lines.append(f"It hasn't fully recovered yet (average offset {m['steady_offset_m']:.2f} m over the last {TAIL_S:.0f} s).")
    if p["gust_std_m_s"] > 0:
        lines.append(
            f"Gusts change faster than the controller can cancel them, so the drone keeps wandering "
            f"(RMS {m['position_wander_rms_m']:.2f} m at the end)."
        )
    if m["max_motor_pct"] >= 99.5:
        lines.append("A motor hit 100 %: this wind is close to the limit of what the drone can fight.")
    return lines


EXPERIMENT = Experiment(
    id="wind",
    number=5,
    title="Wind (constant + gusts)",
    objective="Push the drone with wind and watch the position controller fight back.",
    concept=(
        "Drag depends on speed relative to the AIR, so wind pushes even a hovering drone.",
        "To hold its spot, the drone must lean into the wind.",
        "P reacts to the drift; I learns a steady wind and removes the leftover offset.",
    ),
    params=(
        Param("wind_speed_m_s", "Wind speed", "m/s", 5.0, "Steady wind, switched on at t = 3 s.", 0.0, 12.0, 0.5),
        Param("wind_direction_deg", "Blowing toward", "°", 0.0, "0° = toward east, 90° = toward north.", 0.0, 345.0, 15.0),
        Param("gust_std_m_s", "Gust strength", "m/s", 0.0, "Typical size of random gusts on top of the steady wind.", 0.0, 4.0, 0.25),
        Param("vel_ki", "Speed I-gain", "1/s²", 0.5, "How quickly the controller learns a steady push. 0 = never.", 0.0, 2.0, 0.1),
    ),
    key_metrics=(
        Metric("max_drift_m", "Max drift", "m"),
        Metric("recovery_time_s", "Recovery time (<10 cm)", "s", 1),
        Metric("steady_offset_m", "Offset (last 5 s)", "m"),
        Metric("lean_angle_deg", "Lean angle", "°", 1),
        Metric("max_motor_pct", "Max motor", "%", 0),
    ),
    learn_slug="07-exp-wind",
    runner=run,
)
