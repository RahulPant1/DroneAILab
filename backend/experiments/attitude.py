"""Experiment 3: Roll, pitch and yaw. Which motors make each rotation, and why tilting moves the drone."""

from __future__ import annotations

import math

import numpy as np

from backend.sim.config import get_preset
from backend.sim.control import Setpoint
from backend.sim.metrics import step_response
from backend.sim.physics import State
from backend.sim.runner import RunResult, run_headless

from .base import RAD_TO_DEG, Experiment, Metric, Param, Plot, Series, ch, fmt, motor_series, plots_to_dicts, try_next

ALT_M = 3.0
STEP_START_S = 1.0
TILT_END_S = 3.0  # roll/pitch return to level here, so the drone doesn't fly off forever
DURATION_S = 6.0

# For each axis: (motors that work harder for a POSITIVE step, the others, plain-English description).
AXES = {
    "roll": ((2, 3), (1, 4), "right side down", "the LEFT motors (M2, M3)", "the right motors (M1, M4)"),
    "pitch": ((2, 4), (1, 3), "nose down", "the REAR motors (M2, M4)", "the front motors (M1, M3)"),
    "yaw": ((3, 4), (1, 2), "turn left", "the clockwise propellers (M3, M4)", "the counter-clockwise ones (M1, M2)"),
}


def run(params: dict, seed: int) -> RunResult:
    axis = params["axis"]
    angle_rad = math.radians(params["angle_deg"])
    kp = params["angle_kp"]
    cfg = get_preset("basic").with_changes(gains={"att_kp": kp, "yaw_kp": min(kp, 10.0)})

    def setpoint(t: float) -> Setpoint:
        active = t >= STEP_START_S and (axis == "yaw" or t < TILT_END_S)
        a = angle_rad if active else 0.0
        return Setpoint(
            alt_m=ALT_M,
            roll_rad=a if axis == "roll" else 0.0,
            pitch_rad=a if axis == "pitch" else 0.0,
            yaw_rad=a if axis == "yaw" else 0.0,
        )

    r = run_headless(cfg, setpoint, DURATION_S, seed, initial_state=State.hovering(cfg, ALT_M), scenario={"duration_s": DURATION_S})

    t = ch(r, "t_s")
    angle = ch(r, f"{axis}_rad")
    end = DURATION_S if axis == "yaw" else TILT_END_S
    step = step_response(t, angle, 0.0, angle_rad, STEP_START_S, end, band=math.radians(1.0))
    harder, other, *_ = AXES[axis]
    # Look at the moment the two motor groups differ most, just after the step. (Averaging would mix the
    # "start rotating" push with the "stop rotating" brake that follows, which cancel out.)
    early = np.flatnonzero((t >= STEP_START_S) & (t <= STEP_START_S + 0.5))
    harder_pct = np.mean([ch(r, f"motor{i}_pct") for i in harder], axis=0)
    other_pct = np.mean([ch(r, f"motor{i}_pct") for i in other], axis=0)
    peak = early[np.argmax((harder_pct - other_pct)[early])]
    x, y = ch(r, "x_m"), ch(r, "y_m")
    r.metrics.update(
        {
            "axis": axis,
            "angle_step_deg": params["angle_deg"],
            "rise_time_s": step.get("rise_time_s"),
            "overshoot_pct": step.get("overshoot_pct"),
            "settling_time_s": step.get("settling_time_s"),
            "harder_group_pct": float(harder_pct[peak]),
            "other_group_pct": float(other_pct[peak]),
            "motor_split_time_s": float(t[peak]),
            "final_x_m": float(x[-1]),
            "final_y_m": float(y[-1]),
            "horizontal_distance_m": float(math.hypot(x[-1], y[-1])),
            "altitude_dip_m": float(ALT_M - ch(r, "z_m").min()),
        }
    )
    r.summary = summarise(axis, r)
    r.try_next = [
        try_next("Raise the angle gain to 14: faster, but does it overshoot?", angle_kp=14.0),
        try_next("Lower the angle gain to 2: watch how sluggish it gets.", angle_kp=2.0),
        try_next("Now try pitch: which way does the drone move?", axis="pitch"),
        try_next("Try yaw: notice it doesn't move the drone sideways at all.", axis="yaw"),
    ]
    r.plots = plots_to_dicts(
        Plot("angle", f"{axis.capitalize()} angle", f"{axis} (°)",
             "Solid = actual angle, dashed = requested angle. The gap closing is the attitude controller at work.",
             (Series(f"{axis}_rad", axis, scale=RAD_TO_DEG), Series(f"{axis}_setpoint_rad", "requested", scale=RAD_TO_DEG, style="dashed"))),
        Plot("motors", "Motor commands", "motor command (%)",
             "At the moment of the step, some motors jump up and others drop: that difference makes the rotation.",
             motor_series()),
        Plot("pos", "Horizontal position", "position (m)",
             "Tilting points part of the thrust sideways, so the drone starts to move. Yaw alone doesn't move it.",
             (Series("x_m", "east (x)"), Series("y_m", "north (y)"))),
    )  # fmt: skip
    return r


def summarise(axis: str, r: RunResult) -> list[str]:
    m = r.metrics
    _, _, meaning, harder_text, other_text = AXES[axis]
    lines = [
        f"A {m['angle_step_deg']:.0f}° {axis} step ({meaning}): at t = {m['motor_split_time_s']:.2f} s {harder_text} ran at "
        f"{m['harder_group_pct']:.0f} % while {other_text} dropped to {m['other_group_pct']:.0f} %. "
        "Uneven thrust (or uneven spin drag, for yaw) creates the turning torque. Just after, the split briefly "
        "reverses to STOP the rotation at the right angle.",
        f"It reached 90 % of the angle in {fmt(m['rise_time_s'])} s (rise time) with {fmt(m['overshoot_pct'], 1)} % overshoot.",
    ]
    if axis == "yaw":
        lines.append(
            f"Yaw doesn't tilt the thrust, so the drone stayed put ({m['horizontal_distance_m']:.2f} m from the start). "
            "Yaw is weak: it relies on propeller drag, not thrust, which is why it's the slowest move."
        )
    else:
        direction = {"roll": "to the right (−y)", "pitch": "forward (+x)"}[axis]
        lines.append(
            f"Tilting sent part of the thrust sideways, so the drone moved {direction}: it ended "
            f"{m['horizontal_distance_m']:.2f} m from where it started, even after levelling out at t = {TILT_END_S:.0f} s "
            "(nothing brakes it except air drag)."
        )
    if m["overshoot_pct"] and m["overshoot_pct"] > 10:
        lines.append("Noticeable overshoot: the angle gain is high for this drone.")
    if m["rise_time_s"] and m["rise_time_s"] > 0.6:
        lines.append("Slow response: a low angle gain makes the drone feel lazy.")
    return lines


EXPERIMENT = Experiment(
    id="attitude",
    number=3,
    title="Roll, pitch and yaw",
    objective="See which motors create each rotation, and why tilting makes the drone move.",
    concept=(
        "Roll and pitch come from pushing one side harder than the other.",
        "Yaw comes from speeding up the props that spin one way and slowing the others.",
        "Tilting points part of the thrust sideways, and that's how a quadcopter moves.",
    ),
    params=(
        Param("axis", "Axis", "", "roll", "Which rotation to try.", options=(("roll", "Roll"), ("pitch", "Pitch"), ("yaw", "Yaw"))),
        Param("angle_deg", "Angle step", "°", 15.0, "How far to rotate (held 2 s for roll/pitch).", 5.0, 30.0, 1.0),
        Param("angle_kp", "Angle gain (P)", "1/s", 6.0, "How quickly the controller turns toward the requested angle.", 2.0, 15.0, 0.5),
    ),
    key_metrics=(
        Metric("rise_time_s", "Rise time", "s"),
        Metric("overshoot_pct", "Overshoot", "%", 1),
        Metric("harder_group_pct", "Harder-working motors (peak)", "%", 0),
        Metric("other_group_pct", "Other motors (same moment)", "%", 0),
        Metric("horizontal_distance_m", "Moved sideways", "m"),
    ),
    learn_slug="05-exp-roll-pitch-yaw",
    runner=run,
)
