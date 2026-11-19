"""Experiment 2: Mass vs required thrust. Heavier drones need proportionally more thrust to hover."""

from __future__ import annotations

import numpy as np

from backend.sim.config import get_preset
from backend.sim.runner import RunResult, run_hover

from .base import Experiment, Metric, Param, Plot, Series, add_channel, ch, motor_series, plots_to_dicts, try_next

DURATION_S = 10.0
HOLD_FROM_S = 5.0


def run(params: dict, seed: int) -> RunResult:
    cfg = get_preset("basic").with_changes(mass_kg=params["mass_kg"], motor_max_thrust_n=params["motor_thrust_n"])
    r = run_hover(cfg, alt_m=2.0, duration_s=DURATION_S, seed=seed)

    t = ch(r, "t_s")
    hold = t >= HOLD_FROM_S
    add_channel(r, "weight_n", cfg.weight_n)
    add_channel(r, "expected_hover_pct", min(100.0, cfg.hover_throttle_pct))
    hover_thrust = float(ch(r, "thrust_total_n")[hold].mean())
    r.metrics.update(
        {
            "mass_kg": cfg.mass_kg,
            "weight_n": cfg.weight_n,
            "hover_thrust_n": hover_thrust,
            "hover_thrust_per_motor_n": hover_thrust / 4.0,
            "headroom_pct": 100.0 - r.metrics["hover_motor_pct"],
        }
    )
    r.summary = summarise(cfg, r)
    m = params["mass_kg"]
    r.try_next = [
        try_next("Double the mass and compare the two runs in History.", mass_kg=min(3.0, 2 * m)),
        try_next("Halve the mass: what happens to the hover throttle?", mass_kg=max(0.3, m / 2)),
        try_next("Mass 2.0 kg with 4.9 N motors: T/W is exactly 1. Can it hover AND climb?", mass_kg=2.0, motor_thrust_n=4.9),
    ]
    r.plots = plots_to_dicts(
        Plot("motors", "Motor commands", "motor command (%)",
             "Once hovering, all motors sit near the dashed line: 100 % ÷ T/W. More mass moves that line up.",
             (*motor_series(), Series("expected_hover_pct", "expected hover", style="dashed"))),
        Plot("thrust", "Thrust vs weight", "force (N)",
             "In a steady hover, total thrust (solid) equals weight (dashed), whatever the mass.",
             (Series("thrust_total_n", "total thrust"), Series("weight_n", "weight", style="dashed"))),
        Plot("alt", "Altitude", "altitude (m)", "Solid = actual height, dashed = target (2 m).",
             (Series("z_m", "altitude"), Series("alt_setpoint_m", "target", style="dashed"))),
    )  # fmt: skip
    return r


def summarise(cfg, r: RunResult) -> list[str]:
    m = r.metrics
    if not m["left_ground"]:
        return [
            f"Weight {m['weight_n']:.2f} N is more than the {cfg.max_total_thrust_n:.2f} N the motors can make "
            f"(T/W = {cfg.thrust_to_weight:.2f}). It stayed on the ground.",
            "Either reduce the mass or choose stronger motors.",
        ]
    lines = [
        f"Weight = {m['mass_kg']:.2f} kg × 9.81 = {m['weight_n']:.2f} N. Measured hover thrust: {m['hover_thrust_n']:.2f} N "
        f"({m['hover_thrust_per_motor_n']:.2f} N per motor). Hover thrust equals weight.",
        f"Hover throttle {m['hover_motor_pct']:.1f} % = weight ÷ max thrust = 100 ÷ T/W ({cfg.hover_throttle_pct:.1f} %).",
        "Thrust needed grows in direct proportion to mass: double the mass, double the hover throttle.",
    ]
    if m["headroom_pct"] < 20:
        lines.append(
            f"Only {m['headroom_pct']:.0f} % throttle headroom is left. Climbing, braking and fighting wind all "
            "need extra thrust, so this drone will feel sluggish."
        )
    return lines


EXPERIMENT = Experiment(
    id="mass_thrust",
    number=2,
    title="Mass vs required thrust",
    objective="See how the thrust (and throttle) needed to hover depends on mass.",
    concept=(
        "To hover, total thrust must exactly cancel weight (m × g).",
        "So hover thrust is proportional to mass: twice the mass, twice the thrust.",
        "Throttle headroom (100 % − hover %) is what's left for climbing and manoeuvring.",
    ),
    params=(
        Param("mass_kg", "Mass", "kg", 1.0, "Total mass including battery.", 0.3, 3.0, 0.05),
        Param("motor_thrust_n", "Max thrust per motor", "N", 4.9, "How hard each motor pushes at full speed.", 1.0, 12.0, 0.1),
    ),
    key_metrics=(
        Metric("weight_n", "Weight", "N"),
        Metric("hover_thrust_n", "Hover thrust (measured)", "N"),
        Metric("thrust_to_weight", "T/W", "×"),
        Metric("hover_motor_pct", "Hover throttle", "%", 1),
        Metric("headroom_pct", "Throttle headroom", "%", 1),
    ),
    learn_slug="04-exp-mass-and-thrust",
    runner=run,
)
