"""Experiment 1: Thrust and take-off. How much spare thrust (T/W) does a drone need?"""

from __future__ import annotations

import numpy as np

from backend.sim.config import get_preset
from backend.sim.runner import RunResult, run_hover

from .base import Experiment, Metric, Param, Plot, Series, add_channel, ch, fmt, motor_series, plots_to_dicts, try_next

DURATION_S = 10.0


def run(params: dict, seed: int) -> RunResult:
    base = get_preset("basic")
    tw = params["thrust_to_weight"]
    cfg = base.with_changes(motor_max_thrust_n=tw * base.weight_n / 4.0)
    alt = params["target_alt_m"]
    r = run_hover(cfg, alt_m=alt, duration_s=DURATION_S, seed=seed)

    add_channel(r, "weight_n", cfg.weight_n)
    add_channel(r, "max_thrust_n", cfg.max_total_thrust_n)
    add_channel(r, "expected_hover_pct", min(100.0, cfg.hover_throttle_pct))
    vz = ch(r, "vz_m_s")
    liftoff = next((e["t_s"] for e in r.events if e["type"] == "liftoff"), None)
    r.metrics.update(
        {
            "weight_n": cfg.weight_n,
            "max_total_thrust_n": cfg.max_total_thrust_n,
            "liftoff_time_s": liftoff,
            "max_climb_rate_m_s": float(vz.max()),
            "spare_thrust_n": cfg.max_total_thrust_n - cfg.weight_n,
        }
    )
    r.summary = summarise(cfg, r)
    r.try_next = [
        try_next("Set T/W to 0.9: the motors are weaker than gravity. What happens?", thrust_to_weight=0.9),
        try_next("Set T/W to 1.2: it flies, but notice how slowly it climbs.", thrust_to_weight=1.2),
        try_next("Set T/W to 3.0 and compare the climb rate and hover throttle.", thrust_to_weight=3.0),
    ]
    r.plots = plots_to_dicts(
        Plot("alt", "Altitude", "altitude (m)",
             "Solid = actual height, dashed = target. A steeper start means more spare thrust.",
             (Series("z_m", "altitude"), Series("alt_setpoint_m", "target", style="dashed"))),
        Plot("thrust", "Thrust vs weight", "force (N)",
             "While thrust (solid) is above weight (dashed), the drone accelerates upward. At hover they are equal.",
             (Series("thrust_total_n", "total thrust"), Series("weight_n", "weight", style="dashed"),
              Series("max_thrust_n", "max possible thrust", style="dashed"))),
        Plot("motors", "Motor commands", "motor command (%)",
             "Each motor's command. During a steady hover they sit near the dashed 'expected hover' line.",
             (*motor_series(), Series("expected_hover_pct", "expected hover", style="dashed"))),
    )  # fmt: skip
    return r


def summarise(cfg, r: RunResult) -> list[str]:
    m = r.metrics
    tw = cfg.thrust_to_weight
    lines = [f"Weight {m['weight_n']:.2f} N, max thrust {m['max_total_thrust_n']:.2f} N, so T/W = {tw:.2f}."]
    if not m["left_ground"]:
        lines.append(
            f"It never left the ground. Even at 100 % the motors make only {tw * 100:.0f} % of the force "
            "needed to cancel gravity. No controller can fix that: you need T/W above 1."
        )
        return lines
    lines.append(f"It lifted off at t = {fmt(m['liftoff_time_s'])} s and climbed at up to {m['max_climb_rate_m_s']:.2f} m/s.")
    lines.append(
        f"Hover throttle was {m['hover_motor_pct']:.1f} %, matching 100 / T/W = {cfg.hover_throttle_pct:.1f} %. "
        f"The {m['spare_thrust_n']:.1f} N of spare thrust is what lets it climb and fight wind."
    )
    if tw < 1.4:
        lines.append("With little spare thrust the motors spend the climb near 100 % (saturated): the climb is slow.")
    elif tw > 2.5:
        lines.append("Lots of spare thrust: a quick climb, and a low hover throttle that leaves room for manoeuvres.")
    if m["settling_time_s"] is not None:
        lines.append(f"It settled within ±{m['settling_band_m']} m of the target {m['settling_time_s']:.1f} s after the start.")
    return lines


EXPERIMENT = Experiment(
    id="thrust_takeoff",
    number=1,
    title="Thrust and take-off",
    objective="Find out how much thrust a drone needs to take off, and what spare thrust buys you.",
    concept=(
        "Weight = mass × g pulls down; the four propellers push up.",
        "Thrust-to-weight (T/W) = max thrust ÷ weight. Below 1, it can never lift off.",
        "Hover needs thrust = weight, so hover throttle ≈ 100 % ÷ T/W.",
    ),
    params=(
        Param("thrust_to_weight", "Thrust-to-weight (T/W)", "×", 2.0,
              "Max total thrust divided by weight. 1.0 = just enough to hover, never enough to climb.", 0.6, 3.0, 0.05),
        Param("target_alt_m", "Target altitude", "m", 2.0, "How high to climb.", 1.0, 5.0, 0.5),
    ),  # fmt: skip
    key_metrics=(
        Metric("thrust_to_weight", "T/W", "×"),
        Metric("left_ground", "Left the ground"),
        Metric("liftoff_time_s", "Lift-off time", "s"),
        Metric("max_climb_rate_m_s", "Max climb rate", "m/s"),
        Metric("rise_time_s", "Rise time (10→90 %)", "s"),
        Metric("hover_motor_pct", "Hover throttle", "%", 1),
    ),
    learn_slug="03-exp-thrust-and-takeoff",
    runner=run,
)
