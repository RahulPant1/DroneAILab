"""Experiment 4: PID tuning. Watch an altitude step response change as you turn the P, I and D knobs."""

from __future__ import annotations

import numpy as np

from backend.sim.config import get_preset
from backend.sim.control import Setpoint
from backend.sim.environment import EnvConfig
from backend.sim.metrics import step_response
from backend.sim.physics import State
from backend.sim.runner import RunResult, run_headless

from .base import Experiment, Metric, Param, Plot, Series, ch, fmt, motor_series, plots_to_dicts, try_next

START_ALT_M = 1.0
TARGET_ALT_M = 3.0
STEP_AT_S = 1.0
DURATION_S = 15.0
BAND_M = 0.05


def run(params: dict, seed: int) -> RunResult:
    base = get_preset("basic")
    cfg = base.with_changes(gains={"alt_kp": params["kp"], "alt_ki": params["ki"], "alt_kd": params["kd"]})
    env = EnvConfig(unmodeled_payload_kg=params["payload_kg"])

    def setpoint(t: float) -> Setpoint:
        return Setpoint(alt_m=TARGET_ALT_M if t >= STEP_AT_S else START_ALT_M)

    r = run_headless(cfg, setpoint, DURATION_S, seed, initial_state=State.hovering(base, START_ALT_M), env=env,
                     scenario={"duration_s": DURATION_S})  # fmt: skip

    t, z = ch(r, "t_s"), ch(r, "z_m")
    s = step_response(t, z, START_ALT_M, TARGET_ALT_M, STEP_AT_S, band=BAND_M)
    r.metrics.update(
        {
            "rise_time_s": s["rise_time_s"],
            "overshoot_pct": s["overshoot_pct"],
            "settling_time_s": s["settling_time_s"],
            "oscillations": s["oscillations"],
            "steady_state_error_m": s["final_error"],
            "peak_alt_m": float(z.max()),
            "payload_kg": params["payload_kg"],
            "kp": params["kp"],
            "ki": params["ki"],
            "kd": params["kd"],
        }
    )
    r.summary = summarise(params, r)
    r.try_next = [
        try_next("Kp = 25: a much stronger 'spring'. Count the oscillations.", kp=25.0, ki=0.5, kd=3.5),
        try_next("Kd = 0: remove the damping and compare the overshoot.", kp=4.0, ki=0.5, kd=0.0),
        try_next("Payload 0.3 kg with Ki = 0: P and D alone leave a gap. Then set Ki = 1.", payload_kg=0.3, ki=0.0),
        try_next("Back to the defaults (Kp 4, Ki 0.5, Kd 3.5, no payload).", kp=4.0, ki=0.5, kd=3.5, payload_kg=0.0),
    ]
    r.plots = plots_to_dicts(
        Plot("alt", "Altitude step response", "altitude (m)",
             "Target jumps from 1 m to 3 m at t = 1 s (dashed). Look for: how fast it rises, how far it overshoots, how long it wobbles.",
             (Series("z_m", "altitude"), Series("alt_setpoint_m", "target", style="dashed"))),
        Plot("vz", "Vertical speed", "vertical speed (m/s)",
             "Positive = climbing. The D term pushes against this speed, acting like a brake.",
             (Series("vz_m_s", "vertical speed"),)),
        Plot("motors", "Motor commands", "motor command (%)",
             "Flat at 100 % = saturated: the controller wants more than the motors can give.",
             motor_series()),
    )  # fmt: skip
    return r


def summarise(p: dict, r: RunResult) -> list[str]:
    m = r.metrics
    lines = [
        f"Step 1 m → 3 m: rise time {fmt(m['rise_time_s'])} s, overshoot {m['overshoot_pct']:.1f} %, "
        f"{m['oscillations']} crossing(s) of the target, settled (±{BAND_M} m) "
        + (f"after {m['settling_time_s']:.1f} s." if m["settling_time_s"] is not None else "NEVER within the run.")
    ]
    if m["oscillations"] >= 3 and m["overshoot_pct"] > 20:
        lines.append(
            "It oscillates. The P term (Kp) acts like a spring: a strong spring with too little damping (Kd) "
            "keeps bouncing past the target. Lower Kp or raise Kd."
        )
    elif m["overshoot_pct"] > 10:
        lines.append("Noticeable overshoot: there's not quite enough damping (Kd) for this Kp.")
    elif m["rise_time_s"] is not None and m["rise_time_s"] > 3.0:
        lines.append("Sluggish: Kp is low (or Kd is high), so the push toward the target is gentle.")
    else:
        lines.append("A clean response: quick, with little overshoot.")
    if p["kd"] == 0:
        lines.append("Kd = 0 means nothing brakes the climb, so it overshoots a lot.")
    if p["payload_kg"] > 0:
        sse = m["steady_state_error_m"]
        if p["ki"] == 0:
            lines.append(
                f"The {p['payload_kg']:.2f} kg payload the controller doesn't know about leaves it "
                f"{abs(sse):.2f} m too low for good. P only pushes when there's an error, so some error has to remain. "
                "That's the job of the I term: try Ki = 1."
            )
        elif abs(sse) > BAND_M:
            lines.append(
                f"The I term is slowly learning the extra {p['payload_kg']:.2f} kg payload. It's still {abs(sse):.2f} m low "
                "at the end. A bigger Ki learns faster, but too big causes overshoot."
            )
        else:
            lines.append(
                f"The I term learned the hidden {p['payload_kg']:.2f} kg payload and removed the height error "
                f"(final error {sse * 100:.1f} cm)."
            )
    if any(e["type"] == "motor_saturation" for e in r.events):
        lines.append("Motors hit 100 % at some point: the controller asked for more than the drone can do.")
    return lines


EXPERIMENT = Experiment(
    id="pid_tuning",
    number=4,
    title="PID tuning (step response)",
    objective="Tune the altitude controller and see how P, I and D each change the response.",
    concept=(
        "P pushes in proportion to the error: a spring pulling toward the target.",
        "D pushes against the speed: a shock absorber that stops overshoot.",
        "I adds up error over time: it removes steady offsets, like an unknown payload.",
    ),
    params=(
        Param("kp", "Kp (proportional)", "(m/s²)/m", 4.0, "Spring strength. Higher = faster, but bouncier.", 0.5, 30.0, 0.5),
        Param("ki", "Ki (integral)", "(m/s²)/(m·s)", 0.5, "Removes lasting errors. Too high = slow wobbles.", 0.0, 5.0, 0.1),
        Param("kd", "Kd (derivative)", "(m/s²)/(m/s)", 3.5, "Damping. 0 = no brakes.", 0.0, 10.0, 0.25),
        Param("payload_kg", "Hidden payload", "kg", 0.0, "Extra mass the controller doesn't know about.", 0.0, 0.5, 0.05),
    ),
    key_metrics=(
        Metric("rise_time_s", "Rise time", "s"),
        Metric("overshoot_pct", "Overshoot", "%", 1),
        Metric("settling_time_s", "Settling time (±5 cm)", "s", 1),
        Metric("oscillations", "Target crossings", "", 0),
        Metric("steady_state_error_m", "Final error", "m", 3),
    ),
    learn_slug="06-exp-pid-tuning",
    runner=run,
)
