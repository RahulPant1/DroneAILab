"""Experiment 6: Sensor noise. Truth vs estimate: the drone only knows what its sensors tell it."""

from __future__ import annotations

import numpy as np

from backend.sim.config import get_preset
from backend.sim.control import Setpoint
from backend.sim.physics import State
from backend.sim.runner import RunResult, run_headless
from backend.sim.sensors import SensorConfig

from .base import RAD_TO_DEG, Experiment, Metric, Param, Plot, Series, ch, plots_to_dicts, try_next

ALT_M = 2.0
DURATION_S = 20.0
SKIP_S = 2.0  # ignore the first moments while things settle


def rms(x) -> float:
    return float(np.sqrt(np.mean(np.square(x))))


def run(params: dict, seed: int) -> RunResult:
    cfg = get_preset("basic")
    sensors = SensorConfig(
        enabled=True,
        noise_scale=params["noise_scale"],
        baro_trust=params["baro_trust"],
        gyro_bias_deg_s=params["gyro_bias_deg_s"],
        tilt_correction=params["tilt_correction"],
    )
    r = run_headless(cfg, Setpoint(alt_m=ALT_M, position_control=True), DURATION_S, seed,
                     initial_state=State.hovering(cfg, ALT_M), sensors=sensors, scenario={"duration_s": DURATION_S})  # fmt: skip

    w = ch(r, "t_s") >= SKIP_S
    z, ze, baro = ch(r, "z_m")[w], ch(r, "z_est_m")[w], ch(r, "baro_m")[w]
    x, xe, y, ye = ch(r, "x_m")[w], ch(r, "x_est_m")[w], ch(r, "y_m")[w], ch(r, "y_est_m")[w]
    roll, roll_e = ch(r, "roll_rad")[w], ch(r, "roll_est_rad")[w]
    r.metrics.update(
        {
            "raw_altimeter_error_m": rms(baro - z),
            "est_altitude_error_m": rms(ze - z),
            "est_position_error_m": rms(np.hypot(xe - x, ye - y)),
            "est_roll_error_deg": rms((roll_e - roll) * RAD_TO_DEG),
            "mean_roll_error_deg": float(np.mean((roll_e - roll) * RAD_TO_DEG)),
            "true_altitude_error_m": rms(z - ALT_M),
            "true_position_error_m": rms(np.hypot(x, y)),
        }
    )
    r.summary = summarise(params, r)
    r.try_next = [
        try_next("Altitude trust 1.0: the estimate copies the noisy altimeter. Does the drone bob?", baro_trust=1.0),
        try_next("Altitude trust 0.02: smooth, but how far does it wander?", baro_trust=0.02),
        try_next("Gyro bias 2 °/s: a tiny sensor offset, a big tilt error.", gyro_bias_deg_s=2.0),
        try_next("Gyro bias 2 °/s AND tilt correction 1.0: the accelerometer fixes the bias, but…",
                 gyro_bias_deg_s=2.0, tilt_correction=1.0),
        try_next("Noise ×3: worse sensors everywhere.", noise_scale=3.0),
    ]  # fmt: skip
    r.plots = plots_to_dicts(
        Plot("alt", "Altitude: truth vs estimate", "altitude (m)",
             "Dots = raw altimeter readings. Solid = true height. Dashed = what the drone THINKS its height is.",
             (Series("baro_m", "altimeter reading", style="points"), Series("z_m", "true altitude"),
              Series("z_est_m", "estimated altitude", style="dashed"))),
        Plot("x", "East position: truth vs estimate", "east position x (m)",
             "Dots = GPS readings (10 per second). The controller steers the ESTIMATE to 0, so the true drone wanders.",
             (Series("gps_x_m", "GPS reading", style="points"), Series("x_m", "true x"), Series("x_est_m", "estimated x", style="dashed"))),
        Plot("roll", "Roll: truth vs estimate", "roll (°)",
             "A gap here means the drone believes it's level when it isn't (or the other way round).",
             (Series("roll_rad", "true roll", scale=RAD_TO_DEG), Series("roll_est_rad", "estimated roll", scale=RAD_TO_DEG, style="dashed"))),
    )  # fmt: skip
    return r


def summarise(p: dict, r: RunResult) -> list[str]:
    m = r.metrics
    lines = []
    if p["noise_scale"] == 0 and p["gyro_bias_deg_s"] == 0:
        lines.append("Perfect sensors: the estimate matches the truth, so the controller flies exactly as in the earlier experiments.")
    else:
        raw, est = m["raw_altimeter_error_m"], m["est_altitude_error_m"]
        if est < raw:
            lines.append(
                f"The altimeter was off by {raw * 100:.0f} cm (RMS) per reading, but the estimate only by {est * 100:.0f} cm: "
                f"blending it with the accelerometer removed about {(1 - est / raw) * 100:.0f} % of the noise."
            )
        else:
            lines.append(
                f"The estimate ({est * 100:.0f} cm RMS error) is no better than the raw altimeter ({raw * 100:.0f} cm): "
                "it trusts the noisy readings too much."
            )
        lines.append(
            f"The controller can only steer what it believes. True height wobbled {m['true_altitude_error_m'] * 100:.0f} cm "
            f"and true position {m['true_position_error_m'] * 100:.0f} cm (RMS) around the target."
        )
    if p["baro_trust"] >= 0.5:
        lines.append("High altitude trust: the estimate jumps with every noisy reading, and the drone chases that noise up and down.")
    elif p["baro_trust"] <= 0.03:
        lines.append(
            "Very low altitude trust: the estimate is smooth but leans on integrating the accelerometer, "
            "which drifts. Corrections arrive too slowly to stop it."
        )
    if p["gyro_bias_deg_s"] != 0:
        expected = abs(p["gyro_bias_deg_s"]) / p["tilt_correction"] if p["tilt_correction"] > 0 else float("inf")
        lines.append(
            f"The gyro bias ({p['gyro_bias_deg_s']:.1f} °/s) makes the roll estimate drift; the accelerometer pulls it back. "
            f"The balance leaves an average roll error of {abs(m['mean_roll_error_deg']):.1f}° "
            + (f"(theory: bias ÷ tilt correction ≈ {expected:.1f}°). " if expected != float("inf") else "(no correction at all, so it keeps growing). ")
            + "The drone really tilts to 'fix' a tilt it doesn't have, so it drifts sideways."
        )
    if p["tilt_correction"] >= 0.6:
        lines.append(
            "Strong tilt correction trusts the accelerometer a lot. But while a drone accelerates, its accelerometer "
            "mostly feels its own thrust and 'lies' about which way is down. Watch the roll gap grow."
        )
    return lines


EXPERIMENT = Experiment(
    id="sensor_noise",
    number=6,
    title="Sensor noise (truth vs estimate)",
    objective="See the gap between what's really happening and what the drone believes, and how filtering helps.",
    concept=(
        "The controller never sees the truth, only noisy, biased, slow sensor readings.",
        "An estimator blends fast-but-drifting sensors (gyro, accelerometer) with slow-but-absolute ones (altimeter, GPS).",
        "'Trust' decides the blend: too high copies the noise, too low lets drift creep in.",
    ),
    params=(
        Param("noise_scale", "Noise level", "×", 1.0, "1 = typical hobby sensors, 0 = perfect, 3 = poor.", 0.0, 3.0, 0.25),
        Param("baro_trust", "Altitude trust", "", 0.1, "How much each altimeter reading moves the estimate (0–1).", 0.02, 1.0, 0.02),
        Param("gyro_bias_deg_s", "Gyro bias (roll)", "°/s", 0.0, "A constant error in the roll gyro.", 0.0, 3.0, 0.25),
        Param("tilt_correction", "Tilt correction", "1/s", 0.2, "How strongly the accelerometer corrects the tilt estimate.", 0.0, 1.5, 0.05),
    ),
    key_metrics=(
        Metric("raw_altimeter_error_m", "Altimeter error (raw)", "m", 3),
        Metric("est_altitude_error_m", "Altitude estimate error", "m", 3),
        Metric("est_position_error_m", "Position estimate error", "m", 3),
        Metric("est_roll_error_deg", "Roll estimate error", "°", 2),
        Metric("true_altitude_error_m", "True altitude wobble", "m", 3),
        Metric("true_position_error_m", "True position wander", "m", 3),
    ),
    learn_slug="08-exp-sensor-noise",
    runner=run,
)
