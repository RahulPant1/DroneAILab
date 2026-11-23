"""Experiment 12: Kalman filter vs complementary filter. The same flight, the same sensor noise, two estimators."""

from __future__ import annotations

import math

import numpy as np

from backend.autonomy.agile import run_track
from backend.sim.config import get_preset
from backend.sim.control import Setpoint
from backend.sim.environment import EnvConfig
from backend.sim.physics import State
from backend.sim.runner import RunResult, run_headless
from backend.sim.sensors import SensorConfig

from .base import Experiment, Metric, Param, Plot, Series, plots_to_dicts, try_next

SCENARIOS = {
    "figure_eight": "Race the figure eight (long banked turns)",
    "gps_glitch": "Hover; the GPS jumps 5 m",
    "gyro_bias": "Hover with a gyro that reads 3°/s too much",
    "gps_outage": "Hover in wind; the GPS is lost",
}
HOLD_ALT_M = 5.0
EVENT_START_S = 10.0
GLITCH_M = 5.0
GYRO_BIAS_DEG_S = 3.0
HOVER_S = 40.0  # at least; long glitches / outages get 15 s more after they end
WINDY = EnvConfig(wind_speed_m_s=3.0, wind_direction_deg=90.0, gust_std_m_s=0.8)


def _fly(scenario: str, estimator: str, event_s: float, seed: int) -> RunResult:
    sensors = SensorConfig(enabled=True, estimator=estimator,
                           gyro_bias_deg_s=GYRO_BIAS_DEG_S if scenario == "gyro_bias" else 0.0)

    seen = {}

    def extra(sim) -> dict[str, float]:
        e = seen["est"] = sim.estimator
        if e.kind != "ekf":
            return {"sigma2_m": 0.0, "gps_ratio": 0.0, "bias_deg_s": 0.0}
        sx, sy = e.sigmas()["pos_m"][:2]
        return {"sigma2_m": 2.0 * math.hypot(sx, sy), "gps_ratio": min(e.test_ratio["gps_pos"], 50.0),
                "bias_deg_s": math.degrees(e.gyro_bias[0])}

    if scenario == "figure_eight":
        r = run_track("figure_eight", "geometric", 12.0, sensors=sensors, seed=seed, tilt_deg=30.0, extra_channels=extra)
    else:
        def before(sim) -> None:
            inside = EVENT_START_S <= sim.state.time_s < EVENT_START_S + event_s
            if scenario == "gps_glitch":
                sim.sensors.gps_glitch_m = np.array([GLITCH_M, 0.0]) if inside else np.zeros(2)
            elif scenario == "gps_outage":
                sim.sensors.gps_ok = not inside

        cfg = get_preset("basic")
        r = run_headless(cfg, Setpoint(armed=True, alt_m=HOLD_ALT_M, position_control=True),
                         max(HOVER_S, EVENT_START_S + event_s + 15.0), seed,
                         initial_state=State.hovering(cfg, HOLD_ALT_M), run_type="estimators",
                         env=WINDY if scenario == "gps_outage" else None, sensors=sensors, before_step=before,
                         extra_channels=extra)
    if estimator == "ekf":
        r.metrics["gps_resets"] = seen["est"].gps_resets
    T = r.telemetry
    est_err = np.hypot(np.subtract(T["x_est_m"], T["x_m"]), np.subtract(T["y_est_m"], T["y_m"]))
    tilt_err = np.degrees(np.hypot(np.subtract(T["roll_est_rad"], T["roll_rad"]),
                                   np.subtract(T["pitch_est_rad"], T["pitch_rad"])))
    if scenario == "figure_eight":
        off = np.array([np.nan if v is None else v for v in T["track_err_m"]])
    else:
        off = np.hypot(T["x_m"], T["y_m"])  # distance from the hold point (0, 0)
    T["est_err_m"] = [round(float(v), 4) for v in est_err]
    T["tilt_err_deg"] = [round(float(v), 3) for v in tilt_err]
    T["off_m"] = [None if not np.isfinite(v) else round(float(v), 4) for v in off]
    return r


def _stats(r: RunResult, scenario: str, event_s: float) -> dict:
    T = r.telemetry
    t = np.asarray(T["t_s"])
    est = np.asarray(T["est_err_m"])
    tilt = np.asarray(T["tilt_err_deg"])
    off = np.array([np.nan if v is None else v for v in T["off_m"]])
    out = {"est_err_mean_m": round(float(est.mean()), 3), "est_err_max_m": round(float(est.max()), 3),
           "tilt_err_mean_deg": round(float(tilt.mean()), 2), "off_max_m": round(float(np.nanmax(off)), 2),
           "off_mean_m": round(float(np.nanmean(off)), 3)}
    if scenario == "figure_eight":
        out["rings_passed"] = r.metrics["rings_passed"]
        out["rings_total"] = r.metrics["rings_total"]
        out["lap_time_s"] = r.metrics["lap_time_s"]
    if scenario == "gps_outage":
        end = int(np.searchsorted(t, EVENT_START_S + event_s)) - 1
        inside = (t >= EVENT_START_S) & (t <= EVENT_START_S + event_s)
        out["outage_err_max_m"] = round(float(est[inside].max()), 2)
        out["sigma2_at_outage_end_m"] = round(float(T["sigma2_m"][end]), 2)
    return out


def run(params: dict, seed: int) -> RunResult:
    scenario, event_s = params["scenario"], float(params["event_s"])
    ekf = _fly(scenario, "ekf", event_s, seed)
    cf = _fly(scenario, "complementary", event_s, seed)
    # One telemetry table: the EKF flight, plus the complementary flight's numbers (padded if it ran shorter/longer).
    n = len(ekf.telemetry["t_s"])
    for src, dst in (("est_err_m", "cf_est_err_m"), ("tilt_err_deg", "cf_tilt_err_deg"), ("off_m", "cf_off_m"),
                     ("x_m", "cf_x_m"), ("y_m", "cf_y_m")):
        vals = cf.telemetry[src][:n]
        ekf.telemetry[dst] = vals + [None] * (n - len(vals))
    m_ekf, m_cf = _stats(ekf, scenario, event_s), _stats(cf, scenario, event_s)
    health = {}
    ekf.metrics.update({f"ekf_{k}": v for k, v in m_ekf.items()})
    ekf.metrics.update({f"cf_{k}": v for k, v in m_cf.items()})
    ratio = np.asarray(ekf.telemetry["gps_ratio"])
    health["gps_rejected_s"] = round(float((ratio > 1.0).sum()) / 50.0, 1)  # telemetry is 50 Hz
    health["gps_resets"] = ekf.metrics.get("gps_resets", 0)
    health["bias_learnt_deg_s"] = round(float(ekf.telemetry["bias_deg_s"][-1]), 2)
    ekf.metrics.update(health)
    ekf.scenario = {"scenario": scenario, "event_s": event_s, "event_start_s": EVENT_START_S, **ekf.scenario}
    ekf.summary = summarise(scenario, event_s, ekf.metrics)
    ekf.try_next = suggestions(scenario, event_s)
    ekf.plots = plots_to_dicts(*_plots(scenario, ekf))
    return ekf


def _plots(scenario: str, r: RunResult) -> list[Plot]:
    plots = [
        Plot("est_err", "How wrong is the estimate? (horizontal)", "error (m)",
             "Distance between where the drone THINKS it is and where it really is. Same sensor noise for both. The "
             "dashed line is the EKF's own claim: 2 standard deviations, 'I'm within this'. A good filter's error stays "
             "under it most of the time.",
             (Series("est_err_m", "EKF"), Series("cf_est_err_m", "complementary"), Series("sigma2_m", "EKF says ±2σ", style="dashed"))),
        Plot("tilt_err", "How wrong is the tilt estimate?", "error (deg)",
             "Roll/pitch error of the estimate. A tilt error makes the controller push the wrong way, so it shows up as "
             "a position error a moment later.", (Series("tilt_err_deg", "EKF"), Series("cf_tilt_err_deg", "complementary"))),
        Plot("off", "Where the REAL drone ended up" if scenario != "figure_eight" else "Off the planned trajectory",
             "distance (m)",
             "The real drone's distance from where it should be (the hold point, or the plan at that moment). This is "
             "what a bad estimate costs: the controller flies the estimate, not the truth.",
             (Series("off_m", "EKF"), Series("cf_off_m", "complementary"))),
        Plot("gate", "EKF: GPS innovation test ratio", "ratio",
             "Innovation = GPS fix minus the prediction, divided by 5 standard deviations (EKF3's default gate), squared. "
             "Above 1 = the fix is rejected. ArduPilot logs the same number (XKF4.SP).", (Series("gps_ratio", "GPS position"),)),
    ]
    if scenario == "gyro_bias":
        plots.append(Plot("bias", "EKF: the gyro bias it has learnt", "bias (deg/s)",
                          f"The roll gyro really reads {GYRO_BIAS_DEG_S:g}°/s too much. The EKF has the bias in its state and "
                          "learns it, because a wrong bias makes the tilt drift, which makes the GPS velocity disagree.",
                          (Series("bias_deg_s", "learnt bias"),)))
    if scenario == "figure_eight":
        plan = r.scenario["plan"]
        gates = [[g["x_m"], g["y_m"], g["heading_deg"], g["radius_m"], 1 if t is not None else 0]
                 for g, t in zip(plan["rings"], r.scenario["ring_times_s"])]
        plots.insert(0, Plot("map", "Top-down: the plan and the two flights", "north y (m)",
                             "Dashed yellow = the plan. Solid = flown with the EKF; points = flown with the complementary "
                             "filter. Rings: green = flown through by the EKF flight.",
                             (Series("x_m", "EKF"), Series("y_m", "EKF"), Series("cf_x_m", "complementary", style="points"),
                              Series("cf_y_m", "complementary", style="points")), kind="map",
                             extra={"obstacles": [], "waypoints": [plan["points"][0][:2]], "safety_zone_m": 0.0,
                                    "planned_route": [p[:2] for p in plan["path"]], "gates": gates}))
    return plots


def summarise(scenario: str, event_s: float, m: dict) -> list[str]:
    lines = [f"Same flight, same sensor noise. Estimate error: EKF {m['ekf_est_err_mean_m']:.2f} m on average "
             f"({m['ekf_est_err_max_m']:.2f} m worst), complementary {m['cf_est_err_mean_m']:.2f} m "
             f"({m['cf_est_err_max_m']:.2f} m worst). Tilt error: {m['ekf_tilt_err_mean_deg']:.1f}° vs "
             f"{m['cf_tilt_err_mean_deg']:.1f}°."]
    if scenario == "figure_eight":
        lines.append(f"Rings: EKF {m['ekf_rings_passed']}/{m['ekf_rings_total']}, complementary "
                     f"{m['cf_rings_passed']}/{m['cf_rings_total']}. In a long banked turn the accelerometer feels the "
                     "thrust, not gravity, so the complementary filter's 'accelerometer = down' slowly levels the "
                     "estimate while the drone is really leaning. The EKF uses GPS velocity instead: a wrong tilt would "
                     "push the predicted velocity off, the GPS disagrees, and the covariance knows that the fix is "
                     "the tilt.")
    elif scenario == "gps_glitch":
        lines.append(f"The GPS jumped {GLITCH_M:g} m for {event_s:g} s. The complementary filter can't tell a glitch from "
                     f"real motion: it follows, and the drone flies {m['cf_off_max_m']:.1f} m away to 'correct' it.")
        if event_s <= 10.0:
            lines.append(f"The EKF's innovation gate rejected the fixes for the whole glitch ({m['gps_rejected_s']:.1f} s), "
                         f"so the real drone stayed within {m['ekf_off_max_m']:.2f} m of its spot.")
        else:
            lines.append(f"The EKF rejected the fixes for 10 s, then gave in and reset to the GPS: after that long, maybe "
                         "the filter is the one that's wrong (EKF3 waits 10 s too). The catch: when the glitch ended, the "
                         "CORRECT fixes now disagreed by 5 m, so they were rejected too, until the next 10 s reset. In all "
                         f"the gate failed for {m['gps_rejected_s']:.0f} s and the real drone got {m['ekf_off_max_m']:.1f} m "
                         "off. No filter can tell a long, steady glitch from the truth with only one GPS: that's why "
                         "serious drones carry two receivers (or vision).")
    elif scenario == "gyro_bias":
        lines.append(f"The gyro reads {GYRO_BIAS_DEG_S:g}°/s of roll that isn't there. The complementary filter's gentle "
                     "accelerometer pull can only hold the error at bias / gain, which is a big tilt error. The EKF "
                     f"estimates the bias as part of its state: it learnt {m['bias_learnt_deg_s']:.2f}°/s.")
    else:
        lines.append(f"Without GPS for {event_s:g} s, both filters can only dead-reckon, and both drift (EKF "
                     f"up to {m['ekf_outage_err_max_m']:.1f} m off, complementary up to {m['cf_outage_err_max_m']:.1f} m). "
                     "A Kalman filter is no magic here. The difference: the EKF KNOWS. Its ±2σ grew to "
                     f"{m['ekf_sigma2_at_outage_end_m']:.1f} m. An autopilot uses that number to decide when to stop "
                     "trusting its position (ArduPilot's EKF failsafe).")
    return lines


def suggestions(scenario: str, event_s: float) -> list[dict]:
    out = [try_next(SCENARIOS[s], scenario=s) for s in SCENARIOS if s != scenario][:2]
    if scenario == "gps_glitch":
        out.insert(0, try_next("A 15 s glitch: longer than the EKF's 10 s retry.", event_s=15.0) if event_s <= 10
                   else try_next("A short 3 s glitch.", event_s=3.0))
    if scenario == "gps_outage":
        out.insert(0, try_next("A 30 s outage: how far does the ±2σ grow?", event_s=30.0))
    return out[:3]


EXPERIMENT = Experiment(
    id="estimators",
    number=12,
    title="Kalman filter (EKF) vs complementary filter",
    objective="Fly the same flight twice with the same noisy sensors: once on our simple filter, once on an EKF.",
    concept=(
        "A Kalman filter tracks its own uncertainty and works out how much to trust each reading, every step.",
        "It links errors: a GPS velocity reading also corrects the tilt and learns the gyro bias.",
        "Innovation gates reject readings that disagree too much (a GPS glitch), like ArduPilot's EKF3.",
    ),
    params=(
        Param("scenario", "Scenario", "", "figure_eight", "What to fly, and what goes wrong.",
              options=tuple(SCENARIOS.items())),
        Param("event_s", "Glitch / outage length", "s", 5.0, "How long the GPS glitch or GPS outage lasts.", 1.0, 30.0, 1.0),
    ),
    key_metrics=(
        Metric("ekf_est_err_mean_m", "EKF estimate error (mean)", "m", 3),
        Metric("cf_est_err_mean_m", "Complementary estimate error (mean)", "m", 3),
        Metric("ekf_tilt_err_mean_deg", "EKF tilt error (mean)", "deg", 2),
        Metric("cf_tilt_err_mean_deg", "Complementary tilt error (mean)", "deg", 2),
        Metric("ekf_off_max_m", "EKF flight: worst distance off", "m", 2),
        Metric("cf_off_max_m", "Complementary flight: worst distance off", "m", 2),
    ),
    learn_slug="24-kalman-filter",
    runner=run,
)
