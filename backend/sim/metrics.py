"""Turn a run's telemetry into a few numbers that summarise what happened.

All functions take plain arrays (time in s, altitude in m, ...) so they work the same
on a live run, a saved run, or a test.
"""

from __future__ import annotations

import math

import numpy as np


def altitude_step_metrics(
    t_s: np.ndarray,
    alt_m: np.ndarray,
    target_m: float,
    band_m: float = 0.05,
    hold_start_s: float | None = None,
) -> dict:
    """How well the drone climbed to and held a target altitude (a "step response").

    rise time   : time to go from 10 % to 90 % of the target.
    overshoot   : how far it went ABOVE the target, as % of the target.
    settling    : time after which it stays within ±band of the target for good.
    hold errors : RMS and worst error from `hold_start_s` (default: second half) to the end.
    """
    t_s = np.asarray(t_s, dtype=float)
    alt_m = np.asarray(alt_m, dtype=float)
    peak_m = float(alt_m.max())

    def first_time(mask: np.ndarray) -> float | None:
        idx = np.flatnonzero(mask)
        return float(t_s[idx[0]]) if idx.size else None

    t10 = first_time(alt_m >= 0.1 * target_m)
    t90 = first_time(alt_m >= 0.9 * target_m)
    rise_time_s = (t90 - t10) if (t10 is not None and t90 is not None) else None

    outside = np.abs(alt_m - target_m) > band_m
    if outside[-1]:
        settling_time_s = None  # still not settled at the end of the run
    elif not outside.any():
        settling_time_s = 0.0
    else:
        settling_time_s = float(t_s[np.flatnonzero(outside)[-1] + 1])

    if hold_start_s is None:
        hold_start_s = float(t_s[-1]) / 2.0
    hold = t_s >= hold_start_s
    hold_err = alt_m[hold] - target_m if hold.any() else np.array([math.nan])

    return {
        "target_alt_m": target_m,
        "peak_alt_m": peak_m,
        "final_alt_m": float(alt_m[-1]),
        "rise_time_s": rise_time_s,
        "overshoot_pct": max(0.0, (peak_m - target_m) / target_m * 100.0) if target_m > 0 else 0.0,
        "settling_time_s": settling_time_s,
        "settling_band_m": band_m,
        "hold_start_s": hold_start_s,
        "hold_rms_error_m": float(np.sqrt(np.mean(hold_err**2))),
        "hold_max_error_m": float(np.max(np.abs(hold_err))),
    }


def mean_motor_pct(t_s: np.ndarray, motor_pct: np.ndarray, start_s: float) -> float:
    """Average motor command (mean of the 4 motors) from start_s to the end.
    During a steady hover this is the 'hover throttle'."""
    t_s = np.asarray(t_s)
    window = t_s >= start_s
    if not window.any():
        return math.nan
    return float(np.asarray(motor_pct)[window].mean())


def horizontal_drift_m(x_m: np.ndarray, y_m: np.ndarray) -> float:
    """Straight-line horizontal distance between where the run started and ended."""
    return float(math.hypot(x_m[-1] - x_m[0], y_m[-1] - y_m[0]))


def step_response(
    t_s: np.ndarray,
    y: np.ndarray,
    start_value: float,
    target: float,
    t_step_s: float,
    t_end_s: float | None = None,
    band: float | None = None,
) -> dict:
    """Describe how a signal moved from `start_value` to `target` after a step at `t_step_s`.

    rise time   : 10 % -> 90 % of the way.
    overshoot   : how far past the target it went, as % of the step size.
    settling    : time (after the step) from which it stays within ±band of the target.
    oscillations: how many times it crossed the target (ignoring tiny wiggles).
    final error : average error over the last 20 % of the window.
    """
    t_s, y = np.asarray(t_s, dtype=float), np.asarray(y, dtype=float)
    t_end_s = float(t_s[-1]) if t_end_s is None else t_end_s
    window = (t_s >= t_step_s) & (t_s <= t_end_s)
    tw, yw = t_s[window], y[window]
    step = target - start_value
    if abs(step) < 1e-9 or tw.size < 2:
        return {}
    progress = (yw - start_value) / step  # 0 at the start, 1 at the target
    band = abs(step) * 0.05 if band is None else band

    def first(mask):
        idx = np.flatnonzero(mask)
        return float(tw[idx[0]]) if idx.size else None

    t10, t90 = first(progress >= 0.1), first(progress >= 0.9)
    error = yw - target
    outside = np.abs(error) > band
    if outside[-1]:
        settling = None
    elif not outside.any():
        settling = 0.0
    else:
        settling = float(tw[np.flatnonzero(outside)[-1] + 1] - t_step_s)
    big = error[np.abs(error) > 0.2 * band]
    crossings = int(np.sum(np.diff(np.sign(big)) != 0)) if big.size > 1 else 0
    tail = tw >= tw[0] + 0.8 * (tw[-1] - tw[0])
    return {
        "rise_time_s": (t90 - t10) if (t10 is not None and t90 is not None) else None,
        "overshoot_pct": float(max(0.0, (progress.max() - 1.0) * 100.0)),
        "settling_time_s": settling,
        "settling_band": band,
        "oscillations": crossings,
        "final_error": float(np.mean(error[tail])),
        "peak_value": float(yw[np.argmax(progress)]),
    }
