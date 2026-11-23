"""Motor vibration and gyro filters on real ArduCopter (SITL): find the noise, set a notch, check it worked.

The workflow a builder follows on a new drone, done in the simulator:
  1. Fly with ArduPilot's batch sampler on (INS_LOG_BAT_MASK = 1, INS_LOG_BAT_OPT = 4: raw gyro before AND after the
     filters, 1 kHz). SITL adds motor vibration at each rotor's spin frequency (SIM_VIB_MOT_MAX, SIM_GYR1_RND).
  2. The log doctor's FFT (backend/logdoctor/spectrum.py) finds the motor-noise peak and recommends a harmonic notch.
  3. Fly again with that notch and compare.

Three flights, each on a fresh SITL (the notch and the batch sampler are set at boot):
  lpf20        INS_GYRO_FILTER = 20 Hz (ArduPilot's default): kills the noise, but delays the gyro.
  lpf80        INS_GYRO_FILTER = 80 Hz: little delay, but the motor noise gets through to the controller.
  lpf80_notch  80 Hz + the recommended notch from the lpf80 flight: little delay AND little noise.

    .venv/bin/python -m backend.sitl.vibration            # about 4 minutes
    .venv/bin/python -m backend.sitl.vibration --write    # also docs/results/vibration.{md,json}
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Callable

import numpy as np

from backend.logdoctor import spectrum as S
from backend.logdoctor.analyze import WANTED, read_log

RESULTS = Path(__file__).resolve().parents[2] / "docs" / "results"
SHAKE = {"SIM_VIB_MOT_MAX": 350, "SIM_GYR1_RND": 20}  # as ArduPilot's own notch autotest
SAMPLER = {"INS_LOG_BAT_MASK": 1, "INS_LOG_BAT_OPT": 4}
CONFIGS = {
    "lpf20": ("Low-pass 20 Hz (default)", {"INS_GYRO_FILTER": 20}),
    "lpf80": ("Low-pass 80 Hz", {"INS_GYRO_FILTER": 80}),
    "lpf80_notch": ("Low-pass 80 Hz + harmonic notch", {"INS_GYRO_FILTER": 80}),
}
HOVER_ALT_M = 5.0
HOVER_S = 15.0
DASH_M = 10.0


def fly(params: dict[str, float], speedup: float = 1.0) -> dict:
    """One flight: take off, hover (for the spectrum), dash 10 m out and back (for tracking), land."""
    from . import launch
    from .fly import Flight
    from .offboard import _hold_at

    before = set((launch.RUN_DIR / "logs").glob("*.BIN"))
    sitl = launch.start(speedup=speedup, extra_params=params)
    ev: dict[str, float] = {}
    try:
        f = Flight(sitl.url, log=lambda _t: None, timeout_scale=max(1.0, 1.0 / speedup))
        boot = lambda: (f.boot_ms0 or 0) / 1000.0 + f.t_s  # noqa: E731
        try:
            f.wait_for_home(90)
            f.wait_until("EKF position", lambda: f.ekf_ok, 90)
            f.request_rate(108, 25)  # SIM_STATE, for _hold_at
            f.set_mode("GUIDED")
            f.arm_when_ready()
            f.takeoff(HOVER_ALT_M)
            home = np.array([0.0, 0.0, HOVER_ALT_M])
            _hold_at(f, home, print)
            ev["hover_start_s"] = boot()
            f.sleep(HOVER_S)
            ev["hover_end_s"] = ev["dash_start_s"] = boot()
            _hold_at(f, home + [0.0, DASH_M, 0.0], print, tol_m=0.6, settle_s=0.5)
            _hold_at(f, home, print, tol_m=0.6, settle_s=0.5)
            ev["dash_end_s"] = boot()
            f.set_mode("LAND")
            f.wait_until("landed", lambda: not f.armed, 90)
        finally:
            f.close()
    finally:
        sitl.stop()
    new = sorted(set((launch.RUN_DIR / "logs").glob("*.BIN")) - before, key=lambda p: p.stat().st_mtime)
    if not new:
        raise RuntimeError("SITL wrote no log")
    return {"log": str(new[-1]), "events": {k: round(v, 2) for k, v in ev.items()}}


def measure(log_path: str, events: dict) -> dict:
    """Spectrum in the hover, attitude tracking in the dash (both from the log)."""
    from pymavlink import mavutil

    L = read_log(log_path)
    # read_log's times start at its first message; the events are autopilot boot times. Find the offset.
    first_us = None
    m = mavutil.mavlink_connection(log_path)
    while first_us is None:
        msg = m.recv_match(type=list(WANTED))
        if msg is None:
            break
        first_us = getattr(msg, "TimeUS", None)
    t0 = (first_us or 0) / 1e6
    a, b = events["hover_start_s"] - t0, events["hover_end_s"] - t0
    spec = S.analyze(L, flying=lambda t: a <= t <= b)
    att = L.get("ATT", [])
    ta = np.array([r["t"] for r in att])
    sel = (ta >= events["dash_start_s"] - t0) & (ta <= events["dash_end_s"] - t0)
    err = np.array([max(abs(r["DesRoll"] - r["Roll"]), abs(r["DesPitch"] - r["Pitch"])) for r in att])[sel]
    return {"spectrum": spec, "att_err_rms_deg": round(float(np.sqrt(np.mean(err**2))), 3) if len(err) else None,
            "att_err_max_deg": round(float(err.max()), 2) if len(err) else None,
            "dash_s": round(events["dash_end_s"] - events["dash_start_s"], 1)}


def compare(speedup: float = 1.0, say: Callable[[str], None] = print) -> dict:
    out: dict = {"shake": SHAKE, "runs": {}}
    notch = None
    for key in ("lpf80", "lpf20", "lpf80_notch"):  # lpf80 first: its spectrum gives the notch
        name, cfg = CONFIGS[key]
        params = {**SHAKE, **SAMPLER, **cfg, **(notch if key == "lpf80_notch" and notch else {})}
        say(f"{name}: flying...")
        rec = fly(params, speedup)
        res = measure(rec["log"], rec["events"])
        spec = res["spectrum"] or {}
        if key == "lpf80":
            notch = spec.get("recommendation")
            if not notch:
                raise RuntimeError("No motor-noise peak found: can't recommend a notch")
            say(f"  log doctor: motor noise at {spec['pre_peak']['freq_hz']:.0f} Hz -> notch "
                + ", ".join(f"{k}={v}" for k, v in notch.items()))
        peak_hz = (spec.get("pre_peak") or {}).get("freq_hz")
        res["filter_cost"] = S.filter_cost(cfg["INS_GYRO_FILTER"], notch if key == "lpf80_notch" else None, 10.0, peak_hz)
        res["params"] = {k: v for k, v in params.items() if k.startswith("INS_")}
        res["log"] = Path(rec["log"]).name
        out["runs"][key] = res
        post = spec.get("post_peak") or {}
        say(f"  after the filters: {post.get('amp_deg_s', 0):.2f} deg/s at the motor frequency, RMS "
            f"{spec.get('post_rms_deg_s', 0):.3f} deg/s; gyro delay at 10 Hz {res['filter_cost']['lag_ms']:.1f} ms; "
            f"attitude error in the dash RMS {res['att_err_rms_deg']:.2f}°")
    out["notch"] = notch
    return out


def tracking_note(runs: dict) -> list[str]:
    rms = [runs[k]["att_err_rms_deg"] for k in ("lpf20", "lpf80", "lpf80_notch")]
    if max(rms) - min(rms) < 0.1:
        return [f"- **Attitude tracking was the same with all three** ({min(rms):.2f}–{max(rms):.2f}° RMS): ArduPilot's default "
                "rate PIDs are gentle enough to work behind 12 ms of delay. The delay you save is room to tune them "
                "sharper (AUTOTUNE does that); we didn't retune here, so it doesn't show yet."]
    best = min(("lpf20", "lpf80", "lpf80_notch"), key=lambda k: runs[k]["att_err_rms_deg"])
    return [f"- Best attitude tracking in the dash: {CONFIGS[best][0]} ({runs[best]['att_err_rms_deg']:.2f}° RMS)."]


def write_results(res: dict) -> Path:
    RESULTS.mkdir(parents=True, exist_ok=True)
    (RESULTS / "vibration.json").write_text(json.dumps(res, separators=(",", ":")))
    runs = res["runs"]
    pre = runs["lpf80"]["spectrum"]["pre_peak"]
    md = [
        "# Motor vibration and gyro filters (ArduCopter 4.7.3 SITL)",
        "",
        f"SITL shakes the gyro at each rotor's spin frequency (`SIM_VIB_MOT_MAX` = {res['shake']['SIM_VIB_MOT_MAX']}, "
        f"`SIM_GYR1_RND` = {res['shake']['SIM_GYR1_RND']}, as ArduPilot's own notch test). The batch sampler logged the "
        "raw gyro at 1 kHz before and after the filters; DroneLab's log doctor found the peak with an FFT and recommended "
        "the notch. Generated by `python -m backend.sitl.vibration --write`.",
        "",
        f"**Motor noise found:** {pre['freq_hz']:.0f} Hz, {pre['above_floor_db']:.0f} dB above the noise floor (before "
        "the filters). **Recommended notch:** " + ", ".join(f"`{k}` = {v}" for k, v in res["notch"].items()) + ".",
        "",
        "| Filters | Noise left at the motor frequency (after filters) | Gyro noise above 30 Hz (RMS, after filters) "
        "| Gyro delay at 10 Hz (theory) | Attitude error in a 10 m dash (RMS / max) |",
        "|---|---|---|---|---|",
    ]
    for key in ("lpf20", "lpf80", "lpf80_notch"):
        r = runs[key]
        sp = r["spectrum"]
        post = sp.get("post_peak")
        left = f"{post['amp_deg_s']:.2f} deg/s" if post else "no peak left"
        md.append(f"| {CONFIGS[key][0]} | {left} | {sp['post_rms_deg_s']:.3f} deg/s | {r['filter_cost']['lag_ms']:.1f} ms "
                  f"({r['filter_cost']['lag_deg']:.0f}°) | {r['att_err_rms_deg']:.2f}° / {r['att_err_max_deg']:.1f}° |")
    md += [
        "",
        "- *Delay* is the phase lag of the low-pass (and notch) at 10 Hz, computed from the filter equations: the "
        "controller sees the drone's rotation that much late. Less delay = the rate PIDs can be tuned sharper.",
        *tracking_note(runs),
        "- The 20 Hz default is safe on almost any drone, but it costs about four times the delay. A notch lets you raise "
        "the low-pass: it removes only a narrow band around the motor frequency (and follows it with the throttle, "
        "`INS_HNTCH_MODE` = 1).",
        "- On a real drone the peak comes from your props and motors; balance props and soft-mount the flight "
        "controller first, then measure, then notch.",
    ]
    (RESULTS / "vibration.md").write_text("\n".join(md) + "\n")
    return RESULTS / "vibration.md"


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--speedup", type=float, default=1.0)
    ap.add_argument("--write", action="store_true", help="save docs/results/vibration.md and .json")
    args = ap.parse_args(argv)
    res = compare(args.speedup)
    if args.write:
        print(f"Wrote {write_results(res)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
