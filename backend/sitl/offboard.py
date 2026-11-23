"""Offboard control of REAL ArduPilot: stream DroneLab's own trajectory to ArduCopter SITL, and compare.

"Offboard" = your code (a companion computer, a ground station) decides where the drone goes, many times a second,
and the autopilot just flies it. This is how companies put custom autonomy on top of ArduPilot and PX4.

ArduCopter GUIDED mode accepts SET_POSITION_TARGET_LOCAL_NED with position, velocity AND acceleration (checked in
ArduCopter 4.7 source: GCS_MAVLink_Copter.cpp -> ModeGuided::set_pos_vel_accel_NED_m). Older docs say acceleration
is ignored; that's out of date. The setpoint must keep coming: after GUID_TIMEOUT (3 s) without one it stops.

The same rings track is flown three ways, one after another on one SITL:
  auto        ArduPilot's own planner: the ring centres uploaded as an AUTO mission (it rounds the corners with
              its S-curve planner, limited by WP_SPD / WP_ACC; it knows nothing about the rings' direction).
  guided_pos  Our minimum-snap trajectory, streamed as POSITIONS only (25 Hz). ArduPilot has to chase them.
  guided_pva  The same trajectory streamed as position + velocity + acceleration: feedforward, like experiment 11.

Everything is measured on SITL's ground truth (SIM_STATE): rings flown through, lap time, distance from the plan.

    .venv/bin/python -m backend.sitl.offboard                          # figure eight, 30° tilt, all three
    .venv/bin/python -m backend.sitl.offboard --track square_loop --tilt 20
    .venv/bin/python -m backend.sitl.offboard --write                  # also docs/results/agile-compare.*
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Callable

import numpy as np

from backend.autonomy.agile import METHOD_NAMES, distance_to_path_m, plan_track, run_track
from backend.games.race import TRACKS

from .fly import Flight, enu_to_ned, offset_latlon

AP_METHODS = ("guided_pva", "guided_pos", "auto")
AP_NAMES = {"auto": "ArduPilot AUTO mission (its own planner)",
            "guided_pos": "GUIDED, our trajectory as positions",
            "guided_pva": "GUIDED, our trajectory as pos + vel + accel"}
STREAM_HZ = 25.0
MSG_SIM_STATE, MSG_GLOBAL_POSITION_INT = 108, 33
# type_mask bits: 0-2 position, 3-5 velocity, 6-8 acceleration, 9 force, 10 yaw, 11 yaw rate (1 = IGNORE)
MASK_POS_VEL_ACC = 0b0000_1100_0000_0000  # use position, velocity and acceleration; ignore yaw and yaw rate
MASK_POS_ONLY = 0b0000_1111_1111_1000
RESULTS = Path(__file__).resolve().parents[2] / "docs" / "results"


def ap_limits(plan) -> dict[str, float]:
    """ArduPilot's own limits, opened up to fit the plan (with room for corrections on top).

    Found the hard way: ArduPilot shapes every GUIDED target with a jerk limit (PSC_NE_JERK, default 5 m/s^3). Stream
    a plan with more jerk than that (our 30° figure eight peaks at ~8.5 m/s^3) and its input shaper falls behind and
    overshoots by metres. The autopilot's limits must be at least the plan's.
    """
    peaks = plan.trajectory.peaks()
    jerk = max(5.0, round(1.5 * peaks["jerk_m_s3"], 1))
    return {"ATC_ANGLE_MAX": plan.limits.tilt_deg + 10.0, "PSC_NE_JERK": jerk, "PSC_D_JERK": jerk,
            "WP_ACC": round(plan.limits.accel_m_s2, 2)}


def send_pva(f: Flight, p: np.ndarray, v: np.ndarray | None, a: np.ndarray | None) -> None:
    """One GUIDED setpoint, given in DroneLab's ENU (metres from home) and sent in ArduPilot's NED."""
    n, e, d = enu_to_ned(*p)
    if v is None:
        vn = ve = vd = an = ae = ad = 0.0
        mask = MASK_POS_ONLY
    else:
        vn, ve, vd = enu_to_ned(*v)
        an, ae, ad = enu_to_ned(*a)
        mask = MASK_POS_VEL_ACC
    f.send(f.m.mav.set_position_target_local_ned_send, 0, f.m.target_system, f.m.target_component,
           f.mav.MAV_FRAME_LOCAL_NED, mask, n, e, d, vn, ve, vd, an, ae, ad, 0, 0)


class Recorder:
    """Samples SITL's truth in the trajectory loop and judges the rings (each once the plan heads for it)."""

    def __init__(self, f: Flight, rings, offset: np.ndarray):
        self.f, self.rings, self.offset = f, rings, offset  # offset: track coords = home-relative ENU + offset
        self.samples: list[list[float]] = []  # t, x, y, z, speed, tilt_deg (track coords)
        self.ring_times: list[float | None] = [None] * len(rings)
        self._prev: np.ndarray | None = None

    def sample(self, t: float, heading_for: Callable[[int], bool]) -> np.ndarray | None:
        with self.f.lock:
            tr = self.f.truth_enu3()
        if tr is None:
            return None
        p = np.array(tr[:3]) + self.offset
        if self._prev is not None and np.linalg.norm(p - self._prev) > 1e-6:
            for i, ring in enumerate(self.rings):
                if self.ring_times[i] is None and heading_for(i) and ring.passed(self._prev, p):
                    self.ring_times[i] = round(t, 2)
            self.samples.append([round(t, 3), *(round(float(x), 3) for x in p),
                                 round(float(np.linalg.norm(tr[3:6])), 2), round(math.degrees(tr[6]), 1)])
        self._prev = p
        return p


def _hold_at(f: Flight, p: np.ndarray, say, tol_m: float = 0.4, settle_s: float = 2.0, timeout_s: float = 60.0) -> None:
    """Fly to a point (GUIDED position) and wait until the drone is still there."""
    deadline = time.monotonic() + timeout_s * f.k
    still_since = None
    while True:
        send_pva(f, p, None, None)
        with f.lock:
            tr = f.truth_enu3()
        if tr is not None:
            close = math.dist(tr[:3], p) < tol_m and math.hypot(*tr[3:6]) < 0.2
            still_since = (still_since or f.t_s) if close else None
            if still_since is not None and f.t_s - still_since >= settle_s:
                return
        if time.monotonic() > deadline:
            raise TimeoutError("Couldn't get back to the start point")
        f.sleep(0.1)


def fly_lap(f: Flight, track_id: str, method: str, tilt_deg: float, speedup: float, say=print) -> dict:
    """Fly one lap from a hover at the track start (= ArduPilot's home). The drone must already be in GUIDED."""
    plan = plan_track(track_id, "geometric", 12.0, tilt_deg=tilt_deg)
    offset = plan.points[0].copy()
    offset[2] = 0.0  # track coords = home-relative ENU + (start x, start y, 0)
    local = plan.points - offset
    rec = Recorder(f, plan.rings, offset)
    start = local[0]
    _hold_at(f, start, say)
    tr = plan.trajectory
    if method == "auto":
        f.set_param("WP_SPD", 12.0)
        assert f.home is not None
        pts = [(*offset_latlon(*f.home, p[1], p[0]), float(p[2])) for p in local[1:]]
        f.upload_mission(pts)
        with f.lock:
            f.reached.clear()
        f.set_mode("AUTO")
        t0 = f.t_s
        last_item = len(pts) + 1  # 0 home, 1 take-off, 2.. = points
        deadline = time.monotonic() + 120 * f.k / max(speedup, 1.0)
        while True:
            t = f.t_s - t0
            seq = f.mission_seq
            rec.sample(t, lambda i, s=seq: s >= 2 + i)  # ring i = mission item 2 + i
            with f.lock:
                done = last_item in f.reached
            if done or time.monotonic() > deadline:
                break
            f.sleep(1.0 / STREAM_HZ / max(speedup, 1.0))
        path = plan.points  # what it was asked to fly: straight lines between the ring centres
        f.set_mode("GUIDED")
    else:
        t0 = f.t_s
        knots = tr.knots_s()
        while True:
            t = f.t_s - t0
            p, v, a, _ = tr.sample_one(t)
            if method == "guided_pva":
                send_pva(f, p - offset, v, a)
            else:
                send_pva(f, p - offset, None, None)
            rec.sample(t, lambda i, tt=t: tt >= knots[i] - 0.3)
            if t > tr.duration_s + 2.0:
                break
            f.sleep(1.0 / STREAM_HZ / max(speedup, 1.0))
        path = np.array(tr.polyline(800))
    s = np.array(rec.samples) if rec.samples else np.zeros((0, 6))
    passed = sum(x is not None for x in rec.ring_times)
    lap = max(rec.ring_times) if passed == len(plan.rings) else None
    flying = s[s[:, 0] <= (lap if lap is not None else s[-1, 0])] if len(s) else s
    path_err = distance_to_path_m(flying[:, 1:4], path) if len(flying) else np.zeros(1)
    track_err = None
    if tr is not None and method != "auto" and len(flying):
        ref = np.array([tr.sample_one(t)[0] for t in flying[:, 0]])
        track_err = round(float(np.linalg.norm(flying[:, 1:4] - ref, axis=1).mean()), 3)
    res = {
        "method": method,
        "rings_total": len(plan.rings),
        "rings_passed": passed,
        "ring_times_s": rec.ring_times,
        "lap_time_s": lap,
        "path_err_mean_m": round(float(path_err.mean()), 3),
        "path_err_max_m": round(float(path_err.max()), 3),
        "track_err_mean_m": track_err,
        "max_speed_m_s": round(float(flying[:, 4].max()), 2) if len(flying) else 0.0,
        "max_tilt_deg": round(float(flying[:, 5].max()), 1) if len(flying) else 0.0,
        "track": s[:, :4].tolist(),
    }
    lap_txt = f"lap {lap:.1f} s" if lap is not None else "lap not finished"
    say(f"{AP_NAMES[method]}: {passed}/{len(plan.rings)} rings, {lap_txt}, {res['path_err_mean_m']:.2f} m off the path.")
    return res


def compare(track_id: str = "figure_eight", tilt_deg: float = 30.0, speedup: float = 1.0,
            methods: tuple[str, ...] = AP_METHODS, say: Callable[[str], None] = print) -> dict:
    """The same track on DroneLab (three ways) and on a fresh ArduCopter SITL (three ways)."""
    from . import launch

    plan = plan_track(track_id, "geometric", 12.0, tilt_deg=tilt_deg)
    out: dict = {"track": track_id, "tilt_deg": tilt_deg, "plan": plan.to_dict(), "ap_params": ap_limits(plan),
                 "dronelab": {}, "ardupilot": {}}
    for m in ("waypoints", "pid_ff", "geometric"):
        r = run_track(track_id, m, 12.0, tilt_deg=tilt_deg)
        keep = ("rings_total", "rings_passed", "lap_time_s", "path_err_mean_m", "path_err_max_m", "track_err_mean_m",
                "max_speed_m_s", "max_tilt_deg")
        out["dronelab"][m] = {k: r.metrics[k] for k in keep}
        out["dronelab"][m]["track"] = [[t, x, y, z] for t, x, y, z in zip(*(r.telemetry[k][::5] for k in ("t_s", "x_m", "y_m", "z_m")))]
        say(f"DroneLab {METHOD_NAMES[m]}: {r.metrics['rings_passed']}/{r.metrics['rings_total']} rings, lap "
            f"{r.metrics['lap_time_s']} s, {r.metrics['path_err_mean_m']:.2f} m off the path.")
    sitl = launch.start(speedup=speedup)
    try:
        f = Flight(sitl.url, log=lambda _t: None, timeout_scale=max(1.0, 1.0 / speedup))
        try:
            say("ArduPilot: waiting for GPS + EKF...")
            f.wait_for_home(90)
            f.wait_until("EKF position", lambda: f.ekf_ok, 90)
            f.request_rate(MSG_SIM_STATE, STREAM_HZ * 2)
            f.request_rate(MSG_GLOBAL_POSITION_INT, STREAM_HZ)
            for k, v in ap_limits(plan).items():
                f.set_param(k, v)
            f.set_mode("GUIDED")
            f.arm_when_ready()
            f.takeoff(float(plan.points[0][2]))
            for m in methods:
                say(f"ArduPilot: {AP_NAMES[m]}...")
                out["ardupilot"][m] = fly_lap(f, track_id, m, tilt_deg, speedup, say=lambda t: say("ArduPilot " + t))
            f.set_mode("LAND")
            f.wait_until("landed", lambda: not f.armed, 60)
        finally:
            f.close()
    finally:
        sitl.stop()
    return out


def write_results(res: dict) -> Path:
    RESULTS.mkdir(parents=True, exist_ok=True)
    (RESULTS / "agile-compare.json").write_text(json.dumps(res, separators=(",", ":")))
    title = TRACKS[res["track"]]["title"]

    def row(name: str, m: dict) -> str:
        lap = f"{m['lap_time_s']:.1f} s" if m["lap_time_s"] is not None else "not finished"
        te = f"{m['track_err_mean_m']:.2f} m" if m.get("track_err_mean_m") is not None else "–"
        return (f"| {name} | {m['rings_passed']}/{m['rings_total']} | {lap} | {m['path_err_mean_m']:.2f} m "
                f"({m['path_err_max_m']:.2f} max) | {te} | {m['max_speed_m_s']:.1f} m/s | {m['max_tilt_deg']:.0f}° |")

    md = [
        f"# Agile flight: one rings track, six ways ({title}, max tilt {res['tilt_deg']:.0f}°)",
        "",
        f"The plan: a minimum-snap trajectory through {len(res['plan']['rings'])} rings, "
        f"{res['plan']['duration_s']:.1f} s long, top speed {res['plan']['peaks']['speed_m_s']:.1f} m/s. "
        "Calm air. Measured on ground truth (DroneLab physics; ArduPilot SITL's SIM_STATE). Generated by "
        "`python -m backend.sitl.offboard --write`.",
        "",
        "| Who flies it | Rings | Lap | Off the planned path (mean) | Behind the plan (mean) | Top speed | Max tilt |",
        "|---|---|---|---|---|---|---|",
    ]
    for m, v in res["dronelab"].items():
        md.append(row(f"DroneLab: {METHOD_NAMES[m]}", v))
    for m, v in res["ardupilot"].items():
        md.append(row(f"ArduCopter 4.7: {AP_NAMES[m]}", v))
    md += [
        "",
        "- *Off the planned path* = distance from the line it was asked to fly (straight lines for waypoints / AUTO).",
        "- *Behind the plan* = distance from where the trajectory said to be at that moment (trajectory methods only).",
        "- ArduPilot parameters set for these laps (its limits opened up to fit the plan): "
        + ", ".join(f"`{k}` = {v:g}" for k, v in res["ap_params"].items()) + ". Why: ArduPilot smooths every GUIDED target with a jerk limit (PSC_NE_JERK, default 5 m/s³). In our test a "
        "30° square-loop plan (peak jerk 8.4 m/s³) streamed with the defaults diverged: 2.3 m off the path on average, "
        "5.2 m at worst, 1 of 4 rings. A gentler 20° plan (4.2 m/s³) flew fine with the defaults.",
        "- AUTO is ArduPilot's own planner given only the ring centres: it rounds corners its own way and doesn't know "
        "which way each ring faces.",
    ]
    (RESULTS / "agile-compare.md").write_text("\n".join(md) + "\n")
    return RESULTS / "agile-compare.md"


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--track", default="figure_eight", choices=list(TRACKS))
    ap.add_argument("--tilt", type=float, default=30.0, help="max tilt in degrees (sets the plan's acceleration)")
    ap.add_argument("--speedup", type=float, default=1.0)
    ap.add_argument("--methods", default=",".join(AP_METHODS), help="comma list of: " + ", ".join(AP_METHODS))
    ap.add_argument("--write", action="store_true", help="save docs/results/agile-compare.md and .json")
    args = ap.parse_args(argv)
    methods = tuple(m for m in args.methods.split(",") if m)
    bad = [m for m in methods if m not in AP_METHODS]
    if bad:
        ap.error(f"unknown method(s): {', '.join(bad)}")
    res = compare(args.track, args.tilt, args.speedup, methods)
    if args.write:
        print(f"Wrote {write_results(res)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
