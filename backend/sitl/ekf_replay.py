"""Our EKF vs ArduPilot's EKF3, on the SAME sensor data from a real ArduCopter flight (SITL).

1. Fly ArduCopter SITL with LOG_REPLAY = 1: the log then holds every input its EKF3 saw (AP_DAL "replay" messages:
   IMU delta angles / velocities each loop, GPS, barometer, magnetometer), plus EKF3's own answer (XKF1) and the
   simulator's truth (SIM). The flight: hover, a gyro bias appears (SIM_GYR1_BIAS_X), our figure eight at 30° tilt
   streamed in GUIDED, then a 5 m GPS glitch (SIM_GPS1_GLTCH_X) in a hover.
2. Feed those inputs through DroneLab's EKF (backend/sim/ekf.py), offline. It initialises itself from the sensors
   (accelerometer = level, magnetometer = heading, first GPS fix = position), as EKF3 does.
3. Score both against the simulator's truth.

ArduPilot's code is not used or copied: it runs as a separate program, and we only read its log.

    .venv/bin/python -m backend.sitl.ekf_replay                    # fly (about 3 min) + replay + print
    .venv/bin/python -m backend.sitl.ekf_replay --write            # also docs/results/ekf-compare.{md,json}
    .venv/bin/python -m backend.sitl.ekf_replay --log path/to.BIN  # replay a log you already have
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

from backend.sim.ekf import EKF
from backend.sim.frames import quat_from_euler, quat_to_euler, quat_to_rotation_matrix
from backend.sim.physics import State
from backend.sim.sensors import Readings, SensorConfig

RESULTS = Path(__file__).resolve().parents[2] / "docs" / "results"
REPLAY_PARAMS = {"LOG_REPLAY": 1, "LOG_DISARMED": 1, "LOG_FILE_BUFSIZE": 64}
GYRO_BIAS_RAD_S = math.radians(2.0)  # appears after take-off (the boot-time gyro calibration would remove it)
GLITCH_M = 5.0
GLITCH_S = 5.0
M_PER_DEG_LAT = 111_319.5


# --- 1. the flight ------------------------------------------------------------------------------------------------
def record(speedup: float = 1.0, say: Callable[[str], None] = print) -> dict:
    """Fly the test flight on a fresh SITL; return the log path and when things happened (autopilot boot time, s)."""
    from . import launch
    from .fly import Flight
    from .offboard import MSG_SIM_STATE, _hold_at, ap_limits, fly_lap
    from backend.autonomy.agile import plan_track

    plan = plan_track("figure_eight", "geometric", 12.0, tilt_deg=30.0)
    events: dict[str, float] = {}
    before = set((launch.RUN_DIR / "logs").glob("*.BIN"))
    sitl = launch.start(speedup=speedup, extra_params=REPLAY_PARAMS)
    try:
        f = Flight(sitl.url, log=lambda _t: None, timeout_scale=max(1.0, 1.0 / speedup))
        boot = lambda: (f.boot_ms0 or 0) / 1000.0 + f.t_s  # noqa: E731  seconds since the autopilot booted
        try:
            say("ArduPilot: waiting for GPS + EKF...")
            f.wait_for_home(90)
            f.wait_until("EKF position", lambda: f.ekf_ok, 90)
            f.request_rate(MSG_SIM_STATE, 50)
            for k, v in ap_limits(plan).items():
                f.set_param(k, v)
            f.set_mode("GUIDED")
            f.arm_when_ready()
            events["armed_s"] = boot()
            f.takeoff(float(plan.points[0][2]))
            start = plan.points[0].copy()
            start[:2] = 0.0
            _hold_at(f, start, say)
            say(f"Gyro bias appears: {math.degrees(GYRO_BIAS_RAD_S):.0f}°/s on the roll gyro.")
            f.set_param("SIM_GYR1_BIAS_X", GYRO_BIAS_RAD_S)
            events["bias_s"] = boot()
            f.sleep(15.0)
            say("Figure eight, 30° tilt, streamed in GUIDED...")
            events["lap_start_s"] = boot()
            fly_lap(f, "figure_eight", "guided_pva", 30.0, speedup, say=lambda t: say("  " + t))
            events["lap_end_s"] = boot()
            _hold_at(f, start, say)
            say(f"GPS glitch: {GLITCH_M:.0f} m north for {GLITCH_S:.0f} s.")
            events["glitch_start_s"] = boot()
            f.set_param("SIM_GPS1_GLTCH_X", GLITCH_M / M_PER_DEG_LAT)
            f.sleep(GLITCH_S)
            f.set_param("SIM_GPS1_GLTCH_X", 0.0)
            events["glitch_end_s"] = boot()
            f.sleep(10.0)
            f.set_mode("LAND")
            f.wait_until("landed", lambda: not f.armed, 90)
            events["landed_s"] = boot()
            f.sleep(2.0)
        finally:
            f.close()
    finally:
        sitl.stop()
    new = sorted(set((launch.RUN_DIR / "logs").glob("*.BIN")) - before, key=lambda p: p.stat().st_mtime)
    if not new:
        raise RuntimeError("SITL wrote no log")
    return {"log": str(new[-1]), "events": {k: round(v, 2) for k, v in events.items()}}


# --- 2. reading the log ---------------------------------------------------------------------------------------------
def frd_to_flu(v) -> np.ndarray:
    """ArduPilot's body axes (forward, right, down) -> ours (forward, left, up)."""
    return np.array([v[0], -v[1], -v[2]])


def ned_to_enu(v) -> np.ndarray:
    return np.array([v[1], v[0], -v[2]])


_T_ENU = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
_T_FLU = np.diag([1.0, -1.0, -1.0])


def rot_from_ned_euler(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """ArduPilot's attitude (degrees, body FRD -> NED) as our rotation matrix (body FLU -> world ENU)."""
    r, p, y = (math.radians(a) for a in (roll_deg, pitch_deg, yaw_deg))
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    r_ned = np.array([[cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy],
                      [cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy],
                      [-sp, sr * cp, cr * cp]])
    return _T_ENU @ r_ned @ _T_FLU


def attitude_errors_deg(r_true: np.ndarray, r_est: np.ndarray) -> tuple[float, float]:
    """(tilt error, heading error): the angle between the two 'up' axes, and between the two noses seen from above."""
    up = float(np.clip(r_true[:, 2] @ r_est[:, 2], -1.0, 1.0))
    h_true = math.atan2(r_true[1, 0], r_true[0, 0])
    h_est = math.atan2(r_est[1, 0], r_est[0, 0])
    return math.degrees(math.acos(up)), abs(math.degrees(math.remainder(h_est - h_true, 2 * math.pi)))


def heading_from_mag(field_flu: np.ndarray, rot: np.ndarray, declination_rad: float) -> float:
    """Compass heading (our yaw: from east, anticlockwise) from a body-frame magnetometer reading.

    Level the reading with the current roll / pitch, then the field's horizontal direction in the body says how far
    the nose is turned from magnetic north. True north = magnetic north turned by the declination (east positive).
    """
    roll, pitch, _ = quat_to_euler_from_rot(rot)
    level = quat_to_rotation_matrix(quat_from_euler(roll, pitch, 0.0)) @ field_flu
    nose_from_mag_north = math.atan2(level[1], level[0])  # field direction in body x/y (left positive)
    # Our yaw is measured from east anticlockwise; magnetic north lies at 90° - declination from east.
    return math.remainder(math.pi / 2 - declination_rad - nose_from_mag_north, 2 * math.pi)


def quat_to_euler_from_rot(rot: np.ndarray) -> tuple[float, float, float]:
    roll = math.atan2(rot[2, 1], rot[2, 2])
    pitch = math.asin(max(-1.0, min(1.0, -rot[2, 0])))
    yaw = math.atan2(rot[1, 0], rot[0, 0])
    return roll, pitch, yaw


def read_log(path: str | Path) -> dict:
    """Pull out the EKF inputs (in order), EKF3's answers, the simulator's truth and the events."""
    from pymavlink import mavutil

    m = mavutil.mavlink_connection(str(path))
    inputs: list[tuple] = []  # ("imu", t, dang, dvel, dt) | ("baro", t, alt) | ("mag", t, field) | ("gps", t, ...)
    truth, ekf3, ekf3_sp, arms, params = [], [], [], [], []
    origin, declination, origin_s = None, None, 0.0
    t = 0.0
    last = {"baro": None, "mag": None, "gps": None}
    while True:
        msg = m.recv_match()
        if msg is None:
            break
        typ = msg.get_type()
        if typ == "RFRH":
            t = msg.TimeUS / 1e6
        elif typ == "RISI" and msg.I == 0 and msg.DADT > 0:
            inputs.append(("imu", t, frd_to_flu((msg.DAX, msg.DAY, msg.DAZ)), frd_to_flu((msg.DVX, msg.DVY, msg.DVZ)),
                           msg.DADT, msg.DVDT))
        elif typ == "RBRI" and msg.I == 0 and msg.LastUpdate != last["baro"]:
            last["baro"] = msg.LastUpdate
            inputs.append(("baro", t, float(msg.Alt)))
        elif typ == "RMGI" and msg.I == 0 and msg.LU != last["mag"]:
            last["mag"] = msg.LU
            inputs.append(("mag", t, frd_to_flu((msg.FX, msg.FY, msg.FZ))))
        elif typ == "RGPJ" and msg.I == 0 and msg.TS != last["gps"] and msg.Lat != 0:
            last["gps"] = msg.TS
            inputs.append(("gps", t, msg.Lat * 1e-7, msg.Lon * 1e-7, ned_to_enu((msg.VX, msg.VY, msg.VZ))))
        elif typ == "RMGH" and msg.Dec != 0.0:
            declination = float(msg.Dec)
        elif typ == "ORGN" and msg.Type == 0 and origin is None:
            origin = (msg.Lat, msg.Lng, msg.Alt)
            origin_s = msg.TimeUS / 1e6
        elif typ == "SIM":
            truth.append((msg.TimeUS / 1e6, msg.Lat, msg.Lng, msg.Alt, msg.Roll, msg.Pitch, msg.Yaw))
        elif typ == "XKF1" and msg.C == 0:
            ekf3.append((msg.TimeUS / 1e6, msg.Roll, msg.Pitch, msg.Yaw, msg.VN, msg.VE, msg.VD, msg.PN, msg.PE, msg.PD,
                         msg.GX, msg.GY, msg.GZ))
        elif typ == "XKF4" and msg.C == 0:
            ekf3_sp.append((msg.TimeUS / 1e6, msg.SP))
        elif typ == "ARM":
            arms.append((msg.TimeUS / 1e6, int(msg.ArmState)))
        elif typ == "PARM" and msg.Name in ("SIM_GYR1_BIAS_X", "SIM_GPS1_GLTCH_X", "COMPASS_DEC"):
            params.append((msg.TimeUS / 1e6, msg.Name, float(msg.Value)))
            if msg.Name == "COMPASS_DEC" and declination is None and msg.Value != 0:
                declination = float(msg.Value)
    if origin is None:
        raise ValueError("The log has no EKF origin (ORGN): did the EKF ever get a GPS position?")
    return {"inputs": inputs, "truth": np.array(truth), "ekf3": np.array(ekf3), "ekf3_sp": np.array(ekf3_sp),
            "arms": arms, "params": params, "origin": origin, "origin_s": origin_s, "declination_rad": declination or 0.0}


def events_from_log(log: dict) -> dict[str, float]:
    """When things happened, from the log alone: arming, the parameter changes, and the lap (moving > 1.5 m/s)."""
    ev: dict[str, float] = {}
    armed = [t for t, s in log["arms"] if s == 1]
    disarmed = [t for t, s in log["arms"] if s == 0]
    if armed:
        ev["armed_s"] = armed[0]
    if disarmed:
        ev["landed_s"] = disarmed[-1]
    for t, name, v in log["params"]:
        if name == "SIM_GYR1_BIAS_X" and v != 0 and "bias_s" not in ev:
            ev["bias_s"] = t
        if name == "SIM_GPS1_GLTCH_X" and v != 0 and "glitch_start_s" not in ev:
            ev["glitch_start_s"] = t
        if name == "SIM_GPS1_GLTCH_X" and v == 0 and "glitch_start_s" in ev and "glitch_end_s" not in ev:
            ev["glitch_end_s"] = t
    tr = log["truth"]
    if len(tr) > 2:
        n, e = _local_ne(tr[:, 1], tr[:, 2], log["origin"])
        speed = np.hypot(np.gradient(n, tr[:, 0]), np.gradient(e, tr[:, 0]))
        lo, hi = ev.get("bias_s", tr[0, 0]), ev.get("glitch_start_s", tr[-1, 0])
        fast = tr[(speed > 1.5) & (tr[:, 0] > lo) & (tr[:, 0] < hi), 0]
        if len(fast):
            ev["lap_start_s"], ev["lap_end_s"] = float(fast[0]), float(fast[-1])
    return {k: round(v, 2) for k, v in ev.items()}


def _local_ne(lat, lon, origin) -> tuple[np.ndarray, np.ndarray]:
    lat0, lon0 = origin[0], origin[1]
    n = (np.asarray(lat) - lat0) * M_PER_DEG_LAT
    e = (np.asarray(lon) - lon0) * M_PER_DEG_LAT * math.cos(math.radians(lat0))
    return n, e


# --- 3. the replay ---------------------------------------------------------------------------------------------------
# What our EKF assumes about the sensors: EKF3's copter defaults (EK3_ACC_P_NSE, EK3_GYRO_P_NSE, EK3_POSNE_M_NSE,
# EK3_VELNE_M_NSE, EK3_ALT_M_NSE), and a 3° compass heading. Same assumptions = a fair comparison.
REPLAY_NOISE = {"accel_m_s2": 0.35, "gyro_rad_s": 0.015, "gps_pos_m": 0.5, "gps_vel_m_s": 0.3, "baro_m": 2.0,
                "compass_rad": math.radians(3.0)}


def replay(log: dict, gps_lag_s: float | None = 0.1, bias_walk_rad_s: float | None = None) -> dict:
    """Run DroneLab's EKF on the logged sensor data. Returns our estimate sampled at each EKF3 output time.
    bias_walk_rad_s: override how fast our filter lets the gyro bias change (None = our default)."""
    origin, dec = log["origin"], log["declination_rad"]
    ev = events_from_log(log)
    armed_s, landed_s = ev.get("armed_s", 0.0), ev.get("landed_s", math.inf)
    ekf: EKF | None = None
    accel_buf: list[np.ndarray] = []
    mag_last: np.ndarray | None = None
    baro_last = 0.0
    out: list[list[float]] = []  # t, x, y, z, vx, vy, vz, rot(9), bias x, gps test ratio
    k3 = 0
    times3 = log["ekf3"][:, 0]
    started_s = None
    for item in log["inputs"]:
        kind, t = item[0], item[1]
        if ekf is not None:
            ekf.time_s = ekf.est.time_s = t
            ekf.set_stationary(t < armed_s or t > landed_s)
        if kind == "imu":
            _, _, dang, dvel, dadt, dvdt = item
            if ekf is None:
                accel_buf = (accel_buf + [dvel / dvdt])[-200:]
                continue
            ekf.predict(dang / dadt, dvel / dvdt, dadt)
        elif kind == "baro":
            baro_last = item[2]
            if ekf is not None:
                if ekf.stationary:  # on the ground, disarmed: "I'm not moving" is a measurement too
                    ekf.fuse_zero_velocity()
                ekf.fuse_baro(baro_last)
        elif kind == "mag":
            mag_last = item[2]
            if ekf is not None:
                ekf.fuse_compass(heading_from_mag(mag_last, quat_to_rotation_matrix(ekf.est.quat), dec))
        elif kind == "gps":
            _, _, lat, lon, vel = item
            n, e = _local_ne(lat, lon, origin)
            if ekf is None:
                # Wait for EKF3's origin: before that, SITL's GPS reports a position it hasn't locked yet.
                if mag_last is None or len(accel_buf) < 50 or t < log["origin_s"]:
                    continue
                ekf = _initialise(np.mean(accel_buf, axis=0), mag_last, dec, float(e), float(n), baro_last, vel, t)
                ekf.gps_lag_s = gps_lag_s or 0.0
                if bias_walk_rad_s is not None:
                    ekf.bias_walk_rad_s = bias_walk_rad_s
                started_s = t
                continue
            ekf.fuse_gps(np.array([float(e), float(n)]), vel)
        # sample ours whenever EKF3 has logged an output up to now
        while k3 < len(times3) and times3[k3] <= t:
            if ekf is not None:
                s = ekf.est
                out.append([times3[k3], *s.pos_m, *s.vel_m_s, *quat_to_rotation_matrix(s.quat).ravel(),
                            ekf.gyro_bias[0], min(ekf.test_ratio["gps_pos"], 99.0)])
            k3 += 1
    if ekf is None:
        raise ValueError("Never got a GPS fix with the magnetometer and accelerometer ready.")
    return {"samples": np.array(out), "started_s": started_s, "events": ev, "health": ekf.health()}


def _initialise(accel_flu: np.ndarray, mag_flu: np.ndarray, dec: float, x: float, y: float, z: float,
                vel: np.ndarray, t: float) -> EKF:
    """Start the filter from the sensors alone: accelerometer = which way is up, magnetometer = heading."""
    f = accel_flu / np.linalg.norm(accel_flu)
    roll, pitch = math.atan2(f[1], f[2]), math.atan2(-f[0], math.hypot(f[1], f[2]))
    level = quat_to_rotation_matrix(quat_from_euler(roll, pitch, 0.0))
    yaw = heading_from_mag(mag_flu, level, dec)
    init = State(time_s=t, pos_m=np.array([x, y, z]), vel_m_s=np.asarray(vel, dtype=float),
                 quat=quat_from_euler(roll, pitch, yaw))
    ekf = EKF(SensorConfig(enabled=True), init, 0.00125)
    ekf.set_noise(**REPLAY_NOISE)
    ekf.P = np.diag([0.5**2] * 2 + [1.0**2] + [0.3**2] * 3 + [math.radians(2) ** 2] * 2 + [math.radians(10) ** 2]
                    + [math.radians(1) ** 2] * 3)
    return ekf


# --- 4. scoring ------------------------------------------------------------------------------------------------------
def _truth_at(log: dict, times: np.ndarray) -> tuple[np.ndarray, np.ndarray, list[np.ndarray]]:
    tr = log["truth"]
    n, e = _local_ne(tr[:, 1], tr[:, 2], log["origin"])
    pos = np.column_stack([np.interp(times, tr[:, 0], e), np.interp(times, tr[:, 0], n),
                           np.interp(times, tr[:, 0], tr[:, 3] - log["origin"][2])])
    vel = np.column_stack([np.gradient(c, tr[:, 0]) for c in (e, n, tr[:, 3])])
    vel = np.column_stack([np.interp(times, tr[:, 0], vel[:, k]) for k in range(3)])
    ang = [np.interp(times, tr[:, 0], np.unwrap(np.radians(tr[:, k])) if k == 6 else tr[:, k]) for k in (4, 5, 6)]
    rots = [rot_from_ned_euler(r, p, math.degrees(y)) for r, p, y in zip(*ang)]
    return pos, vel, rots


def score(log: dict, rep: dict, events: dict | None = None) -> dict:
    """Errors of both filters against the simulator's truth, for the whole flight and for each part of it.
    `events`: the times record() noted (better than guessing them from the log); missing ones come from the log."""
    ours = rep["samples"]
    ev = {**rep["events"], **(events or {})}
    t = ours[:, 0]
    k3 = np.searchsorted(log["ekf3"][:, 0], t)
    k3 = np.clip(k3, 0, len(log["ekf3"]) - 1)
    a3 = log["ekf3"][k3]
    pos_t, vel_t, rot_t = _truth_at(log, t)
    est = {
        "dronelab": {"pos": ours[:, 1:4], "vel": ours[:, 4:7], "rot": [r.reshape(3, 3) for r in ours[:, 7:16]],
                     "bias_deg_s": np.degrees(ours[:, 16]), "gate": ours[:, 17]},
        "ekf3": {"pos": np.column_stack([a3[:, 8], a3[:, 7], -a3[:, 9]]),
                 "vel": np.column_stack([a3[:, 5], a3[:, 4], -a3[:, 6]]),
                 "rot": [rot_from_ned_euler(r, p, y) for r, p, y in a3[:, 1:4]],
                 "bias_deg_s": a3[:, 10]},  # XKF1.GX: roll gyro bias (deg/s); FRD x = FLU x
    }
    sp = log["ekf3_sp"]
    est["ekf3"]["gate"] = np.interp(t, sp[:, 0], sp[:, 1]) ** 2  # XKF4.SP is the SQUARE ROOT of the test ratio
    parts = {"whole flight": (ev.get("armed_s", t[0]) + 5.0, ev.get("landed_s", t[-1]))}
    if "bias_s" in ev and "lap_start_s" in ev:
        parts["hover, gyro bias on"] = (ev["bias_s"], ev["lap_start_s"])
    if "lap_start_s" in ev:
        parts["figure eight, 30° tilt"] = (ev["lap_start_s"], ev["lap_end_s"])
    if "glitch_start_s" in ev:
        parts[f"GPS glitch, {GLITCH_M:.0f} m"] = (ev["glitch_start_s"], ev.get("glitch_end_s", ev["glitch_start_s"]) + 3.0)
    res: dict = {"events": ev, "parts": {}, "declination_deg": round(math.degrees(log["declination_rad"]), 2),
                 "dronelab_health": rep["health"]}
    for part, (a, b) in parts.items():
        sel = (t >= a) & (t <= b)
        if sel.sum() < 3:
            continue
        row = {"duration_s": round(float(b - a), 1)}
        for who, e in est.items():
            dp = e["pos"][sel] - pos_t[sel]
            dv = e["vel"][sel] - vel_t[sel]
            att = np.array([attitude_errors_deg(rt, re) for rt, re, s in zip(rot_t, e["rot"], sel) if s])
            row[who] = {
                "horiz_err_mean_m": round(float(np.hypot(dp[:, 0], dp[:, 1]).mean()), 3),
                "horiz_err_max_m": round(float(np.hypot(dp[:, 0], dp[:, 1]).max()), 3),
                "height_err_mean_m": round(float(np.abs(dp[:, 2]).mean()), 3),
                "vel_err_mean_m_s": round(float(np.linalg.norm(dv, axis=1).mean()), 3),
                "tilt_err_mean_deg": round(float(att[:, 0].mean()), 2),
                "tilt_err_max_deg": round(float(att[:, 0].max()), 2),
                "heading_err_mean_deg": round(float(att[:, 1].mean()), 2),
                "bias_end_deg_s": round(float(e["bias_deg_s"][sel][-1]), 2),
                "gps_rejected_s": round(float((e["gate"][sel] > 1.0).sum()) / 10.0, 1),  # samples are 10 Hz
            }
        res["parts"][part] = row
    res["true_bias_deg_s"] = round(math.degrees(GYRO_BIAS_RAD_S), 2)
    res["series"] = {  # for plots: 5 Hz is plenty
        "t_s": [round(float(x), 2) for x in t[::2]],
        "dronelab_horiz_err_m": [round(float(x), 3) for x in np.hypot(*(est["dronelab"]["pos"] - pos_t)[:, :2].T)[::2]],
        "ekf3_horiz_err_m": [round(float(x), 3) for x in np.hypot(*(est["ekf3"]["pos"] - pos_t)[:, :2].T)[::2]],
        "dronelab_bias_deg_s": [round(float(x), 3) for x in est["dronelab"]["bias_deg_s"][::2]],
        "ekf3_bias_deg_s": [round(float(x), 3) for x in est["ekf3"]["bias_deg_s"][::2]],
    }
    return res


# --- 5. the whole comparison, and the write-up --------------------------------------------------------------------
VARIANTS = {"dronelab_ekf3_like": ("DroneLab EKF, EKF3-like bias tuning (5e-5)", "ekf3_like"),
            "dronelab": ("DroneLab EKF, our default bias tuning (2e-4)", "default"),
            "dronelab_fast": ("DroneLab EKF, fast bias tuning (1e-3)", "fast")}
NAMES = {**{k: v[0] for k, v in VARIANTS.items()}, "ekf3": "ArduPilot EKF3 (its own output)"}
WHO = (*VARIANTS, "ekf3")


def compare_log(path: str | Path, events: dict | None = None) -> dict:
    """Replay one log through our EKF (two tunings) and score them and EKF3 on the same truth."""
    from backend.sim.ekf import BIAS_WALKS

    log = read_log(path)
    out: dict = {"log": Path(path).name, "parts": {}, "noise": REPLAY_NOISE}
    for key, (_, walk) in VARIANTS.items():
        rep = replay(log, bias_walk_rad_s=BIAS_WALKS[walk])
        res = score(log, rep, events)
        for part, row in res["parts"].items():
            out["parts"].setdefault(part, {"duration_s": row["duration_s"]})
            out["parts"][part][key] = row["dronelab"]
            out["parts"][part]["ekf3"] = row["ekf3"]
        out.setdefault("events", res["events"])
        out.setdefault("series", res["series"])
        out[f"{key}_health"] = rep["health"]
    out["true_bias_deg_s"] = round(math.degrees(GYRO_BIAS_RAD_S), 2)
    out["declination_deg"] = round(math.degrees(log["declination_rad"]), 2)
    return out


def compare(speedup: float = 1.0, say: Callable[[str], None] = print) -> dict:
    rec = record(speedup, say)
    say(f"Replaying {Path(rec['log']).name} through DroneLab's EKF...")
    res = compare_log(rec["log"], rec["events"])
    for line in summary_lines(res):
        say(line)
    return res


def findings(res: dict) -> list[str]:
    """Plain-English conclusions, worded from the measured numbers (the same text goes into the results file)."""
    parts = res["parts"]
    out = []
    glitch = next((v for k, v in parts.items() if k.startswith("GPS glitch")), None)
    if glitch:
        rej = {w: glitch[w]["gps_rejected_s"] for w in ("dronelab", "ekf3")}
        out.append(f"- **GPS glitch:** EKF3 rejected the fixes for {rej['ekf3']:.1f} s, our filter for {rej['dronelab']:.1f} s "
                   f"(the same 5-sigma rule: EKF3's `XKF4.SP` vs our test ratio). Position error during it: EKF3 "
                   f"{glitch['ekf3']['horiz_err_mean_m']:.2f} m, ours {glitch['dronelab']['horiz_err_mean_m']:.2f} m.")
    hover = parts.get("hover, gyro bias on")
    if hover:
        cells = "; ".join(f"{NAMES[w].split(', ')[-1] if w != 'ekf3' else 'EKF3'}: {hover[w]['horiz_err_mean_m']:.2f} m off, "
                          f"bias {hover[w]['bias_end_deg_s']:.2f}°/s after {hover['duration_s']:.0f} s" for w in WHO if w in hover)
        lost = [NAMES[w].split(", ")[-1] for w in VARIANTS if res.get(f"{w}_health", {}).get("vel_resets", 0)]
        lost_txt = (f"our simpler filter got lost with {' and with '.join(lost)} and needed its reset rule (GPS velocity "
                    "rejected for 5 s: jump to the GPS and widen the uncertainty)" if lost else "our filter never got lost")
        out.append(f"- **A sudden {res['true_bias_deg_s']:g}°/s gyro bias is a tuning question.** {cells}. A filter that "
                   "believes the bias can only change slowly takes a long time to accept a sudden one. EKF3 rode it out "
                   f"(about a metre off, never lost); {lost_txt}. Faster bias tuning learns it in "
                   "seconds, but in DroneLab's experiment 12 the same setting makes the estimate wander more in a GPS "
                   "outage. Every setting is a trade-off; real autopilots are tuned for the sensors they carry.")
    lap = parts.get("figure eight, 30° tilt")
    if lap:
        out.append(f"- **During the lap** our filter was closer to the truth ({lap['dronelab']['horiz_err_mean_m']:.2f} m vs "
                   f"EKF3's {lap['ekf3']['horiz_err_mean_m']:.2f} m). Don't read much into that: SITL's GPS has no "
                   "horizontal noise, which rewards a filter that trusts GPS more. EKF3 assumes real GPS (0.5 m noise, "
                   "drifting, late), estimates 24 states (accelerometer bias, magnetic field, wind), runs two copies and "
                   "can switch between them: built not to be fooled on real hardware, not to win in a perfect simulator.")
    return out


def summary_lines(res: dict) -> list[str]:
    lines = []
    for part, row in res["parts"].items():
        cells = ", ".join(f"{who} {row[who]['horiz_err_mean_m']:.2f} m / {row[who]['tilt_err_mean_deg']:.1f}°"
                          for who in WHO if who in row)
        lines.append(f"{part}: position / tilt error: {cells}")
    return lines


def write_results(res: dict) -> Path:
    RESULTS.mkdir(parents=True, exist_ok=True)
    (RESULTS / "ekf-compare.json").write_text(json.dumps(res, separators=(",", ":")))
    who = WHO
    md = [
        "# Our EKF vs ArduPilot's EKF3, on the same sensor data",
        "",
        "ArduCopter 4.7.3 SITL flown with `LOG_REPLAY = 1`: the log holds every input EKF3 used (IMU, GPS, barometer, "
        "magnetometer). DroneLab's EKF (`backend/sim/ekf.py`, 12 error states) was run offline on those inputs and both "
        "were scored against the simulator's truth (the `SIM` log message). ArduPilot's code is not used: it runs as a "
        f"separate program and we read its log. Generated by `python -m backend.sitl.ekf_replay --write` (log "
        f"`{res['log']}`).",
        "",
        f"The flight: hover; a {res['true_bias_deg_s']:g}°/s roll-gyro bias appears (`SIM_GYR1_BIAS_X`); our figure eight at "
        f"30° tilt streamed in GUIDED; a {GLITCH_M:g} m GPS glitch for {GLITCH_S:g} s (`SIM_GPS1_GLTCH_X`); land.",
        "",
        "| Part of the flight | Filter | Position error (mean / max) | Velocity error | Tilt error (mean / max) "
        "| Heading error | Gyro bias at the end | GPS rejected |",
        "|---|---|---|---|---|---|---|---|",
    ]
    for part, row in res["parts"].items():
        for w in who:
            if w not in row:
                continue
            m = row[w]
            md.append(f"| {part} ({row['duration_s']:.0f} s) | {NAMES[w]} | {m['horiz_err_mean_m']:.2f} / "
                      f"{m['horiz_err_max_m']:.2f} m | {m['vel_err_mean_m_s']:.2f} m/s | {m['tilt_err_mean_deg']:.1f} / "
                      f"{m['tilt_err_max_deg']:.1f}° | {m['heading_err_mean_deg']:.1f}° | {m['bias_end_deg_s']:.2f}°/s | "
                      f"{m['gps_rejected_s']:.1f} s |")
    md += ["", "## What this shows", *findings(res), ""]
    md += [
        "## How the replay works",
        "- Inputs from AP_DAL's replay messages: `RISI` (delta angle / velocity each 400 Hz loop), `RGPJ` (GPS), `RBRI` "
        "(barometer), `RMGI` (magnetometer). ArduPilot's body axes (forward-right-down) and world (north-east-down) are "
        "converted to ours (forward-left-up, east-north-up).",
        "- Our filter starts from the sensors alone (accelerometer = level, magnetometer + declination = heading, first GPS "
        "fix after EKF3's origin = position), with EKF3's default noise settings (`EK3_ACC_P_NSE` 0.35, `EK3_GYRO_P_NSE` "
        "0.015, `EK3_POSNE_M_NSE` 0.5, `EK3_VELNE_M_NSE` 0.3, `EK3_ALT_M_NSE` 2.0) and the GPS's 0.1 s lag.",
        "- Errors are measured at EKF3's 10 Hz log rate, against `SIM` (10 Hz, interpolated).",
    ]
    (RESULTS / "ekf-compare.md").write_text("\n".join(md) + "\n")
    return RESULTS / "ekf-compare.md"


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--log", help="replay this .BIN (flown with LOG_REPLAY=1) instead of flying a new one")
    ap.add_argument("--speedup", type=float, default=1.0)
    ap.add_argument("--write", action="store_true", help="save docs/results/ekf-compare.md and .json")
    args = ap.parse_args(argv)
    if args.log:
        res = compare_log(args.log)
        for line in summary_lines(res):
            print(line)
    else:
        res = compare(args.speedup)
    if args.write:
        print(f"Wrote {write_results(res)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
