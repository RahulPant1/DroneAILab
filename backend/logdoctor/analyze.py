"""Read an ArduPilot (ArduCopter) DataFlash log and check it the way a flight-test engineer would.

Each check gives a status (ok / warn / bad / info), the measured value, the limit it was compared with, and what it
means in plain English. Thresholds follow ArduPilot's own guidance where it has some:
  - vibration: below 30 m/s/s is normally fine, above 60 nearly always a problem; any accelerometer clipping is bad;
  - EKF innovation test ratios: above 1.0 means a sensor reading was rejected; up to ~0.5 is normal;
  - GPS: HDOP above 1.4 fails ArduPilot's own arming check; fewer than 6 satellites is poor.

Usage:
  .venv/bin/python -m backend.logdoctor.analyze path/to/00000042.BIN
"""

from __future__ import annotations

import json
import math
import re
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

from . import spectrum

# ---- ArduPilot code tables (from AP_Logger.h, ArduPilot 4.7) ---------------------------------------

ERR_SUBSYSTEMS = {
    1: "Main", 2: "Radio", 3: "Compass", 5: "Radio failsafe", 6: "Battery failsafe", 7: "GPS failsafe",
    8: "GCS failsafe", 9: "Fence failsafe", 10: "Flight mode change", 11: "GPS", 12: "Crash check", 13: "Flip",
    15: "Parachute", 16: "EKF check", 17: "EKF failsafe", 18: "Barometer", 19: "CPU", 20: "ADSB failsafe",
    21: "Terrain", 22: "Navigation", 23: "Terrain failsafe", 24: "EKF primary lane", 25: "Thrust loss check",
    26: "Sensor failsafe", 27: "Leak failsafe", 28: "Pilot input", 29: "Vibration failsafe", 30: "Internal error",
    31: "Dead-reckoning failsafe",
}
FAILSAFE_SUBSYSTEMS = {5, 6, 7, 8, 9, 17, 20, 23, 26, 27, 29, 31}

EVENTS = {
    10: "armed", 11: "disarmed", 15: "auto-armed", 17: "maybe landed", 18: "landed", 19: "lost GPS",
    25: "home set", 28: "took off", 41: "fence enabled", 42: "fence disabled", 54: "motors emergency-stopped",
    60: "EKF altitude reset", 62: "EKF yaw reset", 67: "GPS primary changed",
}

MAX_CHART_POINTS = 400
MAX_TIMELINE = 40

# One check = {"id", "title", "status", "value", "limit", "meaning"}
OK, WARN, BAD, INFO = "ok", "warn", "bad", "info"


def _err_text(subsys: int, code: int) -> str:
    name = ERR_SUBSYSTEMS.get(subsys, f"subsystem {subsys}")
    if code == 0:
        return f"{name}: resolved"
    if subsys in FAILSAFE_SUBSYSTEMS:
        return f"{name}: TRIGGERED"
    specific = {
        (12, 1): "crash detected, motors stopped", (12, 2): "loss of control", (11, 2): "GPS glitch",
        (16, 2): "bad EKF variance", (25, 1): "thrust loss detected", (10, 1): "couldn't enter the mode",
    }
    if subsys == 10:
        return f"{name}: couldn't enter mode {code}"
    if subsys == 24:
        return f"{name}: switched to lane {code}"
    return f"{name}: " + specific.get((subsys, code), {1: "failed to start", 4: "unhealthy"}.get(code, f"code {code}"))


def _mode_name(num: int) -> str:
    from pymavlink import mavutil

    return mavutil.mode_mapping_acm.get(num, f"mode {num}")  # ArduCopter: number -> name


# Start-up chatter that says nothing about the flight (kept out of the timeline)
BOOT_NOISE = re.compile(r"^(ArduCopter V|[0-9a-f]{32}$|Param space used|RC Protocol|New (mission|rally|fence)|Frame: |"
                        r"GPS \d: (probing|detected)|u-blox|IMU\d: fast sampling|Barometer \d calibration|"
                        r"EKF3 IMU\d (initialised|tilt alignment|MAG\d initial yaw)|AHRS: (DCM|EKF3) active|"
                        r"RC\d: SaveWaypoint|ArduPilot Ready|Field Elevation Set|EKF3 IMU\d origin set)")


# ---- reading --------------------------------------------------------------------------------------

WANTED = ("MODE", "MSG", "ERR", "EV", "VIBE", "XKF4", "GPS", "ATT", "CTUN", "PSCN", "PSCE", "BAT", "RCOU",
          "POS", "SIM", "PARM", "ISBH", "ISBD")


def read_log(path: str | Path) -> dict[str, list[dict]]:
    """All the messages we use, as lists of dicts with a time in seconds ('t')."""
    from pymavlink import mavutil

    m = mavutil.mavlink_connection(str(path), dialect="ardupilotmega", zero_time_base=True)
    out: dict[str, list[dict]] = defaultdict(list)
    t0 = None
    while True:
        msg = m.recv_match(type=list(WANTED))
        if msg is None:
            break
        d = msg.to_dict()
        us = d.get("TimeUS")
        if us is None:
            continue
        t0 = us if t0 is None else t0
        d["t"] = (us - t0) / 1e6
        out[msg.get_type()].append(d)
    return out


def _col(rows: list[dict], key: str) -> np.ndarray:
    return np.array([r.get(key, np.nan) for r in rows], dtype=float)


def _thin(t: np.ndarray, *ys: np.ndarray) -> list[list[float]]:
    """Chart data with at most MAX_CHART_POINTS points (rounded for JSON)."""
    if len(t) == 0:
        return [[] for _ in range(1 + len(ys))]
    idx = np.unique(np.linspace(0, len(t) - 1, min(len(t), MAX_CHART_POINTS)).astype(int))
    clean = lambda a: [None if not math.isfinite(v) else round(float(v), 3) for v in a[idx]]  # noqa: E731
    return [clean(t), *(clean(y) for y in ys)]


def _flying_mask(t: np.ndarray, armed_spans: list[tuple[float, float]], ctun: list[dict]) -> np.ndarray:
    """True where the drone was armed and more than 1 m up (so ground time doesn't skew tracking checks)."""
    armed = np.zeros(len(t), bool)
    for a, b in armed_spans:
        armed |= (t >= a) & (t <= b)
    if ctun:
        alt = np.interp(t, _col(ctun, "t"), _col(ctun, "Alt"))
        armed &= alt > 1.0
    return armed


# ---- the checks ---------------------------------------------------------------------------------------

def analyze(path: str | Path) -> dict:
    """The full report for one log: overview, checks, timeline and chart data."""
    path = Path(path)
    return analyze_messages(read_log(path), path.name)


def analyze_messages(L: dict[str, list[dict]], name: str = "log") -> dict:
    """The checks on already-read messages ({type: [dict with 't' in seconds, ...]}). Tests feed this directly."""
    L = defaultdict(list, L)
    checks: list[dict] = []

    def check(cid, title, status, value, limit, meaning):
        checks.append({"id": cid, "title": title, "status": status, "value": value, "limit": limit, "meaning": meaning})

    # Timeline: modes, events, errors, messages
    timeline: list[tuple[float, str, str]] = []  # (t, kind, text)
    for r in L["MODE"]:
        timeline.append((r["t"], "mode", f"mode {_mode_name(int(r['ModeNum']))}"))
    armed_spans, armed_at = [], None
    for r in L["EV"]:
        eid = int(r["Id"])
        if eid in (10, 15):
            armed_at = r["t"]
        if eid == 11 and armed_at is not None:
            armed_spans.append((armed_at, r["t"]))
            armed_at = None
        if eid in EVENTS:
            timeline.append((r["t"], "event", EVENTS[eid]))
    end_t = max((rows[-1]["t"] for rows in L.values() if rows), default=0.0)
    if armed_at is not None:
        armed_spans.append((armed_at, end_t))
    errors = []
    for r in L["ERR"]:
        text = _err_text(int(r["Subsys"]), int(r["ECode"]))
        errors.append({"t": round(r["t"], 1), "text": text})
        timeline.append((r["t"], "error", text))
    seen = set()
    for r in L["MSG"]:
        text = str(r.get("Message", "")).strip()
        if text and text not in seen and not BOOT_NOISE.match(text):
            seen.add(text)
            timeline.append((r["t"], "message", text))
    timeline.sort(key=lambda x: x[0])
    modes = [x[2][5:] for x in timeline if x[1] == "mode"]

    # Overview
    ctun = L["CTUN"]
    alt = _col(ctun, "Alt") if ctun else np.array([])
    gps = L["GPS"]
    overview = {
        "file": name,
        "log_duration_s": round(end_t, 1),
        "armed_time_s": round(sum(b - a for a, b in armed_spans), 1),
        "max_altitude_m": round(float(np.nanmax(alt)), 1) if len(alt) else None,
        "max_ground_speed_m_s": round(float(np.nanmax(_col(gps, "Spd"))), 1) if gps else None,
        "modes_used": list(dict.fromkeys(modes)),
        "frame": _frame_name(L["PARM"]),
        "simulated": bool(L["SIM"]),
    }

    # 1. Vibration
    vibe = L["VIBE"]
    if vibe:
        vx, vy, vz = (_col(vibe, k) for k in ("VibeX", "VibeY", "VibeZ"))
        peak = float(np.nanmax(np.vstack([vx, vy, vz])))
        clip = int(np.nanmax(_col(vibe, "Clip"))) if "Clip" in vibe[0] else 0
        st = BAD if peak > 60 or clip > 100 else WARN if peak > 30 or clip > 0 else OK
        check("vibration", "Vibration", st, f"peak {peak:.1f} m/s², {clip} clipping events", "< 30 m/s², no clipping",
              "How much the flight controller is shaken. High vibration confuses the accelerometers: altitude and "
              "position hold get worse, and clipping (the sensor maxed out) can make the EKF lose track.")
    # 1b. Gyro noise spectrum (only with the batch sampler on)
    flying_t = [(a, b) for a, b in armed_spans]
    spec = spectrum.analyze(L, flying=lambda t: any(a + 5.0 <= t <= b - 2.0 for a, b in flying_t)) if L["ISBH"] else None
    if spec:
        post, pre = spec.get("post_peak"), spec.get("pre_peak")
        have_post = spec.get("post_rms_deg_s") is not None  # filtered samples logged (INS_LOG_BAT_OPT 2 or 4)
        seen_peak = post if have_post else pre
        left = seen_peak["amp_deg_s"] if seen_peak else 0.0  # without post-filter data: judge the raw peak
        st = BAD if left > 3.0 else WARN if left > 0.5 else OK
        where = "after the filters" if have_post else "before the filters"
        value = (f"motor-noise peak at {seen_peak['freq_hz']:.0f} Hz, {seen_peak['amp_deg_s']:.2f} deg/s {where}"
                 if seen_peak else f"no motor-noise peak {where}")
        if pre and post:
            value += f" (before the filters: {pre['amp_deg_s']:.2f} deg/s)"
        rec = spec.get("recommendation")
        advice = (" Suggested notch: " + ", ".join(f"{k} = {v}" for k, v in rec.items()) + "." if rec else
                  " A harmonic notch is on." if spec["notch_on"] else "")
        check("gyro_noise", "Gyro noise (FFT of the batch sampler)", st, value, "< 0.5 deg/s left at the motor frequency",
              "Motors and props shake the flight controller at the rotor's spin frequency; the gyro feels it as fake "
              f"rotation and the motors buzz. Low-pass filter: INS_GYRO_FILTER = {spec['gyro_filter_hz']:g} Hz (lower "
              "removes more noise but delays the gyro). A harmonic notch removes just the motor band." + advice)
    # 2. EKF health
    xk = L["XKF4"]
    if xk:
        worst = {k: float(np.nanmax(_col(xk, k))) for k in ("SV", "SP", "SH", "SM")}
        top = max(worst, key=worst.get)
        names = {"SV": "velocity", "SP": "position", "SH": "height", "SM": "compass"}
        st = BAD if worst[top] > 1.0 else WARN if worst[top] > 0.5 else OK
        check("ekf", "EKF health (innovation test ratios)", st,
              ", ".join(f"{names[k]} {v:.2f}" for k, v in worst.items()), "< 0.5 normal, > 1.0 = reading rejected",
              "How much each sensor disagreed with the EKF's prediction. Above 1.0 the EKF rejected that sensor's "
              f"reading: a glitch, interference or a failing sensor. Worst here: {names[top]}.")
    # 3. GPS quality
    if gps:
        sats = _col(gps, "NSats")
        hdop = _col(gps, "HDop")
        fix = _col(gps, "Status")
        flying_gps = fix >= 3
        min_sats = int(np.nanmin(sats[flying_gps])) if flying_gps.any() else int(np.nanmin(sats))
        max_hdop = float(np.nanmax(hdop[flying_gps])) if flying_gps.any() else float(np.nanmax(hdop))
        lost = int(np.sum(np.diff((fix >= 3).astype(int)) == -1))
        st = BAD if min_sats < 6 or max_hdop > 2.5 or lost else WARN if min_sats < 9 or max_hdop > 1.4 else OK
        check("gps", "GPS quality", st, f"min {min_sats} satellites, max HDOP {max_hdop:.2f}, lost fix {lost}×",
              "≥ 9 satellites, HDOP ≤ 1.4, no lost fix",
              "Satellites and HDOP (how good their geometry is). ArduPilot won't arm above HDOP 1.4. "
              "Losing the 3D fix in flight forces a failsafe.")
    # 4. Attitude tracking
    att = L["ATT"]
    if att:
        t = _col(att, "t")
        fly = _flying_mask(t, armed_spans, ctun)
        if fly.any():
            err = np.maximum(np.abs(_col(att, "DesRoll") - _col(att, "Roll")),
                             np.abs(_col(att, "DesPitch") - _col(att, "Pitch")))[fly]
            mx, rms = float(np.nanmax(err)), float(np.sqrt(np.nanmean(err ** 2)))
            st = BAD if mx > 15 else WARN if mx > 5 else OK
            check("attitude", "Attitude tracking (roll/pitch)", st, f"max {mx:.1f}°, RMS {rms:.2f}°", "max < 5°",
                  "How closely the drone tilted to what its controller asked for. Big errors mean the inner PID loop "
                  "can't keep up: poor tuning, a weak or failed motor, or a lot of wind.")
    # 5. Altitude tracking
    if ctun:
        t = _col(ctun, "t")
        fly = _flying_mask(t, armed_spans, ctun)
        if fly.any():
            err = np.abs(_col(ctun, "DAlt") - _col(ctun, "Alt"))[fly]
            mx = float(np.nanmax(err))
            st = BAD if mx > 3 else WARN if mx > 1 else OK
            check("altitude", "Altitude tracking", st, f"max error {mx:.2f} m", "< 1 m",
                  "Desired vs estimated altitude while flying. Large errors: not enough thrust (heavy, weak battery, "
                  "failed motor) or vibration upsetting the altitude estimate.")
    # 6. Horizontal position tracking
    if L["PSCN"] and L["PSCE"] and len(L["PSCN"]) == len(L["PSCE"]):
        n, e = L["PSCN"], L["PSCE"]
        t = _col(n, "t")
        fly = _flying_mask(t, armed_spans, ctun)
        if fly.any():
            err = np.hypot(_col(n, "TPN") - _col(n, "PN"), _col(e, "TPE") - _col(e, "PE"))[fly]
            mx = float(np.nanmax(err))
            st = BAD if mx > 5 else WARN if mx > 2 else OK
            check("position", "Position tracking", st, f"max error {mx:.2f} m", "< 2 m",
                  "Target vs estimated horizontal position in position-controlled modes. Large errors: wind beyond "
                  "what it can fight, a failing motor, or the target jumping (e.g. a new waypoint).")
    # 7. Motors: balance and saturation
    rcou = L["RCOU"]
    n_motors = {"hexa": 6, "octa": 8}.get(overview["frame"] or "", 4)
    if rcou:
        t = _col(rcou, "t")
        fly = _flying_mask(t, armed_spans, ctun)
        if fly.any():
            pwm = np.vstack([_col(rcou, f"C{i}") for i in range(1, n_motors + 1)])[:, fly]
            means = np.nanmean(pwm, axis=1)
            spread = float(means.max() - means.min())
            sat = float(np.mean(np.any(pwm >= 1950, axis=0)))
            hi = int(np.argmax(means)) + 1
            st = BAD if sat > 0.2 or spread > 150 else WARN if sat > 0.02 or spread > 75 else OK
            check("motors", "Motor balance and saturation", st,
                  f"average outputs {', '.join(f'{m:.0f}' for m in means)} µs (spread {spread:.0f}); "
                  f"at full power {sat * 100:.0f} % of the time", "spread < 75 µs, full power < 2 % of the time",
                  "If one motor always works harder (here motor " + str(hi) + " is highest), suspect the centre of "
                  "gravity, a bent prop or a weak motor/ESC. Motors at full power means no margin left to control "
                  "the drone.")
    # 8. Battery
    bat = L["BAT"]
    if bat:
        v = _col(bat, "Volt")
        v = v[v > 0.5]
        if len(v):
            sag = float(v.max() - v.min())
            fs = any("Battery failsafe" in e["text"] and "TRIGGERED" in e["text"] for e in errors)
            st = BAD if fs else WARN if sag > 1.5 else OK
            check("battery", "Battery", st, f"{v.max():.2f} V → min {v.min():.2f} V (sag {sag:.2f} V)"
                  + ("; battery failsafe triggered" if fs else ""), "no failsafe, sag < 1.5 V",
                  "Voltage drop under load. A big sag means a tired or undersized battery; the battery failsafe "
                  "(BATT_FS_LOW_ACT) brings the drone home before it's too late.")
    # 9. Failsafes, crashes, errors
    trig = [e for e in errors if "TRIGGERED" in e["text"] or "crash" in e["text"].lower()
            or "loss of control" in e["text"]]
    check("failsafes", "Failsafes and errors", BAD if trig else WARN if errors else OK,
          "; ".join(f"{e['t']:.0f} s {e['text']}" for e in (trig or errors)[:6]) or "none",
          "none", "Every failsafe the autopilot triggered (radio, battery, EKF, fence...) and every logged error. "
                  "A failsafe means something went wrong, even if the drone then did the right thing.")
    # 10. Truth vs belief (SITL only)
    if L["SIM"] and L["POS"]:
        sim, pos = L["SIM"], L["POS"]
        ts = _col(sim, "t")
        tp = _col(pos, "t")
        lat_s = np.interp(tp, ts, _col(sim, "Lat"))
        lon_s = np.interp(tp, ts, _col(sim, "Lng"))
        lat_p, lon_p = _col(pos, "Lat"), _col(pos, "Lng")
        dn = np.radians(lat_p - lat_s) * 6378137.0
        de = np.radians(lon_p - lon_s) * 6378137.0 * np.cos(np.radians(lat_s))
        gap = np.hypot(dn, de)
        mx = float(np.nanmax(gap))
        st = BAD if mx > 5 else WARN if mx > 2 else OK
        check("truth", "Belief vs truth (simulator only)", st, f"max {mx:.1f} m", "< 2 m",
              "How far the EKF's position was from where the simulator says the drone really was. Only a simulator "
              "knows the truth; on a real drone you never see this number, which is why spoofing is dangerous.")
        truth_chart = _thin(tp, gap)
    else:
        truth_chart = None

    charts = {}
    if ctun:
        charts["altitude"] = {"labels": ["desired (m)", "estimated (m)"], "data": _thin(_col(ctun, "t"), _col(ctun, "DAlt"), _col(ctun, "Alt"))}
    if vibe:
        charts["vibration"] = {"labels": ["X", "Y", "Z"], "data": _thin(_col(vibe, "t"), *(_col(vibe, k) for k in ("VibeX", "VibeY", "VibeZ")))}
    if rcou:
        charts["motors"] = {"labels": [f"motor {i}" for i in range(1, n_motors + 1)],
                            "data": _thin(_col(rcou, "t"), *(_col(rcou, f"C{i}") for i in range(1, n_motors + 1)))}
    if xk:
        charts["ekf"] = {"labels": ["velocity", "position", "height", "compass"],
                         "data": _thin(_col(xk, "t"), *(_col(xk, k) for k in ("SV", "SP", "SH", "SM")))}
    if spec:
        charts["gyro_spectrum"] = spec["chart"]
    if truth_chart:
        charts["truth"] = {"labels": ["belief vs truth (m)"], "data": truth_chart}

    worst = BAD if any(c["status"] == BAD for c in checks) else WARN if any(c["status"] == WARN for c in checks) else OK
    return {
        "overview": overview,
        "verdict": worst,
        "checks": checks,
        "timeline": [{"t": round(t, 1), "kind": k, "text": x} for t, k, x in timeline[:MAX_TIMELINE]],
        "timeline_total": len(timeline),
        "charts": charts,
    }


def _frame_name(parms: list[dict]) -> str | None:
    vals = {p.get("Name"): p.get("Value") for p in parms}
    return {1: "quad", 2: "hexa", 3: "octa", 4: "octaquad", 5: "y6", 7: "tri"}.get(int(vals.get("FRAME_CLASS", 0) or 0))


def main(argv: list[str] | None = None) -> int:
    args = argv if argv is not None else sys.argv[1:]
    if not args:
        print(__doc__)
        return 1
    r = analyze(args[0])
    print(json.dumps({k: v for k, v in r.items() if k != "charts"}, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
