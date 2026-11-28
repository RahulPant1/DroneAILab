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

from . import preflight, spectrum

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
          "POS", "SIM", "PARM", "ISBH", "ISBD", "MAG", "ANG")


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
    # 8b. Battery per cell (+ internal resistance when current is logged)
    if bat:
        tb = _col(bat, "t")
        v_all = _col(bat, "Volt")
        good = v_all > 0.5
        if good.any():
            cells = max(1, int(round(float(np.nanmax(v_all[good])) / 4.0)))
            fly = _flying_mask(tb, armed_spans, ctun) & good
            v_load = v_all[fly] if fly.any() else v_all[good]
            per = float(np.nanmin(v_load)) / cells
            r_txt, r_cell = "", None
            if "Curr" in bat[0]:
                cur = _col(bat, "Curr")[fly if fly.any() else good]
                ok_i = np.isfinite(cur)
                if ok_i.sum() > 20 and float(np.nanstd(cur[ok_i])) > 1.0:
                    slope = np.polyfit(cur[ok_i], v_load[ok_i], 1)[0]  # volts per amp (negative)
                    r_cell = max(0.0, -slope) / cells * 1000.0
                    r_txt = f", internal resistance ≈ {r_cell:.0f} mΩ per cell"
            st = BAD if per < 3.3 else WARN if per < 3.5 or (r_cell is not None and r_cell > 25) else OK
            check("battery_cells", "Battery per cell", st, f"{cells}S, lowest {per:.2f} V per cell under load{r_txt}",
                  "≥ 3.5 V per cell in flight; < 25 mΩ per cell",
                  "LiPo cells are damaged below about 3.3 V under load. Internal resistance (voltage drop per amp) "
                  "grows as a pack ages: a tired pack sags more and gives a shorter flight.")
    # 8c. Compass interference: does the magnetic field change with throttle?
    mag = L["MAG"]
    if mag and ctun and "ThO" in ctun[0]:
        tm = _col(mag, "t")
        fly = _flying_mask(tm, armed_spans, ctun)
        if fly.sum() > 30:
            field = np.sqrt(sum(_col(mag, k) ** 2 for k in ("MagX", "MagY", "MagZ")))[fly]
            thr = np.interp(tm[fly], _col(ctun, "t"), _col(ctun, "ThO"))
            if float(np.ptp(thr)) >= 0.1:
                slope = np.polyfit(thr, field, 1)[0]  # mGauss per unit throttle
                pct = abs(slope) / float(np.median(field)) * 100.0  # change at full throttle, like COMPASSMOT
                st = BAD if pct > 60 else WARN if pct > 30 else OK
                check("compass_motor", "Compass interference from the motors", st,
                      f"field changes {pct:.0f} % from idle to full throttle", "< 30 % (ArduPilot's compassmot guide)",
                      "Current in the power wires makes its own magnetic field. If the compass sees it, the heading "
                      "swings with throttle and the drone circles ('toilet-bowls') in Loiter. Move the compass away "
                      "from the power wiring (a GPS mast), twist the battery leads, or run COMPASSMOT.")
            else:
                check("compass_motor", "Compass interference from the motors", INFO,
                      f"throttle only varied {np.ptp(thr) * 100:.0f} %", "needs ≥ 10 % throttle change",
                      "Climb and descend a few times in one flight so the check can compare low and high throttle.")
    # 8d. Motors fighting each other on yaw (quad X: motors 1, 2 spin one way, 3, 4 the other)
    if rcou and n_motors == 4:
        t = _col(rcou, "t")
        fly = _flying_mask(t, armed_spans, ctun)
        if fly.any():
            m = [float(np.nanmean(_col(rcou, f"C{i}")[fly])) for i in range(1, 5)]
            diff = (m[0] + m[1]) / 2 - (m[2] + m[3]) / 2
            st = BAD if abs(diff) > 100 else WARN if abs(diff) > 50 else OK
            pair = "1 and 2 (counter-clockwise props)" if diff > 0 else "3 and 4 (clockwise props)"
            check("motor_yaw", "Motors fighting on yaw", st, f"motors {pair} work {abs(diff):.0f} µs harder",
                  "< 50 µs", "To hold its heading, one pair of motors keeps working harder than the other: usually "
                  "motors twisted on their arms, or a frame that twists. It wastes battery and leaves less margin "
                  "to turn. Check that every motor sits straight.")
    # 8e. Wobble: does the attitude error oscillate (tuning too aggressive)?
    #     ArduCopter 4.7: ATT is always 10 Hz; the controller's own ANG is at the loop rate with LOG_BITMASK bit 0.
    src = att
    if L["ANG"] and att and len(L["ANG"]) > 1.5 * len(att):
        src = L["ANG"]
    if src:
        t = _col(src, "t")
        fly = _flying_mask(t, armed_spans, ctun)
        osc = _oscillation(t, _col(src, "DesRoll") - _col(src, "Roll"), _col(src, "DesPitch") - _col(src, "Pitch"), fly)
        log_hz = 1.0 / float(np.median(np.diff(t))) if len(t) > 2 else 0.0
        if osc:
            amp, f_hz = osc
            st = BAD if amp > 0.4 else WARN if amp > 0.15 else OK
            advice = ("Above about 5 Hz it's usually the D term or noisy gyro filtering; slower wobble is too much P. "
                      "Lower ATC_RAT_RLL_P / ATC_RAT_PIT_P (and D) by 10-20 %, or run AUTOTUNE on a calm day.")
            slow_log = log_hz < 50
            check("wobble", "Wobble (attitude oscillation)", st,
                  f"{amp:.2f}° RMS, strongest at {f_hz:.1f} Hz (attitude logged at {log_hz:.0f} Hz)",
                  "< 0.15° RMS above 3 Hz", "Overshooting and correcting again and again: the rate loop is tuned too "
                  "aggressively for this airframe. " + advice + (
                      " This log has attitude at only 10 Hz, so fast wobble (5-20 Hz, the usual kind) is invisible: "
                      "for tuning flights add 1 to LOG_BITMASK (ATTITUDE_FAST)." if slow_log else ""))

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
    params = preflight.params_from_log(L["PARM"])
    pre = preflight.check(params) if len(params) >= 100 else None  # real logs carry every parameter
    return {
        "fix_first": fix_first(checks),
        "preflight": pre,
        "overview": overview,
        "verdict": worst,
        "checks": checks,
        "timeline": [{"t": round(t, 1), "kind": k, "text": x} for t, k, x in timeline[:MAX_TIMELINE]],
        "timeline_total": len(timeline),
        "charts": charts,
    }


# What to do about each check, most important first (safety, then things that break hardware, then quality).
ACTIONS = {
    "failsafes": "Find out why each failsafe triggered (timeline) before flying again.",
    "motors": "A motor at its limit or one working much harder: check the props, the centre of gravity, that motor and its ESC.",
    "compass_motor": "Move the compass away from the power wiring (GPS mast, twisted battery leads), or run COMPASSMOT.",
    "vibration": "Balance or replace the props, tighten the motors, soft-mount the flight controller.",
    "gyro_noise": "Set the harmonic notch filter as suggested (lesson 25).",
    "ekf": "A sensor disagreed with the EKF: look at which one (GPS, compass, baro) and what happened at that moment.",
    "battery_cells": "Land earlier (raise BATT_LOW_VOLT) or replace a tired pack.",
    "battery": "Check the pack and the low-battery failsafe settings.",
    "wobble": "Lower the rate P/D gains 10-20 % or run AUTOTUNE on a calm day.",
    "motor_yaw": "Straighten the motors on their arms (all perfectly vertical).",
    "attitude": "The drone couldn't tilt as asked: check tuning, motors and wind.",
    "gps": "Fly where the sky is open, and move the GPS away from cameras / video transmitters.",
    "altitude": "Altitude wandered: check thrust margin and vibration.",
    "position": "Position wandered: wind, compass or GPS. Check those checks first.",
    "truth": "The EKF believed something false: see the EKF and GPS checks.",
}
PRIORITY = list(ACTIONS)


def fix_first(checks: list[dict], n: int = 3) -> list[dict]:
    """The few things to fix before the next flight: worst status first, then by importance."""
    bad = [c for c in checks if c["status"] in (BAD, WARN) and c["id"] in ACTIONS]
    bad.sort(key=lambda c: (c["status"] != BAD, PRIORITY.index(c["id"])))
    return [{"id": c["id"], "title": c["title"], "status": c["status"], "action": ACTIONS[c["id"]]} for c in bad[:n]]


WOBBLE_FROM_HZ = 3.0  # wind, pilot and manoeuvres are slower than this; a ringing controller is faster


def _oscillation(t: np.ndarray, err_roll: np.ndarray, err_pitch: np.ndarray, fly: np.ndarray) -> tuple[float, float] | None:
    """(RMS of the attitude error above WOBBLE_FROM_HZ in degrees, the strongest frequency in Hz), or None.

    Each continuous stretch of flying is checked on its own (mean removed); the power above 3 Hz is added up
    (Parseval: the RMS of that band). On ArduCopter SITL: 0.01° with default gains, 0.49° at 23 Hz with the
    roll/pitch rate P and D x 3.5 (docs/results/hardware-checks.md)."""
    if fly.sum() < 50:
        return None
    dt = float(np.median(np.diff(t[fly])))
    if not 0 < dt < 0.2 or 1.0 / (2 * dt) <= WOBBLE_FROM_HZ:
        return None
    idx = np.flatnonzero(fly)
    cuts = np.flatnonzero(np.diff(idx) > 1) + 1
    best = None
    for e_all in (err_roll, err_pitch):
        power, n_total, peak = 0.0, 0, (0.0, 0.0)
        for seg in np.split(idx, cuts):
            if len(seg) < 32:
                continue
            e = np.nan_to_num(e_all[seg])
            e = e - e.mean()
            X = np.fft.rfft(e)
            f = np.fft.rfftfreq(len(e), dt)
            band = f >= WOBBLE_FROM_HZ
            p = np.abs(X[band]) ** 2
            power += 2.0 * float(p.sum()) / len(e)
            n_total += len(e)
            if p.size and float(p.max()) / len(e) > peak[0]:
                peak = (float(p.max()) / len(e), float(f[band][np.argmax(p)]))
        if n_total:
            rms = math.sqrt(power / n_total)
            if best is None or rms > best[0]:
                best = (rms, peak[1])
    return best


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
