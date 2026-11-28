"""✈ Pre-flight check: read a drone's parameters and say what is missing BEFORE the first arm.

Works on a parameter file saved from QGroundControl (.params, tab-separated) or Mission Planner (.param,
"NAME,VALUE"), or on the parameters every ArduPilot log starts with (the PARM messages).

Each item: (status, title, what was found, what to do). Status: ok / warn / bad / info.
Thresholds and names are ArduCopter 4.7's (checked against real 4.7.3 SITL for the parameter generator, lesson 28).
"""

from __future__ import annotations

import re

OK, WARN, BAD, INFO = "ok", "warn", "bad", "info"

# Names that changed in ArduCopter 4.7: an old name in a file is silently ignored by the flight controller.
RENAMED = {
    "RTL_ALT": "RTL_ALT_M (now in metres, not cm)",
    "ATC_ACCEL_R_MAX": "ATC_ACC_R_MAX (now in deg/s/s, not centideg/s/s)",
    "ATC_ACCEL_P_MAX": "ATC_ACC_P_MAX (now in deg/s/s, not centideg/s/s)",
    "ATC_ACCEL_Y_MAX": "ATC_ACC_Y_MAX (now in deg/s/s, not centideg/s/s)",
    "ARMING_CHECK": "ARMING_SKIPCHK (works the other way round: 0 = skip nothing)",
}
RTL_MODE = 6
RC_OPTION_RTL = 4
FRAME_CLASSES = {1: "quad", 2: "hexa", 3: "octa", 4: "octa-quad", 5: "Y6", 7: "tri", 10: "single", 12: "dodeca-hexa", 14: "deca"}
LINE = re.compile(r"^\s*([A-Z][A-Z0-9_]{1,16})\s*[,\s]\s*(-?[0-9.eE+-]+)\s*$")


def parse_param_text(text: str) -> dict[str, float]:
    """Parameters from a Mission Planner (NAME,VALUE) or QGroundControl (vehicle comp NAME VALUE type) file."""
    out: dict[str, float] = {}
    for raw in text.splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        parts = line.split("\t")
        if len(parts) >= 4 and parts[0].strip().isdigit():  # QGC: sysid compid name value type
            name, value = parts[2].strip(), parts[3].strip()
        else:
            m = LINE.match(line)
            if not m:
                continue
            name, value = m.group(1), m.group(2)
        try:
            out[name] = float(value)
        except ValueError:
            continue
    return out


def params_from_log(parm_rows: list[dict]) -> dict[str, float]:
    """The last value of every parameter in a log's PARM messages."""
    return {str(r["Name"]): float(r["Value"]) for r in parm_rows if "Name" in r and "Value" in r}


def check(p: dict[str, float], partial: bool = False) -> dict:
    """The pre-flight checklist. `partial` = a hand-made file (like DroneLab's starting parameters) rather than a
    full dump from the flight controller: missing calibrations are then expected, so they are 'to do' items."""
    items: list[dict] = []

    def add(cid, status, title, found, todo):
        items.append({"id": cid, "status": status, "title": title, "found": found, "todo": todo})

    have = lambda *names: all(n in p for n in names)  # noqa: E731
    g = p.get

    # 0. Old names
    old = [n for n in RENAMED if n in p]
    if old:
        add("renamed", WARN, "Out-of-date parameter names", ", ".join(old),
            "ArduCopter 4.7 ignores these: use " + "; ".join(f"{n} -> {RENAMED[n]}" for n in old) + ".")

    # 1. Frame
    fc = g("FRAME_CLASS")
    if fc is None:
        add("frame", INFO if partial else BAD, "Frame type", "not set", "Set FRAME_CLASS (1 = quad) and FRAME_TYPE (1 = X).")
    elif fc == 0:
        add("frame", BAD, "Frame type", "FRAME_CLASS = 0 (undefined)", "Set FRAME_CLASS and FRAME_TYPE: ArduPilot won't arm.")
    else:
        add("frame", OK, "Frame type", f"{FRAME_CLASSES.get(int(fc), int(fc))}, FRAME_TYPE {int(g('FRAME_TYPE', 1))}", "")

    # 2. Calibrations (only a full dump can tell)
    if have("INS_ACCOFFS_X", "INS_ACCOFFS_Y", "INS_ACCOFFS_Z"):
        raw = all(g(f"INS_ACCOFFS_{a}") == 0 for a in "XYZ") and all(g(f"INS_ACCSCAL_{a}", 1) == 1 for a in "XYZ")
        add("accel_cal", BAD if raw else OK, "Accelerometer calibration", "never done (offsets 0, scales 1)" if raw else "done",
            "QGroundControl -> Vehicle Setup -> Sensors -> Accelerometer (6 positions)." if raw else "")
    else:
        add("accel_cal", INFO, "Accelerometer calibration", "not in this file",
            "Do it on the real board: QGroundControl -> Sensors -> Accelerometer.")
    if g("COMPASS_USE", 1) and have("COMPASS_OFS_X", "COMPASS_OFS_Y", "COMPASS_OFS_Z"):
        raw = all(g(f"COMPASS_OFS_{a}") == 0 for a in "XYZ")
        add("compass_cal", BAD if raw else OK, "Compass calibration", "never done (offsets 0)" if raw else
            f"offsets {g('COMPASS_OFS_X'):.0f}, {g('COMPASS_OFS_Y'):.0f}, {g('COMPASS_OFS_Z'):.0f}",
            "QGroundControl -> Sensors -> Compass, outdoors, away from metal." if raw else "")
        big = max(abs(g(f"COMPASS_OFS_{a}")) for a in "XYZ")
        if not raw and big > 600:
            add("compass_ofs", WARN, "Compass offsets large", f"{big:.0f} mGauss", "Over 600: a magnet, battery lead or "
                "metal near the compass. Move the GPS/compass mast away from the power wiring.")
    else:
        add("compass_cal", INFO, "Compass calibration", "not in this file", "Do it on the real board, outdoors.")
    if have("RC1_MIN", "RC1_MAX", "RC3_MIN", "RC3_MAX"):
        default = all(g(f"RC{i}_MIN") == 1100 and g(f"RC{i}_MAX") == 1900 for i in (1, 2, 3, 4) if f"RC{i}_MIN" in p)
        add("radio_cal", WARN if default else OK, "Radio calibration", "default 1100-1900 (probably not done)" if default
            else f"throttle {g('RC3_MIN'):.0f}-{g('RC3_MAX'):.0f} µs",
            "QGroundControl -> Radio: move every stick and switch to its ends." if default else "")
    else:
        add("radio_cal", INFO, "Radio calibration", "not in this file", "Do it with your transmitter: QGroundControl -> Radio.")

    # 3. Failsafes
    thr = g("FS_THR_ENABLE")
    if thr is not None:
        add("fs_radio", BAD if thr == 0 else OK, "Radio-loss failsafe", "OFF" if thr == 0 else f"on (FS_THR_ENABLE {int(thr)})",
            "Set FS_THR_ENABLE = 1 (RTL), and set the receiver to 'no pulses' on signal loss." if thr == 0 else
            "Test it with the props off: switch the transmitter off, the mode must change to RTL.")
    mon = g("BATT_MONITOR")
    if mon == 0:
        add("fs_batt", BAD, "Battery monitor", "off (BATT_MONITOR 0)",
            "Set BATT_MONITOR to your power module (4 = analog voltage + current); without it there is no low-battery failsafe.")
    if g("BATT_FS_LOW_ACT") is not None or g("BATT_LOW_VOLT") is not None:
        act, low = g("BATT_FS_LOW_ACT", 0), g("BATT_LOW_VOLT", 0)
        bad = act == 0 or low == 0
        add("fs_batt_low", BAD if bad else OK, "Low-battery failsafe",
            f"BATT_LOW_VOLT {low:g} V, action {int(act)}", "Set BATT_LOW_VOLT (3.5 V x cells) and BATT_FS_LOW_ACT = 2 (RTL)." if bad else "")
    ekf = g("FS_EKF_ACTION")
    if ekf is not None:
        add("fs_ekf", WARN if ekf == 0 else OK, "Bad-position (EKF) failsafe", "report only" if ekf == 0 else f"action {int(ekf)}",
            "Set FS_EKF_ACTION = 1 (land)." if ekf == 0 else "")
    fence = g("FENCE_ENABLE")
    if fence is not None:
        add("fence", WARN if fence == 0 else OK, "Geofence", "off" if fence == 0 else
            f"up to {g('FENCE_ALT_MAX', 0):g} m, {g('FENCE_RADIUS', 0):g} m out", "Turn on FENCE_ENABLE: a safety net "
            "for a fly-away or a confused pilot." if fence == 0 else "")
    rtl = g("RTL_ALT_M")
    if rtl is not None:
        add("rtl_alt", WARN if rtl < 10 else OK, "Return-home height", f"{rtl:g} m",
            "Under 10 m it may fly home through trees. Set RTL_ALT_M above the tallest thing around." if rtl < 10 else "")

    # 4. A way to call it home
    modes = [int(g(f"FLTMODE{i}")) for i in range(1, 7) if f"FLTMODE{i}" in p]
    switch = [k for k, v in p.items() if re.fullmatch(r"RC\d+_OPTION", k) and int(v) == RC_OPTION_RTL]
    if modes or switch:
        ok = RTL_MODE in modes or bool(switch)
        add("rtl_switch", OK if ok else WARN, "RTL on a switch", "yes" if ok else "no RTL among the flight modes",
            "" if ok else "Put RTL on a switch (a FLTMODE slot, or RCx_OPTION = 4): one flick brings it home.")

    # 5. Arming checks
    skip = g("ARMING_SKIPCHK")
    if skip is not None and skip != 0:
        add("arming", BAD, "Arming checks", f"some skipped (ARMING_SKIPCHK {int(skip)})",
            "Set ARMING_SKIPCHK = 0: the checks catch a missing calibration or a bad GPS before take-off.")
    elif g("ARMING_CHECK") is not None and g("ARMING_CHECK") != 1:
        add("arming", BAD, "Arming checks", f"ARMING_CHECK {int(g('ARMING_CHECK'))}", "Enable all arming checks.")

    # 6. Motors and tuning starting points
    hov = g("MOT_THST_HOVER")
    if hov is not None:
        add("hover", WARN if not 0.125 <= hov <= 0.6 else OK, "Hover throttle", f"{hov * 100:.0f} %",
            "Above 60 %: too heavy for its motors (sluggish, short flights, no margin in wind). Below 12.5 %: very "
            "over-powered, it will feel twitchy." if not 0.125 <= hov <= 0.6 else "")
    if g("MOT_SPIN_ARM") is not None and g("MOT_SPIN_MIN") is not None and g("MOT_SPIN_ARM") >= g("MOT_SPIN_MIN"):
        add("spin", WARN, "Motor idle speeds", f"MOT_SPIN_ARM {g('MOT_SPIN_ARM'):g} >= MOT_SPIN_MIN {g('MOT_SPIN_MIN'):g}",
            "MOT_SPIN_ARM must be lower than MOT_SPIN_MIN (use the motor test to find where they spin reliably).")
    if g("INS_LOG_BAT_MASK") is not None:
        on = g("INS_LOG_BAT_MASK") != 0
        add("batch_log", OK if on else INFO, "Raw-gyro logging (for the notch filter)", "on" if on else "off",
            "" if on else "Set INS_LOG_BAT_MASK = 1 for the first flights: the log doctor then finds the motor noise.")

    mask = g("LOG_BITMASK")
    if mask is not None:
        fast = int(mask) & 1
        add("fast_att_log", OK if fast else INFO, "Fast attitude logging (for tuning)", "on" if fast else "off (10 Hz)",
            "" if fast else "For the first and the tuning flights, add 1 to LOG_BITMASK (ATTITUDE_FAST): the log doctor "
            "can then see fast wobble. Turn it off later; it makes logs bigger.")

    order = {BAD: 0, WARN: 1, INFO: 2, OK: 3}
    todo = [i for i in items if i["status"] in (BAD, WARN)]
    todo.sort(key=lambda i: order[i["status"]])
    verdict = BAD if any(i["status"] == BAD for i in items) else WARN if todo else OK
    return {"verdict": verdict, "items": items, "fix_first": todo[:5], "count": len(p), "partial": partial}
