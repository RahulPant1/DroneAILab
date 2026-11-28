"""From a Drone Builder design to ArduPilot: a SITL frame model (the "digital twin") and a starting parameter file.

1. sitl_model(design)  -> the JSON frame model ArduCopter SITL can fly instead of its generic quad
   (`--model x:<file>.json`, loaded by libraries/SITL/SIM_Frame.cpp): mass, size, hover throttle, battery, rotor disc
   area, drag. So ArduPilot's own physics flies *your* build: same weight, same thrust margin, same battery.
2. param_file(design)  -> a .param file to load in QGroundControl / Mission Planner on the real drone, for the FIRST
   flights: frame type, battery and its failsafes, radio-loss and EKF failsafes, a geofence, return-home height,
   motor hover throttle, and gyro filters / acceleration limits scaled for the prop size.

Every name below was checked against ArduCopter 4.7.3's source (several changed in 4.7: RTL_ALT is now RTL_ALT_M in
metres; ATC_ACCEL_*_MAX in centidegrees is now ATC_ACC_*_MAX in deg/s/s; ARMING_CHECK became ARMING_SKIPCHK).
The names, values and MAVLink types were read back from a 4.7.3 SITL booted with this file (test_twin.py). The prop-size values follow ArduPilot's
"Setting the Aircraft Up for Tuning" page: INS_GYRO_FILTER 80 Hz at 5", 40 Hz at 10", 20 Hz at 20" (= 400 / inches),
and ATC_ACC_R/P_MAX 1100 deg/s/s at 10", 500 at 20", 200 at 30".

What this file does NOT do (it can't know your hardware): calibrations (accelerometer, compass, radio, ESCs), the
ESC protocol and outputs, serial ports, the battery monitor's sensor scaling, flight modes, and the notch filter
(measure that from the first hover with the 🩺 log doctor, lesson 25). It's a safe start, not a tune.
"""

from __future__ import annotations

import math

from .config import GRAVITY_M_S2

PROP_EXPO = 0.65  # MOT_THST_EXPO (ArduCopter's default): how thrust bends with throttle
SITL_HOME_ALT_M = 584.0  # launch.HOME's ground height: the model's reference altitude
FENCE_RADIUS_M = 100.0
FENCE_ALT_MAX_M = 60.0  # well under 120 m (400 ft), the usual legal ceiling for small drones (India: check the
                        # DGCA Digital Sky airspace map for your spot; many areas near airports are lower or red)
RTL_ALT_M = 15.0
CELL_LOW_V, CELL_CRIT_V, CELL_FULL_V, CELL_MIN_V = 3.5, 3.3, 4.2, 3.3
LOW_RESERVE, CRIT_RESERVE = 0.30, 0.15  # failsafe when this much of the pack is left


def hover_throttle(thrust_to_weight: float, expo: float = PROP_EXPO) -> float:
    """ArduPilot's throttle at hover (MOT_THST_HOVER, 0-1). Thrust vs throttle is (1-e)*t + e*t^2 (MOT_THST_EXPO),
    and hover needs thrust = weight = 1 / (T/W) of full thrust. Solve for t."""
    f = 1.0 / thrust_to_weight
    if expo <= 0:
        return f
    return (-(1 - expo) + math.sqrt((1 - expo) ** 2 + 4 * expo * f)) / (2 * expo)


def _ref_speed_m_s(cfg, angle_deg: float = 45.0) -> float:
    """Top speed leaning at angle_deg, from DroneLab's drag model: m g tan(angle) = lin v + quad v^2."""
    force = cfg.mass_kg * GRAVITY_M_S2 * math.tan(math.radians(angle_deg))
    a, b = cfg.drag_quadratic_n_s2_m2, cfg.drag_linear_n_s_m
    return (-b + math.sqrt(b * b + 4 * a * force)) / (2 * a) if a > 0 else force / max(b, 1e-6)


def gyro_filter_hz(prop_in: float) -> int:
    """INS_GYRO_FILTER for a prop size: ArduPilot's table (80 Hz at 5", 40 at 10", 20 at 20") = 400 / inches.
    Kept within 20-80 Hz: the table doesn't go below 5" (smaller props: measure, then raise)."""
    return int(round(min(80.0, max(20.0, 400.0 / prop_in)) / 5.0) * 5)


def accel_max_degss(prop_in: float) -> int:
    """ATC_ACC_R_MAX / ATC_ACC_P_MAX for a prop size, between ArduPilot's table points (10": 1100, 20": 500,
    30": 200 deg/s/s). Below 10" it stays at 1100 (the default): small props could go higher, after tuning."""
    pts = [(10.0, 1100.0), (20.0, 500.0), (30.0, 200.0)]
    if prop_in <= pts[0][0]:
        return int(pts[0][1])
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        if prop_in <= x1:
            return int(round((y0 + (y1 - y0) * (prop_in - x0) / (x1 - x0)) / 10.0) * 10)
    return int(pts[-1][1])


def sitl_model(design: dict) -> dict:
    """The SITL frame model (JSON fields of SIM_Frame.cpp's Model) for a Builder design."""
    from .builder import INCH, PROPS

    cfg, bat, rep, parts = design["config"], design["battery"], design["report"], design["parts"]
    prop_in = PROPS[parts["prop"]].diameter_in
    tw = rep["thrust_to_weight"]
    return {
        "mass": round(cfg.mass_kg, 3),
        "diagonal_size": round(2 * cfg.arm_length_m, 3),  # motor to opposite motor
        "refSpd": round(_ref_speed_m_s(cfg), 2), "refAngle": 45.0,
        # SIM_Frame.cpp uses refCurrent x refVoltage as the HOVER power (its variable is called hover_power), whatever
        # the example files' comments say about a test at refAngle. So: the Builder's hover current. (Found by
        # reading the source after a first version scaled it for a 45° lean and SITL drew 60 % too much.)
        "refVoltage": round(bat.cells * 3.85, 2), "refCurrent": rep["hover_current_a"],
        "refAlt": SITL_HOME_ALT_M, "refTempC": 25.0,
        "refBatRes": round(bat.internal_resistance_ohm, 4),
        "maxVoltage": round(bat.cells * CELL_FULL_V, 2),
        "battCapacityAh": round(bat.capacity_mah / 1000.0, 3),
        "hoverThrOut": round(hover_throttle(tw), 3),
        "propExpo": PROP_EXPO,
        "refRotRate": 120.0,
        "pwmMin": 1000.0, "pwmMax": 2000.0, "spin_min": 0.15, "spin_max": 0.95, "slew_max": 150.0,
        "disc_area": round(4 * math.pi * (prop_in * INCH / 2) ** 2, 4),
        "mdrag_coef": 0.2,
        "num_motors": 4,
    }


def params(design: dict) -> list[tuple[str, float, str]]:
    """(name, value, why) for the first flights of this build. Names checked against ArduCopter 4.7.3."""
    from .builder import PROPS

    bat, rep, parts = design["battery"], design["report"], design["parts"]
    cells, cap = bat.cells, bat.capacity_mah
    prop_in = PROPS[parts["prop"]].diameter_in
    gyro = gyro_filter_hz(prop_in)
    acc = accel_max_degss(prop_in)
    hover = hover_throttle(rep["thrust_to_weight"])
    return [
        # frame
        ("FRAME_CLASS", 1, "quad"),
        ("FRAME_TYPE", 1, "X layout (DroneLab's builds are X quads)"),
        # motors
        ("MOT_THST_HOVER", round(hover, 3), f"hover throttle predicted by the Builder (T/W {rep['thrust_to_weight']:.2f})"),
        ("MOT_HOVER_LEARN", 2, "keep learning the real hover throttle in flight, and save it"),
        ("MOT_THST_EXPO", PROP_EXPO, "thrust curve (ArduCopter default)"),
        ("MOT_SPIN_ARM", 0.10, "motors idle when armed: CHECK with props off (MOT_SPIN_ARM must spin all motors smoothly)"),
        ("MOT_SPIN_MIN", 0.15, "lowest throttle in flight: CHECK it is above MOT_SPIN_ARM and keeps every motor turning"),
        ("MOT_BAT_VOLT_MAX", round(cells * CELL_FULL_V, 2), f"{cells}S full"),
        ("MOT_BAT_VOLT_MIN", round(cells * CELL_MIN_V, 2), f"{cells}S empty: thrust is scaled between these"),
        # battery + its failsafe
        ("BATT_CAPACITY", int(cap), f"{cap:.0f} mAh pack"),
        ("BATT_LOW_VOLT", round(cells * CELL_LOW_V, 2), f"{CELL_LOW_V} V per cell under load"),
        ("BATT_CRT_VOLT", round(cells * CELL_CRIT_V, 2), f"{CELL_CRIT_V} V per cell under load"),
        ("BATT_LOW_MAH", int(round(cap * LOW_RESERVE)), f"{LOW_RESERVE:.0%} of the pack left"),
        ("BATT_CRT_MAH", int(round(cap * CRIT_RESERVE)), f"{CRIT_RESERVE:.0%} of the pack left"),
        ("BATT_FS_LOW_ACT", 2, "low battery: RTL (come home)"),
        ("BATT_FS_CRT_ACT", 1, "critical battery: land where it is"),
        # other failsafes
        ("FS_THR_ENABLE", 1, "radio lost: RTL (also needs the receiver set to cut throttle / no pulses on signal loss)"),
        ("FS_EKF_ACTION", 1, "position estimate bad: land (if the mode needs position)"),
        # geofence + return home
        ("FENCE_ENABLE", 1, "geofence on"),
        ("FENCE_TYPE", 3, "max altitude + circle around home"),
        ("FENCE_ALT_MAX", FENCE_ALT_MAX_M, "metres above home: stay well below the legal ceiling"),
        ("FENCE_RADIUS", FENCE_RADIUS_M, "metres from home: keep it in sight"),
        ("FENCE_ACTION", 1, "fence breached: RTL (or land)"),
        ("RTL_ALT_M", RTL_ALT_M, "come home at this height (4.7: metres; older firmware used RTL_ALT in cm)"),
        # filters + limits for the prop size (ArduPilot's tuning setup table)
        ("INS_GYRO_FILTER", gyro, f"gyro low-pass for {prop_in:g}\" props (400 / inches)"),
        ("INS_ACCEL_FILTER", 10, "accelerometer low-pass (tuning setup page)"),
        ("ATC_RAT_RLL_FLTD", gyro / 2, "roll D-term filter = half the gyro filter"),
        ("ATC_RAT_RLL_FLTT", gyro / 2, "roll target filter"),
        ("ATC_RAT_PIT_FLTD", gyro / 2, "pitch D-term filter"),
        ("ATC_RAT_PIT_FLTT", gyro / 2, "pitch target filter"),
        ("ATC_RAT_YAW_FLTT", gyro / 2, "yaw target filter"),
        ("ATC_ACC_R_MAX", acc, f"roll acceleration limit for {prop_in:g}\" props (deg/s/s)"),
        ("ATC_ACC_P_MAX", acc, f"pitch acceleration limit for {prop_in:g}\" props (deg/s/s)"),
        # logging for the first flights (the log doctor's vibration + notch check)
        ("INS_LOG_BAT_MASK", 1, "log raw gyro for an FFT (🩺 log doctor: find the motor noise, set a notch)"),
        ("INS_LOG_BAT_OPT", 4, "before AND after the filters"),
        ("ARMING_SKIPCHK", 0, "skip NO pre-arm checks (4.7's replacement for ARMING_CHECK): never skip one to make it arm"),
    ]


# MAVLink parameter types (MAV_PARAM_TYPE) as ArduCopter 4.7.3 reports them: QGroundControl's file format needs them
# and refuses a parameter whose type doesn't match. 2 = INT8, 4 = INT16, 6 = INT32, 9 = REAL32.
PARAM_TYPES = {
    "FRAME_CLASS": 2, "FRAME_TYPE": 2, "MOT_HOVER_LEARN": 2, "BATT_CAPACITY": 6, "BATT_FS_LOW_ACT": 2,
    "BATT_FS_CRT_ACT": 2, "FS_THR_ENABLE": 2, "FS_EKF_ACTION": 2, "FENCE_ENABLE": 2, "FENCE_TYPE": 2,
    "FENCE_ACTION": 2, "INS_GYRO_FILTER": 4, "INS_ACCEL_FILTER": 4, "INS_LOG_BAT_MASK": 2, "INS_LOG_BAT_OPT": 2,
    "ARMING_SKIPCHK": 6,
}  # everything else is REAL32 (9)


def _header(design: dict, how: str) -> list[str]:
    p, rep = design["parts"], design["report"]
    return [
        "# DroneLab: starting parameters for the FIRST flights of this build (ArduCopter 4.7)",
        f"# {design['config'].name}",
        f"# Builder: {rep['mass_g']} g, T/W {rep['thrust_to_weight']}, hover {rep['hover_throttle_pct']} % thrust, "
        f"~{rep['flight_time_min']} min, verdict: {rep['verdict']}",
        f"# parts: {', '.join(f'{k}={v}' for k, v in p.items())}",
        f"# {how}",
        "# NOT included (do these in the ground station): accelerometer / compass / radio / ESC calibration, ESC",
        "# protocol + motor outputs + motor order test (props OFF), battery monitor sensor scaling, flight modes.",
        "# First flight: props on only after a props-off motor test; open field, Stabilize / AltHold, low and slow.",
    ]


def _value(v) -> str:
    return f"{v:g}" if isinstance(v, float) else str(v)


def qgc_param_file(design: dict) -> str:
    """The same parameters in QGroundControl's format (tab-separated, with each parameter's MAVLink type)."""
    lines = _header(design, "QGroundControl: Vehicle Setup > Parameters > Tools > Load from file.")
    lines += ["# Vehicle-Id\tComponent-Id\tName\tValue\tType"]
    for name, value, why in params(design):
        lines.append(f"# {why}")
        lines.append(f"1\t1\t{name}\t{_value(value)}\t{PARAM_TYPES.get(name, 9)}")
    return "\n".join(lines) + "\n"


def param_file(design: dict) -> str:
    """Mission Planner / MAVProxy format: NAME,VALUE lines, comments on their own lines (not after a value)."""
    lines = _header(design, "Mission Planner: Config > Full Parameter List > Load from file. MAVProxy: param load <file>.")
    for name, value, why in params(design):
        lines.append(f"# {why}")
        lines.append(f"{name},{_value(value)}")
    return "\n".join(lines) + "\n"
