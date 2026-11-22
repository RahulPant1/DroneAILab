"""Fault-injection lab: break a real ArduCopter on purpose (in SITL) and check that it does the safe thing.

Each scenario starts a fresh SITL (default parameters), takes off to 10 m and holds 20 m north of home in GUIDED
mode, then injects one fault through SITL's own SIM_* parameters (the same switches ArduPilot's developers use in
their autotests). We record what the autopilot BELIEVES (its EKF position) and where the simulator says the drone
REALLY is (SIMSTATE), every mode change and every message, then judge the outcome against what a correctly
configured ArduCopter should do.

Run all scenarios and write the report (docs/results/fault-injection.md + SVG charts):
  .venv/bin/python -m backend.sitl.faults --all
One scenario:
  .venv/bin/python -m backend.sitl.faults gps_loss
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Callable

from backend.sitl import launch
from backend.sitl.fly import Flight

HOLD_ENU = (0.0, 20.0, 10.0)  # where it hovers before the fault: 20 m north of home, 10 m up
SAMPLE_HZ = 5.0
RESULTS_DIR = launch.ROOT / "docs" / "results"


@dataclass
class FaultResult:
    id: str
    title: str
    frame: str
    injected: str  # what we changed
    expected: str  # what a correctly configured ArduCopter should do
    observed: str = ""  # what it actually did (one sentence, filled by the judge)
    kind: str = "safety"  # "safety": it must do the safe thing | "limitation": a known weakness we reproduce
    verdict: str = "not run"  # safety: PASS | FAIL; limitation: CONFIRMED | NOT REPRODUCED; ERROR if the run broke
    metrics: dict = field(default_factory=dict)
    events: list[tuple[float, str]] = field(default_factory=list)  # (seconds after the fault, "mode X" / message)
    samples: dict = field(default_factory=lambda: {k: [] for k in
                                                   ("t_s", "est_e", "est_n", "alt", "true_e", "true_n", "mode")})
    wall_s: float = 0.0
    log: str | None = None  # the ArduPilot flight log of this run (an id for the 🩺 log doctor)

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass
class Fault:
    id: str
    title: str
    injected: str
    expected: str
    inject: dict[str, float]  # SIM_* (or other) parameters set at the moment of the fault
    observe_s: float  # how long to watch afterwards (autopilot seconds), unless it lands first
    judge: Callable[["FaultResult"], tuple[bool, str]]
    frame: str = "quad"
    setup: dict[str, float] = field(default_factory=dict)  # parameters set before take-off
    clear_after_s: float | None = None  # undo the fault after this many seconds (e.g. a short GPS glitch)
    clear: dict[str, float] = field(default_factory=dict)
    after_inject: Callable[[Flight], None] | None = None  # e.g. "now fly out of the fence"
    kind: str = "safety"  # "safety" (PASS/FAIL) or "limitation" (CONFIRMED / NOT REPRODUCED)
    keep_doing: Callable[[Flight], None] | None = None  # called ~5x a second while watching (e.g. keep pushing)


# ---- judges: each returns (passed, one sentence describing what happened) -------------------------------

def _modes_after(r: FaultResult) -> list[str]:
    return [e[1][5:] for e in r.events if e[1].startswith("mode ")]


def _landed_text(r: FaultResult) -> str:
    m = r.metrics
    if not m.get("landed"):
        return "still flying when we stopped watching"
    return f"landed {m['landed_from_home_m']:.1f} m from home ({m['landed_from_fault_m']:.1f} m from where the fault hit)"


def judge_land(r: FaultResult) -> tuple[bool, str]:
    modes, m = _modes_after(r), r.metrics
    ok = "LAND" in modes and m.get("landed") and not m.get("crashed")
    t = next((e[0] for e in r.events if e[1] == "mode LAND"), None)
    when = f"switched to LAND after {t:.1f} s" if t is not None else f"modes after the fault: {modes or 'none'}"
    return bool(ok), f"{when}; {_landed_text(r)}; drifted {m['max_true_dev_m']:.1f} m (true position)."


def judge_rtl(r: FaultResult) -> tuple[bool, str]:
    modes, m = _modes_after(r), r.metrics
    ok = "RTL" in modes and m.get("landed") and m.get("landed_from_home_m", 99) < 3.0 and not m.get("crashed")
    t = next((e[0] for e in r.events if e[1] == "mode RTL"), None)
    when = f"switched to RTL after {t:.1f} s" if t is not None else f"modes after the fault: {modes or 'none'}"
    return bool(ok), f"{when}; {_landed_text(r)}."


def judge_hold(limit_m: float) -> Callable[[FaultResult], tuple[bool, str]]:
    def judge(r: FaultResult) -> tuple[bool, str]:
        m = r.metrics
        failsafe = [x for x in _modes_after(r) if x != "GUIDED"]
        ok = m["max_true_dev_m"] < limit_m and not failsafe and not m.get("crashed")
        extra = f"; mode changes: {failsafe}" if failsafe else "; no failsafe needed"
        return bool(ok), (f"kept flying: at most {m['max_true_dev_m']:.1f} m from its hover point (limit {limit_m:g} m), "
                          f"belief vs truth at most {m['max_est_err_m']:.1f} m{extra}.")
    return judge


def judge_spoof(r: FaultResult) -> tuple[bool, str]:
    m = r.metrics
    ok = 15.0 <= m["max_true_dev_m"] <= 26.0 and not m.get("crashed")
    return bool(ok), (f"believed the false GPS: really flew {m['max_true_dev_m']:.1f} m away from its hover point while "
                      f"its belief was off by up to {m['max_est_err_m']:.1f} m, then came back when the offset ended.")


def _first_impact(r: FaultResult) -> tuple[float, float] | None:
    """(seconds after the fault, impact speed m/s) of the first 'SIM Hit ground at X m/s' message."""
    for t, text in r.events:
        if text.startswith("SIM Hit ground at"):
            try:
                return t, float(text.split("at")[1].split("m/s")[0])
            except (IndexError, ValueError):
                return t, 0.0
    return None


def judge_quad_motor(r: FaultResult) -> tuple[bool, str]:
    hit = _first_impact(r)
    ok = hit is not None and hit[0] < 4.0 and hit[1] > 5.0
    if not hit:
        return False, "it did not hit the ground."
    return ok, f"fell out of the sky: hit the ground {hit[0]:.1f} s after the motor stopped, at {hit[1]:.1f} m/s."


def judge_hexa_motor(r: FaultResult) -> tuple[bool, str]:
    hit = _first_impact(r)
    crash = next((t for t, text in r.events if text.startswith("Crash")), None)
    airborne = hit[0] if hit else r.samples["t_s"][-1]
    yaw_lost = r.metrics["max_true_dev_m"] > 5.0
    ok = airborne >= 5.0 and (crash is not None or yaw_lost)
    tail = f"; ArduPilot's crash check stopped the motors at {crash:.1f} s" if crash is not None else ""
    return ok, (f"stayed in the air {airborne:.1f} s (a quad: under 2 s) but could not hold position: swung up to "
                f"{r.metrics['max_true_dev_m']:.0f} m away while losing heading control{tail}.")


def judge_fence(r: FaultResult) -> tuple[bool, str]:
    m = r.metrics
    rtl = "RTL" in _modes_after(r)
    # It's flying out at 4 m/s when the breach is detected, so it needs a few metres to brake and turn (measured: it
    # peaks ~45 m for a 40 m fence). Pass = never more than 10 m past the fence.
    ok = rtl and m["max_dist_from_home_m"] < 50.0 and m.get("landed") and not m.get("crashed")
    t = next((e[0] for e in r.events if e[1] == "mode RTL"), None)
    what = f"switched to RTL after {t:.1f} s" if rtl else "never switched to RTL"
    s = r.samples  # the "fly to" was sent at t = 0; the velocity push only starts at t = 2 s
    early = [math.hypot(e - HOLD_ENU[0], n - HOLD_ENU[1]) for t_, e, n in zip(s["t_s"], s["true_e"], s["true_n"])
             if t_ < 2.0 and e is not None]
    refused = bool(early) and max(early) < 1.0
    ok = ok and refused
    first = "'fly to 70 m north' was refused (it didn't move)" if refused else "'fly to 70 m north' was NOT refused"
    return bool(ok), (f"{first}; pushed north at 4 m/s instead, it got "
                      f"{m['max_dist_from_home_m']:.1f} m from home (fence 40 m), {what}; {_landed_text(r)}.")


def _fly_out_of_fence(f: Flight) -> None:
    f.send_position_target(0.0, 70.0, 10.0)  # GUIDED quietly refuses a destination outside the fence


def _push_north(f: Flight) -> None:
    """Velocity command: 4 m/s north. GUIDED doesn't check velocity commands against the fence, so this
    really flies out (velocity commands expire after ~3 s, so we keep sending them)."""
    if f.mode == "GUIDED":
        velocity_only = 0b0000_1111_1100_0111  # use vx, vy, vz only
        f.send(f.m.mav.set_position_target_local_ned_send, 0, f.m.target_system, f.m.target_component,
               f.mav.MAV_FRAME_LOCAL_NED, velocity_only, 0, 0, 0, 4.0, 0, 0, 0, 0, 0, 0, 0)


FAULTS: list[Fault] = [
    Fault("gps_loss", "GPS lost", "SIM_GPS1_ENABLE = 0 (the GPS stops giving a fix)",
          "EKF failsafe: without GPS it can't hold position, so it lands where it is (FS_EKF_ACTION = LAND)",
          {"SIM_GPS1_ENABLE": 0}, observe_s=90, judge=judge_land),
    Fault("gps_glitch", "GPS glitch (100 m, 3 s)", "SIM_GPS1_GLTCH_X = 0.0009° (GPS suddenly reports 100 m north) for 3 s",
          "EKF rejects the jump (it disagrees with the IMU by far more than its gate) and the drone stays put",
          {"SIM_GPS1_GLTCH_X": 0.0009}, observe_s=25, clear_after_s=3, clear={"SIM_GPS1_GLTCH_X": 0.0},
          judge=judge_hold(3.0)),
    Fault("gps_spoof", "GPS spoofing (steady 20 m offset)", "SIM_GPS1_GLTCH_X = 0.00018° (GPS reports 20 m north) for 15 s",
          "Known weakness: a steady offset looks like real movement, so the EKF believes it and the drone REALLY "
          "moves ~20 m south (only big or brief jumps are rejected: EK3_GLITCH_RAD = 25 m)",
          {"SIM_GPS1_GLTCH_X": 0.00018}, observe_s=30, clear_after_s=15, clear={"SIM_GPS1_GLTCH_X": 0.0},
          judge=judge_spoof, kind="limitation"),
    Fault("motor_weak", "Weak motor (80 % thrust)", "SIM_ENGINE_FAIL = motor 1, SIM_ENGINE_MUL = 0.8 (one motor loses 20 %)",
          "The attitude controller works the other motors harder and it keeps holding position "
          "(at 60 % it sinks: hovering already needs ~59 % throttle, so there's no margin left)",
          {"SIM_ENGINE_FAIL": 1, "SIM_ENGINE_MUL": 0.8}, observe_s=30, judge=judge_hold(3.0)),
    Fault("motor_quad", "Motor dead (quad)", "SIM_ENGINE_FAIL = motor 1, SIM_ENGINE_MUL = 0 (one motor stops)",
          "Physics: a quad can't fly on 3 motors (it can't balance roll, pitch AND yaw), so it falls",
          {"SIM_ENGINE_FAIL": 1, "SIM_ENGINE_MUL": 0}, observe_s=20, judge=judge_quad_motor, kind="limitation"),
    Fault("motor_hexa", "Motor dead (hexa)", "SIM_ENGINE_FAIL = motor 1, SIM_ENGINE_MUL = 0 on a 6-motor '+' hexacopter",
          "With DEFAULT settings a hexa buys time (it stays up several seconds) but loses heading control and can't hold "
          "position; ArduCopter reports 'Potential Thrust Loss' and its crash check disarms",
          {"SIM_ENGINE_FAIL": 1, "SIM_ENGINE_MUL": 0}, observe_s=30, judge=judge_hexa_motor, frame="hexa",
          kind="limitation"),
    Fault("rc_loss", "Radio link lost", "SIM_RC_FAIL = 1 (no signal from the pilot's transmitter)",
          "Radio failsafe (FS_THR_ENABLE = 1): return to launch and land at home",
          {"SIM_RC_FAIL": 1}, observe_s=90, judge=judge_rtl),
    Fault("low_battery", "Battery low", "SIM_BATT_VOLTAGE = 10.8 V (below BATT_LOW_VOLT = 11.0 V)",
          "Battery failsafe (BATT_FS_LOW_ACT = RTL): after 10 s below the limit, fly home and land",
          {"SIM_BATT_VOLTAGE": 10.8}, observe_s=100, judge=judge_rtl,
          setup={"BATT_LOW_VOLT": 11.0, "BATT_FS_LOW_ACT": 2}),
    Fault("strong_wind", "Strong gusty wind", "SIM_WIND_SPD = 10 m/s from the west, SIM_WIND_TURB = 2",
          "It leans into the wind and still holds position within a couple of metres",
          {"SIM_WIND_DIR": 270, "SIM_WIND_TURB": 2, "SIM_WIND_SPD": 10}, observe_s=30, judge=judge_hold(3.0)),
    Fault("compass_fail", "Compass failure", "SIM_MAG1_FAIL = 1 (the first compass stops)",
          "It switches to its second compass and keeps flying normally",
          {"SIM_MAG1_FAIL": 1}, observe_s=30, judge=judge_hold(3.0)),
    Fault("geofence", "Geofence breach", "FENCE_ENABLE = 1, 40 m circle, fence avoidance off (AVOID_ENABLE = 0); "
          "'fly to 70 m north', then push north at 4 m/s",
          "Layer 1: GUIDED refuses a destination outside the fence. Layer 3: once really outside, the breach triggers "
          "RTL and it lands at home. (Layer 2, avoidance, is switched off here: with defaults it brakes at the edge; "
          "we measured it stopping at 40.6 m)",
          {}, observe_s=90, judge=judge_fence,
          setup={"FENCE_ENABLE": 1, "FENCE_TYPE": 2, "FENCE_RADIUS": 40, "AVOID_ENABLE": 0},
          after_inject=_fly_out_of_fence, keep_doing=_push_north),
]
BY_ID = {f.id: f for f in FAULTS}


def run_scenario(f: Flight, fault: Fault, say: Callable[[str], None] = print) -> FaultResult:
    """Run one scenario on a FRESH, connected SITL (default parameters). Fills and returns a FaultResult."""
    r = FaultResult(fault.id, fault.title, fault.frame, fault.injected, fault.expected, kind=fault.kind)
    t_wall = time.monotonic()
    try:
        f.wait_for_home(90)
        f.wait_until("EKF position", lambda: f.ekf_ok, 90)
        for name, value in fault.setup.items():
            f.set_param(name, value)
        say("Taking off to 10 m...")
        f.set_mode("GUIDED")
        f.arm_when_ready()
        f.takeoff(HOLD_ENU[2])
        say("Flying to the hover point, 20 m north...")
        f.fly_to_local(*HOLD_ENU, settle_s=3.0)
        hold_e, hold_n = HOLD_ENU[:2]
        say(f"Injecting the fault: {fault.injected}")
        t0 = f.t_s
        texts_before, modes_before = len(f.text_log), len(f.mode_log)
        for name, value in fault.inject.items():
            f.set_param(name, value)
        if fault.after_inject:
            fault.after_inject(f)
        cleared = False
        next_sample = 0.0
        landed_at = None
        while f.t_s - t0 < fault.observe_s:
            dt = f.t_s - t0
            if fault.clear_after_s is not None and not cleared and dt >= fault.clear_after_s:
                for name, value in fault.clear.items():
                    f.set_param(name, value)
                cleared = True
                r.events.append((round(dt, 1), "fault cleared"))
            if dt >= next_sample:
                if fault.keep_doing and dt > 2.0:  # first 2 s: see whether the plain "fly to" is refused
                    fault.keep_doing(f)
                with f.lock:
                    est = f.local_enu()
                    true = f.truth_enu()
                    s = r.samples
                    s["t_s"].append(round(dt, 2))
                    s["est_e"].append(round(est[0], 2))
                    s["est_n"].append(round(est[1], 2))
                    s["alt"].append(round(est[2], 2))
                    s["true_e"].append(round(true[0], 2) if true else None)
                    s["true_n"].append(round(true[1], 2) if true else None)
                    s["mode"].append(f.mode)
                next_sample += 1.0 / SAMPLE_HZ
            if not f.armed and landed_at is None:
                landed_at = dt
            if landed_at is not None and dt - landed_at > 2.0:
                break  # landed (or crashed) and disarmed: nothing more to see
            f.sleep(0.02)
        with f.lock:
            r.events += [(round(t - t0, 1), f"mode {m}") for t, m in f.mode_log[modes_before:]]
            r.events += [(round(t - t0, 1), text) for t, text in f.text_log[texts_before:]]
        r.events.sort(key=lambda e: e[0])
        from backend.logdoctor.store import latest_sitl_log_id

        r.log = latest_sitl_log_id()  # SITL starts a new log when it arms, so this is this flight's log
        r.metrics = _metrics(r, f, hold_e, hold_n, landed_at)
        passed, r.observed = fault.judge(r)
        if fault.kind == "limitation":
            r.verdict = "CONFIRMED" if passed else "NOT REPRODUCED"
        else:
            r.verdict = "PASS" if passed else "FAIL"
    except Exception as exc:  # noqa: BLE001  (report it, don't hide it)
        r.verdict, r.observed = "ERROR", f"{type(exc).__name__}: {exc}"
    r.wall_s = round(time.monotonic() - t_wall, 1)
    return r


def _metrics(r: FaultResult, f: Flight, hold_e: float, hold_n: float, landed_at: float | None) -> dict:
    s = r.samples
    pts = [(te, tn) for te, tn in zip(s["true_e"], s["true_n"]) if te is not None]
    est_err = [math.hypot(e - te, n - tn) for e, n, te, tn in zip(s["est_e"], s["est_n"], s["true_e"], s["true_n"])
               if te is not None]
    dev = [math.hypot(e - hold_e, n - hold_n) for e, n in pts]
    crashed = any("crash" in text.lower() for _, text in r.events)
    m = {
        "max_true_dev_m": round(max(dev), 2) if dev else None,  # how far it REALLY got from the hover point
        "max_est_err_m": round(max(est_err), 2) if est_err else None,  # how wrong its belief was at worst
        "max_dist_from_home_m": round(max(math.hypot(e, n) for e, n in pts), 2) if pts else None,
        "min_alt_m": round(min(s["alt"]), 2) if s["alt"] else None,
        "landed": landed_at is not None,
        "landed_after_s": round(landed_at, 1) if landed_at is not None else None,
        "crashed": crashed,
    }
    if landed_at is not None and pts:
        e, n = pts[-1]
        m["landed_from_home_m"] = round(math.hypot(e, n), 2)
        m["landed_from_fault_m"] = round(math.hypot(e - hold_e, n - hold_n), 2)
    return m


def run_fault(fault_id: str, speedup: float = 3.0, say: Callable[[str], None] = print) -> FaultResult:
    """Start a fresh SITL for one scenario, run it, stop SITL."""
    fault = BY_ID[fault_id]
    with launch.start(speedup=speedup, wipe=True, frame=fault.frame) as sitl:
        f = Flight(sitl.url, log=lambda *_: None, timeout_scale=1.0)
        try:
            return run_scenario(f, fault, say)
        finally:
            f.close()


# ---- report -------------------------------------------------------------------------------------------

def svg_paths(r: FaultResult, width: int = 360, height: int = 300) -> str:
    """Top-down view: where it REALLY flew (solid) vs where it THOUGHT it was (dashed), home and hover point."""
    s = r.samples
    true_pts = [(e, n) for e, n in zip(s["true_e"], s["true_n"]) if e is not None]
    est_pts = list(zip(s["est_e"], s["est_n"]))
    every = true_pts + est_pts + [(0.0, 0.0), HOLD_ENU[:2]]
    xs, ys = [p[0] for p in every], [p[1] for p in every]
    span = max(max(xs) - min(xs), max(ys) - min(ys), 10.0) * 1.15
    cx, cy = (max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2
    pad = 28
    k = (min(width, height) - 2 * pad) / span

    def px(e: float, n: float) -> str:
        return f"{width / 2 + (e - cx) * k:.1f},{height / 2 - (n - cy) * k:.1f}"

    def line(pts, color, dash=""):
        if len(pts) < 2:
            return ""
        d = f' stroke-dasharray="{dash}"' if dash else ""
        return f'<polyline points="{" ".join(px(*p) for p in pts)}" fill="none" stroke="{color}" stroke-width="2"{d}/>'

    hx, hy = px(0, 0).split(",")
    fx, fy = px(*HOLD_ENU[:2]).split(",")
    scale_m = 10 if span > 25 else 5
    bar = scale_m * k
    color = {"PASS": "#2a9d8f", "CONFIRMED": "#b5651d", "FAIL": "#e63946"}.get(r.verdict, "#f4a261")
    return f"""<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height + 34}" viewBox="0 0 {width} {height + 34}" font-family="sans-serif" font-size="12">
<rect width="100%" height="100%" fill="#ffffff"/>
<text x="10" y="18" font-weight="bold">{r.title}</text><text x="{width - 10}" y="18" text-anchor="end" fill="{color}" font-weight="bold">{r.verdict}</text>
<g transform="translate(0,22)">
<rect x="0.5" y="0.5" width="{width - 1}" height="{height - 1}" fill="#f8f9fb" stroke="#d0d5dd"/>
{line(est_pts, "#ff8c42", "5,4")}
{line(true_pts, "#1d3557")}
<circle cx="{hx}" cy="{hy}" r="6" fill="none" stroke="#2a9d8f" stroke-width="2"/><text x="{float(hx) + 9}" y="{float(hy) + 4}" fill="#2a9d8f">home</text>
<circle cx="{fx}" cy="{fy}" r="4" fill="#e63946"/><text x="{float(fx) + 8}" y="{float(fy) - 6}" fill="#e63946">fault</text>
<line x1="{width - pad - bar:.1f}" y1="{height - 14}" x2="{width - pad}" y2="{height - 14}" stroke="#555" stroke-width="2"/><text x="{width - pad - bar / 2:.1f}" y="{height - 19}" text-anchor="middle" fill="#555">{scale_m} m</text>
<text x="10" y="{height - 10}" fill="#1d3557">— true path</text><text x="100" y="{height - 10}" fill="#ff8c42">- - what the EKF believed</text>
<text x="10" y="16" fill="#555">N ↑</text>
</g></svg>"""


def write_report(results: list[FaultResult], out_dir: Path = RESULTS_DIR) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "faults").mkdir(exist_ok=True)
    for r in results:
        (out_dir / "faults" / f"{r.id}.svg").write_text(svg_paths(r))
    (out_dir / "faults" / "results.json").write_text(json.dumps([r.to_dict() for r in results], indent=1))
    safety = [r for r in results if r.kind == "safety"]
    limits = [r for r in results if r.kind == "limitation"]
    lines = [
        "# Fault-injection results: real ArduCopter 4.7.3 in SITL",
        "",
        f"**Safety checks: {sum(r.verdict == 'PASS' for r in safety)} of {len(safety)} passed** (the autopilot did the "
        f"safe thing). **Known limitations: {sum(r.verdict == 'CONFIRMED' for r in limits)} of {len(limits)} "
        "reproduced** (weaknesses worth knowing before you fly a real one).",
        "",
        "Generated by `.venv/bin/python -m backend.sitl.faults --all` (see [lesson 13](../learn/13-fault-injection.md)).",
        "",
        "Each run: fresh SITL with default parameters, take off to 10 m, hold 20 m north of home (GUIDED), inject one "
        "fault with SITL's `SIM_*` parameters, then watch. \"True\" positions come from the simulator (SIMSTATE); "
        "\"belief\" is the autopilot's EKF.",
        "",
    ]
    for heading, group in (("Safety checks", safety), ("Known limitations", limits)):
        lines += ["", f"## {heading}", "", "| Fault | Injected | Expected | Observed | Result |", "|---|---|---|---|---|"]
        lines += [f"| **{r.title}** | {r.injected} | {r.expected} | {r.observed} | **{r.verdict}** |" for r in group]
    lines += ["", "## Flight paths (top-down)", ""]
    lines += [f'<img src="faults/{r.id}.svg" width="360" alt="{r.title}: true path vs EKF belief">' for r in results]
    lines += ["", "## Timelines (seconds after the fault)", ""]
    for r in results:
        lines.append(f"**{r.title}**: " + "; ".join(f"{t:+.1f} s {text}" for t, text in r.events[:12]))
        lines.append("")
    path = out_dir / "fault-injection.md"
    path.write_text("\n".join(lines))
    return path


def load_results(path: Path = RESULTS_DIR / "faults" / "results.json") -> list[FaultResult]:
    out = []
    for d in json.loads(path.read_text()):
        d["events"] = [tuple(e) for e in d["events"]]
        out.append(FaultResult(**d))
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ids", nargs="*", help=f"scenarios: {', '.join(BY_ID)}")
    ap.add_argument("--all", action="store_true", help="run every scenario and write the report")
    ap.add_argument("--speedup", type=float, default=3.0)
    ap.add_argument("--report-only", action="store_true", help="rebuild the report from the last saved results")
    args = ap.parse_args(argv)
    if args.report_only:
        print(f"Report: {write_report(load_results())}")
        return 0
    ids = list(BY_ID) if args.all else args.ids
    if not ids or any(i not in BY_ID for i in ids):
        ap.error(f"pick scenarios from: {', '.join(BY_ID)} (or --all)")
    results = []
    for i in ids:
        print(f"\n=== {BY_ID[i].title} ===")
        r = run_fault(i, args.speedup, say=lambda text: print("  " + text))
        print(f"  {r.verdict}: {r.observed}  ({r.wall_s:.0f} s)")
        results.append(r)
    if args.all:
        print(f"\nReport: {write_report(results)}")
    return 0 if all(r.verdict in ("PASS", "CONFIRMED") for r in results) else 1


if __name__ == "__main__":
    sys.exit(main())
