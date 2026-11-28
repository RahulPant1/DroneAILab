"""🛠 -> 🛰 Digital twin: fly YOUR Drone Builder design on real ArduCopter (SITL), with the parameter file it would get.

  1. The design becomes a SITL frame model (backend/sim/apparams.sitl_model): ArduPilot's own physics flies an X quad
     with your build's mass, size, thrust margin (hover throttle), battery and rotor area.
  2. The starting parameter file (apparams.params) is loaded at boot, exactly as you would load it on the real drone.
  3. Test flight, measured from ArduPilot's own telemetry:
       hover          throttle, current and battery use -> flight time, vs what the Builder predicted
       radio lost     SIM_RC_FAIL = 1 away from home: does the failsafe bring it back (RTL)?
       battery low    the pack 'empties' (BATT_LOW_VOLT raised above the voltage): RTL, as the file says?
       AUTOTUNE       optional (--autotune): ArduPilot's own tuning flight finds rate PIDs for this airframe.

It tests the PARAMETERS and ArduPilot's behaviour on an airframe like yours. It can't test your real motors, ESCs,
wiring, vibration or compass: that's what the props-off checks and the first flight log (🩺 log doctor) are for.

    .venv/bin/python -m backend.sitl.twin                       # default build (450 mm, 10" props, 3S 3000 mAh)
    .venv/bin/python -m backend.sitl.twin --parts '{"frame": "racer_250", "motor": "2306_2450kv", "prop": "5in", "cells": 4, "capacity_mah": 1500}'
    .venv/bin/python -m backend.sitl.twin --autotune --write   # + AUTOTUNE, and save docs/results/twin.{md,json}
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Callable

from backend.sim import apparams, builder

RESULTS = Path(__file__).resolve().parents[2] / "docs" / "results"
OUT = Path(__file__).resolve().parents[2] / "data" / "twin"
HOVER_ALT_M = 10.0
HOVER_S = 30.0
AWAY_M = 30.0  # failsafe tests start this far north of home
AUTOTUNE_PIDS = ("ATC_RAT_RLL_P", "ATC_RAT_RLL_I", "ATC_RAT_RLL_D", "ATC_RAT_PIT_P", "ATC_RAT_PIT_I", "ATC_RAT_PIT_D",
                 "ATC_RAT_YAW_P", "ATC_RAT_YAW_I", "ATC_ANG_RLL_P", "ATC_ANG_PIT_P", "ATC_ANG_YAW_P")


def build_files(parts: dict, out_dir: Path = OUT) -> dict:
    """Design the drone and write its twin model + parameter files. Returns the paths and the design."""
    d = builder.design(parts)
    if d["report"]["thrust_to_weight"] < 1.2:
        raise builder.DesignError(f"T/W {d['report']['thrust_to_weight']}: this build can't fly well enough to test.")
    out_dir.mkdir(parents=True, exist_ok=True)
    model, prm, qgc = out_dir / "model.json", out_dir / "dronelab.param", out_dir / "dronelab.params"
    model.write_text(json.dumps(apparams.sitl_model(d), indent=1))
    prm.write_text(apparams.param_file(d))
    qgc.write_text(apparams.qgc_param_file(d))
    return {"design": d, "model": model, "param": prm, "qgc": qgc}


def _wait_mode(f, modes: tuple[str, ...], timeout_s: float) -> float | None:
    """Seconds until the autopilot is in one of `modes` (None = it never was)."""
    t0 = f.t_s
    deadline = time.monotonic() + timeout_s * f.k
    while time.monotonic() < deadline:
        if f.mode in modes:
            return round(f.t_s - t0, 1)
        f.sleep(0.2)
    return None


def _wait_for(f, cond, timeout_s: float) -> bool:
    """Like Flight.wait_until, but returns False instead of raising: a test records what happened."""
    deadline = time.monotonic() + timeout_s * f.k
    while time.monotonic() < deadline:
        if cond():
            return True
        f.sleep(0.5)
    return False


def _launch_point(f) -> tuple[float, float]:
    """Where it is now (call on the ground, just before take-off): ArduPilot's HOME is wherever it last armed."""
    return f.pos["lat"], f.pos["lon"]


def _dist(f, point) -> float:
    return f.distance_to_m(*point)


def twin_boot_params(design: dict) -> dict:
    """The parameters SITL boots with: the starting file, plus the simulator's own battery set to the same pack."""
    bat = design["battery"]
    boot = {name: value for name, value, _ in apparams.params(design)}
    boot.update({"SIM_BATT_VOLTAGE": round(bat.cells * apparams.CELL_FULL_V, 2), "SIM_BATT_CAP_AH": bat.capacity_mah / 1000.0})
    return boot


def fly(files: dict, speedup: float = 1.0, autotune: bool = False, say: Callable[[str], None] = print) -> dict:
    from . import launch
    from .fly import Flight

    d = files["design"]
    bat = d["battery"]
    boot = twin_boot_params(d)
    res: dict = {"parts": d["parts"], "builder": d["report"], "model": json.loads(files["model"].read_text()), "tests": {}}
    sitl = launch.start(speedup=speedup, extra_params=boot, model_json=files["model"])
    try:
        f = Flight(sitl.url, log=lambda _t: None, timeout_scale=max(1.0, 1.0 / speedup))
        try:
            say("ArduPilot is flying your design: waiting for GPS + EKF...")
            f.wait_for_home(90)
            f.wait_until("EKF position", lambda: f.ekf_ok, 90)
            f.set_mode("GUIDED")
            launch_at = _launch_point(f)
            f.arm_when_ready()
            f.takeoff(HOVER_ALT_M)
            # 1. hover
            f.sleep(5)
            thr, amps, volts = [], [], []
            t0 = f.t_s
            pct0 = f.battery_pct
            while f.t_s - t0 < HOVER_S:
                f.sleep(1.0)
                if f.throttle_pct is not None:
                    thr.append(f.throttle_pct)
                if f.battery_a is not None:
                    amps.append(f.battery_a)
                if f.battery_v is not None:
                    volts.append(f.battery_v)
            hover_a = sum(amps) / len(amps) if amps else None
            used = (pct0 - f.battery_pct) if pct0 is not None and f.battery_pct is not None else None
            usable = builder.USABLE_FRACTION
            res["tests"]["hover"] = {
                "throttle_pct": round(sum(thr) / len(thr), 1) if thr else None,
                "current_a": round(hover_a, 2) if hover_a else None,
                "voltage_v": round(sum(volts) / len(volts), 2) if volts else None,
                "flight_time_min": round(bat.capacity_mah / 1000.0 * usable / hover_a * 60.0, 1) if hover_a else None,
                "battery_pct_used": used,
            }
            h = res["tests"]["hover"]
            say(f"  hover: throttle {h['throttle_pct']} % (Builder: {apparams.hover_throttle(d['report']['thrust_to_weight']) * 100:.0f} %), "
                f"{h['current_a']} A (Builder: {d['report']['hover_current_a']} A) -> ~{h['flight_time_min']} min "
                f"(Builder: {d['report']['flight_time_min']} min)")
            # 2. radio lost, away from home
            f.fly_to_local(0.0, AWAY_M, HOVER_ALT_M, settle_s=2.0)
            f.set_param("SIM_RC_FAIL", 1)
            t_rtl = _wait_mode(f, ("RTL", "LAND"), 30)
            _wait_for(f, lambda: _dist(f, launch_at) < 3.0, 180)
            res["tests"]["radio_lost"] = {"mode": f.mode, "seconds_to_rtl": t_rtl, "back_home_m": round(_dist(f, launch_at), 1),
                                          "ok": t_rtl is not None and f.mode in ("RTL", "LAND")}
            say(f"  radio lost: {f.mode} after {t_rtl} s, back over home ({res['tests']['radio_lost']['back_home_m']} m)")
            f.set_param("SIM_RC_FAIL", 0)
            _wait_for(f, lambda: not f.armed, 180)
            if f.armed:  # still flying (e.g. hovering over home): bring it down before the next test
                f.set_mode("LAND")
                f.wait_until("landed", lambda: not f.armed, 180)
            # 3. battery low, away from home (last: ArduPilot's battery failsafe stays latched until a reboot)
            f.set_mode("GUIDED")
            f.arm_when_ready()
            f.takeoff(HOVER_ALT_M)
            f.fly_to_local(0.0, AWAY_M, HOVER_ALT_M, settle_s=2.0)
            low = round((f.battery_v or bat.cells * 3.8) + 1.0, 2)
            f.set_param("BATT_LOW_VOLT", low)  # 'the pack is now below the low-voltage line'
            t_rtl = _wait_mode(f, ("RTL", "LAND"), 40)
            res["tests"]["battery_low"] = {"mode": f.mode, "seconds_to_rtl": t_rtl, "ok": f.mode == "RTL",
                                           "messages": [m for m in f.report.messages if "atter" in m][-2:]}
            say(f"  battery low: {f.mode} after {t_rtl} s ({'; '.join(res['tests']['battery_low']['messages'])})")
            _wait_for(f, lambda: not f.armed, 240)
            f.set_param("BATT_LOW_VOLT", boot["BATT_LOW_VOLT"])
        finally:
            f.close()
    finally:
        sitl.stop()
    if autotune:  # a dedicated tuning flight on a fresh boot, as you'd do on the real drone
        res["tests"]["autotune"] = _autotune_session(files, boot, speedup, say)
    res["files"] = {k: str(files[k].relative_to(files[k].parents[2])) for k in ("model", "param", "qgc")}
    return res


class Sticks:
    """A transmitter held still: RC override sent 10 times a second from a background thread. ArduPilot drops an
    override it hasn't heard for a moment, and SITL's own 'radio' then has the throttle at the bottom: in AltHold or
    AUTOTUNE that means 'descend' (found the hard way: the twin fell out of AltHold)."""

    def __init__(self, f):
        import threading

        self.f, self.throttle, self.stop_flag = f, 1500, threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def _run(self):
        f = self.f
        while not self.stop_flag.is_set():
            f.send(f.m.mav.rc_channels_override_send, f.m.target_system, f.m.target_component,
                   1500, 1500, self.throttle, 1500, 0, 0, 0, 0)
            time.sleep(0.1)
        f.send(f.m.mav.rc_channels_override_send, f.m.target_system, f.m.target_component, 0, 0, 0, 0, 0, 0, 0, 0)

    def __enter__(self):
        self.thread.start()
        time.sleep(0.5)
        return self

    def __exit__(self, *exc):
        self.stop_flag.set()
        self.thread.join(2)


def _autotune_session(files: dict, boot: dict, speedup: float, say) -> dict:
    from . import launch
    from .fly import Flight

    sitl = launch.start(speedup=speedup, extra_params=boot, model_json=files["model"])
    try:
        f = Flight(sitl.url, log=lambda _t: None, timeout_scale=max(1.0, 1.0 / speedup))
        try:
            f.wait_for_home(90)
            f.wait_until("EKF position", lambda: f.ekf_ok, 90)
            f.set_mode("GUIDED")
            f.arm_when_ready()
            f.takeoff(HOVER_ALT_M)
            a = _autotune(f, say)
            say(f"  AUTOTUNE: {a['result']} (roll rate P {a['before'].get('ATC_RAT_RLL_P'):.3g} -> "
                f"{a['after'].get('ATC_RAT_RLL_P'):.3g})")
            return a
        finally:
            f.close()
    finally:
        sitl.stop()


def _autotune(f, say) -> dict:
    """ArduPilot's AUTOTUNE flight: it twitches each axis and measures the response to find rate and angle gains."""
    for name in AUTOTUNE_PIDS:
        f.m.mav.param_request_read_send(f.m.target_system, f.m.target_component, name.encode(), -1)
    f.sleep(2)
    before = {n: f.params.get(n) for n in AUTOTUNE_PIDS}
    f.set_param("AUTOTUNE_AXES", 7)  # roll, pitch, yaw
    done = None
    with Sticks(f) as sticks:
        f.set_mode("ALT_HOLD")
        f.sleep(2)
        f.set_mode("AUTOTUNE")
        say("  AUTOTUNE: ArduPilot is twitching each axis to find the gains (several minutes of flight)...")
        deadline = time.monotonic() + 1500 * f.k
        while time.monotonic() < deadline and done is None:
            f.sleep(1.0)
            for m in f.report.messages[-5:]:
                if "AutoTune: Success" in m or "AutoTune: Failed" in m:
                    done = m
        # Land and disarm while still in AUTOTUNE: that is how ArduPilot saves the new gains (switching mode first
        # would throw them away). Throttle low = descend.
        sticks.throttle = 1100
        f.wait_until("landed", lambda: not f.armed, 240)
    for name in AUTOTUNE_PIDS:
        f.m.mav.param_request_read_send(f.m.target_system, f.m.target_component, name.encode(), -1)
    f.sleep(2)
    after = {n: f.params.get(n) for n in AUTOTUNE_PIDS}
    return {"result": done, "before": before, "after": after, "ok": bool(done and "Success" in done)}


def write_results(res: dict) -> Path:
    RESULTS.mkdir(parents=True, exist_ok=True)
    (RESULTS / "twin.json").write_text(json.dumps(res, indent=1))
    b, t = res["builder"], res["tests"]
    h = t["hover"]
    hover_pred = apparams.hover_throttle(b["thrust_to_weight"]) * 100
    md = [
        "# Digital twin: a Drone Builder design flown on real ArduCopter (SITL)",
        "",
        f"Build: `{res['parts']}` ({b['mass_g']} g, T/W {b['thrust_to_weight']}). ArduCopter 4.7.3 SITL flew it as a "
        "custom frame model (`data/twin/model.json`), booted with the generated starting parameters "
        "(`data/twin/dronelab.param`, `.params` for QGroundControl). Generated by `python -m backend.sitl.twin --write`.",
        "",
        "| Check | Builder predicted | ArduPilot SITL measured |",
        "|---|---|---|",
        f"| Hover throttle | {hover_pred:.0f} % | {h['throttle_pct']} % |",
        f"| Hover current | {b['hover_current_a']} A | {h['current_a']} A *(not independent: SITL is given this as its hover power)* |",
        f"| Flight time (to {builder.USABLE_FRACTION:.0%} of the pack) | {b['flight_time_min']} min | {h['flight_time_min']} min *(follows from the current)* |",
        f"| Radio lost {AWAY_M:.0f} m from home | RTL (FS_THR_ENABLE 1) | {t['radio_lost']['mode']} after {t['radio_lost']['seconds_to_rtl']} s, "
        f"back within {t['radio_lost']['back_home_m']} m of home |",
        f"| Battery low {AWAY_M:.0f} m from home | RTL (BATT_FS_LOW_ACT 2) | {t['battery_low']['mode']} after {t['battery_low']['seconds_to_rtl']} s |",
    ]
    if "autotune" in t:
        a = t["autotune"]
        md += ["", f"**AUTOTUNE** (a separate tuning flight on a fresh boot): {a['result'] or 'did not finish'}.", "",
               "These are ArduPilot's gains for the SIMULATED airframe, which responds perfectly crisply (no flexing arms, "
               "no vibration, no motor lag beyond the model). Don't copy them to the real drone: they show what AUTOTUNE "
               "does. On the real drone, fly AUTOTUNE yourself once the basics are proven (calm day, open field, enough "
               "battery for ~10 minutes, after the notch filter is set).", "",
               "| Gain | Before (ArduCopter default) | After AUTOTUNE (simulated airframe) |", "|---|---|---|"]
        for n in AUTOTUNE_PIDS:
            if a["before"].get(n) is not None and a["after"].get(n) is not None:
                md.append(f"| `{n}` | {a['before'][n]:.4g} | {a['after'][n]:.4g} |")
    md += [
        "",
        "- What is really tested: ArduPilot itself (its controllers, modes and failsafes) and the parameter file, on an "
        "airframe with your build's weight, thrust margin and battery. The hover throttle is ArduPilot's own control "
        "loop finding it. Hover current and flight time are NOT an independent check: SITL's power model is set from "
        "the Builder's hover current. The real number comes from your first flight log (BAT.Curr at hover).",
        "- It can't check your real hardware: motor direction and order, ESC calibration, vibration, compass "
        "interference. Do the props-off motor test, then read the first hover's log in the 🩺 log doctor.",
    ]
    (RESULTS / "twin.md").write_text("\n".join(md) + "\n")
    return RESULTS / "twin.md"


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--parts", default="{}", help="Builder parts as JSON (default: the Builder's default build)")
    ap.add_argument("--speedup", type=float, default=1.0)
    ap.add_argument("--autotune", action="store_true", help="also fly ArduPilot's AUTOTUNE (several minutes)")
    ap.add_argument("--write", action="store_true", help="save docs/results/twin.md and .json")
    args = ap.parse_args(argv)
    files = build_files(json.loads(args.parts))
    print(f"Wrote {files['param']} (Mission Planner / MAVProxy) and {files['qgc']} (QGroundControl).")
    res = fly(files, args.speedup, args.autotune)
    if args.write:
        print(f"Wrote {write_results(res)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
