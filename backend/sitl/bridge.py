"""The 🛰 ArduPilot panel in DroneLab's Fly view: one ArduCopter SITL, shared by all browser tabs.

DroneLab starts SITL itself and talks to it on its first MAVLink link (TCP 127.0.0.1:5760). The second link
(127.0.0.1:5762) stays free, so a ground station (MAVProxy, QGroundControl) can watch the same flight.

Positions go to the 3D view in DroneLab's ENU frame, shifted so ArduPilot's home sits on its own pad
PAD_OFFSET_M from DroneLab's start (so the two drones fly side by side). Commands from the panel are
relative to that pad: "fly to x=0, y=10, alt=5" means 10 m north of ArduPilot's own take-off point.
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque

import numpy as np

from backend.sim.frames import quat_from_euler
from backend.sitl import launch
from backend.sitl.fly import Cancelled, Flight, attitude_ned_to_enu, offset_latlon, square_mission

PAD_OFFSET_M = (0.0, -4.0)  # ArduPilot's pad: 4 m south of DroneLab's start pad
MAX_ALT_M = 30.0
MAX_DIST_M = 60.0
COMPARE_DISTANCE_M = 10.0
COMPARE_ALT_M = 5.0
COMPARE_DURATION_S = 15.0
COMPARE_HZ = 10.0


def step_metrics(t_s: np.ndarray, pos_m: np.ndarray, target_m: float) -> dict:
    """How a 'fly to a point target_m away' went: rise time (10% -> 90%), overshoot past the target,
    and settle time (from the command until it stays within 0.2 m)."""
    t_s, pos_m = np.asarray(t_s, float), np.asarray(pos_m, float)
    keep = np.concatenate([[True], np.diff(t_s) > 0]) if len(t_s) else np.zeros(0, bool)  # no repeated times
    t_s, pos_m = t_s[keep], pos_m[keep]
    frac = pos_m / target_m
    i10 = np.flatnonzero(frac >= 0.1)
    i90 = np.flatnonzero(frac >= 0.9)
    rise = round(float(t_s[i90[0]] - t_s[i10[0]]), 2) if len(i10) and len(i90) else None
    overshoot = max(0.0, float(pos_m.max() - target_m))
    outside = np.flatnonzero(np.abs(pos_m - target_m) > 0.2)
    if len(outside) == 0:
        settle = 0.0
    elif outside[-1] == len(t_s) - 1:
        settle = None  # never settled within the window
    else:
        settle = float(t_s[outside[-1] + 1])
    peak_speed = float(np.max(np.abs(np.gradient(pos_m, t_s)))) if len(t_s) > 2 else None
    return {"rise_s": rise, "overshoot_m": round(overshoot, 3), "settle_s": settle,
            "peak_speed_m_s": None if peak_speed is None else round(peak_speed, 2)}


def dronelab_step(distance_m: float = COMPARE_DISTANCE_M, alt_m: float = COMPARE_ALT_M,
                  duration_s: float = COMPARE_DURATION_S) -> dict:
    """The same 'fly distance_m north' on DroneLab's own controller (basic preset, perfect sensors, no wind)."""
    from backend.sim.config import get_preset
    from backend.sim.control import Setpoint
    from backend.sim.physics import State
    from backend.sim.runner import run_headless

    cfg = get_preset("basic")
    sp = Setpoint(alt_m=alt_m, x_m=0.0, y_m=distance_m, position_control=True)
    r = run_headless(cfg, sp, duration_s, 0, initial_state=State.hovering(cfg, alt_m))
    t = r.channel("t_s")
    keep = np.arange(0, len(t), max(1, int(round(50 / COMPARE_HZ))))  # 50 Hz telemetry -> 10 Hz like ArduPilot
    return {"t_s": t[keep].round(2).tolist(), "north_m": r.channel("y_m")[keep].round(3).tolist()}


class SitlBridge:
    def __init__(self) -> None:
        self.lock = threading.RLock()
        self.sitl: launch.Sitl | None = None
        self.flight: Flight | None = None
        self.state = "off"  # off | starting | waiting_gps | ready | error
        self.message = ""
        self.action: str | None = None  # what the action thread is doing ("take-off", "mission", ...)
        self.action_thread: threading.Thread | None = None
        self.log: deque[str] = deque(maxlen=8)  # recent autopilot messages, newest last
        self.compare: dict | None = None
        self.speedup = 1.0

    # --- life cycle ------------------------------------------------------------------------------
    def start(self, speedup: float = 1.0) -> str:
        with self.lock:
            if self.state in ("starting", "waiting_gps", "ready"):
                return "ArduPilot is already running."
            if not launch.is_built():
                self.state, self.message = "error", "ArduCopter SITL isn't built yet: run ./scripts/sitl_setup.sh"
                return self.message
            if not 0.5 <= speedup <= 5:
                raise ValueError("speedup must be between 0.5 and 5")
            self.state, self.message, self.compare, self.speedup = "starting", "Starting ArduCopter SITL...", None, speedup
            self.log.clear()
        threading.Thread(target=self._start_worker, args=(speedup,), name="sitl-start", daemon=True).start()
        return "Starting ArduPilot (the real ArduCopter flight code). GPS + EKF take about 30–45 s to get ready."

    def _start_worker(self, speedup: float) -> None:
        try:
            sitl = launch.start(speedup=speedup, wipe=True, own_group=False)  # dies with the server's group
            with self.lock:
                if self.state != "starting":  # stopped while starting
                    sitl.stop()
                    return
                self.sitl = sitl
            flight = Flight(sitl.url, log=self._log, timeout_scale=max(1.0, 1.0 / speedup))
            with self.lock:
                self.flight = flight
                self.state, self.message = "waiting_gps", "Connected. Waiting for GPS + EKF (pre-arm checks)..."
            flight.wait_for_home(timeout_s=90)
            flight.wait_until("EKF position", lambda: flight.ekf_ok, 90)
            with self.lock:
                self.state, self.message = "ready", "Ready: GPS + EKF have a position. Press Take off."
        except Cancelled:
            pass
        except Exception as exc:  # noqa: BLE001  (report anything to the UI)
            with self.lock:
                if self.state != "off":
                    self.state, self.message = "error", f"Couldn't start ArduPilot: {exc}"
            self._teardown()

    def stop(self) -> str:
        with self.lock:
            if self.state == "off":
                return "ArduPilot isn't running."
            self.state, self.message, self.action = "off", "Stopped.", None
        self._teardown()
        return "ArduPilot stopped."

    def _teardown(self) -> None:
        with self.lock:
            flight, sitl = self.flight, self.sitl
            self.flight, self.sitl = None, None
        if flight:
            flight.close()
        if self.action_thread and self.action_thread.is_alive():
            self.action_thread.join(timeout=3)
        if sitl:
            sitl.stop()

    def _log(self, text: str) -> None:
        text = text.strip()
        if text.startswith("autopilot: "):
            self.log.append(text[len("autopilot: "):])

    # --- actions (each runs in its own thread; a new one replaces the old) -------------------------
    def _run(self, name: str, work) -> str:
        with self.lock:
            flight = self.flight
            if self.state != "ready" or flight is None:
                raise ValueError("ArduPilot isn't ready yet." if self.state != "off" else "Start ArduPilot first.")
            old = self.action_thread
        if old and old.is_alive():
            flight.cancel.set()
            old.join(timeout=3)
        flight.cancel.clear()

        def body() -> None:
            try:
                work(flight)
                with self.lock:
                    if self.action == name:
                        self.action = None
            except Cancelled:
                pass
            except Exception as exc:  # noqa: BLE001
                with self.lock:
                    self.message, self.action = f"{name}: {exc}", None

        with self.lock:
            self.action = name
            self.message = f"{name}..."
            self.action_thread = threading.Thread(target=body, name=f"sitl-{name}", daemon=True)
            self.action_thread.start()
        return f"ArduPilot: {name}."

    def _say(self, text: str) -> None:
        with self.lock:
            self.message = text

    def _ensure_flying(self, f: Flight, alt_m: float) -> None:
        if f.armed and f.pos["rel_alt_m"] > 0.5:
            if f.mode != "GUIDED":
                f.set_mode("GUIDED")
            return
        f.set_mode("GUIDED")
        self._say("Arming (waits for the pre-arm checks)...")
        f.arm_when_ready()
        self._say(f"Taking off to {alt_m:.0f} m...")
        f.takeoff(alt_m)

    def takeoff(self, alt_m: float = 5.0) -> str:
        alt_m = _check_alt(alt_m)

        def work(f: Flight) -> None:
            self._ensure_flying(f, alt_m)
            f.send_position_target(*f.local_enu()[:2], alt_m)  # also changes height if it was already flying
            self._say(f"Holding at {alt_m:.0f} m (GUIDED).")

        return self._run("take-off", work)

    def goto(self, x_m: float, y_m: float, alt_m: float) -> str:
        alt_m = _check_alt(alt_m)
        if math.hypot(x_m, y_m) > MAX_DIST_M:
            raise ValueError(f"Stay within {MAX_DIST_M:.0f} m of ArduPilot's pad.")

        def work(f: Flight) -> None:
            self._ensure_flying(f, alt_m)
            self._say(f"Flying to x={x_m:g}, y={y_m:g}, alt={alt_m:g} m (GUIDED)...")
            err = f.fly_to_local(x_m, y_m, alt_m, settle_s=2.0)
            self._say(f"Arrived: {err:.2f} m from the target.")

        return self._run("fly-to", work)

    def hover(self) -> str:
        def work(f: Flight) -> None:
            if not f.armed:
                self._say("It's on the ground.")
                return
            if f.mode != "GUIDED":
                f.set_mode("GUIDED")
            e, n, u = f.local_enu()
            f.send_position_target(e, n, max(u, 1.0))
            self._say("Holding position (GUIDED).")

        return self._run("hover", work)

    def square(self, side_m: float = 10.0, alt_m: float = 5.0) -> str:
        alt_m = _check_alt(alt_m)
        if not 4 <= side_m <= 40:
            raise ValueError("side_m must be 4–40 m")

        def work(f: Flight) -> None:
            self._ensure_flying(f, alt_m)
            points = square_mission(*f.home, side_m=side_m, alt_m=alt_m)
            self._say("Uploading a 4-waypoint mission...")
            f.upload_mission(points)
            self._say("Flying the mission in AUTO...")
            errors = f.fly_mission(points)
            self._say("Mission done: corners " + ", ".join(f"{e:.1f}" for e in errors) + " m off (WP_RADIUS_M = 2 m).")

        return self._run("mission", work)

    def set_mode(self, mode: str) -> str:
        if mode not in ("RTL", "LAND"):
            raise ValueError("mode must be RTL or LAND")
        label = {"RTL": "return home", "LAND": "land"}[mode]

        def work(f: Flight) -> None:
            f.set_mode(mode)
            self._say(f"{mode}: {'flying home, then landing' if mode == 'RTL' else 'landing here'}...")
            f.wait_until("landed", lambda: not f.armed, 180)
            self._say("Landed and disarmed.")

        return self._run(label, work)

    def run_compare(self) -> str:
        """Fly the same 'COMPARE_DISTANCE_M north' step on ArduPilot (live) and DroneLab (headless), then plot both."""

        def work(f: Flight) -> None:
            self._ensure_flying(f, COMPARE_ALT_M)
            self._say("Comparing: lining up over the pad...")
            f.fly_to_local(0.0, 0.0, COMPARE_ALT_M, settle_s=3.0)
            n0 = f.local_enu()[1]
            self._say(f"Comparing: 'fly {COMPARE_DISTANCE_M:.0f} m north', recording {COMPARE_DURATION_S:.0f} s...")
            f.send_position_target(0.0, COMPARE_DISTANCE_M, COMPARE_ALT_M)
            t0 = f.t_s
            ts, ns = [], []
            while f.t_s - t0 < COMPARE_DURATION_S:
                with f.lock:
                    t = round(f.t_s - t0, 2)
                    if not ts or t > ts[-1]:  # one sample per new autopilot time
                        ts.append(t)
                        ns.append(round(f.local_enu()[1] - n0, 3))
                f.sleep(1.0 / COMPARE_HZ / self.speedup)
            ours = dronelab_step()
            with self.lock:
                self.compare = {
                    "distance_m": COMPARE_DISTANCE_M,
                    "ardupilot": {"t_s": ts, "north_m": ns, **step_metrics(ts, ns, COMPARE_DISTANCE_M)},
                    "dronelab": {**ours, **step_metrics(ours["t_s"], ours["north_m"], COMPARE_DISTANCE_M)},
                }
            self._say("Comparison done: see the chart.")

        return self._run("compare", work)

    # --- the picture for the UI -------------------------------------------------------------------
    def status(self) -> dict | None:
        with self.lock:
            if self.state == "off" and not self.message:
                return None
            out: dict = {"state": self.state, "message": self.message, "action": self.action,
                         "log": list(self.log), "pad_m": list(PAD_OFFSET_M), "compare": self.compare,
                         "speedup": self.speedup}
            f = self.flight
        if f is None:
            return out
        with f.lock:
            enu = f.local_enu()
            roll, pitch, yaw = attitude_ned_to_enu(*f.att)
            out.update({
                "mode": f.mode,
                "armed": f.armed,
                "gps_fix": f.gps["fix"],  # 3 = 3D fix
                "sats": f.gps["sats"],
                "ekf_ok": f.ekf_ok,
                "t_s": round(f.t_s, 2),
                "motor_pct": [round(p, 1) for p in f.motor_pct],
                "euler_rad": [round(roll, 4), round(pitch, 4), round(yaw, 4)],
                "heading_deg": round(math.degrees(f.att[2]) % 360, 1),
                "vel_m_s": [round(f.pos["ve"], 3), round(f.pos["vn"], 3), round(-f.pos["vd"], 3)],
            })
        out["quat"] = [round(float(v), 5) for v in quat_from_euler(roll, pitch, yaw)]
        if enu is not None:
            out["local_m"] = [round(v, 3) for v in enu]
            out["pos_m"] = [round(enu[0] + PAD_OFFSET_M[0], 3), round(enu[1] + PAD_OFFSET_M[1], 3), round(enu[2], 3)]
        return out

    def handle(self, args: dict) -> str:
        """One WebSocket {"cmd": "sitl", "action": ...} message."""
        action = args.get("action")
        if action == "start":
            return self.start(float(args.get("speedup", 1.0)))
        if action == "stop":
            return self.stop()
        if action == "takeoff":
            return self.takeoff(float(args.get("alt_m", 5.0)))
        if action == "goto":
            return self.goto(float(args["x_m"]), float(args["y_m"]), float(args.get("alt_m", 5.0)))
        if action == "hover":
            return self.hover()
        if action == "square":
            return self.square(float(args.get("side_m", 10.0)), float(args.get("alt_m", 5.0)))
        if action in ("rtl", "land"):
            return self.set_mode(action.upper())
        if action == "compare":
            return self.run_compare()
        raise ValueError(f"Unknown ArduPilot action: {action!r}")


def _check_alt(alt_m: float) -> float:
    if not 1.0 <= alt_m <= MAX_ALT_M:
        raise ValueError(f"alt_m must be 1–{MAX_ALT_M:.0f} m")
    return float(alt_m)


_ = offset_latlon  # re-exported for tests
BRIDGE = SitlBridge()  # one per server process
