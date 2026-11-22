"""Fly ArduCopter SITL from a script, the way a ground station does it, and report what happened.

The flight:
  1. Wait for the autopilot's heartbeat, then its pre-arm checks (GPS lock, EKF ready).
  2. GUIDED mode: arm, take off to 10 m, fly to a point 20 m north (a "Fly to", like DroneLab's).
  3. Upload a 4-waypoint square mission and fly it in AUTO mode.
  4. RTL (return to launch): it flies home and lands by itself (it first climbs to RTL_ALT_M = 15 m,
     unless it's already close to home, as it is at the end of this square).

Run (starts SITL itself, then stops it):
  .venv/bin/python -m backend.sitl.fly
  .venv/bin/python -m backend.sitl.fly --speedup 5        # 5x faster than real time
  .venv/bin/python -m backend.sitl.fly --connect tcp:127.0.0.1:5760   # SITL you started yourself

The same Flight class drives the 🛰 ArduPilot panel in DroneLab's Fly view (backend/sitl/bridge.py).

Frames: DroneLab uses ENU (x east, y north, z up). ArduPilot uses NED (x north, y east, z DOWN) and
compass headings (0 = north, clockwise). enu_to_ned() and attitude_ned_to_enu() convert.
"""

from __future__ import annotations

import argparse
import math
import queue
import sys
import threading
import time
from dataclasses import dataclass, field

EARTH_RADIUS_M = 6378137.0


def enu_to_ned(east_m: float, north_m: float, up_m: float) -> tuple[float, float, float]:
    """DroneLab's (east, north, up) -> ArduPilot's (north, east, down). Swap x/y, flip z."""
    return north_m, east_m, -up_m


def attitude_ned_to_enu(roll_rad: float, pitch_rad: float, yaw_rad: float) -> tuple[float, float, float]:
    """ArduPilot's roll/pitch/yaw (body x forward, y RIGHT, z DOWN; yaw = compass heading) -> DroneLab's
    (body x forward, y LEFT, z UP; yaw measured from east, anticlockwise).

    Roll is the same. Pitch flips sign (nose-up is positive in NED, negative in DroneLab's frame).
    Yaw: heading 0 (north) is DroneLab yaw 90°, and headings grow clockwise, so yaw = 90° - heading.
    """
    yaw = math.pi / 2 - yaw_rad
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))  # wrap to -180..180°
    return roll_rad, -pitch_rad, yaw


def offset_latlon(lat_deg: float, lon_deg: float, north_m: float, east_m: float) -> tuple[float, float]:
    """A point north_m / east_m away from (lat, lon). Flat-earth approximation: fine for a few km."""
    dlat = north_m / EARTH_RADIUS_M
    dlon = east_m / (EARTH_RADIUS_M * math.cos(math.radians(lat_deg)))
    return lat_deg + math.degrees(dlat), lon_deg + math.degrees(dlon)


def latlon_to_ne(lat_deg: float, lon_deg: float, home_lat_deg: float, home_lon_deg: float) -> tuple[float, float]:
    """(north, east) metres of a lat/lon from home. The inverse of offset_latlon."""
    n = math.radians(lat_deg - home_lat_deg) * EARTH_RADIUS_M
    e = math.radians(lon_deg - home_lon_deg) * EARTH_RADIUS_M * math.cos(math.radians(home_lat_deg))
    return n, e


def square_mission(lat_deg: float, lon_deg: float, side_m: float = 20.0, alt_m: float = 10.0) -> list[tuple[float, float, float]]:
    """Corners of a square starting north of home, flown clockwise (N, NE, E, back home), as lat/lon/alt."""
    corners_ne = [(side_m, 0.0), (side_m, side_m), (0.0, side_m), (0.0, 0.0)]
    return [(*offset_latlon(lat_deg, lon_deg, n, e), alt_m) for n, e in corners_ne]


class Cancelled(Exception):
    """A newer command replaced the one that was waiting (only used by the live panel)."""


@dataclass
class FlightReport:
    steps: list[tuple[str, float]] = field(default_factory=list)  # (what, autopilot seconds since we connected)
    messages: list[str] = field(default_factory=list)  # STATUSTEXT from the autopilot
    max_alt_m: float = 0.0
    fly_to_error_m: float = float("nan")
    corner_errors_m: list[float] = field(default_factory=list)
    landed_from_home_m: float = float("nan")
    ok: bool = False

    def text(self) -> str:
        lines = ["", "ArduCopter SITL flight report", "-" * 40]
        lines += [f"  {t:6.1f} s  {what}" for what, t in self.steps]
        lines += [
            "-" * 40,
            f"  Highest altitude:            {self.max_alt_m:5.1f} m",
            f"  'Fly to' arrival error:      {self.fly_to_error_m:5.2f} m",
            "  Mission corner errors:       " + ", ".join(f"{e:.2f}" for e in self.corner_errors_m) + " m",
            f"  Landed this far from home:   {self.landed_from_home_m:5.2f} m",
            f"  Result: {'OK' if self.ok else 'FAILED'}",
        ]
        return "\n".join(lines)


class Flight:
    """A small ground station: one MAVLink connection, read by a background thread that keeps an up-to-date
    picture of the drone, plus commands that wait on what the autopilot reports."""

    def __init__(self, url: str, log=print, timeout_scale: float = 1.0):
        from pymavlink import mavutil  # imported here so the rest of DroneLab never needs pymavlink

        self.mavutil = mavutil
        self.mav = mavutil.mavlink
        self.log = log
        self.k = timeout_scale  # >1 when SITL runs slower than real time
        self.report = FlightReport()
        self.lock = threading.Lock()  # guards the picture below
        self.send_lock = threading.Lock()
        self.cancel = threading.Event()  # set = abandon whatever command is waiting
        self.closed = threading.Event()
        self.m = mavutil.mavlink_connection(url, source_system=255, source_component=190)
        if self.m.wait_heartbeat(timeout=30) is None:
            raise TimeoutError(f"No heartbeat from {url}")
        # The picture of the drone (updated by the reader thread)
        self.boot_ms0: int | None = None
        self.t_s = 0.0  # autopilot time since we connected (follows --speedup)
        self.pos = {"lat": 0.0, "lon": 0.0, "rel_alt_m": 0.0, "vn": 0.0, "ve": 0.0, "vd": 0.0}
        self.att = (0.0, 0.0, 0.0)  # NED roll, pitch, yaw (rad)
        self.home: tuple[float, float] | None = None
        self.armed = False
        self.mode = self.m.flightmode
        self.gps = {"fix": 0, "sats": 0}
        self.ekf_ok = False  # EKF has a good horizontal position
        self.motor_pct = [0.0, 0.0, 0.0, 0.0]
        self.truth: dict | None = None  # SITL only: where the drone REALLY is (SIMSTATE), vs pos = what the EKF believes
        self.params: dict[str, float] = {}  # from PARAM_VALUE (only the ones we set or asked for)
        self.mode_log: list[tuple[float, str]] = []  # (autopilot time, mode) each time the mode changes
        self.text_log: list[tuple[float, str]] = []  # (autopilot time, STATUSTEXT)
        self.battery_v: float | None = None
        self.acks: dict[int, int] = {}  # command id -> result, from COMMAND_ACK
        self.reached: list[int] = []  # mission item numbers, from MISSION_ITEM_REACHED
        self.mission_msgs: queue.Queue = queue.Queue()  # MISSION_REQUEST(_INT) / MISSION_ACK during an upload
        self.m.mav.request_data_stream_send(self.m.target_system, self.m.target_component,
                                            mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)  # 10 Hz of everything
        self.reader = threading.Thread(target=self._read_loop, name="mavlink-reader", daemon=True)
        self.reader.start()
        self.log(f"Connected to ArduCopter (system {self.m.target_system}).")

    # --- reading (background thread) -------------------------------------------------------------
    def _read_loop(self) -> None:
        while not self.closed.is_set():
            try:
                msg = self.m.recv_match(blocking=True, timeout=0.2)
            except (OSError, ValueError):
                break  # connection closed
            if msg is not None:
                self._handle(msg)
        self.closed.set()

    def _handle(self, msg) -> None:
        kind = msg.get_type()
        mav = self.mav
        with self.lock:
            if hasattr(msg, "time_boot_ms"):
                if self.boot_ms0 is None:
                    self.boot_ms0 = msg.time_boot_ms
                self.t_s = (msg.time_boot_ms - self.boot_ms0) / 1000.0
            if kind == "GLOBAL_POSITION_INT":
                self.pos = {"lat": msg.lat / 1e7, "lon": msg.lon / 1e7, "rel_alt_m": msg.relative_alt / 1000.0,
                            "vn": msg.vx / 100.0, "ve": msg.vy / 100.0, "vd": msg.vz / 100.0}
                self.report.max_alt_m = max(self.report.max_alt_m, self.pos["rel_alt_m"])
            elif kind == "ATTITUDE":
                self.att = (msg.roll, msg.pitch, msg.yaw)
            elif kind == "HOME_POSITION":
                self.home = (msg.latitude / 1e7, msg.longitude / 1e7)
            elif kind == "HEARTBEAT" and msg.get_srcSystem() == self.m.target_system and msg.type != mav.MAV_TYPE_GCS:
                self.armed = bool(msg.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED)
                self.mode = self.m.flightmode
                if not self.mode_log or self.mode_log[-1][1] != self.mode:
                    self.mode_log.append((self.t_s, self.mode))
            elif kind == "SIMSTATE":  # the simulator's ground truth (lat/lng in degE7)
                self.truth = {"lat": msg.lat / 1e7, "lon": msg.lng / 1e7}
            elif kind == "PARAM_VALUE":
                self.params[msg.param_id] = msg.param_value
            elif kind == "SYS_STATUS":
                self.battery_v = msg.voltage_battery / 1000.0 if msg.voltage_battery not in (0, 65535) else None
            elif kind == "GPS_RAW_INT":
                self.gps = {"fix": msg.fix_type, "sats": msg.satellites_visible}
            elif kind == "EKF_STATUS_REPORT":
                self.ekf_ok = bool(msg.flags & mav.EKF_POS_HORIZ_ABS)
            elif kind == "SERVO_OUTPUT_RAW":  # PWM 1000 (stopped) .. 2000 (full)
                pwm = [msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw]
                self.motor_pct = [max(0.0, min(100.0, (p - 1000) / 10.0)) for p in pwm]
            elif kind == "COMMAND_ACK":
                self.acks[msg.command] = msg.result
            elif kind == "MISSION_ITEM_REACHED":
                if msg.seq not in self.reached:
                    self.reached.append(msg.seq)
            elif kind in ("MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"):
                self.mission_msgs.put(msg)
            elif kind == "STATUSTEXT":
                text = msg.text.strip()
                if text and (not self.report.messages or self.report.messages[-1] != text):
                    self.report.messages.append(text)
                    self.text_log.append((self.t_s, text))
                    self.log(f"  autopilot: {text}")

    def close(self) -> None:
        self.cancel.set()
        self.closed.set()
        self.reader.join(timeout=2)
        try:
            self.m.close()
        except OSError:
            pass

    # --- waiting -------------------------------------------------------------------------------
    def sleep(self, seconds: float) -> None:
        if self.cancel.wait(seconds):
            raise Cancelled()

    def wait_until(self, what: str, cond, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s * self.k
        while True:
            with self.lock:
                if cond():
                    return
            if self.closed.is_set():
                raise ConnectionError("Lost the connection to the autopilot")
            if time.monotonic() > deadline:
                raise TimeoutError(f"Timed out waiting for: {what}")
            self.sleep(0.05)

    def step(self, what: str) -> None:
        self.report.steps.append((what, self.t_s))
        self.log(f"[{self.t_s:6.1f} s] {what}")

    def distance_to_m(self, lat: float, lon: float) -> float:
        n, e = latlon_to_ne(self.pos["lat"], self.pos["lon"], lat, lon)
        return math.hypot(n, e)

    def local_enu(self) -> tuple[float, float, float] | None:
        """Where the drone is, in metres east / north / up of home (DroneLab's frame). None before home is known."""
        if self.home is None:
            return None
        n, e = latlon_to_ne(self.pos["lat"], self.pos["lon"], *self.home)
        return e, n, self.pos["rel_alt_m"]

    # --- commands --------------------------------------------------------------------------------
    def send(self, fn, *args) -> None:
        with self.send_lock:
            fn(*args)

    def command(self, cmd: int, *params: float, timeout_s: float = 5.0) -> bool:
        """Send a COMMAND_LONG and wait for its ACK. Returns True if the autopilot accepted it."""
        p = list(params) + [0.0] * (7 - len(params))
        with self.lock:
            self.acks.pop(cmd, None)
        self.send(self.m.mav.command_long_send, self.m.target_system, self.m.target_component, cmd, 0, *p)
        try:
            self.wait_until(f"reply to command {cmd}", lambda: cmd in self.acks, timeout_s)
        except TimeoutError:
            return False
        return self.acks.get(cmd) == self.mav.MAV_RESULT_ACCEPTED

    def set_param(self, name: str, value: float, timeout_s: float = 5.0) -> None:
        """Change one autopilot parameter and wait until it confirms the new value (PARAM_VALUE)."""
        deadline = time.monotonic() + timeout_s * self.k
        while True:
            self.send(self.m.mav.param_set_send, self.m.target_system, self.m.target_component, name.encode(),
                      float(value), self.mav.MAV_PARAM_TYPE_REAL32)
            try:
                self.wait_until(f"param {name}", lambda: abs(self.params.get(name, math.nan) - value) < 1e-4, 1.0)
                return
            except TimeoutError:
                if time.monotonic() > deadline:
                    raise TimeoutError(f"The autopilot didn't accept {name} = {value}") from None

    def truth_enu(self) -> tuple[float, float] | None:
        """Where the drone REALLY is (east, north of home), from the simulator. None outside SITL."""
        if self.home is None or self.truth is None:
            return None
        n, e = latlon_to_ne(self.truth["lat"], self.truth["lon"], *self.home)
        return e, n

    def set_mode(self, mode: str) -> None:
        mode_id = self.m.mode_mapping()[mode]
        self.send(self.m.set_mode, mode_id)
        self.wait_until(f"mode {mode}", lambda: self.mode == mode, 10)
        self.step(f"Mode {mode}")

    def wait_for_home(self, timeout_s: float = 60.0) -> None:
        deadline = time.monotonic() + timeout_s * self.k
        while self.home is None:
            if time.monotonic() > deadline:
                raise TimeoutError("No home position (GPS never got a fix)")
            self.command(self.mav.MAV_CMD_GET_HOME_POSITION, timeout_s=0.5)
            self.sleep(0.5)

    def arm_when_ready(self, timeout_s: float = 120.0) -> None:
        """Keep trying to arm. Until GPS and the EKF are happy, the autopilot refuses ("PreArm: ...")."""
        deadline = time.monotonic() + timeout_s * self.k
        while not self.armed:
            if time.monotonic() > deadline:
                raise TimeoutError("Never passed the pre-arm checks: " + " | ".join(self.report.messages[-3:]))
            self.command(self.mav.MAV_CMD_COMPONENT_ARM_DISARM, 1)
            self.sleep(1.0)
        self.step("Armed (pre-arm checks passed)")

    def takeoff(self, alt_m: float, tries: int = 5) -> None:
        """GUIDED take-off. Right after arming ArduCopter sometimes refuses it for a moment, so try a few times."""
        for attempt in range(tries):
            if self.command(self.mav.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, alt_m):
                break
            if not self.armed:
                raise RuntimeError("Take-off refused: the motors disarmed (" + " | ".join(self.report.messages[-2:]) + ")")
            self.sleep(1.0)
        else:
            raise RuntimeError("Take-off was refused (" + " | ".join(self.report.messages[-2:]) + ")")
        self.wait_until(f"altitude {alt_m} m", lambda: self.pos["rel_alt_m"] >= 0.95 * alt_m, 60)
        self.step(f"Reached {self.pos['rel_alt_m']:.1f} m")

    def send_position_target(self, east_m: float, north_m: float, up_m: float) -> None:
        """GUIDED "fly to", given in DroneLab's ENU and sent in ArduPilot's NED (relative to the EKF origin = home)."""
        n, e, d = enu_to_ned(east_m, north_m, up_m)
        position_only = 0b0000_1111_1111_1000  # ignore velocity, acceleration, yaw: just "go there"
        self.send(self.m.mav.set_position_target_local_ned_send, 0, self.m.target_system, self.m.target_component,
                  self.mav.MAV_FRAME_LOCAL_NED, position_only, n, e, d, 0, 0, 0, 0, 0, 0, 0, 0)

    def fly_to_local(self, east_m: float, north_m: float, up_m: float, settle_s: float = 3.0) -> float:
        """Fly to a point and wait there. Returns the horizontal error after settling."""
        self.send_position_target(east_m, north_m, up_m)
        assert self.home is not None
        lat, lon = offset_latlon(*self.home, north_m, east_m)
        self.wait_until("arrival", lambda: self.distance_to_m(lat, lon) < 1.0, 60)
        self.sleep(settle_s)  # let it settle, then measure
        err = self.distance_to_m(lat, lon)
        self.step(f"Arrived at {east_m:.0f} m east, {north_m:.0f} m north (error {err:.2f} m)")
        return err

    def upload_mission(self, points: list[tuple[float, float, float]]) -> None:
        """Mission upload, MAVLink style: send the count, then answer each MISSION_REQUEST with that item."""
        mav = self.mav
        assert self.home is not None
        # Item 0 is home (ArduPilot overwrites it), then a take-off, then the waypoints.
        items = [(mav.MAV_CMD_NAV_WAYPOINT, *self.home, 0.0), (mav.MAV_CMD_NAV_TAKEOFF, *self.home, points[0][2])]
        items += [(mav.MAV_CMD_NAV_WAYPOINT, lat, lon, alt) for lat, lon, alt in points]
        while not self.mission_msgs.empty():
            self.mission_msgs.get_nowait()
        self.send(self.m.mav.mission_count_send, self.m.target_system, self.m.target_component, len(items),
                  mav.MAV_MISSION_TYPE_MISSION)
        deadline = time.monotonic() + 20 * self.k
        while True:
            if time.monotonic() > deadline:
                raise TimeoutError("Mission upload timed out")
            try:
                msg = self.mission_msgs.get(timeout=0.5)
            except queue.Empty:
                continue
            if msg.get_type() == "MISSION_ACK":
                if msg.type != mav.MAV_MISSION_ACCEPTED:
                    raise RuntimeError(f"Mission rejected (code {msg.type})")
                break
            cmd, lat, lon, alt = items[msg.seq]
            self.send(self.m.mav.mission_item_int_send, self.m.target_system, self.m.target_component, msg.seq,
                      mav.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, cmd, 0, 1, 0, 0, 0, 0,
                      int(lat * 1e7), int(lon * 1e7), alt, mav.MAV_MISSION_TYPE_MISSION)
        self.step(f"Uploaded a {len(points)}-waypoint mission")

    def fly_mission(self, points: list[tuple[float, float, float]]) -> list[float]:
        """AUTO mode: the autopilot flies the uploaded mission by itself; we just watch and measure.

        Mission item n (n >= 2) is waypoint n-1. ArduCopter says "reached" when the drone gets within
        WP_RADIUS_M of it (2 m by default) and moves on without stopping, so a metre or two off is normal.
        """
        with self.lock:
            self.reached.clear()
        self.set_mode("AUTO")
        errors: list[float] = []
        deadline = time.monotonic() + 180 * self.k
        while len(errors) < len(points):
            if time.monotonic() > deadline:
                raise TimeoutError("Mission didn't finish")
            self.sleep(0.05)
            with self.lock:
                new = [q for q in self.reached if q >= 2][len(errors):]
                for seq in new:
                    lat, lon, _ = points[seq - 2]
                    errors.append(self.distance_to_m(lat, lon))
                    self.step(f"Reached waypoint {seq - 1} of {len(points)} ({errors[-1]:.2f} m off)")
        return errors

    def return_to_launch(self) -> float:
        self.set_mode("RTL")
        self.wait_until("landing and disarm", lambda: not self.armed, 120)
        assert self.home is not None
        err = self.distance_to_m(*self.home)
        self.step(f"Landed and disarmed, {err:.2f} m from home")
        return err


def fly(url: str, log=print, timeout_scale: float = 1.0) -> FlightReport:
    """The whole demo flight. Raises on any failure; returns the report on success."""
    f = Flight(url, log=log, timeout_scale=timeout_scale)
    try:
        f.step("Connected, waiting for GPS + EKF")
        f.wait_for_home()
        f.set_mode("GUIDED")
        f.arm_when_ready()
        f.takeoff(10.0)
        f.report.fly_to_error_m = f.fly_to_local(east_m=0.0, north_m=20.0, up_m=10.0)
        assert f.home is not None
        points = square_mission(*f.home, side_m=20.0, alt_m=10.0)
        f.upload_mission(points)
        f.report.corner_errors_m = f.fly_mission(points)
        f.report.landed_from_home_m = f.return_to_launch()
        f.report.ok = True
        return f.report
    finally:
        f.close()


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--connect", help="MAVLink address of a SITL you started yourself, e.g. tcp:127.0.0.1:5760")
    ap.add_argument("--speedup", type=float, default=1.0, help="simulation speed vs real time (when we start SITL)")
    args = ap.parse_args(argv)
    if args.connect:
        report = fly(args.connect)
    else:
        from backend.sitl import launch

        print(f"Starting ArduCopter SITL (speedup {args.speedup})...")
        with launch.start(speedup=args.speedup) as sitl:
            print(f"SITL is up on {sitl.url} (log: {sitl.log_path})")
            report = fly(sitl.url)
    print(report.text())
    return 0 if report.ok else 1


if __name__ == "__main__":
    sys.exit(main())
