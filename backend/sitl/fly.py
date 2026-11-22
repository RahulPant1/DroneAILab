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

Frames: DroneLab uses ENU (x east, y north, z up). ArduPilot uses NED (x north, y east, z DOWN) and
compass headings (0 = north, clockwise). enu_to_ned() converts.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass, field

EARTH_RADIUS_M = 6378137.0


def enu_to_ned(east_m: float, north_m: float, up_m: float) -> tuple[float, float, float]:
    """DroneLab's (east, north, up) -> ArduPilot's (north, east, down). Swap x/y, flip z."""
    return north_m, east_m, -up_m


def offset_latlon(lat_deg: float, lon_deg: float, north_m: float, east_m: float) -> tuple[float, float]:
    """A point north_m / east_m away from (lat, lon). Flat-earth approximation: fine for a few km."""
    dlat = north_m / EARTH_RADIUS_M
    dlon = east_m / (EARTH_RADIUS_M * math.cos(math.radians(lat_deg)))
    return lat_deg + math.degrees(dlat), lon_deg + math.degrees(dlon)


def square_mission(lat_deg: float, lon_deg: float, side_m: float = 20.0, alt_m: float = 10.0) -> list[tuple[float, float, float]]:
    """Corners of a square starting north of home, flown clockwise (N, NE, E, back home), as lat/lon/alt."""
    corners_ne = [(side_m, 0.0), (side_m, side_m), (0.0, side_m), (0.0, 0.0)]
    return [(*offset_latlon(lat_deg, lon_deg, n, e), alt_m) for n, e in corners_ne]


@dataclass
class FlightReport:
    steps: list[tuple[str, float]] = field(default_factory=list)  # (what, sim seconds since start)
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
    """A small ground station: one MAVLink connection, plus helpers that wait on what the autopilot reports."""

    def __init__(self, url: str, log=print, timeout_scale: float = 1.0):
        from pymavlink import mavutil  # imported here so the rest of DroneLab never needs pymavlink

        self.mavutil = mavutil
        self.log = log
        self.k = timeout_scale  # >1 when SITL runs slower than real time
        self.report = FlightReport()
        self.m = mavutil.mavlink_connection(url, source_system=255, source_component=190)
        hb = self.m.wait_heartbeat(timeout=30)
        if hb is None:
            raise TimeoutError(f"No heartbeat from {url}")
        self.boot_ms0: int | None = None
        self.t_s = 0.0  # autopilot time since we connected (follows --speedup)
        self.pos = {"lat": 0.0, "lon": 0.0, "rel_alt_m": 0.0}
        self.home: tuple[float, float] | None = None
        self.armed = False
        self.acks: dict[int, int] = {}  # command id -> result, from COMMAND_ACK
        self.reached: list[int] = []  # mission item numbers, from MISSION_ITEM_REACHED
        # Ask for position and status at 10 Hz (a real radio link would ask for less).
        self.m.mav.request_data_stream_send(self.m.target_system, self.m.target_component,
                                            mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
        self.log(f"Connected to ArduCopter (system {self.m.target_system}).")

    # --- reading -------------------------------------------------------------------------------
    def pump(self, wait_s: float = 0.1) -> None:
        """Read everything the autopilot sent and update our picture of it."""
        deadline = time.monotonic() + wait_s
        while True:
            msg = self.m.recv_match(blocking=True, timeout=max(0.0, deadline - time.monotonic()))
            if msg is None:
                return
            kind = msg.get_type()
            if hasattr(msg, "time_boot_ms"):
                if self.boot_ms0 is None:
                    self.boot_ms0 = msg.time_boot_ms
                self.t_s = (msg.time_boot_ms - self.boot_ms0) / 1000.0
            if kind == "GLOBAL_POSITION_INT":
                self.pos = {"lat": msg.lat / 1e7, "lon": msg.lon / 1e7, "rel_alt_m": msg.relative_alt / 1000.0}
                self.report.max_alt_m = max(self.report.max_alt_m, self.pos["rel_alt_m"])
            elif kind == "HOME_POSITION":
                self.home = (msg.latitude / 1e7, msg.longitude / 1e7)
            elif kind == "HEARTBEAT" and msg.get_srcSystem() == self.m.target_system and msg.type != 6:
                self.armed = bool(msg.base_mode & self.mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            elif kind == "COMMAND_ACK":
                self.acks[msg.command] = msg.result
            elif kind == "MISSION_ITEM_REACHED":
                if msg.seq not in self.reached:
                    self.reached.append(msg.seq)
            elif kind == "STATUSTEXT":
                text = msg.text.strip()
                if text and (not self.report.messages or self.report.messages[-1] != text):
                    self.report.messages.append(text)
                    self.log(f"  autopilot: {text}")

    def wait_until(self, what: str, cond, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s * self.k
        while not cond():
            if time.monotonic() > deadline:
                raise TimeoutError(f"Timed out waiting for: {what}")
            self.pump()

    def step(self, what: str) -> None:
        self.report.steps.append((what, self.t_s))
        self.log(f"[{self.t_s:6.1f} s] {what}")

    def distance_to_m(self, lat: float, lon: float) -> float:
        dn = math.radians(self.pos["lat"] - lat) * EARTH_RADIUS_M
        de = math.radians(self.pos["lon"] - lon) * EARTH_RADIUS_M * math.cos(math.radians(lat))
        return math.hypot(dn, de)

    # --- commands --------------------------------------------------------------------------------
    def command(self, cmd: int, *params: float, timeout_s: float = 5.0) -> bool:
        """Send a COMMAND_LONG and wait for its ACK. Returns True if the autopilot accepted it."""
        p = list(params) + [0.0] * (7 - len(params))
        self.acks.pop(cmd, None)
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component, cmd, 0, *p)
        deadline = time.monotonic() + timeout_s * self.k
        while cmd not in self.acks and time.monotonic() < deadline:
            self.pump(0.1)  # keeps reading positions too, so nothing goes stale while we wait
        return self.acks.get(cmd) == self.mavutil.mavlink.MAV_RESULT_ACCEPTED

    def set_mode(self, mode: str) -> None:
        mode_id = self.m.mode_mapping()[mode]
        self.m.set_mode(mode_id)
        self.wait_until(f"mode {mode}", lambda: self.m.flightmode == mode, 10)
        self.step(f"Mode {mode}")

    def arm_when_ready(self, timeout_s: float = 120.0) -> None:
        """Keep trying to arm. Until GPS and the EKF are happy, the autopilot refuses ("PreArm: ...")."""
        mav = self.mavutil.mavlink
        deadline = time.monotonic() + timeout_s * self.k
        while not self.armed:
            if time.monotonic() > deadline:
                raise TimeoutError("Never passed the pre-arm checks: " + " | ".join(self.report.messages[-3:]))
            self.command(mav.MAV_CMD_COMPONENT_ARM_DISARM, 1)
            self.pump(1.0)
        self.step("Armed (pre-arm checks passed)")

    def takeoff(self, alt_m: float) -> None:
        if not self.command(self.mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, alt_m):
            raise RuntimeError("Take-off was refused")
        self.wait_until(f"altitude {alt_m} m", lambda: self.pos["rel_alt_m"] >= 0.95 * alt_m, 60)
        self.step(f"Reached {self.pos['rel_alt_m']:.1f} m")

    def fly_to_local(self, east_m: float, north_m: float, up_m: float) -> None:
        """GUIDED "fly to", given in DroneLab's ENU and sent in ArduPilot's NED (relative to the home/EKF origin)."""
        n, e, d = enu_to_ned(east_m, north_m, up_m)
        mav = self.mavutil.mavlink
        position_only = 0b0000_1111_1111_1000  # ignore velocity, acceleration, yaw: just "go there"
        self.m.mav.set_position_target_local_ned_send(0, self.m.target_system, self.m.target_component,
                                                      mav.MAV_FRAME_LOCAL_NED, position_only,
                                                      n, e, d, 0, 0, 0, 0, 0, 0, 0, 0)
        assert self.home is not None
        lat, lon = offset_latlon(*self.home, north_m, east_m)
        self.wait_until("arrival", lambda: self.distance_to_m(lat, lon) < 1.0, 60)
        self.pump(3.0)  # let it settle, then measure
        self.report.fly_to_error_m = self.distance_to_m(lat, lon)
        self.step(f"Arrived {north_m:.0f} m north (error {self.report.fly_to_error_m:.2f} m)")

    def upload_mission(self, points: list[tuple[float, float, float]]) -> None:
        """Mission upload, MAVLink style: send the count, then answer each MISSION_REQUEST with that item."""
        mav = self.mavutil.mavlink
        assert self.home is not None
        # Item 0 is home (ArduPilot overwrites it), then a take-off, then the waypoints.
        items = [(mav.MAV_CMD_NAV_WAYPOINT, *self.home, 0.0), (mav.MAV_CMD_NAV_TAKEOFF, *self.home, points[0][2])]
        items += [(mav.MAV_CMD_NAV_WAYPOINT, lat, lon, alt) for lat, lon, alt in points]
        self.m.mav.mission_count_send(self.m.target_system, self.m.target_component, len(items), mav.MAV_MISSION_TYPE_MISSION)
        deadline = time.monotonic() + 20 * self.k
        while True:
            if time.monotonic() > deadline:
                raise TimeoutError("Mission upload timed out")
            msg = self.m.recv_match(type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"], blocking=True, timeout=1)
            if msg is None:
                continue
            if msg.get_type() == "MISSION_ACK":
                if msg.type != mav.MAV_MISSION_ACCEPTED:
                    raise RuntimeError(f"Mission rejected (code {msg.type})")
                break
            cmd, lat, lon, alt = items[msg.seq]
            self.m.mav.mission_item_int_send(self.m.target_system, self.m.target_component, msg.seq,
                                             mav.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, cmd, 0, 1, 0, 0, 0, 0,
                                             int(lat * 1e7), int(lon * 1e7), alt, mav.MAV_MISSION_TYPE_MISSION)
        self.step(f"Uploaded a {len(points)}-waypoint mission")

    def fly_mission(self, points: list[tuple[float, float, float]]) -> None:
        """AUTO mode: the autopilot flies the uploaded mission by itself; we just watch and measure.

        Mission item n (n >= 2) is waypoint n-1. ArduCopter says "reached" when the drone gets within
        WPNAV_RADIUS of it (2 m by default) and moves on without stopping, so a few tens of cm off is normal.
        """
        self.reached.clear()
        self.set_mode("AUTO")
        deadline = time.monotonic() + 180 * self.k
        while len(self.report.corner_errors_m) < len(points):
            if time.monotonic() > deadline:
                raise TimeoutError("Mission didn't finish")
            self.pump(0.05)
            for seq in [q for q in self.reached if q >= 2][len(self.report.corner_errors_m):]:
                lat, lon, _ = points[seq - 2]
                err = self.distance_to_m(lat, lon)
                self.report.corner_errors_m.append(err)
                self.step(f"Reached waypoint {seq - 1} of {len(points)} ({err:.2f} m off)")

    def return_to_launch(self) -> None:
        self.set_mode("RTL")
        self.wait_until("landing and disarm", lambda: not self.armed, 120)
        assert self.home is not None
        self.report.landed_from_home_m = self.distance_to_m(*self.home)
        self.step(f"Landed and disarmed, {self.report.landed_from_home_m:.2f} m from home")


def fly(url: str, log=print, timeout_scale: float = 1.0) -> FlightReport:
    """The whole demo flight. Raises on any failure; returns the report on success."""
    f = Flight(url, log=log, timeout_scale=timeout_scale)
    f.step("Connected, waiting for GPS + EKF")
    f.wait_until("home position", lambda: f.home is not None or _ask_home(f), 60)
    f.set_mode("GUIDED")
    f.arm_when_ready()
    f.takeoff(10.0)
    f.fly_to_local(east_m=0.0, north_m=20.0, up_m=10.0)
    assert f.home is not None
    points = square_mission(*f.home, side_m=20.0, alt_m=10.0)
    f.upload_mission(points)
    f.fly_mission(points)
    f.return_to_launch()
    f.report.ok = True
    return f.report


def _ask_home(f: Flight) -> bool:
    """Ask for the home position (sent once GPS has a fix). Returns False so wait_until keeps pumping."""
    f.command(f.mavutil.mavlink.MAV_CMD_GET_HOME_POSITION, timeout_s=0.5)
    return f.home is not None


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
