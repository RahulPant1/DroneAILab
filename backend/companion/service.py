"""Companion-computer service: camera + lidar perception that feeds ArduPilot over MAVLink.

The split real drones use: the AUTOPILOT (Pixhawk / ArduPilot) keeps flying, estimating and doing failsafes; the
COMPANION (Jetson, Raspberry Pi) runs the heavier perception and only sends what it found, as standard messages:

  down camera -> marker detection -> LANDING_TARGET    (ArduPilot precision landing, PLND_TYPE = 1 "MAVLink")
  lidar       -> 72 sectors        -> OBSTACLE_DISTANCE (ArduPilot proximity, PRX1_TYPE = 2 "MAVLink"), used by
                                                         its avoidance and its BendyRuler path planner (OA_TYPE = 1)

The perception is DroneLab's own (autonomy/precland.py: detect; autonomy/lidar.py: scan). What the companion
knows about the drone comes from the autopilot over MAVLink (ATTITUDE, GLOBAL_POSITION_INT), like on a real drone.

SIMULATION BOUNDARY: two classes stand in for hardware, and only they read SITL's ground truth (SIM_STATE):
  SimCamera -> on a real drone: a V4L2 / GStreamer camera (e.g. OpenCV VideoCapture), same frame() interface.
  SimLidar  -> on a real drone: an RPLidar / ToF driver over serial, same scan() interface.
Everything else in this file would run unchanged on the real companion computer.

    .venv/bin/python -m backend.companion.service --connect tcp:127.0.0.1:5763 --world precland
"""

from __future__ import annotations

import argparse
import json
import math
import signal
import sys
import threading
import time

import numpy as np

from backend.autonomy.lidar import scan
from backend.autonomy.precland import FOCAL_PX, IMAGE_H, IMAGE_W, detect, render
from backend.autonomy.world import Box
from backend.sim.frames import quat_from_euler
from backend.sim.physics import State
from backend.sitl.fly import attitude_ned_to_enu, latlon_to_ne

CAMERA_HZ = 15.0
LIDAR_HZ = 10.0
LIDAR_SECTORS = 72  # 5° each, clockwise from the nose (OBSTACLE_DISTANCE's layout)
LIDAR_MIN_M, LIDAR_MAX_M = 0.2, 8.0
MSG_SIM_STATE = 108

# The worlds the companion's simulated sensors can see. Coordinates: metres east / north of ArduPilot's home.
WORLDS: dict[str, dict] = {
    "precland": {"pads": [(6.0, 4.0, 1.0)], "boxes": []},  # one 1 m marker on a light mat, 7 m from home
    "avoid": {"pads": [], "boxes": [Box(-5.0, 5.0, 11.0, 12.0, 30.0)]},  # a 10 m wide, 30 m tall wall, 11 m north
}


class VehicleView:
    """What the companion knows about the drone, from its own MAVLink link (a background reader thread)."""

    def __init__(self, url: str):
        from pymavlink import mavutil

        self.mav = mavutil.mavlink
        self.m = mavutil.mavlink_connection(url, source_system=1, source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER)
        if self.m.wait_heartbeat(timeout=30) is None:
            raise TimeoutError(f"No heartbeat from the autopilot on {url}")
        self.lock = threading.Lock()
        self.home: tuple[float, float, float] | None = None  # lat, lon, alt AMSL (m)
        self.att = (0.0, 0.0, 0.0)  # NED roll, pitch, yaw (the AUTOPILOT's estimate)
        self.rel_alt_m = 0.0
        self.sim: dict | None = None  # SIM_STATE: only the simulated sensor drivers may use this
        self.closed = threading.Event()
        self.m.mav.request_data_stream_send(self.m.target_system, self.m.target_component, self.mav.MAV_DATA_STREAM_ALL, 20, 1)
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component, self.mav.MAV_CMD_SET_MESSAGE_INTERVAL,
                                     0, MSG_SIM_STATE, 1e6 / 50, 0, 0, 0, 0, 0)
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self) -> None:
        while not self.closed.is_set():
            try:
                msg = self.m.recv_match(blocking=True, timeout=0.2)
            except (OSError, ValueError):
                break
            if msg is None:
                continue
            kind = msg.get_type()
            with self.lock:
                if kind == "ATTITUDE":
                    self.att = (msg.roll, msg.pitch, msg.yaw)
                elif kind == "GLOBAL_POSITION_INT":
                    self.rel_alt_m = msg.relative_alt / 1000.0
                elif kind == "HOME_POSITION" and self.home is None:  # the first home: ArduPilot moves it on re-arming
                    self.home = (msg.latitude / 1e7, msg.longitude / 1e7, msg.altitude / 1000.0)
                elif kind == "SIM_STATE":
                    self.sim = {"lat": msg.lat_int / 1e7, "lon": msg.lon_int / 1e7, "alt": msg.alt,
                                "roll": msg.roll, "pitch": msg.pitch, "yaw": msg.yaw}
        self.closed.set()

    def ask_home(self) -> None:
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component, self.mav.MAV_CMD_GET_HOME_POSITION,
                                     0, 0, 0, 0, 0, 0, 0, 0)

    def close(self) -> None:
        self.closed.set()
        try:
            self.m.close()
        except OSError:
            pass


def truth_state(view: VehicleView) -> State | None:
    """SIM ONLY: the drone's true pose from SIM_STATE, as a DroneLab State (ENU, metres from home)."""
    with view.lock:
        s, home = view.sim, view.home
    if s is None or home is None:
        return None
    n, e = latlon_to_ne(s["lat"], s["lon"], home[0], home[1])
    st = State.at(max(s["alt"] - home[2], 0.0))
    st.pos_m[:2] = (e, n)
    st.quat = quat_from_euler(*attitude_ned_to_enu(s["roll"], s["pitch"], s["yaw"]))
    st.on_ground = False
    return st


class SimCamera:
    """SIM ONLY: the down camera, rendered from SITL's true pose. A real driver returns a picture the same way."""

    def __init__(self, view: VehicleView, pads, seed: int = 0):
        self.view, self.pads, self.rng = view, pads, np.random.default_rng(seed)

    def frame(self) -> np.ndarray | None:
        st = truth_state(self.view)
        return None if st is None else render(st, self.pads, self.rng)


class SimLidar:
    """SIM ONLY: a 360° lidar at the drone's true position. Returns ranges per sector, clockwise from the nose."""

    def __init__(self, view: VehicleView, boxes: list[Box], seed: int = 0):
        self.view, self.boxes, self.rng = view, boxes, np.random.default_rng(seed)

    def scan(self) -> np.ndarray | None:
        st = truth_state(self.view)
        if st is None:
            return None
        with self.view.lock:
            heading = self.view.sim["yaw"]  # compass heading of the nose (true)
        body = np.radians(np.arange(LIDAR_SECTORS) * 360.0 / LIDAR_SECTORS)  # clockwise from the nose
        enu = math.pi / 2 - (heading + body)  # compass -> DroneLab angle (0 = east, counter-clockwise)
        r = scan(self.boxes, float(st.pos_m[0]), float(st.pos_m[1]), float(st.pos_m[2]), LIDAR_MAX_M, enu)
        hit = r < LIDAR_MAX_M
        return np.where(hit, np.maximum(r + 0.02 * self.rng.standard_normal(r.shape), LIDAR_MIN_M), r)  # noise on hits only


# ---- the part that runs on the real companion computer as-is ----

def landing_target(det: dict, att_ned: tuple[float, float, float], rel_alt_m: float) -> dict | None:
    """Marker pixel -> line of sight in the body frame (FRD: forward, right, down) and its distance.

    The camera points straight down with the top of the picture forward. Distance = height / how steeply down the
    line of sight points in the world, using the AUTOPILOT's attitude estimate (what a real companion knows)."""
    du, dv = det["u"] - IMAGE_W / 2, det["v"] - IMAGE_H / 2
    los = np.array([-dv / FOCAL_PX, du / FOCAL_PX, 1.0])
    los /= np.linalg.norm(los)
    roll, pitch, _ = att_ned
    # body FRD -> level frame (yaw doesn't change the vertical component)
    down = -math.sin(pitch) * los[0] + math.sin(roll) * math.cos(pitch) * los[1] + math.cos(roll) * math.cos(pitch) * los[2]
    if down < 0.2 or rel_alt_m < 0.3:
        return None
    dist = rel_alt_m / down
    return {"x": float(los[0] * dist), "y": float(los[1] * dist), "z": float(los[2] * dist), "distance": float(dist),
            "angle_x": math.atan2(du, FOCAL_PX), "angle_y": math.atan2(dv, FOCAL_PX)}


def obstacle_distance_cm(ranges_m: np.ndarray) -> list[int]:
    """Ranges -> OBSTACLE_DISTANCE's centimetres; 'nothing within range' = max distance + 1 (MAVLink convention)."""
    out = []
    for r in ranges_m:
        out.append(int(LIDAR_MAX_M * 100) + 1 if r >= LIDAR_MAX_M - 1e-3 else max(int(LIDAR_MIN_M * 100), int(r * 100)))
    return out


class Companion:
    def __init__(self, view: VehicleView, camera=None, lidar=None):
        self.view, self.camera, self.lidar = view, camera, lidar
        self.stats = {"frames": 0, "detections": 0, "landing_target_sent": 0, "scans": 0, "obstacle_distance_sent": 0,
                      "closest_obstacle_m": None}
        self.stop = threading.Event()

    def step_camera(self) -> None:
        img = self.camera.frame()
        if img is None:
            return
        self.stats["frames"] += 1
        det = detect(img)
        if det is None:
            return
        self.stats["detections"] += 1
        with self.view.lock:
            att, alt = self.view.att, self.view.rel_alt_m
        lt = landing_target(det, att, alt)
        if lt is None:
            return
        v = self.view
        v.m.mav.landing_target_send(int(time.time() * 1e6), 0, v.mav.MAV_FRAME_BODY_FRD, lt["angle_x"], lt["angle_y"],
                                    lt["distance"], 0.0, 0.0, lt["x"], lt["y"], lt["z"], [1.0, 0.0, 0.0, 0.0],
                                    v.mav.LANDING_TARGET_TYPE_VISION_FIDUCIAL, 1)
        self.stats["landing_target_sent"] += 1

    def step_lidar(self) -> None:
        r = self.lidar.scan()
        if r is None:
            return
        self.stats["scans"] += 1
        close = float(r.min())
        if close < LIDAR_MAX_M:
            prev = self.stats["closest_obstacle_m"]
            self.stats["closest_obstacle_m"] = round(close if prev is None else min(prev, close), 2)
        v = self.view
        v.m.mav.obstacle_distance_send(int(time.time() * 1e6), v.mav.MAV_DISTANCE_SENSOR_LASER, obstacle_distance_cm(r),
                                       0, int(LIDAR_MIN_M * 100), int(LIDAR_MAX_M * 100), 360.0 / LIDAR_SECTORS, 0.0,
                                       v.mav.MAV_FRAME_BODY_FRD)
        self.stats["obstacle_distance_sent"] += 1

    def run(self) -> None:
        """Camera at 15 Hz, lidar at 10 Hz, heartbeat at 1 Hz, until stopped."""
        v = self.view
        nxt_cam = nxt_lidar = nxt_hb = time.monotonic()
        while not self.stop.is_set() and not v.closed.is_set():
            now = time.monotonic()
            if v.home is None and now >= nxt_hb:
                v.ask_home()
            if now >= nxt_hb:
                v.m.mav.heartbeat_send(v.mav.MAV_TYPE_ONBOARD_CONTROLLER, v.mav.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                nxt_hb = now + 1.0
            if self.camera is not None and now >= nxt_cam:
                self.step_camera()
                nxt_cam = now + 1.0 / CAMERA_HZ
            if self.lidar is not None and now >= nxt_lidar:
                self.step_lidar()
                nxt_lidar = now + 1.0 / LIDAR_HZ
            time.sleep(0.005)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--connect", required=True, help="the autopilot's companion port, e.g. tcp:127.0.0.1:5763")
    ap.add_argument("--world", choices=list(WORLDS), required=True, help="what the simulated sensors can see")
    ap.add_argument("--no-camera", action="store_true")
    ap.add_argument("--no-lidar", action="store_true")
    args = ap.parse_args(argv)
    world = WORLDS[args.world]
    view = VehicleView(args.connect)
    comp = Companion(view, None if args.no_camera or not world["pads"] else SimCamera(view, world["pads"]),
                     None if args.no_lidar or not world["boxes"] else SimLidar(view, world["boxes"]))
    signal.signal(signal.SIGTERM, lambda *_: comp.stop.set())
    signal.signal(signal.SIGINT, lambda *_: comp.stop.set())
    print(json.dumps({"companion": "ready", "world": args.world}), flush=True)
    try:
        comp.run()
    finally:
        print(json.dumps({"companion": "stopped", **comp.stats}), flush=True)
        view.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
