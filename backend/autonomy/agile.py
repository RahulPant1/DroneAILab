"""Agile flight: fly a rings-race track three ways and measure the difference.

  waypoints  The classic way (like our missions): a "carrot" slides along straight lines through the ring centres
             at the cruise speed, and the position controller chases it. No idea what comes next, so it lags,
             overshoots corners, and the position loop never lets it go faster than 3 m/s.
  pid_ff     A minimum-snap TRAJECTORY through the rings (autonomy/trajectory.py), flown by the same cascaded PID,
             now told the plan's velocity and acceleration at every instant (feedforward) plus the drag it'll meet.
             This is how PX4 and ArduPilot follow their own trajectories.
  geometric  The same trajectory, flown by a geometric (SE(3)) controller: one 3D loop that computes the exact
             attitude for the wanted thrust vector, and a spin-rate feedforward from the plan's jerk.

What counts is judged on the TRUE position (physics): which rings were flown through, and the lap time
(first ring armed at the start of the trajectory, clock stops at the last ring). Tracking error = distance from
where the plan said to be at that moment (for `waypoints`: from the carrot).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from backend.games.race import TRACKS, Ring
from backend.sim.config import DroneConfig, get_preset
from backend.sim.control import Setpoint
from backend.sim.environment import EnvConfig
from backend.sim.physics import State
from backend.sim.runner import RunResult, run_headless
from backend.sim.sensors import SensorConfig

from .trajectory import Limits, Trajectory, plan_trajectory

METHODS = ("waypoints", "pid_ff", "geometric")
METHOD_NAMES = {"waypoints": "Waypoints (carrot, no plan)", "pid_ff": "Trajectory + PID feedforward",
                "geometric": "Trajectory + geometric controller"}
EXIT_M = 2.0  # the plan ends this far past the last ring, so the drone flies THROUGH it
SETTLE_S = 0.5  # hover this long before starting
HOLD_AFTER_S = 2.0  # keep recording after the end
CARROT_LEASH_M = 2.0
FACE_TRAVEL_ABOVE_M_S = 1.0
WAYPOINT_MAX_SPEED_M_S = 3.0  # what the position loop allows without a plan (control.MAX_HORIZONTAL_SPEED_M_S)


@dataclass
class TrackPlan:
    track_id: str
    rings: list[Ring]
    points: np.ndarray  # start (hover), ring centres, exit point
    directions: dict[int, np.ndarray]
    trajectory: Trajectory | None  # None for the waypoints method
    limits: Limits

    def to_dict(self) -> dict:
        tr = self.trajectory
        return {
            "track": self.track_id,
            "rings": [r.to_dict() for r in self.rings],
            "points": np.round(self.points, 3).tolist(),
            "path": tr.polyline(240) if tr else np.round(self.points, 3).tolist(),
            "duration_s": round(tr.duration_s, 2) if tr else None,
            "knots_s": np.round(tr.knots_s(), 2).tolist() if tr else None,
            "peaks": {k: round(v, 2) for k, v in tr.peaks().items()} if tr else None,
            "limits": {"speed_m_s": self.limits.speed_m_s, "accel_m_s2": self.limits.accel_m_s2,
                       "tilt_deg": self.limits.tilt_deg},
        }


def track_points(track_id: str, start_xy: tuple[float, float] | None = None) -> tuple[list[Ring], np.ndarray, dict]:
    """Waypoints for a track: start (hovering at the first ring's height), each ring's centre, an exit point."""
    if track_id not in TRACKS:
        raise ValueError(f"Unknown track '{track_id}'. Choose one of: {', '.join(TRACKS)}.")
    rings: list[Ring] = TRACKS[track_id]["rings"]
    sx, sy = start_xy if start_xy is not None else TRACKS[track_id]["start"]
    pts = [[sx, sy, rings[0].z_m]] + [list(r.centre) for r in rings]
    last = rings[-1]
    pts.append(list(last.centre + last.normal * EXIT_M))
    directions = {i + 1: r.normal for i, r in enumerate(rings)}
    return rings, np.array(pts, dtype=float), directions


def plan_track(track_id: str, method: str, speed_m_s: float, start_xy: tuple[float, float] | None = None,
               tilt_deg: float = 35.0, accel_m_s2: float | None = None) -> TrackPlan:
    if method not in METHODS:
        raise ValueError(f"Unknown method '{method}'. Choose one of: {', '.join(METHODS)}.")
    rings, pts, dirs = track_points(track_id, start_xy)
    accel = accel_m_s2 if accel_m_s2 is not None else 9.81 * math.tan(math.radians(tilt_deg))  # what that tilt can give
    limits = Limits(speed_m_s=speed_m_s, accel_m_s2=accel, tilt_deg=tilt_deg)
    traj = None if method == "waypoints" else plan_trajectory(pts, limits, dirs)
    return TrackPlan(track_id, rings, pts, dirs, traj, limits)


def face_travel_yaw(vel: np.ndarray, acc: np.ndarray, last_yaw: float) -> tuple[float, float]:
    """Point the nose along the direction of travel (when moving); returns (yaw, yaw rate)."""
    v2 = float(vel[0] ** 2 + vel[1] ** 2)
    if v2 < FACE_TRAVEL_ABOVE_M_S**2:
        return last_yaw, 0.0
    yaw = math.atan2(vel[1], vel[0])
    return yaw, float(vel[0] * acc[1] - vel[1] * acc[0]) / v2


class TrackFollower:
    """Flies a TrackPlan from a hover at its first point. Call `setpoint_for(estimate, t, truth)` every step."""

    def __init__(self, plan: TrackPlan, method: str, t_start_s: float = 0.0):
        self.plan = plan
        self.method = method
        self.phase = "settle"
        self.t_settle_end = t_start_s + SETTLE_S
        self.t0: float | None = None  # when the trajectory (or carrot) started
        self.next_ring = 0  # how many rings have been flown through so far
        self.ring_times_s: list[float | None] = [None] * len(plan.rings)
        self.lap_s: float | None = None
        self.done = False
        self._prev_truth: np.ndarray | None = None
        self._yaw = 0.0
        self._t_end: float | None = None
        p0 = plan.points[0]
        self._sp = Setpoint(armed=True, alt_m=float(p0[2]), position_control=True, x_m=float(p0[0]), y_m=float(p0[1]))
        # carrot along the straight lines (waypoints method)
        self._seg = np.diff(plan.points, axis=0)
        self._cum = np.concatenate([[0.0], np.cumsum(np.linalg.norm(self._seg, axis=1))])
        self.s = 0.0
        self._t_last = t_start_s
        self.ref_log: list[tuple[float, float, float, float]] = []  # (t, x, y, z) of the reference, while tracking

    @property
    def speed_m_s(self) -> float:
        return min(self.plan.limits.speed_m_s, WAYPOINT_MAX_SPEED_M_S)

    def _carrot(self, s: float) -> np.ndarray:
        s = min(max(s, 0.0), self._cum[-1])
        i = int(min(np.searchsorted(self._cum, s, side="right") - 1, len(self._seg) - 1))
        f = (s - self._cum[i]) / max(self._cum[i + 1] - self._cum[i], 1e-9)
        return self.plan.points[i] + self._seg[i] * f

    def reference(self, t: float) -> np.ndarray | None:
        """Where the plan wanted the drone at time t (None outside the tracking phase)."""
        if self.t0 is None:
            return None
        if self.plan.trajectory is not None:
            return self.plan.trajectory.sample_one(t - self.t0)[0]
        return None

    def _heading_for(self, i: int, t: float) -> bool:
        """Ring i counts only once the plan is on its way to it (so crossing it early, on another leg, doesn't)."""
        if self.plan.trajectory is not None:
            return t - self.t0 >= self.plan.trajectory.knots_s()[i] - 0.3
        return self.s >= self._cum[i] - 0.3

    def _rings(self, truth: State, t: float) -> None:
        """Each ring is judged on its own (a miss doesn't hide the rest); the lap counts once all are flown."""
        p = truth.pos_m.copy()
        if self._prev_truth is not None and self.t0 is not None and self.phase == "track":
            for i, ring in enumerate(self.plan.rings):
                if self.ring_times_s[i] is None and self._heading_for(i, t) and ring.passed(self._prev_truth, p):
                    self.ring_times_s[i] = round(t - self.t0, 3)
                    self.next_ring += 1
                    if self.next_ring == len(self.plan.rings):
                        self.lap_s = t - self.t0
        self._prev_truth = p

    def setpoint_for(self, est: State, t: float, truth: State | None = None) -> Setpoint:
        truth = truth or est
        dt = max(0.0, t - self._t_last)
        self._t_last = t
        self._rings(truth, t)
        sp = self._sp
        if self.phase == "settle":
            if t >= self.t_settle_end:
                self.phase = "track"
                self.t0 = t
            return sp
        if self.phase == "track":
            tt = t - self.t0
            if self.method == "waypoints":
                if float(np.linalg.norm(self._carrot(self.s) - est.pos_m)) < CARROT_LEASH_M:
                    self.s = min(self._cum[-1], self.s + self.speed_m_s * dt)
                c = self._carrot(self.s)
                sp.x_m, sp.y_m, sp.alt_m = (float(v) for v in c)
                self.ref_log.append((t, *(float(v) for v in c)))
                if self.s >= self._cum[-1] and float(np.linalg.norm(c - est.pos_m)) < 0.3:
                    self._finish(t)
            else:
                tr = self.plan.trajectory
                p, v, a, j = tr.sample_one(tt)
                self._yaw, yaw_rate = face_travel_yaw(v, a, self._yaw)
                sp.x_m, sp.y_m, sp.alt_m = (float(x) for x in p)
                sp.vel_ff_m_s = tuple(float(x) for x in v)
                sp.acc_ff_m_s2 = tuple(float(x) for x in a)
                sp.jerk_ff_m_s3 = tuple(float(x) for x in j)
                sp.yaw_rad = self._yaw
                sp.yaw_rate_ff_rad_s = yaw_rate
                sp.law = "geometric" if self.method == "geometric" else "cascade"
                self.ref_log.append((t, *(float(x) for x in p)))
                if tt >= tr.duration_s:
                    self._finish(t)
        elif self.phase == "hold" and t >= self._t_end:
            self.phase = "done"
            self.done = True
        return sp

    def _finish(self, t: float) -> None:
        sp = self._sp
        sp.vel_ff_m_s = sp.acc_ff_m_s2 = sp.jerk_ff_m_s3 = None
        sp.yaw_rate_ff_rad_s = 0.0
        self.phase = "hold"
        self._t_end = t + HOLD_AFTER_S


def run_track(track_id: str, method: str, speed_m_s: float = 12.0, config: DroneConfig | None = None,
              env: EnvConfig | None = None, sensors: SensorConfig | None = None, seed: int = 0,
              tilt_deg: float = 35.0, extra_channels=None) -> RunResult:
    """Fly one track headless (from a hover at its start) and measure the lap. `extra_channels`: see run_headless."""
    cfg = config or get_preset("basic")
    plan = plan_track(track_id, method, speed_m_s, tilt_deg=tilt_deg)
    follower = TrackFollower(plan, method)
    x0, y0, z0 = plan.points[0]
    init = State.hovering(cfg, float(z0))
    init.pos_m[:2] = (x0, y0)
    if plan.trajectory is not None:
        expected = plan.trajectory.duration_s
    else:
        expected = float(follower._cum[-1]) / follower.speed_m_s * 1.6 + 5.0
    duration = min(SETTLE_S + expected + HOLD_AFTER_S + 6.0, 300.0)
    r = run_headless(cfg, follower, duration, seed=seed, initial_state=init, run_type="agile", env=env,
                     sensors=sensors, stop_when=lambda: follower.done,
                     extra_channels=extra_channels)
    add_tracking_channels(r, follower)
    r.metrics.update(track_metrics(r, follower))
    r.scenario = {"track": track_id, "method": method, "speed_m_s": speed_m_s, "plan": plan.to_dict(),
                  "ring_times_s": follower.ring_times_s}
    return r


def add_tracking_channels(r: RunResult, f: TrackFollower) -> None:
    """Add ref_x/y/z_m (the plan), track_err_m (distance from it) and speed_m_s to the telemetry."""
    t = np.asarray(r.telemetry["t_s"])
    log = np.array(f.ref_log) if f.ref_log else np.zeros((0, 4))
    ref = np.full((len(t), 3), np.nan)
    if len(log):
        inside = (t >= log[0, 0]) & (t <= log[-1, 0])
        for k in range(3):
            ref[inside, k] = np.interp(t[inside], log[:, 0], log[:, k + 1])
    pos = np.column_stack([r.telemetry["x_m"], r.telemetry["y_m"], r.telemetry["z_m"]])
    err = np.linalg.norm(pos - ref, axis=1)
    speed = np.linalg.norm(np.column_stack([r.telemetry["vx_m_s"], r.telemetry["vy_m_s"], r.telemetry["vz_m_s"]]), axis=1)
    for name, vals in (("ref_x_m", ref[:, 0]), ("ref_y_m", ref[:, 1]), ("ref_z_m", ref[:, 2]), ("track_err_m", err),
                       ("speed_m_s", speed)):
        r.telemetry[name] = [None if not np.isfinite(v) else round(float(v), 4) for v in vals]


def distance_to_path_m(points: np.ndarray, path: np.ndarray) -> np.ndarray:
    """For each point, the distance to the nearest place on a polyline path."""
    a, b = path[:-1], path[1:]
    ab = b - a
    ab2 = np.maximum((ab * ab).sum(axis=1), 1e-12)
    out = np.empty(len(points))
    for i in range(0, len(points), 256):  # in chunks: points x segments
        p = points[i:i + 256, None, :]
        f = np.clip(((p - a) * ab).sum(axis=2) / ab2, 0.0, 1.0)
        out[i:i + 256] = np.linalg.norm(p - (a + f[..., None] * ab), axis=2).min(axis=1)
    return out


def track_metrics(r: RunResult, f: TrackFollower) -> dict:
    t = np.asarray(r.telemetry["t_s"])
    err = np.array([np.nan if v is None else v for v in r.telemetry["track_err_m"]], dtype=float)
    tracking = np.isfinite(err)
    roll = np.degrees(np.asarray(r.telemetry["roll_rad"]))
    pitch = np.degrees(np.asarray(r.telemetry["pitch_rad"]))
    tilt = np.degrees(np.arccos(np.clip(np.cos(np.radians(roll)) * np.cos(np.radians(pitch)), -1, 1)))
    motors = np.column_stack([r.telemetry[f"motor{k}_pct"] for k in range(1, 5)])
    speed = np.asarray(r.telemetry["speed_m_s"], dtype=float)
    sat = (motors.max(axis=1) >= 99.9) | (motors.min(axis=1) <= 0.1)
    path = np.array(f.plan.trajectory.polyline(800)) if f.plan.trajectory is not None else f.plan.points
    pos = np.column_stack([r.telemetry["x_m"], r.telemetry["y_m"], r.telemetry["z_m"]])[tracking]
    path_err = distance_to_path_m(pos, path) if len(pos) else np.zeros(1)
    return {
        "rings_total": len(f.plan.rings),
        "rings_passed": f.next_ring,
        "lap_time_s": round(f.lap_s, 2) if f.lap_s is not None else None,
        "track_err_mean_m": round(float(err[tracking].mean()), 3) if tracking.any() else None,
        "track_err_max_m": round(float(err[tracking].max()), 3) if tracking.any() else None,
        "path_err_mean_m": round(float(path_err.mean()), 3),
        "path_err_max_m": round(float(path_err.max()), 3),
        "max_speed_m_s": round(float(speed[tracking].max()), 2) if tracking.any() else 0.0,
        "max_tilt_deg": round(float(tilt[tracking].max()), 1) if tracking.any() else 0.0,
        "motor_saturated_pct": round(100.0 * float(sat[tracking].mean()), 1) if tracking.any() else 0.0,
        "planned_time_s": round(f.plan.trajectory.duration_s, 2) if f.plan.trajectory else None,
        "flight_time_s": round(float(t[-1]), 2),
    }
