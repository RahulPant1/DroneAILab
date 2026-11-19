"""Multi-drone formation flight: 'wingmen' drones that follow your drone (the leader) in a shape.

How it works (like a real drone swarm, simplified):
  - The leader broadcasts where it BELIEVES it is (its estimate) and its heading, many times a second.
  - Each wingman has its own slot in the shape, measured in the leader's frame ("1.5 m behind and 1.5 m to the
    left"). It turns that into a point in the world and flies there with its own autopilot (Pilot + controller,
    the same code as your drone). Wingmen have perfect sensors, so only the leader's errors matter.
  - The shape is lined up with the leader's nose (so Q/E turn the whole formation). On a mission the autopilot
    doesn't turn the nose, so there the shape lines up with the direction of travel instead, turning smoothly.
  - Wingmen copy the leader's flight phase: they take off when it takes off and land when it lands.
  - Separation ("avoid each other"): each wingman looks LOOKAHEAD_S ahead (where will we both be, at our
    current speeds?). If that predicted gap is under SEPARATION_M, its target is pushed away from the other
    drone, harder the closer it gets, and it also dodges vertically (odd wingmen climb, even ones duck), which
    clears a head-on much faster than running away. It's the potential-field idea from lesson 09, used between
    drones. The same switch also keeps wingmen out of obstacles: if the way to its slot is blocked by a box, a
    wingman "breaks formation" and follows the leader's own path through the gap (the leader got through, so
    its path is safe), queueing in the order they're in, then spreads out again once it's clear; and a wingman
    near a box is pushed away from it. Turn it off to see why swarms
    need it.
  - Two drones whose centres come closer than COLLISION_DIST_M touch propellers: both stop their motors.
    Collisions are judged on the TRUE positions (a physical fact).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from backend.sim.config import DroneConfig
from backend.sim.control import Setpoint
from backend.sim.environment import EnvConfig
from backend.sim.physics import State
from backend.sim.pilot import Pilot
from backend.sim.runner import LightweightSim

from .world import DRONE_RADIUS_M, Box, clearance_m

MAX_WINGMEN = 4
SHAPES = ("v", "line", "column")
SHAPE_NAMES = {"v": "V (like geese)", "line": "line abreast (side by side)", "column": "column (single file)"}
MIN_SPACING_M, MAX_SPACING_M = 1.5, 4.0  # 1.5 m keeps a steady formation outside every avoidance zone
COLLISION_DIST_M = 2 * DRONE_RADIUS_M  # centre-to-centre: closer than this, the propellers touch
SEPARATION_M = 1.5  # avoidance starts when the PREDICTED gap (drones getting closer) is under this
SEPARATION_NOW_M = 1.2  # ... or when the gap right now is under this
CLOSING_SPEED_M_S = 0.3  # 'getting closer' means closing faster than this (ignores tiny wobbles)
SEPARATION_PUSH_M = 2.0  # how far the target is pushed sideways when two drones (almost) touch
VERTICAL_DODGE_M = 2.0  # how far a wingman climbs (or ducks) to get out of the way
LOOKAHEAD_S = 1.0  # avoidance judges where the drones will be this far ahead
OBSTACLE_MARGIN_M = 0.6  # wingmen keep at least this gap between their edge and a box (with avoidance on)
OBSTACLE_PUSH_M = 1.5  # how hard a wingman near a box is pushed away from it
HEADING_RATE_RAD_S = math.radians(60.0)  # the formation's shape turns at most this fast
MOVING_SPEED_M_S = 0.5  # faster than this, the shape points along the direction of travel
TRAIL_STEP_M = 0.1  # the leader's trail (for the column) is recorded every 10 cm
TRAIL_KEEP_M = (MAX_WINGMEN + 1) * 4.0  # enough trail for the longest column


def slot_offsets(shape: str, n: int, spacing_m: float) -> list[tuple[float, float]]:
    """Each wingman's place relative to the leader, in the leader's frame: (metres forward, metres left).
    Wingmen alternate left/right, nearest first."""
    s = spacing_m
    out: list[tuple[float, float]] = []
    for i in range(n):
        rank, side = i // 2 + 1, (1 if i % 2 == 0 else -1)
        if shape == "v":
            out.append((-rank * s, side * rank * s))
        elif shape == "line":
            out.append((0.0, side * rank * s))
        elif shape == "column":
            out.append((-(i + 1) * s, 0.0))
        else:
            raise ValueError(f"Shape must be one of: {', '.join(SHAPES)}.")
    return out


def slot_world(leader_xy: np.ndarray, leader_yaw_rad: float, offset: tuple[float, float]) -> np.ndarray:
    """Turn a slot (forward, left) in the leader's frame into a point (x east, y north) in the world."""
    fwd, left = offset
    c, s = math.cos(leader_yaw_rad), math.sin(leader_yaw_rad)
    return np.array([leader_xy[0] + c * fwd - s * left, leader_xy[1] + s * fwd + c * left])


@dataclass
class LeaderInfo:
    """What the leader tells its wingmen each step (and, for collisions, where it really is)."""

    belief_pos_m: np.ndarray  # where the leader thinks it is (what it broadcasts)
    belief_vel_m_s: np.ndarray  # and how fast it thinks it's moving
    yaw_rad: float  # the leader's commanded heading
    alt_m: float  # the leader's target height
    phase: str  # 'flying' | 'landing' | 'off'
    autopilot: bool  # a mission autopilot is flying (it doesn't turn the nose)
    true_pos_m: np.ndarray  # where it really is (only for collisions)
    on_ground: bool


@dataclass
class Wingman:
    index: int
    sim: LightweightSim
    pilot: Pilot = field(default_factory=Pilot)
    crashed: bool = False
    breaking: bool = False  # left its slot to follow the leader's trail past an obstacle

    @property
    def state(self) -> State:
        return self.sim.state


class Formation:
    """A group of wingmen that follow the leader. Owned by the live sim (MissionSim)."""

    def __init__(self, count: int = 0, shape: str = "v", spacing_m: float = 1.5, avoid: bool = True):
        self.wingmen: list[Wingman] = []
        self.trail: list[np.ndarray] = []  # the leader's path (x, y), oldest first: the column follows it
        self.heading_rad = 0.0  # which way the shape points (see _update_heading)
        self.configure(count, shape, spacing_m, avoid)
        self.min_gap_m: float | None = None  # closest centre-to-centre distance between flying drones, now

    def configure(self, count: int, shape: str, spacing_m: float, avoid: bool) -> None:
        if not 0 <= int(count) <= MAX_WINGMEN:
            raise ValueError(f"Wingmen must be between 0 and {MAX_WINGMEN}.")
        if shape not in SHAPES:
            raise ValueError(f"Shape must be one of: {', '.join(SHAPES)}.")
        if not MIN_SPACING_M <= float(spacing_m) <= MAX_SPACING_M:
            raise ValueError(f"Spacing must be between {MIN_SPACING_M:.0f} and {MAX_SPACING_M:.0f} m.")
        self.count, self.shape, self.spacing_m, self.avoid = int(count), shape, float(spacing_m), bool(avoid)

    def offsets(self) -> list[tuple[float, float]]:
        return slot_offsets(self.shape, self.count, self.spacing_m)

    # ---- creating / removing wingmen ----

    def spawn(self, config: DroneConfig, seed: int, leader_xy: np.ndarray, leader_yaw_rad: float, env: EnvConfig,
              keep_existing: bool = False) -> None:
        """Put wingmen on the ground at their slots around the leader. With keep_existing, only add missing ones
        (and drop extras), so changing the number mid-flight doesn't disturb the others."""
        if not keep_existing:
            self.wingmen = []
            self.heading_rad = leader_yaw_rad
            # Start the trail as a straight line behind the leader, so a column has somewhere to be at once.
            back = -np.array([math.cos(leader_yaw_rad), math.sin(leader_yaw_rad)])
            n = int(TRAIL_KEEP_M / TRAIL_STEP_M)
            self.trail = [np.asarray(leader_xy, float) + back * TRAIL_STEP_M * k for k in range(n, -1, -1)]
        self.wingmen = self.wingmen[: self.count]
        for i in range(len(self.wingmen), self.count):
            xy = self._slot_xy(i, np.asarray(leader_xy, float), self.heading_rad)
            start = State.at(0.0, yaw_rad=leader_yaw_rad)
            start.pos_m[:2] = xy
            sim = LightweightSim()
            sim.reset(config, seed + 101 + i, start, env=env)  # own gusts; perfect sensors
            self.wingmen.append(Wingman(i, sim))

    def set_env(self, env: EnvConfig) -> None:
        for w in self.wingmen:
            w.sim.set_env(env)

    # ---- where each wingman should be ----

    def _record_trail(self, leader_xy: np.ndarray) -> None:
        if not self.trail or float(np.linalg.norm(leader_xy - self.trail[-1])) >= TRAIL_STEP_M:
            self.trail.append(leader_xy.copy())
            if len(self.trail) > TRAIL_KEEP_M / TRAIL_STEP_M + 1:
                self.trail.pop(0)

    def _trail_arcs(self, leader_xy: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """The leader's path, newest first (starting at the leader), and the distance back along it to each point."""
        pts = np.vstack([leader_xy[None, :], np.asarray(self.trail[::-1])]) if self.trail else leader_xy[None, :]
        arcs = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))])
        return pts, arcs

    def _trail_point(self, leader_xy: np.ndarray, back_m: float, arcs=None) -> np.ndarray:
        """The point `back_m` metres back along the leader's path (measured along the path, not straight)."""
        pts, arc = arcs or self._trail_arcs(leader_xy)
        k = int(np.searchsorted(arc, back_m))
        if k >= len(arc):
            return pts[-1]  # trail too short: the oldest point
        if k == 0:
            return pts[0]
        f = (back_m - arc[k - 1]) / max(arc[k] - arc[k - 1], 1e-9)
        return pts[k - 1] + (pts[k] - pts[k - 1]) * f

    @staticmethod
    def _trail_position(p_xy: np.ndarray, arcs) -> float:
        """How far back along the leader's path a drone is (the path point nearest to it)."""
        pts, arc = arcs
        return float(arc[int(np.argmin(np.sum((pts - p_xy) ** 2, axis=1)))])

    def _slot_xy(self, i: int, leader_xy: np.ndarray, heading_rad: float) -> np.ndarray:
        return slot_world(leader_xy, heading_rad, self.offsets()[i])

    def _update_heading(self, leader: LeaderInfo, dt_s: float) -> None:
        """Point the shape along the leader's nose. On a mission (the autopilot doesn't turn the nose) point it
        along the direction of travel instead, turning at most HEADING_RATE_RAD_S."""
        if not leader.autopilot:
            self.heading_rad = leader.yaw_rad
            return
        vx, vy = float(leader.belief_vel_m_s[0]), float(leader.belief_vel_m_s[1])
        want = math.atan2(vy, vx) if math.hypot(vx, vy) > MOVING_SPEED_M_S else self.heading_rad
        err = math.remainder(want - self.heading_rad, 2 * math.pi)
        step = HEADING_RATE_RAD_S * dt_s
        self.heading_rad = math.remainder(self.heading_rad + max(-step, min(step, err)), 2 * math.pi)

    @staticmethod
    def _blocked(p_from: np.ndarray, p_to: np.ndarray, z_m: float, obstacles: list[Box]) -> bool:
        """Is the straight way from p_from to p_to (x, y) too close to a box at this height?"""
        keep = DRONE_RADIUS_M + OBSTACLE_MARGIN_M
        boxes = [b for b in obstacles if b.blocks_altitude(z_m)]
        if not boxes:
            return False
        n = max(1, int(float(np.linalg.norm(p_to - p_from)) / 0.2))
        for k in range(n + 1):
            x, y = p_from + (p_to - p_from) * (k / n)
            if any(b.distance_2d(float(x), float(y)) < keep for b in boxes):
                return True
        return False

    @staticmethod
    def _obstacle_push(p: np.ndarray, obstacles: list[Box]) -> np.ndarray:
        """Push a wingman that is getting close to a box away from it (potential field, lesson 09)."""
        push = np.zeros(3)
        for b in obstacles:
            if not b.blocks_altitude(float(p[2])):
                continue
            gap = b.distance_2d(float(p[0]), float(p[1])) - DRONE_RADIUS_M
            if gap >= OBSTACLE_MARGIN_M or gap <= -DRONE_RADIUS_M:
                continue
            cx, cy = b.closest_point(float(p[0]), float(p[1]))
            d = np.array([p[0] - cx, p[1] - cy])
            n = float(np.linalg.norm(d))
            if n > 1e-6:
                push[:2] += d / n * OBSTACLE_PUSH_M * (OBSTACLE_MARGIN_M - gap) / OBSTACLE_MARGIN_M
        return push

    # ---- every physics step ----

    def _separation_push(self, me: Wingman, others: list[tuple[np.ndarray, np.ndarray, int | None]]) -> np.ndarray:
        """Move my target away from every drone I'm about to get too close to, judged LOOKAHEAD_S ahead.
        `others` = (position, velocity, wingman index or None for the leader). Returns a 3D offset for my target."""
        push = np.zeros(3)
        vertical = 0.0  # + = climb, - = duck
        p_ahead = me.state.pos_m + me.state.vel_m_s * LOOKAHEAD_S
        for q, v, other in others:
            d = p_ahead - (q + v * LOOKAHEAD_S)
            gap_now = float(np.linalg.norm(me.state.pos_m - q))
            gap_ahead = float(np.linalg.norm(d))
            strength = max(0.0, (SEPARATION_NOW_M - gap_now) / SEPARATION_NOW_M)
            closing = (gap_now - gap_ahead) / LOOKAHEAD_S > CLOSING_SPEED_M_S
            if closing:  # getting closer: judge by where we'll be
                strength = max(strength, (SEPARATION_M - gap_ahead) / SEPARATION_M)
            if strength <= 0.0:
                continue
            # Who goes up? Against the leader: odd/even wingmen split up and down. Between two wingmen: the
            # lower-numbered one climbs and the other ducks, so they never dodge into each other.
            up = (me.index % 2 == 0) if other is None else (me.index < other)
            vertical += (1 if up else -1) * min(1.0, 4 * strength)
            away = d if float(np.linalg.norm(d)) > 1e-6 else me.state.pos_m - q
            n = float(np.linalg.norm(away[:2]))
            if n > 1e-6:
                u = away[:2] / n
                push[:2] += u * SEPARATION_PUSH_M * strength
                if closing:  # head-on: also step sideways (like people passing in a corridor)
                    side = 1 if me.index % 2 == 0 else -1
                    push[:2] += side * np.array([-u[1], u[0]]) * SEPARATION_PUSH_M * strength
        push[2] = VERTICAL_DODGE_M * max(-1.0, min(1.0, vertical))
        return push

    def step(self, leader: LeaderInfo, obstacles: list[Box], dt_s: float) -> list[str]:
        """Steer and advance every wingman by one physics step. Returns messages (crashes)."""
        said: list[str] = []
        leader_xy = leader.belief_pos_m[:2]
        if leader.phase == "flying":
            self._record_trail(leader_xy)
            self._update_heading(leader, dt_s)
        # Who has to break formation (way to its slot blocked by a box)? They queue up along the leader's path in
        # the order they're already in, so nobody has to cross anybody to get to their place in the queue.
        arcs = self._trail_arcs(leader_xy)
        slots: dict[int, np.ndarray] = {}
        breaking: list[Wingman] = []
        for w in self.wingmen:
            w.breaking = False
            if w.crashed or leader.phase != "flying":
                continue
            slots[w.index] = self._slot_xy(w.index, leader_xy, self.heading_rad)
            if self.avoid and self._blocked(w.state.pos_m[:2], slots[w.index], leader.alt_m, obstacles):
                w.breaking = True
                breaking.append(w)
        breaking.sort(key=lambda w: self._trail_position(w.state.pos_m[:2], arcs))
        for rank, w in enumerate(breaking):
            # Its place in the queue, but never more than 1 m along the path from where it is now: it follows the
            # path itself (a carrot), instead of cutting a corner straight to its place.
            here = self._trail_position(w.state.pos_m[:2], arcs)
            back = min(max((rank + 1) * self.spacing_m, here - 1.0), here + 1.0)
            slots[w.index] = self._trail_point(leader_xy, back, arcs)
        for w in self.wingmen:
            s = w.state
            if w.crashed:
                w.sim.step(Setpoint(armed=False, position_control=True, x_m=float(s.pos_m[0]), y_m=float(s.pos_m[1])))
                continue
            if leader.phase == "flying":
                if w.pilot.mode in ("idle", "land"):
                    w.pilot.mode = "idle"
                    w.pilot.takeoff(s, max(leader.alt_m, 0.3))
                slot = slots[w.index]
                target = np.array([slot[0], slot[1], leader.alt_m])
                if self.avoid:
                    others = [(leader.belief_pos_m, leader.belief_vel_m_s, None)] + [
                        (o.state.pos_m, o.state.vel_m_s, o.index) for o in self.wingmen if o is not w and not o.crashed]
                    ahead = s.pos_m + s.vel_m_s * 0.3  # judge box closeness a moment ahead
                    target = target + self._separation_push(w, others) + self._obstacle_push(ahead, obstacles)
                w.pilot.sp.x_m, w.pilot.sp.y_m = float(target[0]), float(target[1])
                w.pilot.sp.alt_m = float(max(0.3, target[2]))
                w.pilot.sp.yaw_rad = leader.yaw_rad
            elif w.pilot.mode in ("takeoff", "hover"):
                w.pilot.land(s)  # the leader is landing or has stopped: land where you are
            w.sim.step(w.pilot.update(s, dt_s))
            if not s.on_ground and clearance_m(obstacles, *(float(v) for v in s.pos_m)) < 0:
                self._crash(w)
                said.append(f"💥 Wingman {w.index + 1} hit an obstacle.")
        return said

    def _crash(self, w: Wingman) -> None:
        w.crashed = True
        w.pilot.mode = "idle"
        w.state.vel_m_s[:2] = 0.0
        w.state.rate_rad_s[:] = 0.0

    def check_collisions(self, leader_true_pos_m: np.ndarray, leader_flying: bool) -> tuple[list[str], bool]:
        """Drone-to-drone contact on the TRUE positions. Returns (messages, did the leader get hit?).
        Also updates min_gap_m, the closest distance between any two drones in the air."""
        said: list[str] = []
        leader_hit = False
        drones: list[tuple[str, np.ndarray, Wingman | None]] = []
        if leader_flying:
            drones.append(("your drone", leader_true_pos_m, None))
        drones += [(f"wingman {w.index + 1}", w.state.pos_m, w) for w in self.wingmen if not w.crashed and not w.state.on_ground]
        gaps = []
        for i in range(len(drones)):
            for j in range(i + 1, len(drones)):
                (na, pa, wa), (nb, pb, wb) = drones[i], drones[j]
                dist = float(np.linalg.norm(pa - pb))
                gaps.append(dist)
                if dist < COLLISION_DIST_M:
                    for w in (wa, wb):
                        if w is None:
                            leader_hit = True
                        elif not w.crashed:
                            self._crash(w)
                    said.append(f"💥 {na.capitalize()} and {nb} collided ({dist:.2f} m apart). Both motors stopped.")
        self.min_gap_m = min(gaps) if gaps else None
        return said, leader_hit

    def snapshot(self) -> dict:
        r = lambda v, n=3: [round(float(x), n) for x in v]  # noqa: E731
        return {
            "count": self.count,
            "shape": self.shape,
            "spacing_m": self.spacing_m,
            "avoid": self.avoid,
            "min_gap_m": round(self.min_gap_m, 2) if self.min_gap_m is not None else None,
            "wingmen": [
                {"pos_m": r(w.state.pos_m), "quat": r(w.state.quat, 4), "motor_pct": r(w.sim.motor_pct, 0), "crashed": w.crashed,
                 "breaking": w.breaking}
                for w in self.wingmen
            ],
        }
