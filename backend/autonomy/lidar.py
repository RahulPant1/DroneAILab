"""Sense-and-avoid: a simulated 2D lidar, a map built from what it sees, and a mission follower that replans.

The other planners KNOW the obstacles before take-off. A real drone usually doesn't: it has to sense them.
  1. LIDAR: every 0.1 s, 72 laser beams sweep a full circle at the drone's height (out to 8 m). Each beam
     measures the distance to the first obstacle it hits (from where the drone REALLY is, with a little noise).
  2. MAP: each hit is drawn into an occupancy grid at the position the drone THINKS it is (its estimate), so with
     noisy sensors the map gets smeared, just like on a real drone. Cells near a hit are "blocked" (the drone's
     size + a safety margin).
  3. PLAN: A* on that grid. Space it hasn't seen yet is assumed free ("optimistic"), so it flies straight at the
     goal until it sees something in the way.
  4. REPLAN: whenever the path ahead runs into newly blocked cells, plan again from where it is. If something is
     suddenly close in the direction of travel, brake first.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass

import numpy as np

from backend.sim.control import Setpoint
from backend.sim.physics import State

from .follower import Mission, MissionFollower
from .planner import Point
from .world import DRONE_RADIUS_M, SAFETY_MARGIN_M, Box

BEAMS = 72  # one every 5°
RANGE_M = 8.0
SCAN_PERIOD_S = 0.1
NOISE_STD_M = 0.02
MAP_RES_M = 0.2
KEEP_OUT_M = DRONE_RADIUS_M + SAFETY_MARGIN_M
LOOKAHEAD_M = 4.0  # check this much of the path ahead for newly seen obstacles
BRAKE_CLEARANCE_M = 0.45  # an obstacle edge closer than this (beyond the drone's radius), in the way: brake
BRAKE_CONE_RAD = math.radians(35)
SEARCH_MARGIN_CELLS = 60  # A* may roam this far (12 m) beyond the start/goal box
MAX_EXPANSIONS = 60_000
NO_PATH_GIVE_UP_S = 3.0  # keep trying (and scanning) this long before declaring itself stuck

ANGLES = np.linspace(0.0, 2 * math.pi, BEAMS, endpoint=False)
COS, SIN = np.cos(ANGLES), np.sin(ANGLES)


def scan(boxes: list[Box], x: float, y: float, z: float, range_m: float = RANGE_M,
         angles_rad: np.ndarray | None = None) -> np.ndarray:
    """Distance along each beam (world-frame angles ANGLES, or `angles_rad`: 0 = east, counter-clockwise) to the
    first box tall enough to be hit at height z. range_m = nothing within range. Ray vs box uses the 'slab' method,
    for all beams at once."""
    cos, sin = (COS, SIN) if angles_rad is None else (np.cos(angles_rad), np.sin(angles_rad))
    ranges = np.full(len(cos), range_m)
    with np.errstate(divide="ignore", invalid="ignore"):
        inv_x = np.where(cos != 0, 1.0 / cos, np.inf)
        inv_y = np.where(sin != 0, 1.0 / sin, np.inf)
        for b in boxes:
            if b.height_m <= z:
                continue  # it flies over this one
            tx1, tx2 = (b.x_min - x) * inv_x, (b.x_max - x) * inv_x
            ty1, ty2 = (b.y_min - y) * inv_y, (b.y_max - y) * inv_y
            t_near = np.maximum(np.minimum(tx1, tx2), np.minimum(ty1, ty2))
            t_far = np.minimum(np.maximum(tx1, tx2), np.maximum(ty1, ty2))
            hit = (t_near <= t_far) & (t_far > 0)
            t = np.where(hit, np.maximum(t_near, 0.0), np.inf)
            ranges = np.minimum(ranges, np.nan_to_num(t, nan=np.inf, posinf=np.inf))
    return np.minimum(ranges, range_m)


class OccupancyMap:
    """Cells (MAP_RES_M squares) where the lidar hit something, and the 'blocked' cells around them."""

    def __init__(self, res: float = MAP_RES_M, keep_out_m: float = KEEP_OUT_M):
        self.res = res
        self.occupied: set[tuple[int, int]] = set()
        self.blocked: set[tuple[int, int]] = set()
        r = int(math.ceil(keep_out_m / res))
        self._disc = [(dx, dy) for dx in range(-r, r + 1) for dy in range(-r, r + 1) if math.hypot(dx, dy) * res <= keep_out_m]
        self.version = 0  # goes up whenever a new cell is seen

    def cell(self, x: float, y: float) -> tuple[int, int]:
        return round(x / self.res), round(y / self.res)

    def point(self, c: tuple[int, int]) -> Point:
        return c[0] * self.res, c[1] * self.res

    def add_hits(self, x: float, y: float, ranges: np.ndarray, range_m: float = RANGE_M) -> int:
        """Draw hits seen from (x, y) into the map. Returns how many new cells were found."""
        new = 0
        for r, c, s in zip(ranges, COS, SIN):
            if r >= range_m - 1e-6:
                continue
            cell = self.cell(x + r * c, y + r * s)
            if cell not in self.occupied:
                self.occupied.add(cell)
                new += 1
                for dx, dy in self._disc:
                    self.blocked.add((cell[0] + dx, cell[1] + dy))
        if new:
            self.version += 1
        return new

    def is_blocked(self, x: float, y: float) -> bool:
        return self.cell(x, y) in self.blocked

    def segment_free(self, a: Point, b: Point) -> bool:
        n = max(1, math.ceil(math.dist(a, b) / (self.res / 2)))
        return not any(self.is_blocked(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n) for i in range(n + 1))

    def plan(self, start: Point, goal: Point) -> list[Point] | None:
        """A* from start to goal through cells not known to be blocked (unseen = free). None = no way found.
        The start and the cells right around it are allowed even if blocked (it may already be inside a margin)."""
        s, g = self.cell(*start), self.cell(*goal)
        if g in self.blocked:
            return None
        lo_x, hi_x = min(s[0], g[0]) - SEARCH_MARGIN_CELLS, max(s[0], g[0]) + SEARCH_MARGIN_CELLS
        lo_y, hi_y = min(s[1], g[1]) - SEARCH_MARGIN_CELLS, max(s[1], g[1]) + SEARCH_MARGIN_CELLS
        near_start = {(s[0] + dx, s[1] + dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1)}

        def blocked(c):
            return (c in self.blocked and c not in near_start) or not (lo_x <= c[0] <= hi_x and lo_y <= c[1] <= hi_y)

        moves = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0)] + [(dx, dy, math.sqrt(2)) for dx in (1, -1) for dy in (1, -1)]
        h = lambda c: math.hypot(c[0] - g[0], c[1] - g[1])  # noqa: E731
        heap = [(h(s), 0.0, s)]
        came: dict = {}
        cost = {s: 0.0}
        expanded = 0
        while heap:
            _, cc, c = heapq.heappop(heap)
            if c == g:
                break
            if cc > cost[c]:
                continue
            expanded += 1
            if expanded > MAX_EXPANSIONS:
                return None
            for dx, dy, step in moves:
                n = (c[0] + dx, c[1] + dy)
                if blocked(n) or (dx and dy and (blocked((c[0] + dx, c[1])) or blocked((c[0], c[1] + dy)))):
                    continue
                nc = cc + step
                if nc < cost.get(n, math.inf):
                    cost[n] = nc
                    came[n] = c
                    heapq.heappush(heap, (nc + h(n), nc, n))
        else:
            return None
        cells = [g]
        while cells[-1] != s:
            cells.append(came[cells[-1]])
        raw = [start] + [self.point(c) for c in reversed(cells[1:-1])] + [goal]
        out, i = [raw[0]], 0  # straighten: jump to the farthest point in clear line of sight
        while i < len(raw) - 1:
            j = len(raw) - 1
            while j > i + 1 and not self.segment_free(raw[i], raw[j]):
                j -= 1
            out.append(raw[j])
            i = j
        return out


@dataclass
class LidarScan:
    origin: tuple[float, float, float]  # where the lidar REALLY was (the beams start here)
    ranges: np.ndarray
    t_s: float


class Perception:
    """A lidar plus the map it builds. Used by the sense-and-avoid follower, and by the live view's 📡 toggle."""

    def __init__(self, seed: int = 0):
        self.map = OccupancyMap()
        self.last: LidarScan | None = None
        self._next_t = 0.0
        self._rng = np.random.default_rng([seed, 4242])
        self.scans = 0

    def update(self, t: float, truth: State, estimate: State, boxes: list[Box]) -> int:
        """Scan if it's time. Returns the number of newly seen cells (0 if no scan)."""
        if t < self._next_t:
            return 0
        self._next_t = t + SCAN_PERIOD_S
        tx, ty, tz = (float(v) for v in truth.pos_m)
        ranges = scan(boxes, tx, ty, tz)
        noisy = np.where(ranges < RANGE_M, np.clip(ranges + self._rng.normal(0, NOISE_STD_M, BEAMS), 0.0, RANGE_M), ranges)
        self.last = LidarScan((tx, ty, tz), noisy, t)
        self.scans += 1
        ex, ey = float(estimate.pos_m[0]), float(estimate.pos_m[1])
        return self.map.add_hits(ex, ey, noisy)

    def snapshot(self, with_map: bool) -> dict | None:
        if not self.last:
            return None
        out = {"origin": [round(v, 3) for v in self.last.origin], "ranges": [round(float(r), 2) for r in self.last.ranges],
               "range_m": RANGE_M, "map_version": self.map.version, "cells": len(self.map.occupied), "res_m": self.map.res}
        if with_map:
            out["map"] = [list(self.map.point(c)) for c in sorted(self.map.occupied)]
        return out


class SenseFollower(MissionFollower):
    """Flies a mission knowing only the waypoints: senses obstacles with the lidar and replans around them."""

    def __init__(self, mission: Mission, seed: int = 0):
        super().__init__(mission, list(mission.waypoints), "lidar")
        self.perception = Perception(seed)
        self.replans = 0
        self.brakes = 0
        self._no_path_since: float | None = None
        self.plans: list[list[Point]] = [list(mission.waypoints)]  # every route it decided on (for the map plot)

    # The route is replaced while flying: recompute everything MissionFollower derives from it.
    def _set_route(self, route: list[Point]) -> None:
        self.route = route
        self.cum = [0.0]
        for a, b in zip(route, route[1:]):
            self.cum.append(self.cum[-1] + math.dist(a, b))
        self.total_len = self.cum[-1]
        self.s = 0.0
        wp_s = [0.0] * (len(self.m.waypoints) - 1)
        i = 0
        for k in range(self.next_wp, len(self.m.waypoints)):
            wp = self.m.waypoints[k]
            i = next((j for j in range(i, len(route)) if math.dist(route[j], wp) < 1e-6), len(route) - 1)
            wp_s[k - 1] = self.cum[i]
        self.wp_s = wp_s

    def _replan(self, x: float, y: float, t: float) -> bool:
        route: list[Point] = [(x, y)]
        for k in range(self.next_wp, len(self.m.waypoints)):
            leg = self.perception.map.plan(route[-1], self.m.waypoints[k])
            if leg is None:
                return False
            route += leg[1:]
        self.replans += 1
        self._set_route(route)
        self.plans.append(route)
        self._event(t, "replan", f"Saw an obstacle in the way: replanned ({self.perception.map.version} map updates so far).", x, y)
        return True

    def _path_ahead_blocked(self) -> bool:
        m = self.perception.map
        end = min(self.total_len, self.s + LOOKAHEAD_M)
        n = max(1, int((end - self.s) / (MAP_RES_M / 2)))
        return any(m.is_blocked(*self._point_at(self.s + (end - self.s) * i / n)) for i in range(n + 1))

    def _obstacle_close_ahead(self, x: float, y: float, cx: float, cy: float) -> bool:
        scan_ = self.perception.last
        if scan_ is None or math.hypot(cx - x, cy - y) < 0.05:
            return False
        heading = math.atan2(cy - y, cx - x)
        diff = np.abs((ANGLES - heading + math.pi) % (2 * math.pi) - math.pi)
        close = scan_.ranges[diff < BRAKE_CONE_RAD]
        return bool(close.size and close.min() < DRONE_RADIUS_M + BRAKE_CLEARANCE_M)

    def setpoint_for(self, state: State, t: float, truth: State | None = None) -> Setpoint:
        truth = truth or state
        new_cells = self.perception.update(t, truth, state, self.m.obstacles)
        if self.phase != "cruise":
            return super().setpoint_for(state, t, truth)
        x, y = float(state.pos_m[0]), float(state.pos_m[1])
        if new_cells and self._path_ahead_blocked():
            if self._replan(x, y, t):
                self._no_path_since = None
            elif self._no_path_since is None:
                self._no_path_since = t
        if self._no_path_since is not None:
            # No way through what it has seen: hold position and keep scanning; give up after a while.
            if self._replan(x, y, t):
                self._no_path_since = None
            elif t - self._no_path_since > NO_PATH_GIVE_UP_S:
                self.phase = "stuck"
                self._end_at = t + 1.5
                self._event(t, "stuck", f"Stuck at ({x:.1f}, {y:.1f}): the obstacles it has seen block every way to waypoint {self.next_wp}.", x, y)
            self._sp.x_m, self._sp.y_m = x, y
            self._t_last = t
            return self._sp
        sp = super().setpoint_for(state, t, truth)
        if self.phase == "cruise" and self._obstacle_close_ahead(x, y, sp.x_m, sp.y_m):
            self.brakes += 1
            if self.brakes == 1 or self.brakes % 50 == 0:
                self._event(t, "brake", "Something close ahead: braking and replanning.", x, y)
            sp.x_m, sp.y_m = x, y  # stop where it is
            self.s = max(0.0, self.s - self.m.cruise_speed_m_s * 0.002)  # hold the carrot back
            self._replan(x, y, t)
        return sp
