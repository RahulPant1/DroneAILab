"""Mission follower: turns a route into position setpoints, moment by moment.

Phases: takeoff -> cruise -> land -> done.  (Or: crashed / stuck -> done.)

How it steers during cruise:
  route planners (straight, astar): a "carrot" point slides along the route at the cruise speed, and the
      position controller chases it. If the drone falls too far behind, the carrot waits.
  potential field: no route. Each moment the carrot is placed one step ahead in the direction of
      (pull toward the next waypoint) + (push away from nearby obstacles).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from backend.sim.control import Setpoint
from backend.sim.physics import State

from .planner import Point
from .world import DRONE_RADIUS_M, Box, clearance_m

WAYPOINT_REACHED_M = 0.5
TAKEOFF_DONE_M = 0.15
LAND_SPEED_M_S = 0.6
CARROT_LEASH_M = 2.0  # the carrot waits if the drone is more than this far behind
REPULSION_RANGE_M = 1.5  # obstacles closer than this (edge to edge) push the drone away
REPULSION_GAIN = 0.3
STUCK_WINDOW_S = 6.0  # potential field: no progress for this long = stuck
STUCK_PROGRESS_M = 0.2
END_AFTER_S = 1.5  # keep recording briefly after landing / crashing / getting stuck


@dataclass
class Mission:
    waypoints: list[Point]  # first = start (take-off point), last = landing point
    obstacles: list[Box]
    cruise_alt_m: float = 2.0
    cruise_speed_m_s: float = 1.5


class MissionFollower:
    def __init__(self, mission: Mission, route: list[Point], planner: str):
        self.m = mission
        self.route = route
        self.planner = planner
        self.cum = [0.0]
        for a, b in zip(route, route[1:]):
            self.cum.append(self.cum[-1] + math.dist(a, b))
        self.total_len = self.cum[-1]
        self.phase = "takeoff"
        self.s = 0.0  # carrot position along the route (m)
        self.next_wp = 1  # index of the next waypoint to reach
        self.events: list[dict] = []
        self.done = False
        self._t_last = 0.0
        self._end_at = math.inf
        self._land_alt = mission.cruise_alt_m
        self._ground_time = 0.0
        self._best_dist = math.inf
        self._best_time = 0.0
        self._sp = Setpoint(armed=True, alt_m=mission.cruise_alt_m, position_control=True, x_m=route[0][0], y_m=route[0][1])
        self.cruise_started_s: float | None = None
        self.finished_s: float | None = None
        # Where each waypoint sits along the route (route planners), and the closest approach to each.
        self.wp_s: list[float] = []
        i = 0
        for wp in mission.waypoints[1:]:
            i = next(k for k in range(i + 1, len(route)) if math.dist(route[k], wp) < 1e-6)
            self.wp_s.append(self.cum[i])
        self._wp_closest = math.inf
        self.waypoint_miss_m: list[float] = []

    # ---- helpers ----

    def _event(self, t: float, kind: str, message: str, x: float, y: float) -> None:
        self.events.append({"t_s": round(t, 3), "type": kind, "message": message, "x_m": round(x, 3), "y_m": round(y, 3)})

    def _point_at(self, s: float) -> Point:
        s = min(max(s, 0.0), self.total_len)
        for i in range(len(self.route) - 1):
            if s <= self.cum[i + 1] or i == len(self.route) - 2:
                seg = self.cum[i + 1] - self.cum[i]
                f = 0.0 if seg == 0 else (s - self.cum[i]) / seg
                a, b = self.route[i], self.route[i + 1]
                return a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f
        return self.route[-1]

    def _potential_carrot(self, x: float, y: float, goal: Point) -> Point:
        """Pull toward the goal (length 1) + push away from close obstacles; step one 'lead' in that direction."""
        gx, gy = goal[0] - x, goal[1] - y
        dist = math.hypot(gx, gy)
        lead = self.m.cruise_speed_m_s
        if dist < 1e-9:
            return goal
        fx, fy = gx / dist, gy / dist
        for b in self.m.obstacles:
            if not b.blocks_altitude(self.m.cruise_alt_m):
                continue
            rho = b.distance_2d(x, y) - DRONE_RADIUS_M  # edge-to-edge gap
            if rho >= REPULSION_RANGE_M:
                continue
            rho = max(rho, 0.05)
            cx, cy = b.closest_point(x, y)
            ax, ay = x - cx, y - cy
            n = math.hypot(ax, ay) or 1.0
            push = REPULSION_GAIN * (1.0 / rho - 1.0 / REPULSION_RANGE_M) / (rho * rho)
            fx, fy = fx + push * ax / n, fy + push * ay / n
        mag = math.hypot(fx, fy)
        if mag < 1e-6:
            return x, y
        step = min(lead, dist) if mag > 0 else 0.0
        return x + fx / mag * step, y + fy / mag * step

    # ---- called every physics step ----

    def setpoint_for(self, state: State, t: float) -> Setpoint:
        dt = max(0.0, t - self._t_last)
        self._t_last = t
        x, y, z = (float(v) for v in state.pos_m)
        sp = self._sp
        if t >= self._end_at:
            self.done = True

        if self.phase in ("cruise", "takeoff", "land") and clearance_m(self.m.obstacles, x, y, z) < 0:
            self.phase = "crashed"
            sp.armed = False
            self._end_at = t + END_AFTER_S + 1.5
            self._event(t, "collision", f"Hit an obstacle at ({x:.1f}, {y:.1f}). Motors stopped.", x, y)
            return sp

        if self.phase == "takeoff":
            if z > self.m.cruise_alt_m - TAKEOFF_DONE_M:
                self.phase = "cruise"
                self.cruise_started_s = t
                self._best_time = t
                self._event(t, "cruise", "Reached cruise altitude; starting the route.", x, y)

        elif self.phase == "cruise":
            # Waypoint bookkeeping (in order).
            if self.next_wp < len(self.m.waypoints):
                wx, wy = self.m.waypoints[self.next_wp]
                d = math.hypot(wx - x, wy - y)
                self._wp_closest = min(self._wp_closest, d)
                last = self.next_wp == len(self.m.waypoints) - 1
                # Route planners: a waypoint also counts once the carrot is past it and the drone is moving
                # away again (it cut the corner). The closest approach is recorded, so the miss is visible.
                passed = (
                    self.planner != "potential" and not last
                    and self.s >= self.wp_s[self.next_wp - 1] and d > self._wp_closest + 0.05
                )  # fmt: skip
                if d < WAYPOINT_REACHED_M or passed:
                    miss = self._wp_closest
                    self.waypoint_miss_m.append(miss)
                    how = "Reached" if miss < WAYPOINT_REACHED_M else f"Passed (missed by {miss:.2f} m)"
                    self._event(t, "waypoint", f"{how} waypoint {self.next_wp}{' (final)' if last else ''}.", x, y)
                    self._wp_closest = math.inf
                    self.next_wp += 1
                    self._best_dist, self._best_time = math.inf, t  # progress is measured per waypoint
                    if last:
                        self.phase = "land"
                        self.finished_s = t
                        self._land_alt = self.m.cruise_alt_m
                        sp.x_m, sp.y_m = wx, wy
                        return sp

            if self.planner == "potential":
                goal = self.m.waypoints[min(self.next_wp, len(self.m.waypoints) - 1)]
                sp.x_m, sp.y_m = self._potential_carrot(x, y, goal)
                d = math.hypot(goal[0] - x, goal[1] - y)
                if d < self._best_dist - STUCK_PROGRESS_M:
                    self._best_dist, self._best_time = d, t
                elif t - self._best_time > STUCK_WINDOW_S:
                    self.phase = "stuck"
                    sp.x_m, sp.y_m = x, y
                    self._end_at = t + END_AFTER_S
                    self._event(t, "stuck", f"Stuck at ({x:.1f}, {y:.1f}): no progress toward waypoint {self.next_wp} for {STUCK_WINDOW_S:.0f} s.", x, y)
            else:
                cx, cy = sp.x_m, sp.y_m
                if math.hypot(cx - x, cy - y) < CARROT_LEASH_M:
                    self.s = min(self.total_len, self.s + self.m.cruise_speed_m_s * dt)
                sp.x_m, sp.y_m = self._point_at(self.s)

        elif self.phase == "land":
            self._land_alt = max(-0.3, self._land_alt - LAND_SPEED_M_S * dt)
            sp.alt_m = self._land_alt
            self._ground_time = self._ground_time + dt if state.on_ground else 0.0
            if self._ground_time > 0.3:
                sp.armed = False
                self.phase = "landed"
                self._end_at = t + END_AFTER_S
                self._event(t, "landed", "Landed at the final waypoint.", x, y)
        return sp
