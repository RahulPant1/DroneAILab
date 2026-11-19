"""Missions in the live Fly view: a course (obstacles + waypoints) in the 3D world, and a mission autopilot.

With a course loaded:
  - you can fly it yourself (keyboard / buttons). Touching an obstacle stops the motors, like in the Lab.
  - "Fly mission" puts the drone on the start point, plans a route with the chosen planner, and lets the
    same MissionFollower as the Lab fly it, in real time. It navigates on the estimate (so with sensor
    noise on it can be fooled, like a real autopilot); crashes and ground contact use the truth. When it lands, crashes or gets stuck, the
    controls come back to you (stuck = it hovers where it is).
  - Hover / Land / Fly-to during a mission take over from the autopilot.
"""

from __future__ import annotations

import math

from backend.sim.config import DroneConfig
from backend.sim.control import Setpoint
from backend.sim.physics import State
from backend.sim.pilot import Pilot
from backend.sim.runner import RealtimeSim

from .follower import Mission, MissionFollower
from .mission import PLANNERS, SCENARIOS, MissionError, scenario_mission
from .planner import PlanningError, plan_route
from .world import DRONE_RADIUS_M, SAFETY_MARGIN_M, clearance_m

MIN_SPEED_M_S, MAX_SPEED_M_S = 0.3, 3.0


class MissionSim(RealtimeSim):
    """The live simulator plus an optional course and mission autopilot."""

    def __init__(self, config: DroneConfig, seed: int = 0):
        self.course_id: str | None = None
        self.mission: Mission | None = None
        self.route: list[tuple[float, float]] | None = None
        self.planner: str | None = None
        self.follower: MissionFollower | None = None
        super().__init__(config, seed)

    # ---- course ----

    def start_state(self) -> State | None:
        if not self.mission:
            return None
        s = State.at(0.0)
        s.pos_m[:2] = self.mission.waypoints[0]
        return s

    def reset(self, config: DroneConfig | None = None, seed: int | None = None) -> None:
        self.follower = None
        self.route = None
        self.planner = None
        super().reset(config, seed)

    def load_course(self, scenario_id: str | None) -> str:
        """Load a built-in course (or None for an empty world) and put the drone on its start point."""
        if scenario_id is not None and scenario_id not in SCENARIOS:
            raise ValueError(f"Unknown course '{scenario_id}'. Choose one of: {', '.join(SCENARIOS)}.")
        self.course_id = scenario_id
        self.mission = scenario_mission(scenario_id) if scenario_id else None
        self.reset()
        if not scenario_id:
            return "Free flight: no obstacles."
        return f"Loaded '{SCENARIOS[scenario_id]['title']}'. Fly it yourself, or press Fly mission."

    def course_info(self) -> dict | None:
        """Everything the 3D view needs to draw the course (sent once per change, not every frame)."""
        if not self.mission:
            return None
        m = self.mission
        return {
            "id": self.course_id,
            "title": SCENARIOS[self.course_id]["title"],
            "waypoints": [list(p) for p in m.waypoints],
            "obstacles": [b.to_dict() for b in m.obstacles],
            "cruise_alt_m": m.cruise_alt_m,
            "safety_zone_m": DRONE_RADIUS_M + SAFETY_MARGIN_M,
            "route": [[round(a, 3), round(b, 3)] for a, b in self.route] if self.route else None,
            "planner": self.planner,
        }

    # ---- mission autopilot ----

    def start_mission(self, planner: str, cruise_speed_m_s: float | None = None) -> str:
        if not self.mission:
            raise ValueError("Load a course first.")
        if planner not in PLANNERS:
            raise ValueError(f"Planner must be one of: {', '.join(PLANNERS)}.")
        speed = self.mission.cruise_speed_m_s if cruise_speed_m_s is None else float(cruise_speed_m_s)
        if not MIN_SPEED_M_S <= speed <= MAX_SPEED_M_S:
            raise ValueError(f"Cruise speed must be between {MIN_SPEED_M_S} and {MAX_SPEED_M_S} m/s.")
        self.mission.cruise_speed_m_s = speed
        try:
            route, _ = plan_route(self.mission.waypoints, self.mission.obstacles, planner, self.mission.cruise_alt_m)
        except (PlanningError, MissionError) as exc:
            raise ValueError(str(exc)) from None
        self.reset()  # back to the start point, clock at 0 (the follower times its phases from t = 0)
        self.route, self.planner = route, planner
        self.follower = MissionFollower(self.mission, route, planner)
        self._events_seen = 0
        self.pilot.mode = "mission"
        wait = self.gps_settle_left_s()
        self.pilot.message = f"Mission ({planner}): " + (f"waiting {wait:.0f} s for GPS to settle." if wait > 0 else "taking off.")
        gps = f" Waiting {wait:.0f} s for GPS to settle first." if wait > 0 else ""
        return f"Mission started with the {planner} planner at {speed:.1f} m/s.{gps}"

    def _end_mission(self, state: State) -> None:
        """The autopilot is finished: give the controls back."""
        phase = self.follower.phase
        self.follower = None
        self.pilot = Pilot()
        if phase == "stuck":
            self.pilot.mode = "hover"  # Pilot.hover() needs a flying mode to start from
            self.pilot.hover(self.sim.estimate)
            self.pilot.message = "Stuck: the potential field can't find a way. You have the controls."
        elif phase == "crashed":
            self.pilot.message = "Crashed into an obstacle. Press Reset or Fly mission to try again."
        else:
            self.pilot.message = "Mission complete. Landed at the final waypoint."

    def _take_over(self) -> None:
        """Hover / Land / Fly-to during a mission: stop the autopilot, keep its current target."""
        if not self.follower:
            return
        sp = self.follower._sp
        self.follower = None
        self.pilot = Pilot()
        self.pilot.sp = Setpoint(armed=True, alt_m=sp.alt_m, position_control=True, x_m=sp.x_m, y_m=sp.y_m)
        self.pilot.mode = "hover"
        self.notices.append("Autopilot off: you have the controls.")

    # ---- every physics step ----

    def next_setpoint(self, dt_s: float) -> Setpoint:
        s = self.state
        if self.follower and self.gps_settle_left_s() > 0:
            self.follower._t_last = s.time_s  # the autopilot's clock starts once it may take off
            return Setpoint(armed=False, position_control=True, x_m=float(s.pos_m[0]), y_m=float(s.pos_m[1]))
        if self.follower and self.failsafe_active():
            self._take_over()  # a mission can't navigate without GPS: abort, the failsafe holds the drone
            self.notices.append("Mission aborted: GPS lost.")
        if self.follower:
            sp = self.follower.setpoint_for(self.sim.estimate, s.time_s, s)  # navigates on the estimate
            for ev in self.follower.events[self._events_seen:]:
                msg = ev["message"]
                if ev["type"] == "waypoint" and self.sim.estimator:
                    # The autopilot judged this from its estimate; say where the drone REALLY was.
                    wx, wy = self.mission.waypoints[self.follower.next_wp - 1]
                    real = math.hypot(wx - s.pos_m[0], wy - s.pos_m[1])
                    msg = f"{msg[:-1]}, by its estimate. Really {real:.2f} m away."
                self.notices.append(msg)
                self.pilot.message = msg
            self._events_seen = len(self.follower.events)
            if self.follower.phase == "crashed":
                self._hit_wall(s)
            if self.follower.phase in ("landed", "crashed", "stuck"):
                self._end_mission(s)
            return sp
        sp = self.apply_failsafe(self.pilot.update(self.sim.estimate, dt_s, s))  # estimate decides; ground contact from truth
        if self.mission and sp.armed and clearance_m(self.mission.obstacles, *(float(v) for v in s.pos_m)) < 0:
            sp.armed = False
            self.pilot.mode = "idle"
            self._hit_wall(s)
            msg = f"Hit an obstacle at ({s.pos_m[0]:.1f}, {s.pos_m[1]:.1f}). Motors stopped."
            self.pilot.message = msg
            self.notices.append(msg)
        return sp

    @staticmethod
    def _hit_wall(state: State) -> None:
        """The obstacle stops the drone sideways (and stops it spinning); then it simply drops.
        A crude stand-in for a real impact, so the drone falls beside the wall instead of through it."""
        state.vel_m_s[:2] = 0.0
        state.rate_rad_s[:] = 0.0

    # ---- buttons ----

    def command(self, cmd: str, **args) -> str:
        if cmd == "course":
            return self.load_course(args.get("scenario") or None)
        if cmd == "mission":
            return self.start_mission(str(args.get("planner", "astar")), args.get("cruise_speed_m_s"))
        if self.follower:
            if cmd == "takeoff":
                return "A mission is flying. Press Hover here or Land to take over."
            if cmd in ("hover", "land", "target"):
                self._take_over()
        return super().command(cmd, **args)

    def snapshot(self) -> dict:
        snap = super().snapshot()
        f = self.follower
        snap["mission"] = (
            {"planner": self.planner, "phase": "gps" if self.gps_settle_left_s() > 0 else f.phase, "next_wp": f.next_wp, "waypoints_total": len(f.m.waypoints) - 1}
            if f else None
        )  # fmt: skip
        return snap
