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

from backend.games.race import TRACKS, Race

from .follower import Mission, MissionFollower
from .lidar import Perception, SenseFollower
from .plan import CompiledPlan, PlanError, PlanFollower, compile_plan, make_lander
from .precland import PrecisionLander, image_b64, render
from .site import FARM
from .survey import CAMERA
from .formation import SHAPE_NAMES, Formation, LeaderInfo
from .mission import PLANNERS, SCENARIOS, MissionError, scenario_mission
from .planner import PlanningError, plan_route
from .world import DRONE_RADIUS_M, SAFETY_MARGIN_M, clearance_m

MIN_SPEED_M_S, MAX_SPEED_M_S = 0.3, 3.0
FARM_ID = "farm"  # the 🌾 farm: fields, buildings, no-fly zones, landing pads (site.py)
CAMERA_SEND_EVERY_S = 0.2  # down-camera pictures go to the UI 5 times a second (sim time)
PRECLAND_MAX_DIST_M = 12.0  # 🎯 precision landing needs a pad within this distance


class MissionSim(RealtimeSim):
    """The live simulator plus an optional course and mission autopilot."""

    def __init__(self, config: DroneConfig, seed: int = 0):
        self.course_id: str | None = None
        self.mission: Mission | None = None
        self.route: list[tuple[float, float]] | None = None
        self.planner: str | None = None
        self.follower: MissionFollower | None = None
        self.race: Race | None = None  # rings race (a game); never loaded together with a course
        self._race_prev_pos = None
        self.follower_done: MissionFollower | None = None
        self.formation = Formation()  # wingmen that follow your drone (0 = off); kept across resets
        self.lidar_on = False  # 📡 show the lidar + its map while you fly (a lidar mission always has one)
        self.lidar_view: Perception | None = None
        self._lidar_sent: tuple[int, int] | None = None  # (which map, version) the UI already has
        self.plan: CompiledPlan | None = None  # 🌾 the flight plan being flown / last flown on the farm
        self.prec_lander: PrecisionLander | None = None  # 🎯 precision landing started from the button
        self.camera_on = False  # 📷 show the down camera's picture
        self._camera_img = None
        self._camera_next_s = 0.0
        self._camera_sent_s = -1.0
        self._photos_sent = -1
        self._in_no_fly: str | None = None
        self.cropmaps: list[dict] = []  # 🌱 crop-health maps from the last survey (made in a background thread)
        self.cropmap_busy = False
        self._cropmaps_version = 0
        self._cropmaps_sent = 0
        super().__init__(config, seed)

    # ---- course ----

    def start_state(self) -> State | None:
        if self.race:
            s = State.at(0.0)
            s.pos_m[:2] = self.race.track["start"]
            return s
        if not self.mission:
            return None
        s = State.at(0.0)
        s.pos_m[:2] = self.mission.waypoints[0]
        return s

    def reset(self, config: DroneConfig | None = None, seed: int | None = None) -> None:
        self.follower = None
        self.route = None
        self.planner = None
        if self.race:
            self.race.restart()  # Reset = try the race again
        self._race_prev_pos = None
        self.prec_lander = None
        self._camera_img, self._camera_next_s, self._camera_sent_s = None, 0.0, -1.0
        self._photos_sent = -1
        self._in_no_fly = None
        super().reset(config, seed)
        self.lidar_view = Perception(self.seed) if self.lidar_on else None  # Reset = an empty map
        self._spawn_formation()

    def _spawn_formation(self, keep_existing: bool = False) -> None:
        """Wingmen start on the ground at their slots around your drone."""
        s = self.sim.estimate
        self.formation.spawn(self.config, self.seed, s.pos_m[:2], s.euler_rad()[2], self.env, keep_existing)

    def set_formation(self, count, shape="v", spacing_m=1.5, avoid=True) -> str:
        try:
            count, spacing_m = int(count), float(spacing_m)
        except (TypeError, ValueError):
            raise ValueError("Formation needs numbers: count, spacing_m.") from None
        old = self.formation.count
        self.formation.configure(count, str(shape), spacing_m, bool(avoid))
        if count != old:
            self._spawn_formation(keep_existing=True)  # new wingmen appear on the ground below their slots
        if count == 0:
            return "Formation off: you fly alone."
        avoid_txt = "they avoid crashing into each other and into obstacles" if avoid else "NO collision avoidance: watch out"
        return (f"Formation: {count} wingm{'a' if count == 1 else 'e'}n in a {SHAPE_NAMES[shape]}, {spacing_m:.1f} m apart; "
                f"{avoid_txt}. They take off, fly and land with you.")

    def load_design(self, parts: dict, drain_speedup: float = 1.0) -> tuple[DroneConfig, str]:
        """Fly a Drone Builder design: its config, on its own battery. Returns (config, message)."""
        from dataclasses import replace

        from backend.sim.builder import DesignError, design

        try:
            d = design(parts)
        except DesignError as exc:
            raise ValueError(str(exc)) from None
        bat = replace(d["battery"], drain_speedup=float(drain_speedup))
        self.reset(d["config"])
        self.set_battery(True, cfg=bat)
        r = d["report"]
        if r["verdict"] == "won't fly":
            msg = f"{d['config'].name}: T/W {r['thrust_to_weight']:.2f}. Try Take off: it can't lift itself."
        else:
            msg = (f"{d['config'].name}: {r['mass_g']} g, T/W {r['thrust_to_weight']:.1f}, about "
                   f"{r['flight_time_min']:.0f} min of hover. Battery full: take off!")
        return d["config"], msg

    def set_wind(self, speed_m_s, direction_deg, gust_m_s) -> str:
        msg = super().set_wind(speed_m_s, direction_deg, gust_m_s)
        self.formation.set_env(self.env)  # the wind blows on everyone
        return msg

    def load_course(self, scenario_id: str | None) -> str:
        """Load a built-in course (or None for an empty world) and put the drone on its start point."""
        if scenario_id is not None and scenario_id not in SCENARIOS and scenario_id != FARM_ID:
            raise ValueError(f"Unknown course '{scenario_id}'. Choose one of: {', '.join([*SCENARIOS, FARM_ID])}.")
        self.course_id = scenario_id
        self.plan = None
        if scenario_id == FARM_ID:
            self.mission = Mission([(0.0, 0.0)], [b.box for b in FARM.buildings], 20.0, 3.0)
        else:
            self.mission = scenario_mission(scenario_id) if scenario_id else None
        self.race = None  # a course replaces a race
        self.reset()
        if not scenario_id:
            return "Free flight: no obstacles."
        if scenario_id == FARM_ID:
            return (f"🌾 {FARM.title}: 3 fields, buildings, 2 no-fly zones (red) and 2 landing pads. Plan a flight in the "
                    "🧭 Mission planner, or fly it yourself.")
        return f"Loaded '{SCENARIOS[scenario_id]['title']}'. Fly it yourself, or press Fly mission."

    def load_race(self, track_id: str | None) -> str:
        """Load a rings-race track (or None to stop racing) and put the drone on its start pad."""
        if track_id is not None and track_id not in TRACKS:
            raise ValueError(f"Unknown track '{track_id}'. Choose one of: {', '.join(TRACKS)}.")
        self.race = Race(track_id) if track_id else None
        if track_id:
            self.course_id, self.mission = None, None  # a race replaces a course
        self.reset()
        if not track_id:
            return "Race off."
        best = f" Best time so far: {self.race.best_s:.1f} s." if self.race.best_s is not None else ""
        return f"Race: {self.race.track['title']}. {self.race.track['tip']} The clock starts at take-off.{best}"

    def race_info(self) -> dict | None:
        return self.race.info() if self.race else None

    def _update_race(self, s: State) -> None:
        """Check the rings once per physics step, on the TRUE position (passing a ring is a physical fact)."""
        pos = s.pos_m.copy()
        if self._race_prev_pos is not None:
            for msg in self.race.update(self._race_prev_pos, pos, s.time_s, self.pilot.sp.armed):
                self.notices.append(msg)
                self.pilot.message = msg
        self._race_prev_pos = pos

    def course_info(self) -> dict | None:
        """Everything the 3D view needs to draw the course (sent once per change, not every frame)."""
        if not self.mission:
            return None
        m = self.mission
        if self.course_id == FARM_ID:
            p = self.plan
            return {
                "id": FARM_ID, "title": FARM.title,
                "waypoints": [list(w) for w in (p.waypoints if p else m.waypoints)],
                "obstacles": [b.to_dict() for b in m.obstacles],
                "cruise_alt_m": p.spec["altitude_m"] if p else m.cruise_alt_m,
                "safety_zone_m": DRONE_RADIUS_M + SAFETY_MARGIN_M,
                "route": [[round(a, 3), round(b, 3)] for a, b in p.route] if p else None,
                "planner": "plan" if p else None,
                "site": FARM.to_dict(),
                "plan": p.to_dict() if p else None,
                "footprint_per_m": [CAMERA.footprint_m(1.0)[0], CAMERA.footprint_m(1.0)[1]],
            }
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

    # ---- 🌾 farm: flight plans, precision landing, down camera ----

    def start_plan(self, spec: dict) -> str:
        """Fly a task plan on the farm (from the 🧭 Mission planner). Re-checked here: a plan with a FAIL is refused."""
        if self.course_id != FARM_ID:
            self.load_course(FARM_ID)
        try:
            plan = compile_plan(spec, FARM, self.config, self.battery_cfg)
        except PlanError as exc:
            raise ValueError(str(exc)) from None
        fails = [f"{c['label']}: {c['detail']}" for c in plan.checks if c["status"] == "fail"]
        if fails:
            raise ValueError("Not flying this plan. " + " ".join(fails))
        self.reset()
        self.plan = plan
        self.mission = Mission(list(plan.waypoints), list(plan.buildings), plan.spec["altitude_m"], plan.spec["speed_m_s"])
        self.route, self.planner = list(plan.route), "plan"
        self.follower = PlanFollower(plan, make_lander(plan, FARM, self.seed))
        self._events_seen = 0
        self.pilot.mode = "mission"
        est = plan.estimate
        self.pilot.message = f"Flying the plan: {len(plan.waypoints) - 1} waypoints, about {est['time_s'] / 60:.1f} min."
        return (f"Plan started: {est['distance_m']:.0f} m, about {est['time_s'] / 60:.1f} min, ~{est['battery_used_pct']:.0f} % "
                f"of the battery" + (f", {est['photos']} photos" if est["photos"] else "") + "."
                + (" Try ⏩ to speed up time." if self.time_scale == 1 else ""))

    def _make_cropmaps(self, plan: CompiledPlan, photos: list[dict]) -> None:
        """Stitch each surveyed field's photos into a crop-health map, in a background thread (takes ~0.1-1 s),
        so the live simulation never stutters."""
        import threading

        from .cropmap import build_map, remember

        self.cropmap_busy = True
        self.notices.append("🌱 Stitching the survey photos into a crop-health map...")

        def work() -> None:
            maps = []
            try:
                for sv in plan.surveys:
                    shots = [p for p in photos if p["line"] is not None and plan.labels[p["line"]].startswith(sv["field"] + " ")]
                    if shots:
                        m = build_map(sv["field"], sv["polygon"], shots, seed=self.seed)
                        m["id"] = remember(m)
                        maps.append(m)
                found = sum(len(m["findings"]) for m in maps)
                self.notices.append(f"🌱 Crop-health map ready: {found} problem patch(es) found. Open 🗺 Field map.")
            except Exception as exc:  # noqa: BLE001  (report, don't kill the sim)
                self.notices.append(f"Couldn't make the crop-health map: {exc}")
            self.cropmaps = maps
            self._cropmaps_version += 1
            self.cropmap_busy = False

        threading.Thread(target=work, name="cropmap", daemon=True).start()

    def _pads(self) -> list[tuple[float, float, float]]:
        return [(p.x_m, p.y_m, p.size_m) for p in FARM.pads] if self.course_id == FARM_ID else []

    def start_precland(self, mode: str = "vision") -> str:
        """🎯 Land on the nearest pad (farm only), with the camera or by GPS."""
        if mode not in ("vision", "gps"):
            raise ValueError("Precision landing mode is 'vision' or 'gps'.")
        if self.course_id != FARM_ID:
            raise ValueError("Load the 🌾 farm first: it has landing pads with markers.")
        s, est = self.state, self.sim.estimate
        if s.on_ground or not self.pilot.sp.armed:
            raise ValueError("Take off first.")
        self._take_over()
        pad = min(FARM.pads, key=lambda p: math.hypot(p.x_m - est.pos_m[0], p.y_m - est.pos_m[1]))
        d = math.hypot(pad.x_m - est.pos_m[0], pad.y_m - est.pos_m[1])
        if d > PRECLAND_MAX_DIST_M:
            raise ValueError(f"The nearest pad ({pad.name}) is {d:.0f} m away: fly within {PRECLAND_MAX_DIST_M:.0f} m of it first.")
        self.prec_lander = PrecisionLander((pad.x_m, pad.y_m, pad.size_m), mode, self.seed, self._pads())
        self.pilot.mode = "precland"
        how = "looking for its marker with the down camera" if mode == "vision" else "on its GPS position only"
        self.pilot.message = f"🎯 Landing on {pad.name}, {how}."
        return self.pilot.message

    def set_camera(self, enabled: bool) -> str:
        self.camera_on = bool(enabled)
        self._camera_img = None
        return ("📷 Down camera on: what a camera under the drone sees (128 × 96 pixels)." if enabled else "Down camera off.")

    def _no_fly_check(self, s: State) -> None:
        if self.course_id != FARM_ID:
            return
        x, y = float(s.pos_m[0]), float(s.pos_m[1])
        inside = next((z.name for z in FARM.no_fly if z.box.distance_2d(x, y) == 0.0 and not s.on_ground), None)
        if inside != self._in_no_fly:
            if inside:
                msg = f"⚠ You flew into the no-fly zone '{inside}'. Get out!"
                self.notices.append(msg)
                self.pilot.message = msg
            self._in_no_fly = inside

    def _precland_setpoint(self, s: State) -> Setpoint:
        lander = self.prec_lander
        n = len(lander.events)
        x, y, alt = lander.update(s.time_s, self.sim.estimate, s)
        for _, _, msg in lander.events[n:]:
            self.notices.append(msg)
            self.pilot.message = msg
        sp = Setpoint(armed=True, alt_m=alt, yaw_rad=self.pilot.sp.yaw_rad, position_control=True, x_m=x, y_m=y)
        if s.on_ground and alt < 0.2:
            err = math.hypot(s.pos_m[0] - lander.pad[0], s.pos_m[1] - lander.pad[1])
            msg = f"🎯 Landed {err:.2f} m from the pad centre ({'camera' if lander.mode == 'vision' else 'GPS only'})."
            self.notices.append(msg)
            self.pilot = Pilot()
            self.pilot.message = msg
            self.prec_lander = None
            sp.armed = False
        return sp

    # ---- mission autopilot ----

    def start_mission(self, planner: str, cruise_speed_m_s: float | None = None) -> str:
        if not self.mission:
            raise ValueError("Load a course first.")
        if self.course_id == FARM_ID:
            raise ValueError("On the farm, plan the flight in the 🧭 Mission planner.")
        if planner not in PLANNERS:
            raise ValueError(f"Planner must be one of: {', '.join(PLANNERS)}.")
        speed = self.mission.cruise_speed_m_s if cruise_speed_m_s is None else float(cruise_speed_m_s)
        if not MIN_SPEED_M_S <= speed <= MAX_SPEED_M_S:
            raise ValueError(f"Cruise speed must be between {MIN_SPEED_M_S} and {MAX_SPEED_M_S} m/s.")
        self.mission.cruise_speed_m_s = speed
        try:
            route, _ = plan_route(self.mission.waypoints, self.mission.obstacles,
                                  "straight" if planner == "lidar" else planner, self.mission.cruise_alt_m)
        except (PlanningError, MissionError) as exc:
            raise ValueError(str(exc)) from None
        self.reset()  # back to the start point, clock at 0 (the follower times its phases from t = 0)
        self.route, self.planner = route, planner
        self.follower = SenseFollower(self.mission, self.seed) if planner == "lidar" else MissionFollower(self.mission, route, planner)
        self._events_seen = 0
        self.pilot.mode = "mission"
        wait = self.gps_settle_left_s()
        self.pilot.message = f"Mission ({planner}): " + (f"waiting {wait:.0f} s for GPS to settle." if wait > 0 else "taking off.")
        gps = f" Waiting {wait:.0f} s for GPS to settle first." if wait > 0 else ""
        return f"Mission started with the {planner} planner at {speed:.1f} m/s.{gps}"

    def _end_mission(self, state: State) -> None:
        """The autopilot is finished: give the controls back."""
        phase = self.follower.phase
        self.follower_done = self.follower
        self.follower = None
        self.pilot = Pilot()
        if phase == "stuck":
            self.pilot.mode = "hover"  # Pilot.hover() needs a flying mode to start from
            self.pilot.hover(self.sim.estimate)
            self.pilot.message = "Stuck: the potential field can't find a way. You have the controls."
        elif phase == "crashed":
            self.pilot.message = "Crashed into an obstacle. Press Reset or Fly mission to try again."
        elif isinstance(self.follower_done, PlanFollower) and self.follower_done.plan.land:
            land = self.follower_done.plan.land
            err = math.hypot(state.pos_m[0] - land["x_m"], state.pos_m[1] - land["y_m"])
            cam = self.follower_done.camera
            photos = f" {len(cam.photos)} photos taken." if cam else ""
            self.pilot.message = f"Plan complete. Landed on {land['place']}, {err:.2f} m from its centre.{photos}"
            self.notices.append(self.pilot.message)
            if cam and cam.photos and self.follower_done.plan.surveys:
                self._make_cropmaps(self.follower_done.plan, list(cam.photos))
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
        sp = self._leader_setpoint(dt_s)
        if self.formation.count:
            self._step_formation(sp, dt_s)
        return sp

    def _step_formation(self, sp: Setpoint, dt_s: float) -> None:
        """Tell the wingmen what the leader is doing, step them, and check drone-to-drone collisions."""
        s = self.state
        landing = self.pilot.mode == "land" or (self.follower is not None and self.follower.phase == "land")
        phase = "off" if not sp.armed else "landing" if landing else "flying"
        est = self.sim.estimate
        leader = LeaderInfo(est.pos_m.copy(), est.vel_m_s.copy(), sp.yaw_rad, sp.alt_m, phase,
                            self.follower is not None, s.pos_m, s.on_ground)
        obstacles = self.mission.obstacles if self.mission else []
        self.notices += self.formation.step(leader, obstacles, dt_s)
        said, leader_hit = self.formation.check_collisions(s.pos_m, sp.armed and not s.on_ground)
        self.notices += said
        if said:
            self.pilot.message = said[-1]
        if leader_hit:
            self.follower = None
            self.pilot = Pilot()  # motors off: you're idle until Reset
            self.pilot.message = said[-1]
            sp.armed = False

    def abort_autopilot(self) -> None:
        if self.prec_lander is not None:
            self.prec_lander = None
            self.pilot.mode = "hover"
            self.notices.append("🎯 Precision landing aborted by the battery failsafe.")
        if self.follower:
            self._take_over()
            self.notices.append("Mission aborted by the battery failsafe.")

    def set_lidar(self, enabled: bool) -> str:
        self.lidar_on = bool(enabled)
        self.lidar_view = Perception(self.seed) if self.lidar_on else None
        if not enabled:
            return "Lidar off."
        return ("📡 Lidar on: 72 laser beams sweep a full circle at the drone's height, 10 times a second (8 m range). "
                "Red dots = its map of what it has hit. Load a course to see it map the obstacles.")

    def perception(self) -> Perception | None:
        if isinstance(self.follower, SenseFollower):
            return self.follower.perception
        return self.lidar_view

    def _leader_setpoint(self, dt_s: float) -> Setpoint:
        s = self.state
        self.check_battery()
        if self.lidar_view is not None and not isinstance(self.follower, SenseFollower):
            self.lidar_view.update(s.time_s, s, self.sim.estimate, self.mission.obstacles if self.mission else [])
        if self.race:
            self._update_race(s)
        self._no_fly_check(s)
        if self.camera_on and s.time_s >= self._camera_next_s:
            self._camera_next_s = s.time_s + CAMERA_SEND_EVERY_S
            self._camera_img = render(s, self._pads(), None)
        if self.prec_lander is not None:
            return self._precland_setpoint(s)
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
        self.pilot.attitude_mode = self.failsafe_active()
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
        if cmd == "race":
            return self.load_race(args.get("track") or None)
        if cmd == "formation":
            return self.set_formation(args.get("count", 0), args.get("shape", "v"), args.get("spacing_m", 1.5), args.get("avoid", True))
        if cmd == "mission":
            return self.start_mission(str(args.get("planner", "astar")), args.get("cruise_speed_m_s"))
        if cmd == "lidar":
            return self.set_lidar(bool(args.get("enabled", False)))
        if cmd == "plan":
            return self.start_plan(args.get("spec") or {})
        if cmd == "precland":
            return self.start_precland(str(args.get("mode", "vision")))
        if cmd == "camera":
            return self.set_camera(bool(args.get("enabled", False)))
        if self.prec_lander is not None and cmd in ("hover", "land", "target", "takeoff"):
            self.prec_lander = None
            if cmd != "takeoff":
                self.pilot.mode = "hover"
            self.notices.append("🎯 Precision landing cancelled: you have the controls.")
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
        snap["race"] = self.race.status(self.state.time_s) if self.race else None
        f_cam = getattr(self.follower, "camera", None)
        if f_cam is not None:
            survey = {"photos_taken": len(f_cam.photos), "on_line": f_cam.line is not None}
            if len(f_cam.photos) != self._photos_sent:  # the footprints only when a new photo was taken
                survey["photos"] = [[p["x_m"], p["y_m"], p["alt_m"], round(p["angle_rad"], 4)] for p in f_cam.photos]
                self._photos_sent = len(f_cam.photos)
            snap["survey"] = survey
        else:
            snap["survey"] = None
        lander = self.prec_lander or getattr(self.follower, "lander", None)
        if lander is not None and (self.prec_lander is not None or self.follower.phase == "land"):
            cam = lander.snapshot()
            if self.state.time_s - self._camera_sent_s < CAMERA_SEND_EVERY_S:
                cam["image"] = None  # the UI keeps the last picture
            else:
                self._camera_sent_s = self.state.time_s
            snap["camera"] = {**cam, "landing": True}
        elif self.camera_on and self._camera_img is not None:
            fresh = self.state.time_s - self._camera_sent_s >= CAMERA_SEND_EVERY_S
            if fresh:
                self._camera_sent_s = self.state.time_s
            snap["camera"] = {"landing": False, "image": image_b64(self._camera_img) if fresh else None,
                              "w": self._camera_img.shape[1], "h": self._camera_img.shape[0], "detection": None}
        else:
            snap["camera"] = None
        snap["no_fly"] = self._in_no_fly
        snap["cropmap_busy"] = self.cropmap_busy
        if self._cropmaps_version != self._cropmaps_sent:  # the maps go out once, when new
            snap["cropmaps"] = self.cropmaps
            self._cropmaps_sent = self._cropmaps_version
        snap["formation"] = self.formation.snapshot() if self.formation.count else None
        p = self.perception()
        if p is not None:
            key = (id(p.map), p.map.version)
            snap["lidar"] = p.snapshot(with_map=key != self._lidar_sent)  # the map only when it changed
            if snap["lidar"] is not None:
                snap["lidar"]["replans"] = getattr(self.follower, "replans", None)
                self._lidar_sent = key
        else:
            snap["lidar"] = None
        return snap
