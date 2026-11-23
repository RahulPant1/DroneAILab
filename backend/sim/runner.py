"""The simulation loop: physics at 500 Hz, controller at 250 Hz, telemetry at 50 Hz.

`LightweightSim` is our only simulator. It follows the small `SimBackend` interface, so a
different simulator (e.g. PX4) could be plugged in later without changing the callers.

Two ways to run it:
  headless  (`run_headless`) : as fast as the CPU allows, for experiments and tests;
  real-time (`RealtimeSim`)  : paced to the wall clock, driven by the WebSocket for the 3D view.
"""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, field, replace
from typing import Callable, Protocol

import numpy as np

from .battery import LOW_CELL_V, Battery, BatteryConfig, validate_battery
from .config import ConfigError, DroneConfig, require_valid
from .control import CascadedController, Setpoint
from .environment import EnvConfig, Wind, validate_env
from .ekf import GPS_RETRY_S
from .estimator import make_estimator
from .metrics import altitude_step_metrics, horizontal_drift_m, mean_motor_pct
from .physics import QuadcopterPhysics, State
from .pilot import FAILSAFE_TILT_RAD, Pilot, Sticks
from .sensors import SensorConfig, Sensors, validate_sensors

PHYSICS_DT_S = 0.002  # 500 Hz
CONTROL_EVERY_N_STEPS = 2  # 250 Hz
TELEMETRY_EVERY_N_STEPS = 10  # 50 Hz
MAX_DURATION_S = 600.0  # long enough for a field survey
SATURATION_PCT = 99.9

TELEMETRY_CHANNELS = (
    "t_s",
    "x_m", "y_m", "z_m",
    "vx_m_s", "vy_m_s", "vz_m_s",
    "roll_rad", "pitch_rad", "yaw_rad",
    "motor1_pct", "motor2_pct", "motor3_pct", "motor4_pct",
    "thrust_total_n",
    "alt_setpoint_m",
    "x_setpoint_m", "y_setpoint_m",
    "roll_setpoint_rad", "pitch_setpoint_rad", "yaw_setpoint_rad",
)  # fmt: skip
WIND_CHANNELS = ("wind_x_m_s", "wind_y_m_s", "wind_z_m_s")
ESTIMATE_CHANNELS = (
    "x_est_m", "y_est_m", "z_est_m", "vz_est_m_s",
    "roll_est_rad", "pitch_est_rad", "yaw_est_rad",
    "baro_m", "gps_x_m", "gps_y_m",
)  # fmt: skip
TELEMETRY_DECIMALS = 5


class SimBackend(Protocol):
    """What any simulator must offer: start over, advance one step, report the state."""

    def reset(self, config: DroneConfig, seed: int, initial_state: State | None = None) -> State: ...
    def step(self, command: Setpoint) -> State: ...
    def get_state(self) -> State: ...


class LightweightSim:
    """Our own simple simulator: physics + wind + sensors + estimator + controller, fixed time steps."""

    def __init__(self) -> None:
        self.config: DroneConfig | None = None
        self.ground_align = False  # live view: GPS settle on the ground before take-off (Lab runs: off)

    def reset(
        self,
        config: DroneConfig,
        seed: int = 0,
        initial_state: State | None = None,
        env: EnvConfig | None = None,
        sensors: SensorConfig | None = None,
        battery: BatteryConfig | None = None,
    ) -> State:
        self.config = require_valid(config)
        self.env = env or EnvConfig()
        self.sensor_cfg = sensors or SensorConfig()
        problems = validate_env(self.env) + validate_sensors(self.sensor_cfg)
        if problems:
            raise ConfigError(" ".join(problems))
        self.seed = seed
        # Separate random streams for wind and sensor noise, both derived from the one seed.
        wind_seed, sensor_seed = np.random.SeedSequence(seed).spawn(2)

        # The physics may carry a payload the controller doesn't know about (for the I-term lesson).
        physics_cfg = config
        if self.env.unmodeled_payload_kg > 0:
            physics_cfg = config.with_changes(mass_kg=config.mass_kg + self.env.unmodeled_payload_kg)
        self.physics = QuadcopterPhysics(physics_cfg)
        self.controller = CascadedController(config)
        self.state = initial_state.copy() if initial_state else State.at(0.0)
        self.wind = Wind(self.env, np.random.default_rng(wind_seed), PHYSICS_DT_S) if self.env.active else None
        if self.sensor_cfg.enabled:
            self.sensors = Sensors(self.sensor_cfg, np.random.default_rng(sensor_seed))
            self.estimator = make_estimator(self.sensor_cfg, self.state, PHYSICS_DT_S)
        else:
            self.sensors = None
            self.estimator = None
        self.step_count = 0
        self.motor_pct = np.zeros(4)
        self.rotor_energy_j = 0.0
        self.battery = None
        self.set_battery(battery)
        return self.state

    def set_battery(self, cfg: BatteryConfig | None, soc: float = 1.0) -> None:
        """Fly on a battery (None = unlimited power at the motors' rated voltage, the default)."""
        if cfg is not None:
            problems = validate_battery(cfg)
            if problems:
                raise ConfigError(" ".join(problems))
        self.battery = Battery(cfg, soc) if cfg else None
        self.physics.voltage_scale = self.battery.voltage_scale if self.battery else 1.0

    def set_env(self, env: EnvConfig) -> None:
        """Change the wind without restarting (live view). Gusts restart from calm, with fresh randomness."""
        problems = validate_env(env)
        if problems:
            raise ConfigError(" ".join(problems))
        self.env = env
        rng = np.random.default_rng([self.seed, self.step_count])
        self.wind = Wind(env, rng, PHYSICS_DT_S) if env.active else None

    def set_sensors(self, cfg: SensorConfig) -> None:
        """Switch sensor noise on/off without restarting (live view). A newly switched-on estimator starts
        out knowing the true state, as if just calibrated, and then has to keep up using noisy sensors."""
        problems = validate_sensors(cfg)
        if problems:
            raise ConfigError(" ".join(problems))
        self.sensor_cfg = cfg
        if cfg.enabled:
            self.sensors = Sensors(cfg, np.random.default_rng([self.seed, self.step_count, 1]))
            self.estimator = make_estimator(cfg, self.state, PHYSICS_DT_S)
            # Switched on in the air: it can't stand still to average GPS, so skip the settle.
            self.estimator.settled = not (self.ground_align and self.state.on_ground)
        else:
            self.sensors = None
            self.estimator = None

    @property
    def estimate(self) -> State:
        """What the controller believes. With sensors off ('perfect sensors') this IS the truth."""
        return self.estimator.state if self.estimator else self.state

    def step(self, command: Setpoint) -> State:
        """Advance one physics step (2 ms). The controller runs on every 2nd step."""
        if self.estimator and self.ground_align:
            self.estimator.set_stationary(not command.armed and self.state.on_ground)
        if self.step_count % CONTROL_EVERY_N_STEPS == 0:
            self.motor_pct = self.controller.update(self.estimate, command, PHYSICS_DT_S * CONTROL_EVERY_N_STEPS)
        wind = self.wind.step(self.state.time_s) if self.wind else None
        self.physics.step(self.state, self.motor_pct, PHYSICS_DT_S, wind)
        if self.sensors:
            readings = self.sensors.sample(self.state, self.physics.last_accel_m_s2)
            self.estimator.update(readings, self.state.time_s)
        self.rotor_energy_j += self.physics.last_rotor_power_w * PHYSICS_DT_S
        if self.battery:
            self.battery.step(self.physics.last_rotor_power_w, PHYSICS_DT_S)
            self.physics.voltage_scale = self.battery.voltage_scale
        self.step_count += 1
        return self.state

    def get_state(self) -> State:
        return self.state


BATT_FAILSAFE_AFTER_S = 3.0  # battery below a limit this long (sim time) -> failsafe
GPS_GLITCH_M = 5.0  # live 'GPS glitch' toggle: how far the fixes jump
GPS_FAILSAFE_AFTER_S = 1.0  # GPS lost this long -> stop trusting the position estimate
# Extra air drag in the failsafe (N per m/s; the drone's own is 0.10). Not real physics: a helper so the drone
# doesn't coast so far when you steer it without GPS. Speed now halves in about 1.4 s instead of ~7 s.
FAILSAFE_EXTRA_DRAG_N_S_M = 0.4  # default; the live view has a slider (0 = real physics)
MAX_FAILSAFE_DRAG_N_S_M = 1.0
MIN_FAILSAFE_TILT_DEG, MAX_FAILSAFE_TILT_DEG = 2.0, 20.0  # the live view's tilt slider range
MAX_CATCH_UP_S = 0.1  # if the server stalls, skip ahead rather than racing to catch up
TIME_SCALES = (1.0, 2.0, 4.0, 8.0)  # ⏩ live-view speed-up choices


class RealtimeSim:
    """Runs the simulator in step with the real clock, controlled by a Pilot.

    The WebSocket loop calls `advance(elapsed_wall_time)` about 30 times a second;
    we then do however many 2 ms physics steps fit into that time.
    """

    def __init__(self, config: DroneConfig, seed: int = 0):
        self.sim = LightweightSim()
        self.sim.ground_align = True  # with sensor noise on: GPS must settle before take-off
        self.env = EnvConfig()  # wind; it stays the same across resets until changed
        self.sensor_cfg = SensorConfig()  # off = perfect sensors; also kept across resets
        self.gps_outage = False  # no GPS fixes (only matters with sensor noise on); kept across resets
        self.gps_glitch = False  # GPS positions jump GLITCH_M east (a bad fix); kept across resets
        self.gps_failsafe = True  # on GPS loss: hold level + altitude instead of trusting a drifting position
        self.failsafe_drag_n_s_m = FAILSAFE_EXTRA_DRAG_N_S_M  # extra air drag while the failsafe is active
        self.failsafe_tilt_rad = FAILSAFE_TILT_RAD  # how far a full W/A/S/D press tilts the drone in failsafe
        self._gps_lost_at_s = 0.0
        self._failsafe_was_on = False
        self.battery_cfg: BatteryConfig | None = None  # None = unlimited power (default); kept across resets
        self.time_scale = 1.0  # ⏩ 1 = real time; 2, 4, 8 = faster (handy for long surveys); kept across resets
        self.reset(config, seed)

    def reset(self, config: DroneConfig | None = None, seed: int | None = None) -> None:
        self.config = config or self.config
        self.seed = self.seed if seed is None else seed
        self.sim.reset(self.config, self.seed, self.start_state(), env=self.env, sensors=self.sensor_cfg,
                       battery=self.battery_cfg)
        self.home_xy = (float(self.sim.state.pos_m[0]), float(self.sim.state.pos_m[1]))  # where "fly home" goes
        self._batt_fs: str | None = None  # battery failsafe: None | "rtl" (flying home) | "land"
        self._batt_bad_since: float | None = None
        self.pilot = Pilot()
        self.paused = False
        self._time_debt_s = 0.0
        self.notices: list[str] = []  # one-off messages for the UI's log (the WebSocket loop sends them)
        self._gps_lost_at_s = 0.0
        self._failsafe_was_on = False
        self._apply_gps()

    def _apply_gps(self) -> None:
        if self.sim.sensors:
            self.sim.sensors.gps_ok = not self.gps_outage
            self.sim.sensors.gps_glitch_m = np.array([GPS_GLITCH_M, 0.0]) if self.gps_glitch else np.zeros(2)

    def start_state(self) -> State | None:
        """Where the drone sits after a reset (None = on the ground at the origin)."""
        return None

    def next_setpoint(self, dt_s: float) -> Setpoint:
        """What the controller should aim for during the next physics step (the pilot decides)."""
        self.check_battery()
        self.pilot.attitude_mode = self.failsafe_active()
        return self.apply_failsafe(self.pilot.update(self.sim.estimate, dt_s, self.state))  # decides on the estimate

    def gps_lost_s(self) -> float:
        """How long the GPS has been out (0 = fine, or perfect sensors where GPS isn't used)."""
        if not (self.gps_outage and self.sim.estimator):
            return 0.0
        return max(0.0, self.state.time_s - self._gps_lost_at_s)

    def failsafe_active(self) -> bool:
        return self.gps_failsafe and self.gps_lost_s() >= GPS_FAILSAFE_AFTER_S and not self.state.on_ground

    def apply_failsafe(self, sp: Setpoint) -> Setpoint:
        """GPS-loss failsafe ('altitude mode'): without GPS the position estimate drifts away fast, so stop
        chasing it. Hold the wings level and keep the height (the altimeter still works); the drone then only
        drifts with its leftover speed, small tilt errors and the wind. W/A/S/D now tilt the drone directly (like
        a manual pilot). When GPS returns, position hold resumes at the last target."""
        on = self.failsafe_active() and sp.armed
        if on != self._failsafe_was_on:
            self._failsafe_was_on = on
            self.notices.append(
                "GPS-loss failsafe: holding level and altitude; position is NOT held. W/A/S/D now tilt the drone."
                if on else "Failsafe over: back to holding position."
            )
        self.sim.physics.extra_drag_n_s_m = self.failsafe_drag_n_s_m if on else 0.0
        if not on:
            return sp
        roll, pitch = self.pilot.tilt_from_sticks(self.failsafe_tilt_rad)
        return replace(sp, position_control=False, roll_rad=roll, pitch_rad=pitch)

    # ---- battery ----

    def set_battery(self, enabled: bool, cells=3, capacity_mah=3000.0, drain_speedup=1.0, cfg: BatteryConfig | None = None) -> str:
        """Fly on a battery (or not). Takes effect at once with a full battery; kept across resets."""
        if enabled:
            if cfg is None:
                try:
                    base = self.battery_cfg or BatteryConfig()
                    cfg = replace(base, cells=int(cells), capacity_mah=float(capacity_mah), drain_speedup=float(drain_speedup))
                except (TypeError, ValueError):
                    raise ValueError("Battery needs numbers: cells, capacity_mah, drain_speedup.") from None
            self.sim.set_battery(cfg)  # raises ConfigError (a ValueError) if out of range
            self.battery_cfg = cfg
        else:
            self.sim.set_battery(None)
            self.battery_cfg = None
        self._batt_fs, self._batt_bad_since = None, None
        if not enabled:
            return "Battery off: unlimited power."
        speed = f", draining {cfg.drain_speedup:g}× faster than real" if cfg.drain_speedup > 1 else ""
        return (f"Battery on: {cfg.cells}S {cfg.capacity_mah:.0f} mAh, full{speed}. Watch the voltage sag as you climb; "
                f"below {LOW_CELL_V} V per cell it flies home and lands.")

    def abort_autopilot(self) -> None:
        """A failsafe takes over: stop any autopilot (the live missions override this)."""

    def check_battery(self) -> None:
        """Battery failsafe, like ArduPilot's BATT_FS_LOW_ACT / CRT_ACT: after 3 s below the LOW voltage, fly home
        and land; below CRITICAL, land where it is. Voltage sags under load, so a hard climb can dip it briefly:
        the 3 s wait stops that from triggering it."""
        b = self.sim.battery
        s = self.state
        if b and b.status == "empty" and self._batt_fs != "empty":
            self.abort_autopilot()
            self._batt_fs = "empty"
            msg = "🔋 Battery EMPTY: the motors stopped." + (" It fell." if not s.on_ground else "")
            self.pilot = Pilot()  # motors off until Reset
            self.pilot.message = msg
            self.notices.append(msg)
            return
        if not b or not self.pilot.sp.armed or s.on_ground:
            self._batt_bad_since = None
            return
        status = b.status
        if status == "ok":
            self._batt_bad_since = None
            return
        if self._batt_bad_since is None:
            self._batt_bad_since = s.time_s
            return
        # The wait is in BATTERY time: with a faster-draining demo battery it's shorter, so it can still react.
        if s.time_s - self._batt_bad_since < BATT_FAILSAFE_AFTER_S / b.cfg.drain_speedup:
            return
        est = self.sim.estimate
        if status in ("critical", "empty") and self._batt_fs != "land":
            self.abort_autopilot()
            self._batt_fs = "land"
            msg = f"🔋 Battery CRITICAL ({b.cell_v:.2f} V per cell): landing right here."
            self.pilot.land(est)
            self.pilot.message = msg
            self.notices.append(msg)
        elif status == "low" and self._batt_fs is None:
            self.abort_autopilot()
            self._batt_fs = "rtl"
            msg = f"🔋 Battery low ({b.cell_v:.2f} V per cell): flying home and landing."
            if self.pilot.mode not in ("takeoff", "hover", "land"):
                self.pilot.mode = "hover"
            self.pilot.set_target(self.home_xy[0], self.home_xy[1], None, None)
            self.pilot.message = msg
            self.notices.append(msg)
        if self._batt_fs == "rtl" and math.hypot(est.pos_m[0] - self.home_xy[0], est.pos_m[1] - self.home_xy[1]) < 0.6:
            self._batt_fs = "land"
            self.pilot.land(est)
            self.pilot.message = "🔋 Home: landing."

    @property
    def state(self) -> State:
        return self.sim.get_state()

    def advance(self, wall_dt_s: float) -> int:
        """Simulate `wall_dt_s` seconds of real time (nothing happens while paused)."""
        if self.paused:
            return 0
        self._time_debt_s += min(max(wall_dt_s, 0.0), MAX_CATCH_UP_S) * self.time_scale
        n_steps = int(self._time_debt_s / PHYSICS_DT_S)
        self._time_debt_s -= n_steps * PHYSICS_DT_S
        for _ in range(n_steps):
            self.sim.step(self.next_setpoint(PHYSICS_DT_S))
        return n_steps

    def command(self, cmd: str, **args) -> str:
        """Handle a button press: takeoff | land | hover | pause | resume | reset | target | sticks."""
        s = self.sim.estimate  # buttons act on where the drone BELIEVES it is
        if cmd == "takeoff":
            wait = self.gps_settle_left_s()
            if wait > 0 and self.pilot.mode == "idle" and self.gps_outage:
                return self.pilot._say("No GPS fix (outage): take-off refused. The drone doesn't know where it is.")
            if wait > 0 and self.pilot.mode == "idle":
                return self.pilot._say(f"GPS settling: {wait:.1f} s left. Take-off is allowed once the position estimate is steady.")
            return self.pilot.takeoff(s, args.get("alt_m"))
        if cmd == "land":
            return self.pilot.land(s)
        if cmd == "hover":
            return self.pilot.hover(s)
        if cmd == "pause":
            self.paused = True
            self.pilot.set_sticks(Sticks())
            return "Paused."
        if cmd in ("resume", "start"):
            self.paused = False
            return "Running."
        if cmd == "target":
            return self.pilot.set_target(args.get("x_m"), args.get("y_m"), args.get("alt_m"), args.get("yaw_rad"))
        if cmd == "wind":
            return self.set_wind(args.get("speed_m_s", 0.0), args.get("direction_deg", 0.0), args.get("gust_m_s", 0.0))
        if cmd == "sensors":
            return self.set_sensors(bool(args.get("enabled", False)), args.get("noise_scale", 1.0), bool(args.get("gps_noise", True)),
                                    str(args.get("estimator", "complementary")))
        if cmd == "gps_glitch":
            return self.set_gps_glitch(bool(args.get("on", False)))
        if cmd == "gps":
            drag, tilt = args.get("drag_n_s_m"), args.get("tilt_deg")
            return self.set_gps(bool(args.get("outage", False)), bool(args.get("failsafe", True)),
                                None if drag is None else float(drag), None if tilt is None else float(tilt))
        if cmd == "sticks":
            self.pilot.set_sticks(Sticks(**{k: float(args.get(k, 0.0)) for k in ("forward", "left", "up", "yaw")}))
            return ""
        if cmd == "speed":
            factor = float(args.get("factor", 1.0))
            if factor not in TIME_SCALES:
                raise ValueError(f"Speed must be one of {', '.join(f'{v:g}' for v in TIME_SCALES)}.")
            self.time_scale = factor
            return "Real time." if factor == 1 else f"⏩ Time runs {factor:g}× faster than real."
        if cmd == "battery":
            return self.set_battery(bool(args.get("enabled", False)), args.get("cells", 3), args.get("capacity_mah", 3000.0),
                                    args.get("drain_speedup", 1.0))
        raise ValueError(f"Unknown command '{cmd}'.")

    def set_wind(self, speed_m_s, direction_deg, gust_m_s) -> str:
        """Steady wind (blowing TOWARD direction_deg: 0 = east, 90 = north) plus random gusts. Applies at once."""
        try:
            env = EnvConfig(wind_speed_m_s=float(speed_m_s), wind_direction_deg=float(direction_deg), gust_std_m_s=float(gust_m_s))
        except (TypeError, ValueError):
            raise ValueError("Wind needs numbers: speed_m_s, direction_deg, gust_m_s.") from None
        self.sim.set_env(env)  # raises ConfigError (a ValueError) if out of range
        self.env = env
        if not env.active:
            return "Wind off: calm air."
        return f"Wind {env.wind_speed_m_s:.1f} m/s toward {env.wind_direction_deg:.0f}°, gusts ±{env.gust_std_m_s:.1f} m/s."

    def set_gps(self, outage: bool, failsafe: bool = True, drag_n_s_m: float | None = None,
                tilt_deg: float | None = None) -> str:
        """GPS outage on/off (mid-flight is fine), whether the GPS-loss failsafe is armed, and (optionally) the
        failsafe's extra air drag in N per m/s (0 = real physics: the drone coasts on and on) and how far a full
        W/A/S/D press tilts the drone in failsafe, in degrees."""
        if drag_n_s_m is not None and not 0.0 <= drag_n_s_m <= MAX_FAILSAFE_DRAG_N_S_M:
            raise ValueError(f"Failsafe drag must be between 0 and {MAX_FAILSAFE_DRAG_N_S_M} N per m/s.")
        if tilt_deg is not None and not MIN_FAILSAFE_TILT_DEG <= tilt_deg <= MAX_FAILSAFE_TILT_DEG:
            raise ValueError(f"Failsafe tilt must be between {MIN_FAILSAFE_TILT_DEG:.0f} and {MAX_FAILSAFE_TILT_DEG:.0f} degrees.")
        said: list[str] = []
        if drag_n_s_m is not None and drag_n_s_m != self.failsafe_drag_n_s_m:
            self.failsafe_drag_n_s_m = drag_n_s_m
            how = ("none: real physics, it coasts until you counter-steer" if drag_n_s_m == 0
                   else f"speed halves every {0.693 * self.config.mass_kg / (self.config.drag_linear_n_s_m + drag_n_s_m):.1f} s when you let go")
            said.append(f"Failsafe drag {drag_n_s_m:.1f} N per m/s ({how}).")
        if tilt_deg is not None and abs(math.radians(tilt_deg) - self.failsafe_tilt_rad) > 1e-9:
            self.failsafe_tilt_rad = math.radians(tilt_deg)
            accel = 9.81 * math.tan(self.failsafe_tilt_rad)
            said.append(f"Failsafe tilt {tilt_deg:.0f}° (a full W/A/S/D press speeds it up by {accel:.1f} m/s every second).")
        if said and outage == self.gps_outage and failsafe == self.gps_failsafe:
            return " ".join(said)
        if outage and not self.gps_outage:
            self._gps_lost_at_s = self.state.time_s
        self.gps_outage, self.gps_failsafe = outage, failsafe
        self._apply_gps()
        if not self.sim.estimator:
            return "GPS setting saved. It only matters with sensor noise on (perfect sensors always know the position)."
        if not outage:
            return "GPS back: the estimate snaps back toward the true position, and the drone flies back to its target."
        fs = (f"Failsafe in {GPS_FAILSAFE_AFTER_S:.0f} s: hold level and altitude." if failsafe
              else "No failsafe: it keeps trusting its drifting position estimate.")
        return f"GPS outage: no fixes. Position now comes from the motion sensors only (dead reckoning). {fs}"

    def gps_settle_left_s(self) -> float:
        """Pre-arm check: seconds until the GPS position has settled (0 with perfect sensors)."""
        return self.sim.estimator.settle_left_s if self.sim.estimator else 0.0

    def set_sensors(self, enabled: bool, noise_scale=1.0, gps_noise: bool = True, estimator: str = "complementary") -> str:
        """Noisy sensors + estimator (the controller flies on its ESTIMATE), or perfect sensors (it sees the truth).
        The buttons' autopilot and collision checks always use the true position."""
        try:
            cfg = SensorConfig(enabled=enabled, noise_scale=float(noise_scale), gps_noise_scale=1.0 if gps_noise else 0.0,
                               estimator=estimator)
        except (TypeError, ValueError):
            raise ValueError("noise_scale must be a number.") from None
        self.sim.set_sensors(cfg)  # raises ConfigError (a ValueError) if out of range
        self.sensor_cfg = cfg
        self._gps_lost_at_s = self.state.time_s
        self._apply_gps()
        if not enabled:
            return "Perfect sensors: the controller sees the true state."
        wait = self.gps_settle_left_s()
        settle = f" GPS settling for {wait:.0f} s before take-off." if wait > 0 else ""
        gps = "" if gps_noise else " GPS is perfect (no GPS noise)."
        how = " Estimator: Kalman filter (EKF)." if estimator == "ekf" else ""
        return f"Sensor noise on ({cfg.noise_scale:g}× typical): the controller now flies on its estimate.{how}{gps}{settle}"

    def set_gps_glitch(self, on: bool) -> str:
        """Make every GPS fix jump GPS_GLITCH_M east (or stop). A Kalman filter's innovation gate should reject it."""
        self.gps_glitch = on
        self._apply_gps()
        if not on:
            return "GPS glitch over: fixes are right again."
        if not self.sim.estimator:
            return f"GPS glitch on ({GPS_GLITCH_M:.0f} m east), but sensor noise is off: the controller sees the truth."
        if self.sim.estimator.kind == "ekf":
            return (f"GPS glitch: fixes jump {GPS_GLITCH_M:.0f} m east. The EKF's innovation gate should reject them "
                    f"(after {GPS_RETRY_S:.0f} s it gives in and resets to the GPS).")
        return f"GPS glitch: fixes jump {GPS_GLITCH_M:.0f} m east. The complementary filter follows them, and so does the drone."

    def snapshot(self) -> dict:
        """A compact, JSON-friendly picture of 'right now' for the UI (SI units, radians)."""
        s = self.state
        sp = self.pilot.sp
        r = lambda v, n=4: [round(float(x), n) for x in v]  # noqa: E731
        return {
            "type": "state",
            "t_s": round(s.time_s, 3),
            "mode": self.pilot.mode,
            "paused": self.paused,
            "time_scale": self.time_scale,
            "armed": sp.armed,
            "on_ground": s.on_ground,
            "pos_m": r(s.pos_m),
            "vel_m_s": r(s.vel_m_s),
            "quat": r(s.quat, 5),
            "euler_rad": r(s.euler_rad()),
            "rate_rad_s": r(s.rate_rad_s),
            "motor_pct": r(self.sim.motor_pct, 1),
            "thrust_total_n": round(float(self.sim.physics.last_thrusts_n.sum()), 3),
            "wind_m_s": r(self.sim.wind.current, 2) if self.sim.wind else [0.0, 0.0, 0.0],  # the air's velocity right now
            # Where the drone BELIEVES it is (only with sensor noise on), and how it believes it is tilted.
            "est_pos_m": r(self.sim.estimator.state.pos_m) if self.sim.estimator else None,
            "est_quat": r(self.sim.estimator.state.quat, 5) if self.sim.estimator else None,
            "gps_settle_left_s": round(self.gps_settle_left_s(), 1),  # > 0: take-off not allowed yet
            "gps_outage": bool(self.gps_outage and self.sim.estimator),  # no GPS fixes (sensor noise on)
            "gps_lost_s": round(self.gps_lost_s(), 1),  # how long the outage has lasted
            "failsafe": self.failsafe_active(),  # GPS-loss failsafe: holding level + altitude, not position
            "failsafe_drag_n_s_m": self.failsafe_drag_n_s_m,  # extra air drag in failsafe (0 = real physics)
            "failsafe_tilt_deg": round(math.degrees(self.failsafe_tilt_rad), 1),  # a full W/A/S/D press in failsafe
            "battery": ({**self.sim.battery.snapshot(), "failsafe": self._batt_fs} if self.sim.battery else None),
            "gps_glitch": self.gps_glitch,
            "ekf": self.sim.estimator.health() if self.sim.estimator is not None and self.sim.estimator.kind == "ekf" else None,
            "setpoint": {
                "x_m": round(sp.x_m, 3),
                "y_m": round(sp.y_m, 3),
                "alt_m": round(sp.alt_m, 3),
                "yaw_rad": round(sp.yaw_rad, 4),
            },
            "message": self.pilot.message,
        }


@dataclass
class RunResult:
    """Everything about one run, in one JSON-friendly record.

    It is the single input for plots, history, comparisons and (later) the AI assistant.
    """

    run_type: str
    config: dict
    env: dict
    seed: int
    scenario: dict
    telemetry: dict[str, list[float]]
    metrics: dict
    events: list[dict] = field(default_factory=list)
    summary: list[str] = field(default_factory=list)
    try_next: list[dict] = field(default_factory=list)
    plots: list[dict] = field(default_factory=list)  # how to draw this run (set by experiments)

    def to_dict(self) -> dict:
        return asdict(self)

    def to_json(self, **kwargs) -> str:
        return json.dumps(self.to_dict(), **kwargs)

    def channel(self, name: str) -> np.ndarray:
        return np.asarray(self.telemetry[name])


def run_headless(
    config: DroneConfig,
    setpoint: Setpoint | Callable[[float], Setpoint],
    duration_s: float,
    seed: int = 0,
    initial_state: State | None = None,
    run_type: str = "custom",
    scenario: dict | None = None,
    env: EnvConfig | None = None,
    sensors: SensorConfig | None = None,
    stop_when: Callable[[], bool] | None = None,
    battery: BatteryConfig | None = None,
    telemetry_every_n_steps: int = TELEMETRY_EVERY_N_STEPS,
    before_step: Callable[["LightweightSim"], None] | None = None,
    extra_channels: Callable[["LightweightSim"], dict[str, float]] | None = None,
) -> RunResult:
    """Run a whole flight as fast as possible and return a RunResult.

    `setpoint` is either fixed, a function of time (seconds) returning a Setpoint, or an object with
    `setpoint_for(estimate, time_s, truth)` (e.g. a mission follower: it navigates on the estimate).
    `stop_when` (checked at 50 Hz) can end the run early, e.g. once a mission has landed.
    `before_step(sim)` runs before every physics step (e.g. to inject a GPS glitch); `extra_channels(sim)` returns
    more telemetry values to record (same names every time).
    Telemetry is recorded at 50 Hz; wind and estimate channels are added when those are on.
    """
    if not 0 < duration_s <= MAX_DURATION_S:
        raise ValueError(f"duration_s must be between 0 and {MAX_DURATION_S} s, got {duration_s}.")
    sim = LightweightSim()
    state = sim.reset(config, seed, initial_state, env, sensors, battery=battery)
    if hasattr(setpoint, "setpoint_for"):
        setpoint_at = lambda t: setpoint.setpoint_for(sim.estimate, t, state)  # noqa: E731  (belief, time, truth)
    elif callable(setpoint):
        setpoint_at = setpoint
    else:
        setpoint_at = lambda _t: setpoint  # noqa: E731

    channels = list(TELEMETRY_CHANNELS)
    if sim.wind:
        channels += WIND_CHANNELS
    if sim.estimator:
        channels += ESTIMATE_CHANNELS
    if extra_channels is not None:
        channels += list(extra_channels(sim))
    tel: dict[str, list[float]] = {name: [] for name in channels}
    events: list[dict] = []
    was_on_ground = state.on_ground
    saturated_logged = False

    def record(sp: Setpoint) -> None:
        roll, pitch, yaw = state.euler_rad()
        roll_sp, pitch_sp = sim.controller.last_tilt_sp_rad
        values = [
            state.time_s,
            *state.pos_m,
            *state.vel_m_s,
            roll, pitch, yaw,
            *sim.motor_pct,
            float(sim.physics.last_thrusts_n.sum()),
            sp.alt_m,
            sp.x_m, sp.y_m,
            roll_sp, pitch_sp, sp.yaw_rad,
        ]  # fmt: skip
        if sim.wind:
            values += list(sim.wind.current)
        if sim.estimator:
            est = sim.estimator.state
            readings = sim.sensors.readings
            values += [*est.pos_m, est.vel_m_s[2], *est.euler_rad(), readings.baro_m, *readings.gps_xy_m]
        if extra_channels is not None:
            values += list(extra_channels(sim).values())
        for name, value in zip(channels, values):
            tel[name].append(float(value))

    n_steps = int(round(duration_s / PHYSICS_DT_S))
    record(setpoint_at(0.0))
    for i in range(1, n_steps + 1):
        sp = setpoint_at(state.time_s)
        if before_step is not None:
            before_step(sim)
        sim.step(sp)

        if was_on_ground and not state.on_ground:
            events.append({"t_s": state.time_s, "type": "liftoff", "message": "Left the ground."})
        elif not was_on_ground and state.on_ground:
            events.append({"t_s": state.time_s, "type": "touchdown", "message": "Touched the ground."})
        was_on_ground = state.on_ground
        if not saturated_logged and sim.motor_pct.max() >= SATURATION_PCT:
            saturated_logged = True
            events.append(
                {
                    "t_s": state.time_s,
                    "type": "motor_saturation",
                    "message": "At least one motor hit 100 %: the controller asked for more than the motors can give.",
                }
            )
        if i % telemetry_every_n_steps == 0:
            record(sp)
            if stop_when is not None and stop_when():
                break

    # Round to 5 decimals (10 µm, 0.0006°): plenty for plots, and keeps saved runs small.
    tel = {name: [round(v, TELEMETRY_DECIMALS) for v in values] for name, values in tel.items()}
    for e in events:
        e["t_s"] = round(e["t_s"], 3)

    t = np.asarray(tel["t_s"])
    motors = np.column_stack([tel[f"motor{k}_pct"] for k in range(1, 5)]).mean(axis=1)
    metrics = {
        "thrust_to_weight": config.thrust_to_weight,
        "expected_hover_pct": config.hover_throttle_pct,
        "left_ground": any(e["type"] == "liftoff" for e in events),
        "max_alt_m": float(np.max(tel["z_m"])),
        "horizontal_drift_m": horizontal_drift_m(np.asarray(tel["x_m"]), np.asarray(tel["y_m"])),
        "mean_motor_pct_second_half": mean_motor_pct(t, motors, float(t[-1]) / 2.0),  # actual length (runs can stop early)
        "rotor_energy_j": sim.rotor_energy_j,
    }
    if sim.battery:
        metrics.update({"battery_used_mah": round(sim.battery.used_mah, 1), "battery_left_pct": round(100 * sim.battery.soc, 1)})
    return RunResult(
        run_type=run_type,
        config=config.to_dict(),
        env={"wind": sim.env.to_dict(), "sensors": sim.sensor_cfg.to_dict()},
        seed=seed,
        scenario=scenario or {"duration_s": duration_s},
        telemetry=tel,
        metrics=metrics,
        events=events,
    )


def run_hover(config: DroneConfig, alt_m: float = 2.0, duration_s: float = 10.0, seed: int = 0) -> RunResult:
    """Take off from the ground and hover at `alt_m`. The classic first flight."""
    if not 0.2 <= alt_m <= 50.0:
        raise ValueError(f"alt_m must be between 0.2 and 50 m, got {alt_m}.")
    result = run_headless(
        config,
        Setpoint(armed=True, alt_m=alt_m),
        duration_s,
        seed,
        run_type="hover",
        scenario={"target_alt_m": alt_m, "duration_s": duration_s},
    )
    t = result.channel("t_s")
    hold_start_s = min(5.0, duration_s / 2.0)
    result.metrics.update(altitude_step_metrics(t, result.channel("z_m"), alt_m, hold_start_s=hold_start_s))
    result.metrics["hover_motor_pct"] = mean_motor_pct(t, _mean_motor(result), hold_start_s)
    result.summary = explain_hover(config, result)
    return result


def run_free_fall(config: DroneConfig, start_alt_m: float = 50.0, duration_s: float = 2.0, seed: int = 0) -> RunResult:
    """Drop the drone with motors off. A sanity check that gravity works: z = z0 - ½·g·t²."""
    return run_headless(
        config,
        Setpoint(armed=False),
        duration_s,
        seed,
        initial_state=State.at(start_alt_m),
        run_type="free_fall",
        scenario={"start_alt_m": start_alt_m, "duration_s": duration_s},
    )


def _mean_motor(result: RunResult) -> np.ndarray:
    return np.column_stack([result.channel(f"motor{k}_pct") for k in range(1, 5)]).mean(axis=1)


def explain_hover(config: DroneConfig, result: RunResult) -> list[str]:
    """Plain-English, rule-based explanation of a hover run (no AI involved)."""
    m = result.metrics
    tw = config.thrust_to_weight
    lines = [
        f"Weight is {config.weight_n:.2f} N and the motors can push up to {config.max_total_thrust_n:.2f} N "
        f"in total, so thrust-to-weight (T/W) = {tw:.2f}."
    ]
    if not m["left_ground"]:
        if tw < 1.0:
            lines.append(
                f"The drone never left the ground: even at 100 % the motors give only {tw * 100:.0f} % of the "
                f"thrust needed to cancel gravity (T/W < 1). No controller can fix this. Reduce the mass "
                f"or use stronger motors."
            )
        else:
            lines.append("The drone never left the ground, even though T/W > 1. Check that it was armed.")
        return lines

    lines.append(
        f"To hover, total thrust must equal weight, so each motor needs about 100 / T/W = "
        f"{config.hover_throttle_pct:.0f} %. Measured hover throttle: {m['hover_motor_pct']:.1f} %."
    )
    if m["settling_time_s"] is None:
        lines.append(
            f"It did NOT settle within ±{m['settling_band_m']} m of {m['target_alt_m']} m by the end of the run "
            f"(final altitude {m['final_alt_m']:.2f} m)."
        )
    else:
        lines.append(
            f"It reached {m['target_alt_m']} m (rise time {m['rise_time_s']:.2f} s), overshot by "
            f"{m['overshoot_pct']:.1f} %, and stayed within ±{m['settling_band_m']} m from "
            f"t = {m['settling_time_s']:.2f} s."
        )
    if tw < 1.5:
        lines.append("T/W is low, so there is little spare thrust: climbs are slow and motors saturate easily.")
    return lines
