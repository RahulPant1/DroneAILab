"""The simulation loop: physics at 500 Hz, controller at 250 Hz, telemetry at 50 Hz.

`LightweightSim` is our only simulator. It follows the small `SimBackend` interface, so a
different simulator (e.g. PX4) could be plugged in later without changing the callers.

Two ways to run it:
  headless  (`run_headless`) : as fast as the CPU allows, for experiments and tests;
  real-time (`RealtimeSim`)  : paced to the wall clock, driven by the WebSocket for the 3D view.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from typing import Callable, Protocol

import numpy as np

from .config import ConfigError, DroneConfig, require_valid
from .control import CascadedController, Setpoint
from .environment import EnvConfig, Wind, validate_env
from .estimator import Estimator
from .metrics import altitude_step_metrics, horizontal_drift_m, mean_motor_pct
from .physics import QuadcopterPhysics, State
from .pilot import Pilot, Sticks
from .sensors import SensorConfig, Sensors, validate_sensors

PHYSICS_DT_S = 0.002  # 500 Hz
CONTROL_EVERY_N_STEPS = 2  # 250 Hz
TELEMETRY_EVERY_N_STEPS = 10  # 50 Hz
MAX_DURATION_S = 120.0
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

    def reset(
        self,
        config: DroneConfig,
        seed: int = 0,
        initial_state: State | None = None,
        env: EnvConfig | None = None,
        sensors: SensorConfig | None = None,
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
            self.estimator = Estimator(self.sensor_cfg, self.state, PHYSICS_DT_S)
        else:
            self.sensors = None
            self.estimator = None
        self.step_count = 0
        self.motor_pct = np.zeros(4)
        self.rotor_energy_j = 0.0
        return self.state

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
            self.estimator = Estimator(cfg, self.state, PHYSICS_DT_S)
        else:
            self.sensors = None
            self.estimator = None

    @property
    def estimate(self) -> State:
        """What the controller believes. With sensors off ('perfect sensors') this IS the truth."""
        return self.estimator.state if self.estimator else self.state

    def step(self, command: Setpoint) -> State:
        """Advance one physics step (2 ms). The controller runs on every 2nd step."""
        if self.step_count % CONTROL_EVERY_N_STEPS == 0:
            self.motor_pct = self.controller.update(self.estimate, command, PHYSICS_DT_S * CONTROL_EVERY_N_STEPS)
        wind = self.wind.step(self.state.time_s) if self.wind else None
        self.physics.step(self.state, self.motor_pct, PHYSICS_DT_S, wind)
        if self.sensors:
            readings = self.sensors.sample(self.state, self.physics.last_accel_m_s2)
            self.estimator.update(readings, self.state.time_s)
        self.rotor_energy_j += self.physics.last_rotor_power_w * PHYSICS_DT_S
        self.step_count += 1
        return self.state

    def get_state(self) -> State:
        return self.state


MAX_CATCH_UP_S = 0.1  # if the server stalls, skip ahead rather than racing to catch up


class RealtimeSim:
    """Runs the simulator in step with the real clock, controlled by a Pilot.

    The WebSocket loop calls `advance(elapsed_wall_time)` about 30 times a second;
    we then do however many 2 ms physics steps fit into that time.
    """

    def __init__(self, config: DroneConfig, seed: int = 0):
        self.sim = LightweightSim()
        self.env = EnvConfig()  # wind; it stays the same across resets until changed
        self.sensor_cfg = SensorConfig()  # off = perfect sensors; also kept across resets
        self.reset(config, seed)

    def reset(self, config: DroneConfig | None = None, seed: int | None = None) -> None:
        self.config = config or self.config
        self.seed = self.seed if seed is None else seed
        self.sim.reset(self.config, self.seed, self.start_state(), env=self.env, sensors=self.sensor_cfg)
        self.pilot = Pilot()
        self.paused = False
        self._time_debt_s = 0.0
        self.notices: list[str] = []  # one-off messages for the UI's log (the WebSocket loop sends them)

    def start_state(self) -> State | None:
        """Where the drone sits after a reset (None = on the ground at the origin)."""
        return None

    def next_setpoint(self, dt_s: float) -> Setpoint:
        """What the controller should aim for during the next physics step (the pilot decides)."""
        return self.pilot.update(self.sim.estimate, dt_s, self.state)  # decides on the estimate

    @property
    def state(self) -> State:
        return self.sim.get_state()

    def advance(self, wall_dt_s: float) -> int:
        """Simulate `wall_dt_s` seconds of real time (nothing happens while paused)."""
        if self.paused:
            return 0
        self._time_debt_s += min(max(wall_dt_s, 0.0), MAX_CATCH_UP_S)
        n_steps = int(self._time_debt_s / PHYSICS_DT_S)
        self._time_debt_s -= n_steps * PHYSICS_DT_S
        for _ in range(n_steps):
            self.sim.step(self.next_setpoint(PHYSICS_DT_S))
        return n_steps

    def command(self, cmd: str, **args) -> str:
        """Handle a button press: takeoff | land | hover | pause | resume | reset | target | sticks."""
        s = self.sim.estimate  # buttons act on where the drone BELIEVES it is
        if cmd == "takeoff":
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
            return self.set_sensors(bool(args.get("enabled", False)), args.get("noise_scale", 1.0))
        if cmd == "sticks":
            self.pilot.set_sticks(Sticks(**{k: float(args.get(k, 0.0)) for k in ("forward", "left", "up", "yaw")}))
            return ""
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

    def set_sensors(self, enabled: bool, noise_scale=1.0) -> str:
        """Noisy sensors + estimator (the controller flies on its ESTIMATE), or perfect sensors (it sees the truth).
        The buttons' autopilot and collision checks always use the true position."""
        try:
            cfg = SensorConfig(enabled=enabled, noise_scale=float(noise_scale))
        except (TypeError, ValueError):
            raise ValueError("noise_scale must be a number.") from None
        self.sim.set_sensors(cfg)  # raises ConfigError (a ValueError) if out of range
        self.sensor_cfg = cfg
        if not enabled:
            return "Perfect sensors: the controller sees the true state."
        return f"Sensor noise on ({cfg.noise_scale:g}× typical): the controller now flies on its estimate."

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
) -> RunResult:
    """Run a whole flight as fast as possible and return a RunResult.

    `setpoint` is either fixed, a function of time (seconds) returning a Setpoint, or an object with
    `setpoint_for(estimate, time_s, truth)` (e.g. a mission follower: it navigates on the estimate).
    `stop_when` (checked at 50 Hz) can end the run early, e.g. once a mission has landed.
    Telemetry is recorded at 50 Hz; wind and estimate channels are added when those are on.
    """
    if not 0 < duration_s <= MAX_DURATION_S:
        raise ValueError(f"duration_s must be between 0 and {MAX_DURATION_S} s, got {duration_s}.")
    sim = LightweightSim()
    state = sim.reset(config, seed, initial_state, env, sensors)
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
        for name, value in zip(channels, values):
            tel[name].append(float(value))

    n_steps = int(round(duration_s / PHYSICS_DT_S))
    record(setpoint_at(0.0))
    for i in range(1, n_steps + 1):
        sp = setpoint_at(state.time_s)
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
        if i % TELEMETRY_EVERY_N_STEPS == 0:
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
