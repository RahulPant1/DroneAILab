"""The simulation loop: physics at 500 Hz, controller at 250 Hz, telemetry at 50 Hz.

`LightweightSim` is our only simulator. It follows the small `SimBackend` interface, so a
different simulator (e.g. PX4) could be plugged in later without changing the callers.

Headless mode (this file) runs as fast as the CPU allows; it's for experiments and tests.
Real-time mode (paced to the wall clock, for the 3D view) arrives in Phase 1b.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from typing import Callable, Protocol

import numpy as np

from .config import DroneConfig, require_valid
from .control import CascadedController, Setpoint
from .metrics import altitude_step_metrics, horizontal_drift_m, mean_motor_pct
from .physics import QuadcopterPhysics, State

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
)  # fmt: skip


class SimBackend(Protocol):
    """What any simulator must offer: start over, advance one step, report the state."""

    def reset(self, config: DroneConfig, seed: int, initial_state: State | None = None) -> State: ...
    def step(self, command: Setpoint) -> State: ...
    def get_state(self) -> State: ...


class LightweightSim:
    """Our own simple simulator: physics + controller, with fixed time steps."""

    def __init__(self) -> None:
        self.config: DroneConfig | None = None

    def reset(self, config: DroneConfig, seed: int = 0, initial_state: State | None = None) -> State:
        self.config = require_valid(config)
        self.seed = seed
        self.rng = np.random.default_rng(seed)  # used by sensors/wind from Phase 2 on
        self.physics = QuadcopterPhysics(config)
        self.controller = CascadedController(config)
        self.state = initial_state.copy() if initial_state else State.at(0.0)
        self.step_count = 0
        self.motor_pct = np.zeros(4)
        self.rotor_energy_j = 0.0
        return self.state

    def step(self, command: Setpoint) -> State:
        """Advance one physics step (2 ms). The controller runs on every 2nd step."""
        if self.step_count % CONTROL_EVERY_N_STEPS == 0:
            estimate = self.state  # Phase 2 adds sensors + estimator; with no noise, estimate == truth
            self.motor_pct = self.controller.update(estimate, command, PHYSICS_DT_S * CONTROL_EVERY_N_STEPS)
        self.physics.step(self.state, self.motor_pct, PHYSICS_DT_S)
        self.rotor_energy_j += self.physics.last_rotor_power_w * PHYSICS_DT_S
        self.step_count += 1
        return self.state

    def get_state(self) -> State:
        return self.state


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
) -> RunResult:
    """Run a whole flight as fast as possible and return a RunResult.

    `setpoint` is either fixed, or a function of time (seconds) returning a Setpoint.
    """
    if not 0 < duration_s <= MAX_DURATION_S:
        raise ValueError(f"duration_s must be between 0 and {MAX_DURATION_S} s, got {duration_s}.")
    sim = LightweightSim()
    state = sim.reset(config, seed, initial_state)
    setpoint_at = setpoint if callable(setpoint) else (lambda _t: setpoint)

    tel: dict[str, list[float]] = {name: [] for name in TELEMETRY_CHANNELS}
    events: list[dict] = []
    was_on_ground = state.on_ground
    saturated_logged = False

    def record(sp: Setpoint) -> None:
        roll, pitch, yaw = state.euler_rad()
        values = (
            state.time_s,
            *state.pos_m,
            *state.vel_m_s,
            roll, pitch, yaw,
            *sim.motor_pct,
            float(sim.physics.last_thrusts_n.sum()),
            sp.alt_m,
        )  # fmt: skip
        for name, value in zip(TELEMETRY_CHANNELS, values):
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

    t = np.asarray(tel["t_s"])
    motors = np.column_stack([tel[f"motor{k}_pct"] for k in range(1, 5)]).mean(axis=1)
    metrics = {
        "thrust_to_weight": config.thrust_to_weight,
        "expected_hover_pct": config.hover_throttle_pct,
        "left_ground": any(e["type"] == "liftoff" for e in events),
        "max_alt_m": float(np.max(tel["z_m"])),
        "horizontal_drift_m": horizontal_drift_m(np.asarray(tel["x_m"]), np.asarray(tel["y_m"])),
        "mean_motor_pct_second_half": mean_motor_pct(t, motors, duration_s / 2.0),
        "rotor_energy_j": sim.rotor_energy_j,
    }
    return RunResult(
        run_type=run_type,
        config=config.to_dict(),
        env={"wind_m_s": [0.0, 0.0, 0.0], "sensor_noise": False},
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
