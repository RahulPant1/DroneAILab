"""Drone configuration: the physical numbers that describe one quadcopter.

Everything is in SI units (kg, m, s, N, rad). A config is plain, frozen data:
you never change it in place, you make a modified copy with `with_changes()`.
The same validation limits apply to the UI, the API and AI-generated configs.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field, fields, replace

GRAVITY_M_S2 = 9.81

# Share of the total mass sitting in the four motors (at the arm tips).
# The rest is the central body. Used only to estimate the moments of inertia.
MOTOR_MASS_FRACTION = 0.4
CENTRAL_BODY_SIZE_M = 0.10


class ConfigError(ValueError):
    """Raised when a config is outside the allowed limits. The message is plain English."""


@dataclass(frozen=True)
class ControlGains:
    """Gains for the cascaded controller in control.py.

    Altitude gains turn an altitude error into a vertical acceleration command.
    Attitude gains turn an angle error into a desired spin rate, and rate gains
    turn a spin-rate error into an angular acceleration. Because the outputs are
    accelerations (not forces), the same gains work for light and heavy drones.
    """

    alt_kp: float = 4.0  # (m/s^2) per m of altitude error
    alt_ki: float = 0.5  # (m/s^2) per (m*s) of accumulated error
    alt_kd: float = 3.5  # (m/s^2) per (m/s) of vertical speed
    att_kp: float = 6.0  # (rad/s) per rad of roll/pitch error
    yaw_kp: float = 3.0  # (rad/s) per rad of yaw error
    rate_kp: float = 20.0  # (rad/s^2) per (rad/s) of roll/pitch rate error
    rate_ki: float = 2.0
    rate_kd: float = 0.0
    yaw_rate_kp: float = 8.0
    yaw_rate_ki: float = 0.5
    pos_kp: float = 1.0  # (m/s) of wanted speed per m of position error
    vel_kp: float = 2.0  # (m/s^2) of wanted horizontal acceleration per (m/s) of speed error
    vel_ki: float = 0.5  # removes steady drift (e.g. from wind, in Phase 2)


@dataclass(frozen=True)
class DroneConfig:
    """Physical description of an X-configuration quadcopter."""

    name: str = "Basic learner"
    mass_kg: float = 1.0
    arm_length_m: float = 0.225  # centre to motor (a "450 mm" frame)
    motor_max_thrust_n: float = 4.905  # per motor, at full speed
    motor_max_speed_rad_s: float = 1000.0
    motor_time_constant_s: float = 0.04  # how quickly a motor reaches a new speed
    motor_torque_per_thrust_m: float = 0.016  # kQ / kT: spin-drag torque per newton of thrust
    drag_linear_n_s_m: float = 0.10  # air drag that grows with speed
    drag_quadratic_n_s2_m2: float = 0.02  # air drag that grows with speed squared
    angular_drag_nm_s_rad: float = 0.002  # air resistance to spinning
    gains: ControlGains = field(default_factory=ControlGains)

    # ---- Derived quantities (computed, not stored) ----

    @property
    def weight_n(self) -> float:
        """Force of gravity on the drone: W = m * g."""
        return self.mass_kg * GRAVITY_M_S2

    @property
    def max_total_thrust_n(self) -> float:
        return 4.0 * self.motor_max_thrust_n

    @property
    def thrust_to_weight(self) -> float:
        """Thrust-to-weight ratio (T/W): above 1 the drone can climb, below 1 it can't lift off."""
        return self.max_total_thrust_n / self.weight_n

    @property
    def hover_throttle_pct(self) -> float:
        """Motor % needed to hover: just enough thrust to cancel weight, i.e. 100 / (T/W)."""
        return 100.0 / self.thrust_to_weight

    @property
    def motor_kt_n_per_rad2_s2(self) -> float:
        """Thrust coefficient kT in T = kT * omega^2, chosen so full speed gives max thrust."""
        return self.motor_max_thrust_n / self.motor_max_speed_rad_s**2

    @property
    def motor_kq_nm_per_rad2_s2(self) -> float:
        """Torque coefficient kQ in Q = kQ * omega^2 (the propeller's spin drag)."""
        return self.motor_torque_per_thrust_m * self.motor_kt_n_per_rad2_s2

    @property
    def inertia_kg_m2(self) -> tuple[float, float, float]:
        """Rough moments of inertia (Ixx, Iyy, Izz): how hard the drone is to spin up.

        Model: four point-mass motors at the arm tips plus a small cube in the middle.
        Mass far from the centre (long arms) makes the drone slower to rotate.
        """
        motor_mass_kg = MOTOR_MASS_FRACTION * self.mass_kg / 4.0
        body_mass_kg = (1.0 - MOTOR_MASS_FRACTION) * self.mass_kg
        body_i = body_mass_kg * CENTRAL_BODY_SIZE_M**2 / 6.0
        # In the X layout each motor sits L/sqrt(2) from both the x and the y axis.
        ixx = 4.0 * motor_mass_kg * self.arm_length_m**2 / 2.0 + body_i
        izz = 4.0 * motor_mass_kg * self.arm_length_m**2 + body_i
        return (ixx, ixx, izz)

    # ---- Helpers ----

    def with_changes(self, **changes) -> "DroneConfig":
        """Return a copy with some fields changed (gains can be a dict of changes)."""
        if isinstance(changes.get("gains"), dict):
            changes["gains"] = replace(self.gains, **changes["gains"])
        return replace(self, **changes)

    def to_dict(self) -> dict:
        d = asdict(self)
        d["derived"] = {
            "weight_n": self.weight_n,
            "thrust_to_weight": self.thrust_to_weight,
            "hover_throttle_pct": self.hover_throttle_pct,
            "inertia_kg_m2": list(self.inertia_kg_m2),
        }
        return d

    @classmethod
    def from_dict(cls, data: dict) -> "DroneConfig":
        """Build a config from a dict (e.g. JSON). Unknown keys are an error, not ignored."""
        data = {k: v for k, v in data.items() if k != "derived"}
        allowed = {f.name for f in fields(cls)}
        unknown = set(data) - allowed
        if unknown:
            raise ConfigError(f"Unknown config field(s): {', '.join(sorted(unknown))}")
        gains = data.pop("gains", None)
        if isinstance(gains, dict):
            g_allowed = {f.name for f in fields(ControlGains)}
            g_unknown = set(gains) - g_allowed
            if g_unknown:
                raise ConfigError(f"Unknown gain(s): {', '.join(sorted(g_unknown))}")
            data["gains"] = ControlGains(**gains)
        return cls(**data)


# ---- Validation limits (min, max). Shared by UI, API and AI suggestions. ----

LIMITS: dict[str, tuple[float, float]] = {
    "mass_kg": (0.1, 5.0),
    "arm_length_m": (0.08, 0.5),
    "motor_max_thrust_n": (0.5, 50.0),
    "motor_max_speed_rad_s": (200.0, 5000.0),
    "motor_time_constant_s": (0.01, 0.2),
    "motor_torque_per_thrust_m": (0.005, 0.05),
    "drag_linear_n_s_m": (0.0, 2.0),
    "drag_quadratic_n_s2_m2": (0.0, 1.0),
    "angular_drag_nm_s_rad": (0.0, 0.05),
}

GAIN_LIMITS: dict[str, tuple[float, float]] = {
    "alt_kp": (0.0, 30.0),
    "alt_ki": (0.0, 10.0),
    "alt_kd": (0.0, 20.0),
    "att_kp": (0.0, 20.0),
    "yaw_kp": (0.0, 10.0),
    "rate_kp": (0.0, 80.0),
    "rate_ki": (0.0, 20.0),
    "rate_kd": (0.0, 2.0),
    "yaw_rate_kp": (0.0, 40.0),
    "yaw_rate_ki": (0.0, 10.0),
    "pos_kp": (0.0, 5.0),
    "vel_kp": (0.0, 10.0),
    "vel_ki": (0.0, 5.0),
}


def validate_config(cfg: DroneConfig) -> list[str]:
    """Return a list of plain-English problems (empty list = valid)."""
    problems: list[str] = []
    for name, (lo, hi) in LIMITS.items():
        value = getattr(cfg, name)
        if not isinstance(value, (int, float)) or not math.isfinite(value) or not lo <= value <= hi:
            problems.append(f"{name} = {value!r} is outside the allowed range {lo}–{hi}.")
    for name, (lo, hi) in GAIN_LIMITS.items():
        value = getattr(cfg.gains, name)
        if not isinstance(value, (int, float)) or not math.isfinite(value) or not lo <= value <= hi:
            problems.append(f"gains.{name} = {value!r} is outside the allowed range {lo}–{hi}.")
    return problems


def require_valid(cfg: DroneConfig) -> DroneConfig:
    problems = validate_config(cfg)
    if problems:
        raise ConfigError(" ".join(problems))
    return cfg


# ---- Presets ----


def _motor_thrust_for(thrust_to_weight: float, mass_kg: float) -> float:
    """Per-motor max thrust that gives the requested T/W for this mass."""
    return thrust_to_weight * mass_kg * GRAVITY_M_S2 / 4.0


PRESETS: dict[str, DroneConfig] = {
    "basic": DroneConfig(),  # T/W = 2.0
    "lightweight": DroneConfig(
        name="Lightweight",
        mass_kg=0.5,
        arm_length_m=0.15,
        motor_max_thrust_n=_motor_thrust_for(2.5, 0.5),
        motor_max_speed_rad_s=1400.0,
        motor_time_constant_s=0.03,
        drag_linear_n_s_m=0.06,
        drag_quadratic_n_s2_m2=0.012,
    ),
    "heavy": DroneConfig(
        name="Heavy",
        mass_kg=2.5,
        arm_length_m=0.35,
        motor_max_thrust_n=_motor_thrust_for(1.3, 2.5),
        motor_max_speed_rad_s=700.0,
        motor_time_constant_s=0.05,
        drag_linear_n_s_m=0.2,
        drag_quadratic_n_s2_m2=0.04,
    ),
    "high_thrust": DroneConfig(
        name="High-thrust",
        motor_max_thrust_n=_motor_thrust_for(4.0, 1.0),
        motor_max_speed_rad_s=1200.0,
        motor_time_constant_s=0.03,
    ),
}


def get_preset(preset_id: str) -> DroneConfig:
    try:
        return PRESETS[preset_id]
    except KeyError:
        raise ConfigError(f"Unknown preset '{preset_id}'. Choose one of: {', '.join(PRESETS)}") from None
