"""6-DOF rigid-body physics of an X-configuration quadcopter.

"6-DOF" = six degrees of freedom: the drone can move along 3 axes and rotate about 3 axes.
Each step: motors spin up/down -> propellers make thrust and torque -> Newton's laws
give accelerations -> integrate to get new velocity, position, spin rate and attitude.

Motor layout (top view, nose pointing right):

        3 FL (CW)       1 FR (CCW)        x forward ->
                   \\   /
                    body                  y left ^
                   /    \\
        2 RL (CCW)      4 RR (CW)

Index order 0..3 = FR, RL, FL, RR (the PX4 quad-X order), so a PX4 backend could
reuse the same motor numbering later. CW/CCW = spin direction seen from above.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .config import GRAVITY_M_S2, DroneConfig
from .frames import (
    body_up_in_world,
    cross3,
    integrate_quat,
    quat_from_euler,
    quat_identity,
    quat_to_euler,
)

# Motor positions as multiples of (arm_length / sqrt(2)), in the body frame (x forward, y left).
MOTOR_XY_UNIT = np.array([[1.0, -1.0], [-1.0, 1.0], [1.0, 1.0], [-1.0, -1.0]])
# Spin direction seen from above: +1 = counter-clockwise (CCW), -1 = clockwise (CW).
MOTOR_SPIN_DIR = np.array([1.0, 1.0, -1.0, -1.0])

GROUND_EPS_M = 1e-9


@dataclass
class State:
    """Everything needed to describe the drone at one instant (ground truth)."""

    time_s: float = 0.0
    pos_m: np.ndarray = field(default_factory=lambda: np.zeros(3))  # world ENU
    vel_m_s: np.ndarray = field(default_factory=lambda: np.zeros(3))  # world ENU
    quat: np.ndarray = field(default_factory=quat_identity)  # body -> world
    rate_rad_s: np.ndarray = field(default_factory=lambda: np.zeros(3))  # body frame
    motor_speed_rad_s: np.ndarray = field(default_factory=lambda: np.zeros(4))
    on_ground: bool = True

    def copy(self) -> "State":
        return State(
            self.time_s,
            self.pos_m.copy(),
            self.vel_m_s.copy(),
            self.quat.copy(),
            self.rate_rad_s.copy(),
            self.motor_speed_rad_s.copy(),
            self.on_ground,
        )

    def euler_rad(self) -> tuple[float, float, float]:
        return quat_to_euler(self.quat)

    @classmethod
    def hovering(cls, cfg: DroneConfig, alt_m: float, yaw_rad: float = 0.0) -> "State":
        """Already in the air with the motors at hover speed (for experiments that start mid-air)."""
        s = cls.at(alt_m, yaw_rad=yaw_rad)
        hover_frac = min(1.0, 1.0 / cfg.thrust_to_weight)
        s.motor_speed_rad_s = np.full(4, cfg.motor_max_speed_rad_s * math.sqrt(hover_frac))
        return s

    @classmethod
    def at(cls, alt_m: float = 0.0, roll_rad: float = 0.0, pitch_rad: float = 0.0, yaw_rad: float = 0.0):
        """A drone at rest at the given height and attitude. Altitude 0 = sitting on the ground."""
        return cls(
            pos_m=np.array([0.0, 0.0, alt_m]),
            quat=quat_from_euler(roll_rad, pitch_rad, yaw_rad),
            on_ground=alt_m <= GROUND_EPS_M,
        )


class QuadcopterPhysics:
    """Turns motor commands into motion, one small time step at a time."""

    def __init__(self, cfg: DroneConfig):
        self.cfg = cfg
        self.mass_kg = cfg.mass_kg
        self.kt = cfg.motor_kt_n_per_rad2_s2
        self.kq = cfg.motor_kq_nm_per_rad2_s2
        self.max_speed_rad_s = cfg.motor_max_speed_rad_s
        self.inertia = np.array(cfg.inertia_kg_m2)
        self.motor_xy_m = MOTOR_XY_UNIT * (cfg.arm_length_m / math.sqrt(2.0))
        self.gravity_n = np.array([0.0, 0.0, -cfg.mass_kg * GRAVITY_M_S2])
        # Outputs of the most recent step, handy for telemetry and energy.
        self.last_thrusts_n = np.zeros(4)
        self.last_rotor_power_w = 0.0
        self.last_accel_m_s2 = np.zeros(3)  # true acceleration (world), what an accelerometer feels + g
        # Extra linear drag on top of the drone's own (N per m/s). 0 = real physics. The live view switches it on
        # in the GPS-loss failsafe as a "training wheels" helper, so the drone doesn't coast as far.
        self.extra_drag_n_s_m = 0.0

    def motor_speed_command(self, motor_pct: np.ndarray) -> np.ndarray:
        """Convert motor % (0-100) into a target spin speed.

        Thrust grows with speed squared, so speed = max_speed * sqrt(pct/100).
        That way "50 %" means half of the max THRUST, which is easier to reason about.
        """
        frac = np.clip(motor_pct, 0.0, 100.0) / 100.0
        return self.max_speed_rad_s * np.sqrt(frac)

    def step(self, state: State, motor_pct: np.ndarray, dt_s: float, wind_m_s: np.ndarray | None = None) -> State:
        """Advance the drone by dt seconds (semi-implicit Euler). Updates `state` in place.

        Semi-implicit Euler: update velocity from forces first, then move using the
        NEW velocity. It is as simple as plain Euler but much more stable.
        `wind_m_s` is the air's velocity (world frame); drag depends on speed relative to the air.
        """
        cfg = self.cfg

        # 1. Motors can't change speed instantly: first-order lag toward the commanded speed.
        omega_cmd = self.motor_speed_command(motor_pct)
        lag = 1.0 - math.exp(-dt_s / cfg.motor_time_constant_s)
        omega = state.motor_speed_rad_s + (omega_cmd - state.motor_speed_rad_s) * lag
        state.motor_speed_rad_s = omega

        # 2. Propellers: thrust T = kT*w^2 (pushes up along body z), spin-drag torque Q = kQ*w^2.
        omega_sq = omega * omega
        thrusts_n = self.kt * omega_sq
        drag_torques_nm = self.kq * omega_sq
        total_thrust_n = float(thrusts_n.sum())
        self.last_thrusts_n = thrusts_n
        self.last_rotor_power_w = float((drag_torques_nm * omega).sum())

        # 3. Torques on the body (body frame).
        #    Roll: thrust on the left (+y) side pushes that side up -> positive roll torque.
        #    Pitch: thrust at the front (+x) lifts the nose -> NEGATIVE pitch torque (pitch+ = nose down).
        #    Yaw: each propeller drags the body the opposite way to its own spin.
        torque_nm = np.array(
            [
                float(self.motor_xy_m[:, 1] @ thrusts_n),
                -float(self.motor_xy_m[:, 0] @ thrusts_n),
                -float(MOTOR_SPIN_DIR @ drag_torques_nm),
            ]
        )
        torque_nm -= cfg.angular_drag_nm_s_rad * state.rate_rad_s

        # 4. Forces in the world frame: gravity + thrust (tilted with the body) + air drag.
        vel = state.vel_m_s
        air_vel = vel if wind_m_s is None else vel - wind_m_s  # velocity relative to the air
        airspeed = math.sqrt(air_vel[0] * air_vel[0] + air_vel[1] * air_vel[1] + air_vel[2] * air_vel[2])
        drag_n = -(cfg.drag_linear_n_s_m + self.extra_drag_n_s_m + cfg.drag_quadratic_n_s2_m2 * airspeed) * air_vel
        force_n = self.gravity_n + body_up_in_world(state.quat) * total_thrust_n + drag_n
        accel_m_s2 = force_n / self.mass_kg
        self.last_accel_m_s2 = accel_m_s2

        # 5. Ground contact: while resting on the ground and not pushing up hard enough,
        #    the ground holds the drone still (no sinking, no sliding, no spinning).
        if state.on_ground:
            if accel_m_s2[2] <= 0.0:
                self.last_accel_m_s2 = np.zeros(3)
                state.vel_m_s = np.zeros(3)
                state.rate_rad_s = np.zeros(3)
                state.time_s += dt_s
                return state
            state.on_ground = False  # lift-off!

        # 6. Translation (Newton: F = m*a).
        state.vel_m_s = vel + accel_m_s2 * dt_s
        state.pos_m = state.pos_m + state.vel_m_s * dt_s

        # 7. Rotation (Euler's equation: I*dw/dt = torque - w x (I*w)).
        rate = state.rate_rad_s
        ang_accel = (torque_nm - cross3(rate, self.inertia * rate)) / self.inertia
        state.rate_rad_s = rate + ang_accel * dt_s
        state.quat = integrate_quat(state.quat, state.rate_rad_s, dt_s)

        # 8. Touch-down: stop at the ground, no bounce. We level the drone (keeping its heading),
        #    because tipping over is out of scope for this simple model.
        if state.pos_m[2] < 0.0:
            state.pos_m[2] = 0.0
            state.vel_m_s = np.zeros(3)
            state.rate_rad_s = np.zeros(3)
            _, _, yaw = quat_to_euler(state.quat)
            state.quat = quat_from_euler(0.0, 0.0, yaw)
            state.on_ground = True

        state.time_s += dt_s
        return state
