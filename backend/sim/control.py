"""Flight controller: cascaded PID loops plus the motor mixer.

The chain, outer loop to inner loop:

  altitude error ──PID──> vertical acceleration ──> total (collective) thrust ─┐
  angle error ──P──> desired spin rate ──PID──> angular acceleration ──> torque ─┤
                                                                     mixer <─────┘
                                                                       │
                                                         4 motor commands (0–100 %)

With position control on (Phase 1b), one more loop sits on top and PICKS the angles:

  position error ──P──> wanted speed ──PI──> wanted horizontal acceleration ──> roll/pitch to tilt to

A quadcopter can only push along its own "up" axis, so to accelerate sideways it must tilt.

The controller is given the ESTIMATED state; for now the estimate is simply the true
state (sensor noise arrives in Phase 2).
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass

import numpy as np

from .config import GRAVITY_M_S2, DroneConfig
from .frames import body_up_in_world, quat_conjugate, quat_from_euler, quat_multiply, quat_to_euler
from .physics import MOTOR_SPIN_DIR, MOTOR_XY_UNIT, State

# Safety limits for the commands each loop may send to the next.
MAX_CLIMB_ACCEL_M_S2 = 5.0
MAX_SINK_ACCEL_M_S2 = 4.0
MAX_TILT_RATE_RAD_S = math.radians(220.0)
MAX_YAW_RATE_RAD_S = math.radians(120.0)
MAX_TILT_ANG_ACCEL_RAD_S2 = 300.0
MAX_YAW_ANG_ACCEL_RAD_S2 = 30.0
MIN_TILT_COS = 0.5  # don't boost thrust beyond 2x for tilt compensation (60° tilt)
ALT_INTEGRAL_ZONE_M = 0.3  # altitude integral only collects error within 0.3 m of the target
TILT_AUTHORITY_SHARE = 0.35
MAX_TILT_RAD = math.radians(25.0)  # position control never tilts more than this
MAX_HORIZONTAL_SPEED_M_S = 3.0
MAX_HORIZONTAL_ACCEL_M_S2 = GRAVITY_M_S2 * math.tan(MAX_TILT_RAD)


@dataclass
class Setpoint:
    """What the pilot (or a script) asks for. Angles in radians, distances in metres.

    position_control=False: hold the given roll/pitch angles (x_m, y_m are ignored).
    position_control=True : fly to (x_m, y_m); the controller picks roll/pitch itself.
    """

    armed: bool = True
    alt_m: float = 0.0
    roll_rad: float = 0.0
    pitch_rad: float = 0.0
    yaw_rad: float = 0.0
    position_control: bool = False
    x_m: float = 0.0
    y_m: float = 0.0

    def to_dict(self) -> dict:
        return asdict(self)


class PID:
    """Classic PID: output = Kp*error + Ki*(integral of error) + Kd*(rate of change of error).

    P reacts to the error now, I removes long-lasting small errors, D damps (brakes)
    fast changes. Safety features against "integrator windup" (the integral storing up a
    huge correction during a big move, which then causes overshoot):
      - output clamp: the output is limited to what the next stage can actually do;
      - while the output is clamped, the integral stops growing;
      - optional integral zone: the integral only collects error once we're close to the
        target (|error| < zone). Big moves are handled by P and D alone.
    """

    def __init__(self, kp, ki, kd, out_min, out_max, integral_limit, integral_zone=None):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.integral_limit = integral_limit
        self.integral_zone = integral_zone
        self.reset()

    def reset(self) -> None:
        self.integral = 0.0
        self._prev_error = None

    def update(self, error: float, dt_s: float, error_rate: float | None = None) -> float:
        """One controller tick. Pass `error_rate` if you know it (e.g. from a velocity),
        which avoids a "derivative kick" when the setpoint jumps."""
        if error_rate is None:
            error_rate = 0.0 if self._prev_error is None else (error - self._prev_error) / dt_s
        self._prev_error = error

        unclamped = self.kp * error + self.ki * self.integral + self.kd * error_rate
        output = min(self.out_max, max(self.out_min, unclamped))

        pushing_further_into_limit = (unclamped > self.out_max and error > 0) or (
            unclamped < self.out_min and error < 0
        )
        outside_zone = self.integral_zone is not None and abs(error) > self.integral_zone
        if not pushing_further_into_limit and not outside_zone:
            lim = self.integral_limit
            self.integral = min(lim, max(-lim, self.integral + error * dt_s))
        return output


class Mixer:
    """Shares the wanted total thrust and torques out among the four motors.

    Each motor's thrust contributes to: total lift, roll torque (by its y position),
    pitch torque (by its x position) and yaw torque (by its spin direction).
    That's 4 equations and 4 unknowns, so we solve them with a matrix inverse.
    """

    def __init__(self, cfg: DroneConfig):
        xy = MOTOR_XY_UNIT * (cfg.arm_length_m / math.sqrt(2.0))
        k = cfg.motor_torque_per_thrust_m
        allocation = np.vstack([np.ones(4), xy[:, 1], -xy[:, 0], -MOTOR_SPIN_DIR * k])
        self.allocation = allocation
        self.inverse = np.linalg.inv(allocation)
        self.max_thrust_n = cfg.motor_max_thrust_n

    def mix(self, total_thrust_n: float, torque_nm: np.ndarray) -> np.ndarray:
        """Return motor commands in % (0–100), with priorities when motors run out of range.

        1. Roll/pitch first: if a motor would need more than 100 % (or less than 0 %),
           shift ALL motors down (or up) together. That gives up a little lift but keeps
           the drone upright: staying level matters more than climbing.
        2. Yaw last: yaw is weak and rarely urgent, so it only gets the headroom that is
           left over. It never steals lift or tilt control.
        """
        thrusts = self.inverse @ np.array([total_thrust_n, torque_nm[0], torque_nm[1], 0.0])
        over = thrusts.max() - self.max_thrust_n
        if over > 0:
            thrusts = thrusts - over
        under = -thrusts.min()
        if under > 0:
            thrusts = thrusts + under
        thrusts = np.clip(thrusts, 0.0, self.max_thrust_n)

        yaw = self.inverse[:, 3] * torque_nm[2]  # each motor's share of the yaw torque
        scale = 1.0
        for t, dy in zip(thrusts, yaw):
            if dy > 0:
                scale = min(scale, (self.max_thrust_n - t) / dy)
            elif dy < 0:
                scale = min(scale, t / -dy)
        thrusts = thrusts + max(0.0, scale) * yaw
        return 100.0 * thrusts / self.max_thrust_n

    def max_tilt_torque_nm(self) -> float:
        """Largest roll (or pitch) torque: two motors at full thrust, the other two off."""
        return float(np.max(np.abs(self.allocation[1]))) * 2.0 * self.max_thrust_n


class CascadedController:
    """Altitude + attitude controller (Phase 1a). Call `update` at the controller rate."""

    def __init__(self, cfg: DroneConfig):
        g = cfg.gains
        self.cfg = cfg
        self.inertia = np.array(cfg.inertia_kg_m2)
        self.mixer = Mixer(cfg)
        # Don't ask for more spin-up than the motors can give. Using at most ~1/3 of the full
        # tilt authority leaves room for lift, so a heavy drone gets sluggish, not dropping.
        authority_rad_s2 = TILT_AUTHORITY_SHARE * self.mixer.max_tilt_torque_nm() / self.inertia[0]
        tilt = min(MAX_TILT_ANG_ACCEL_RAD_S2, authority_rad_s2)
        yaw = MAX_YAW_ANG_ACCEL_RAD_S2
        self.alt_pid = PID(
            g.alt_kp, g.alt_ki, g.alt_kd, -MAX_SINK_ACCEL_M_S2, MAX_CLIMB_ACCEL_M_S2,
            integral_limit=2.0, integral_zone=ALT_INTEGRAL_ZONE_M,
        )  # fmt: skip
        self.rate_pids = [
            PID(g.rate_kp, g.rate_ki, g.rate_kd, -tilt, tilt, 0.5),
            PID(g.rate_kp, g.rate_ki, g.rate_kd, -tilt, tilt, 0.5),
            PID(g.yaw_rate_kp, g.yaw_rate_ki, 0.0, -yaw, yaw, 0.5),
        ]
        self.att_kp = np.array([g.att_kp, g.att_kp, g.yaw_kp])
        self.max_rate = np.array([MAX_TILT_RATE_RAD_S, MAX_TILT_RATE_RAD_S, MAX_YAW_RATE_RAD_S])
        acc = MAX_HORIZONTAL_ACCEL_M_S2
        self.pos_kp = g.pos_kp
        self.vel_pids = [PID(g.vel_kp, g.vel_ki, 0.0, -acc, acc, integral_limit=2.0) for _ in range(2)]
        self.last_thrust_cmd_n = 0.0
        self.last_tilt_sp_rad = (0.0, 0.0)

    def reset(self) -> None:
        self.alt_pid.reset()
        for pid in (*self.rate_pids, *self.vel_pids):
            pid.reset()
        self.last_thrust_cmd_n = 0.0
        self.last_tilt_sp_rad = (0.0, 0.0)

    def update(self, est: State, sp: Setpoint, dt_s: float) -> np.ndarray:
        """Compute the four motor commands (%) from the estimated state and the setpoint."""
        if not sp.armed:
            self.reset()
            return np.zeros(4)

        if sp.position_control:
            roll_sp, pitch_sp = self._position_loop(est, sp, dt_s)
        else:
            roll_sp, pitch_sp = sp.roll_rad, sp.pitch_rad
        self.last_tilt_sp_rad = (roll_sp, pitch_sp)

        thrust_n = self._altitude_loop(est, sp, dt_s)
        torque_nm = self._attitude_loop(est, roll_sp, pitch_sp, sp.yaw_rad, dt_s)
        self.last_thrust_cmd_n = thrust_n
        return self.mixer.mix(thrust_n, torque_nm)

    def _position_loop(self, est: State, sp: Setpoint, dt_s: float) -> tuple[float, float]:
        """Position error -> wanted speed (P) -> wanted acceleration (PI) -> roll & pitch.

        The acceleration is worked out in the world frame (east/north), then turned into
        the drone's own forward/left directions using its heading (yaw). Tilting the nose
        down (positive pitch) accelerates forward; positive roll (right side down) goes right.
        """
        err = np.array([sp.x_m - est.pos_m[0], sp.y_m - est.pos_m[1]])
        vel_sp = self.pos_kp * err
        speed = math.hypot(vel_sp[0], vel_sp[1])
        if speed > MAX_HORIZONTAL_SPEED_M_S:
            vel_sp *= MAX_HORIZONTAL_SPEED_M_S / speed

        ax = self.vel_pids[0].update(float(vel_sp[0] - est.vel_m_s[0]), dt_s)
        ay = self.vel_pids[1].update(float(vel_sp[1] - est.vel_m_s[1]), dt_s)
        accel = math.hypot(ax, ay)
        if accel > MAX_HORIZONTAL_ACCEL_M_S2:
            ax, ay = ax * MAX_HORIZONTAL_ACCEL_M_S2 / accel, ay * MAX_HORIZONTAL_ACCEL_M_S2 / accel

        _, _, yaw = quat_to_euler(est.quat)
        accel_forward = math.cos(yaw) * ax + math.sin(yaw) * ay
        accel_left = -math.sin(yaw) * ax + math.cos(yaw) * ay
        pitch_sp = math.atan2(accel_forward, GRAVITY_M_S2)
        roll_sp = math.atan2(-accel_left * math.cos(pitch_sp), GRAVITY_M_S2)
        return roll_sp, pitch_sp

    def _altitude_loop(self, est: State, sp: Setpoint, dt_s: float) -> float:
        """Altitude PID -> wanted vertical acceleration -> total thrust.

        Hover needs thrust = weight (m*g). On top of that we add m*a for the wanted
        acceleration. When tilted only the vertical part of thrust holds the drone up,
        so we divide by cos(tilt) to make up for it.
        """
        alt_error_m = sp.alt_m - est.pos_m[2]
        accel_cmd_m_s2 = self.alt_pid.update(alt_error_m, dt_s, error_rate=-est.vel_m_s[2])
        cos_tilt = max(MIN_TILT_COS, body_up_in_world(est.quat)[2])
        return self.cfg.mass_kg * (GRAVITY_M_S2 + accel_cmd_m_s2) / cos_tilt

    def _attitude_loop(self, est: State, roll_sp: float, pitch_sp: float, yaw_sp: float, dt_s: float) -> np.ndarray:
        """Angle error -> desired spin rate (P) -> rate PID -> torque.

        The angle error comes from the "difference" quaternion between where we are and
        where we want to be. For small errors, 2 * its vector part = error angle per axis.
        """
        q_des = quat_from_euler(roll_sp, pitch_sp, yaw_sp)
        q_err = quat_multiply(quat_conjugate(q_des), est.quat)
        if q_err[0] < 0:  # take the short way round
            q_err = -q_err
        angle_error_rad = 2.0 * q_err[1:]

        rate_sp = np.clip(-self.att_kp * angle_error_rad, -self.max_rate, self.max_rate)
        rate_error = rate_sp - est.rate_rad_s
        ang_accel = np.array([pid.update(float(rate_error[i]), dt_s) for i, pid in enumerate(self.rate_pids)])
        return self.inertia * ang_accel
