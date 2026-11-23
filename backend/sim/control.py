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
from .frames import (
    body_up_in_world, cross3, quat_conjugate, quat_from_euler, quat_from_rotation_matrix, quat_multiply,
    quat_to_euler,
)
from .physics import MOTOR_SPIN_DIR, MOTOR_XY_UNIT, State

# Safety limits for the commands each loop may send to the next.
MAX_CLIMB_ACCEL_M_S2 = 5.0
MAX_SINK_ACCEL_M_S2 = 4.0
MAX_TILT_RATE_RAD_S = math.radians(220.0)
MAX_YAW_RATE_RAD_S = math.radians(120.0)
MAX_TILT_ANG_ACCEL_RAD_S2 = 300.0
MAX_YAW_ANG_ACCEL_RAD_S2 = 30.0
MIN_TILT_COS = 0.5  # don't boost thrust beyond 2x for tilt compensation (60° tilt)
ALT_MAX_I_ACCEL_M_S2 = 4.0  # the altitude I-term can make up for ~40 % extra unknown weight
ALT_INTEGRAL_MAX_SPEED_M_S = 0.5  # altitude I-term only learns while climbing/sinking slower than this
TILT_AUTHORITY_SHARE = 0.35
MAX_TILT_RAD = math.radians(25.0)  # position control never tilts more than this
MAX_HORIZONTAL_SPEED_M_S = 3.0
MAX_HORIZONTAL_ACCEL_M_S2 = GRAVITY_M_S2 * math.tan(MAX_TILT_RAD)
# Following a planned trajectory (feedforward given) the drone may tilt and fly further: the plan already
# respects these limits, so they only catch the corrections on top.
MAX_TILT_AGILE_RAD = math.radians(50.0)
MAX_ACCEL_AGILE_M_S2 = GRAVITY_M_S2 * math.tan(MAX_TILT_AGILE_RAD)
MAX_SPEED_AGILE_M_S = 15.0
# Geometric controller (Lee, Leok & McClamroch 2010; Mellinger & Kumar 2011): one 3D loop, no small-angle tricks.
GEO_KP = np.array([2.0, 2.0, 4.0])  # (m/s^2) per m of position error: same stiffness as the cascade's pos_kp*vel_kp
GEO_KV = np.array([2.0, 2.0, 3.5])  # (m/s^2) per (m/s): same as vel_kp / alt_kd
GEO_KI = np.array([0.5, 0.5, 0.5])  # (m/s^2) per (m*s): removes steady offsets (wind, a heavier drone)
GEO_MAX_I_M_S2 = 4.0
GEO_LEARN_BELOW_M = 1.0  # the integral learns only while the drone is close to the plan


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
    # Trajectory feedforward (world frame, ENU), from a planned trajectory (autonomy/trajectory.py). With these set,
    # the controller doesn't wait for an error to appear: it already knows the speed and acceleration it needs.
    vel_ff_m_s: tuple[float, float, float] | None = None
    acc_ff_m_s2: tuple[float, float, float] | None = None
    jerk_ff_m_s3: tuple[float, float, float] | None = None
    yaw_rate_ff_rad_s: float = 0.0
    law: str = "cascade"  # "cascade" = the PID chain below; "geometric" = GeometricController (needs position_control)

    def to_dict(self) -> dict:
        return asdict(self)


class PID:
    """Classic PID: output = Kp*error + Ki*(integral of error) + Kd*(rate of change of error).

    P reacts to the error now, I removes long-lasting small errors, D damps (brakes)
    fast changes. Safety features against "integrator windup" (the integral storing up a
    huge correction during a big move, which then causes overshoot):
      - output clamp: the output is limited to what the next stage can actually do;
      - while the output is clamped, the integral stops growing;
      - the caller can pause learning (`learn=False`), e.g. while the drone is still
        moving fast toward a far-away target. Big moves are handled by P and D alone.
    """

    def __init__(self, kp, ki, kd, out_min, out_max, max_i_output):
        """`max_i_output`: the most the I-term alone may add to the output (same units as the output)."""
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.max_i_output = max_i_output
        self.reset()

    def reset(self) -> None:
        self.integral = 0.0
        self._prev_error = None

    def update(self, error: float, dt_s: float, error_rate: float | None = None, learn: bool = True) -> float:
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
        if learn and not pushing_further_into_limit and self.ki > 0:
            lim = self.max_i_output / self.ki
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
            max_i_output=ALT_MAX_I_ACCEL_M_S2,
        )  # fmt: skip
        self.rate_pids = [
            PID(g.rate_kp, g.rate_ki, g.rate_kd, -tilt, tilt, max_i_output=0.5 * tilt),
            PID(g.rate_kp, g.rate_ki, g.rate_kd, -tilt, tilt, max_i_output=0.5 * tilt),
            PID(g.yaw_rate_kp, g.yaw_rate_ki, 0.0, -yaw, yaw, max_i_output=0.5 * yaw),
        ]
        self.att_kp = np.array([g.att_kp, g.att_kp, g.yaw_kp])
        self.max_rate = np.array([MAX_TILT_RATE_RAD_S, MAX_TILT_RATE_RAD_S, MAX_YAW_RATE_RAD_S])
        acc = MAX_HORIZONTAL_ACCEL_M_S2
        self.pos_kp = g.pos_kp
        self.vel_pids = [PID(g.vel_kp, g.vel_ki, 0.0, -acc, acc, max_i_output=acc) for _ in range(2)]
        self.last_thrust_cmd_n = 0.0
        self.last_tilt_sp_rad = (0.0, 0.0)
        self.geometric = GeometricController(cfg)

    def reset(self) -> None:
        self.geometric.reset()
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

        if sp.position_control and sp.law == "geometric":
            thrust_n, q_des, rate_ff = self.geometric.update(est, sp, dt_s)
            roll_sp, pitch_sp, _ = quat_to_euler(q_des)
            self.last_tilt_sp_rad = (roll_sp, pitch_sp)
            torque_nm = self._attitude_loop_q(est, q_des, dt_s, rate_ff)
            self.last_thrust_cmd_n = thrust_n
            return self.mixer.mix(thrust_n, torque_nm)
        self.geometric.reset()

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
        agile = sp.vel_ff_m_s is not None
        max_speed = MAX_SPEED_AGILE_M_S if agile else MAX_HORIZONTAL_SPEED_M_S
        max_accel = MAX_ACCEL_AGILE_M_S2 if agile else MAX_HORIZONTAL_ACCEL_M_S2
        if agile:
            # Feedforward: the plan's own speed, plus a correction for any position error.
            corr = math.hypot(vel_sp[0], vel_sp[1])
            if corr > MAX_HORIZONTAL_SPEED_M_S:
                vel_sp *= MAX_HORIZONTAL_SPEED_M_S / corr
            vel_sp = vel_sp + np.asarray(sp.vel_ff_m_s[:2])
        speed = math.hypot(vel_sp[0], vel_sp[1])
        if speed > max_speed:
            vel_sp *= max_speed / speed

        ax = self.vel_pids[0].update(float(vel_sp[0] - est.vel_m_s[0]), dt_s)
        ay = self.vel_pids[1].update(float(vel_sp[1] - est.vel_m_s[1]), dt_s)
        if sp.acc_ff_m_s2 is not None:  # ... and the plan's own acceleration (plus the drag it will meet)
            ff = np.asarray(sp.acc_ff_m_s2) + drag_feedforward_m_s2(self.cfg, sp.vel_ff_m_s)
            ax, ay = ax + ff[0], ay + ff[1]
        accel = math.hypot(ax, ay)
        if accel > max_accel:
            ax, ay = ax * max_accel / accel, ay * max_accel / accel

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
        vz = est.vel_m_s[2]
        # The I-term only "learns" while the drone is nearly holding its height. During a fast climb
        # it would otherwise store up a big correction and overshoot (integrator windup).
        vz_ff = sp.vel_ff_m_s[2] if sp.vel_ff_m_s is not None else 0.0
        learn = abs(vz - vz_ff) < ALT_INTEGRAL_MAX_SPEED_M_S
        accel_cmd_m_s2 = self.alt_pid.update(alt_error_m, dt_s, error_rate=vz_ff - vz, learn=learn)
        if sp.acc_ff_m_s2 is not None:
            accel_cmd_m_s2 += sp.acc_ff_m_s2[2] + drag_feedforward_m_s2(self.cfg, sp.vel_ff_m_s)[2]
        cos_tilt = max(MIN_TILT_COS, body_up_in_world(est.quat)[2])
        return self.cfg.mass_kg * (GRAVITY_M_S2 + accel_cmd_m_s2) / cos_tilt

    def _attitude_loop(self, est: State, roll_sp: float, pitch_sp: float, yaw_sp: float, dt_s: float) -> np.ndarray:
        """Angle error -> desired spin rate (P) -> rate PID -> torque.

        The angle error comes from the "difference" quaternion between where we are and
        where we want to be. For small errors, 2 * its vector part = error angle per axis.
        """
        return self._attitude_loop_q(est, quat_from_euler(roll_sp, pitch_sp, yaw_sp), dt_s)

    def _attitude_loop_q(self, est: State, q_des: np.ndarray, dt_s: float, rate_ff: np.ndarray | None = None) -> np.ndarray:
        """The same loop, given the wanted attitude as a quaternion, plus an optional spin-rate feedforward
        (body frame): how fast the plan itself needs the drone to be turning right now."""
        q_err = quat_multiply(quat_conjugate(q_des), est.quat)
        if q_err[0] < 0:  # take the short way round
            q_err = -q_err
        angle_error_rad = 2.0 * q_err[1:]

        rate_sp = -self.att_kp * angle_error_rad
        if rate_ff is not None:
            rate_sp = rate_sp + rate_ff
        rate_sp = np.clip(rate_sp, -self.max_rate, self.max_rate)
        rate_error = rate_sp - est.rate_rad_s
        ang_accel = np.array([pid.update(float(rate_error[i]), dt_s) for i, pid in enumerate(self.rate_pids)])
        return self.inertia * ang_accel


def drag_feedforward_m_s2(cfg: DroneConfig, vel_m_s) -> np.ndarray:
    """The acceleration needed just to cancel air drag at the planned speed (calm air assumed).

    The physics pushes back with (linear + quadratic*speed) * velocity newtons; at 8 m/s that is about 2 m/s^2
    on the 1 kg drone, a big error if nobody plans for it. A real autopilot learns this from flight logs.
    """
    if vel_m_s is None:
        return np.zeros(3)
    v = np.asarray(vel_m_s, dtype=float)
    return (cfg.drag_linear_n_s_m + cfg.drag_quadratic_n_s2_m2 * float(np.linalg.norm(v))) * v / cfg.mass_kg


class GeometricController:
    """Position control in one 3D step, the way research and racing drones do it (an "SE(3)" controller).

    1. Wanted acceleration = the plan's acceleration + drag compensation + P*(position error) + D*(velocity error)
       + I*(integral of the position error).                        (a 3D vector, no separate altitude loop)
    2. Add gravity: F = m*(a + g*up). A quadcopter can only push along its own "up" axis, so F's DIRECTION is the
       attitude it needs (exactly, at any tilt: no small-angle approximation), and the thrust is F projected on
       where the body points NOW (if it isn't tilted yet, it doesn't push hard in the wrong direction).
    3. The heading (yaw) picks the rotation about that axis.
    4. Spin-rate feedforward from the plan's JERK (differential flatness): to change its acceleration the drone
       must turn its thrust axis, and the jerk says how fast. Telling the inner loop this removes its lag.
    """

    def __init__(self, cfg: DroneConfig):
        self.cfg = cfg
        self.reset()

    def reset(self) -> None:
        self.integral = np.zeros(3)

    def update(self, est: State, sp: Setpoint, dt_s: float) -> tuple[float, np.ndarray, np.ndarray]:
        """Returns (thrust N, wanted attitude quaternion, spin-rate feedforward rad/s in the body frame)."""
        m = self.cfg.mass_kg
        ref_p = np.array([sp.x_m, sp.y_m, sp.alt_m])
        ref_v = np.zeros(3) if sp.vel_ff_m_s is None else np.asarray(sp.vel_ff_m_s, dtype=float)
        ref_a = np.zeros(3) if sp.acc_ff_m_s2 is None else np.asarray(sp.acc_ff_m_s2, dtype=float)
        e_p = ref_p - est.pos_m
        e_v = ref_v - est.vel_m_s
        if float(np.linalg.norm(e_p)) < GEO_LEARN_BELOW_M:
            lim = GEO_MAX_I_M_S2 / GEO_KI
            self.integral = np.clip(self.integral + e_p * dt_s, -lim, lim)
        a_cmd = ref_a + drag_feedforward_m_s2(self.cfg, ref_v) + GEO_KP * e_p + GEO_KV * e_v + GEO_KI * self.integral
        a_cmd[2] = min(max(a_cmd[2], -MAX_SINK_ACCEL_M_S2), MAX_CLIMB_ACCEL_M_S2 + MAX_ACCEL_AGILE_M_S2)

        f_vec = m * (a_cmd + np.array([0.0, 0.0, GRAVITY_M_S2]))
        # Tilt limit: keep the thrust direction within MAX_TILT_AGILE of straight up (shrink the sideways part).
        horiz = math.hypot(f_vec[0], f_vec[1])
        max_h = f_vec[2] * math.tan(MAX_TILT_AGILE_RAD)
        if f_vec[2] <= 0.1 * m * GRAVITY_M_S2:
            f_vec[2] = 0.1 * m * GRAVITY_M_S2  # never ask to point the thrust down
            max_h = f_vec[2] * math.tan(MAX_TILT_AGILE_RAD)
        if horiz > max_h:
            f_vec[:2] *= max_h / horiz
        z_b = f_vec / float(np.linalg.norm(f_vec))

        x_c = np.array([math.cos(sp.yaw_rad), math.sin(sp.yaw_rad), 0.0])  # where the nose should point
        y_b = cross3(z_b, x_c)
        y_b /= float(np.linalg.norm(y_b))
        x_b = cross3(y_b, z_b)
        q_des = quat_from_rotation_matrix(np.column_stack([x_b, y_b, z_b]))

        thrust_n = max(0.0, float(f_vec @ body_up_in_world(est.quat)))

        rate_ff = np.zeros(3)
        if sp.jerk_ff_m_s3 is not None:
            f_mag = max(float(np.linalg.norm(f_vec)), 0.1)
            j = np.asarray(sp.jerk_ff_m_s3, dtype=float)
            h = (m / f_mag) * (j - float(z_b @ j) * z_b)  # how fast the thrust axis must swing
            rate_ff[0] = -float(h @ y_b)
            rate_ff[1] = float(h @ x_b)
        rate_ff[2] = sp.yaw_rate_ff_rad_s * float(z_b[2])
        return thrust_n, q_des, rate_ff
