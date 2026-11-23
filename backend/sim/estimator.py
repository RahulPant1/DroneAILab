"""State estimator: turns noisy sensor readings into a best guess of the drone's state.

The trick used everywhere here is the COMPLEMENTARY FILTER: combine two imperfect sources
that are bad in opposite ways.

  - PREDICT with fast sensors (gyro, accelerometer). Smooth, but small errors pile up
    over time (drift), because we integrate them.
  - CORRECT with slow, absolute sensors (altimeter, GPS, compass, gravity direction).
    Noisy, but they don't drift.

A "trust" gain decides how much each correction moves the estimate:
  high trust -> follows the noisy sensor closely (jittery estimate);
  low trust  -> smooth, but slower to notice real changes and more prone to drift.
"""

from __future__ import annotations

import math

import numpy as np

from .config import GRAVITY_M_S2
from .frames import cross3, integrate_quat, quat_from_euler, quat_multiply, quat_normalize, quat_to_euler, quat_to_rotation_matrix
from .physics import State
from .sensors import BARO_EVERY_N_STEPS, GPS_EVERY_N_STEPS, IMU_EVERY_N_STEPS, Readings, SensorConfig

COMPASS_CORRECTION_PER_S = 1.0
GPS_HZ = 10.0
GPS_SETTLE_S = 5.0  # pre-arm: average this long of GPS fixes on the ground before take-off is allowed
ACCEL_TRUSTED_BAND = 0.15  # only use the accelerometer for tilt when it reads within ±15 % of g


def alpha_beta_gains(alpha: float) -> tuple[float, float]:
    """Position and velocity correction gains for a trust level alpha (0–1).

    beta = alpha² / (2 − alpha) is the classic 'alpha-beta filter' pairing, which keeps
    the position and velocity corrections in balance.
    """
    return alpha, alpha * alpha / (2.0 - alpha)


class Estimator:
    kind = "complementary"

    def __init__(self, cfg: SensorConfig, initial: State, physics_dt_s: float):
        self.cfg = cfg
        self.est = initial.copy()  # starts out knowing where it is (as if just calibrated)
        self.imu_dt_s = physics_dt_s * IMU_EVERY_N_STEPS
        self.baro_dt_s = physics_dt_s * BARO_EVERY_N_STEPS
        self.gps_dt_s = physics_dt_s * GPS_EVERY_N_STEPS
        self.baro_gains = alpha_beta_gains(cfg.baro_trust)
        self.gps_gains = alpha_beta_gains(cfg.gps_trust)

        # Ground alignment ("GPS settle"): while the drone sits disarmed on the ground, it KNOWS it isn't moving.
        # So instead of filtering, average every GPS fix (and altimeter reading) and hold the velocity at zero.
        # Averaging N noisy fixes shrinks the error by about sqrt(N). Set by the sim each step.
        self.stationary = False
        self.settled = False  # latched once enough fixes have been averaged
        self._gps_sum, self._gps_n = np.zeros(2), 0
        self._baro_sum, self._baro_n = 0.0, 0

    @property
    def state(self) -> State:
        return self.est

    @property
    def settle_left_s(self) -> float:
        """Seconds of ground averaging still needed before take-off is allowed (0 once settled)."""
        return 0.0 if self.settled else max(0.0, GPS_SETTLE_S - self._gps_n / GPS_HZ)

    def set_stationary(self, stationary: bool) -> None:
        if stationary == self.stationary:
            return
        self.stationary = stationary
        self._gps_sum, self._gps_n = np.zeros(2), 0  # each stay on the ground starts a fresh average
        self._baro_sum, self._baro_n = 0.0, 0

    def update(self, r: Readings, time_s: float) -> State:
        e = self.est
        e.time_s = time_s
        if r.new_imu:
            self._predict(r)
        if self.stationary:
            self._ground_align(r)
            if r.new_compass:
                self._correct_heading(r.compass_rad)
            return e
        if r.new_baro:
            self._correct_axis(2, r.baro_m, self.baro_gains, self.baro_dt_s)
        if r.new_gps:
            self._correct_axis(0, r.gps_xy_m[0], self.gps_gains, self.gps_dt_s)
            self._correct_axis(1, r.gps_xy_m[1], self.gps_gains, self.gps_dt_s)
        if r.new_compass:
            self._correct_heading(r.compass_rad)
        return e

    def _predict(self, r: Readings) -> None:
        """Gyro -> attitude (with a gentle pull toward the accelerometer's 'up'); accelerometer -> velocity/position."""
        e = self.est
        rate = r.gyro_rad_s.copy()

        # Tilt correction: where the accelerometer says "up" is vs where we THINK up is (both in body frame).
        # Their cross product is the small rotation that would line them up (the Mahony idea).
        f = r.accel_m_s2
        f_norm = math.sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2])
        if abs(f_norm - GRAVITY_M_S2) < ACCEL_TRUSTED_BAND * GRAVITY_M_S2:
            up_measured = f / f_norm
            up_estimated = quat_to_rotation_matrix(e.quat)[2, :]  # world 'up' seen from the body
            rate = rate + self.cfg.tilt_correction * cross3(up_measured, up_estimated)

        e.rate_rad_s = r.gyro_rad_s  # the controller uses the raw (noisy) gyro for spin rates
        e.quat = integrate_quat(e.quat, rate, self.imu_dt_s)

        # Accelerometer -> world acceleration -> integrate velocity and position.
        accel_world = quat_to_rotation_matrix(e.quat) @ f - np.array([0.0, 0.0, GRAVITY_M_S2])
        dt = self.imu_dt_s
        e.pos_m = e.pos_m + e.vel_m_s * dt + 0.5 * accel_world * dt * dt
        e.vel_m_s = e.vel_m_s + accel_world * dt

    def _ground_align(self, r: Readings) -> None:
        """On the ground, disarmed: position = average of all fixes so far, velocity = 0."""
        e = self.est
        e.vel_m_s = np.zeros(3)
        e.pos_m = e.pos_m.copy()
        if r.new_gps:
            self._gps_sum = self._gps_sum + r.gps_xy_m
            self._gps_n += 1
            if self._gps_n >= GPS_SETTLE_S * GPS_HZ:
                self.settled = True
        if r.new_baro:
            self._baro_sum += r.baro_m
            self._baro_n += 1
        if self._gps_n:
            e.pos_m[:2] = self._gps_sum / self._gps_n
        if self._baro_n:
            e.pos_m[2] = self._baro_sum / self._baro_n

    def _correct_axis(self, axis: int, measured: float, gains: tuple[float, float], dt_s: float) -> None:
        """Nudge position (and velocity) on one axis toward a position measurement."""
        e = self.est
        residual = measured - e.pos_m[axis]
        alpha, beta = gains
        e.pos_m = e.pos_m.copy()
        e.vel_m_s = e.vel_m_s.copy()
        e.pos_m[axis] += alpha * residual
        e.vel_m_s[axis] += beta / dt_s * residual

    def _correct_heading(self, compass_rad: float) -> None:
        """Turn the estimate slightly (about the vertical) toward the compass heading."""
        e = self.est
        yaw_est = quat_to_euler(e.quat)[2]
        error = math.remainder(compass_rad - yaw_est, 2 * math.pi)
        turn = COMPASS_CORRECTION_PER_S * error * self.baro_dt_s  # compass runs at the baro rate
        e.quat = quat_normalize(quat_multiply(quat_from_euler(0.0, 0.0, turn), e.quat))


def make_estimator(cfg: SensorConfig, initial: State, physics_dt_s: float):
    """The estimator chosen in the sensor settings: this complementary filter, or the Kalman filter (ekf.py)."""
    if cfg.estimator == "ekf":
        from .ekf import EKF

        return EKF(cfg, initial, physics_dt_s)
    return Estimator(cfg, initial, physics_dt_s)
