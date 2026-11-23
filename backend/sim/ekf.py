"""Extended Kalman filter (EKF): the state estimator real autopilots use (ArduPilot's EKF3, PX4's EKF2).

The complementary filter in estimator.py blends sensors with FIXED trust gains. A Kalman filter works the gains out
itself, every step, from two things it keeps track of:

  - how UNCERTAIN its own estimate is (the covariance P: "I think I'm within ±0.3 m"), which grows while it
    dead-reckons on the motion sensors and shrinks each time a measurement arrives;
  - how noisy each sensor is (R: "GPS is good to ±0.3 m").

The gain is their ratio: trust the GPS a lot when the estimate is unsure, little when it is already sure. And because P
also records how the errors are LINKED (a tilt error makes a velocity error grow, because the accelerometer's push is
turned the wrong way), a GPS VELOCITY measurement corrects the tilt too. That is what keeps the attitude right in a
long banked turn, where the complementary filter's "accelerometer = down" assumption is fooled.

This is an "error-state" EKF, the design of EKF3 and most modern autopilots. It keeps the full nominal state
(position, velocity, attitude quaternion, gyro bias) and a 12-number error vector with its covariance:

    error = [dp (3), dv (3), dtheta (3, a small rotation in the WORLD frame), d_gyro_bias (3)]

Predict (250 Hz, every IMU sample):  integrate gyro - bias and accelerometer; grow P with the sensor noise.
Correct  (when a reading arrives):   GPS position (10 Hz), GPS velocity (10 Hz), altimeter (50 Hz), compass (50 Hz).
Every correction first passes an INNOVATION GATE, like EKF3: if the reading is further from the prediction than
GATE standard deviations, it is rejected (a GPS glitch is ignored instead of followed). If GPS position keeps failing
for 10 s, the filter gives in and resets to it (EKF3's posRetryTimeUseVel_ms).

Written from the textbook equations (Sola, "Quaternion kinematics for the error-state Kalman filter", 2017), not
from ArduPilot code. The gate sizes and the 10 s retry match EKF3's defaults (EK3_*_I_GATE: 500 = 5 sigma,
magnetometer 300 = 3 sigma).
"""

from __future__ import annotations

import math
from collections import deque

import numpy as np

from .config import GRAVITY_M_S2
from .frames import integrate_quat, quat_multiply, quat_normalize, quat_to_euler, quat_to_rotation_matrix
from .physics import State
from .sensors import (ACCEL_NOISE_M_S2, BARO_EVERY_N_STEPS, BARO_NOISE_M, COMPASS_NOISE_RAD, GPS_EVERY_N_STEPS,
                      GPS_NOISE_M, GPS_VEL_NOISE_M_S, GYRO_NOISE_RAD_S, IMU_EVERY_N_STEPS, Readings, SensorConfig)

GPS_HZ = 10.0
GPS_SETTLE_S = 5.0  # same pre-arm rule as the complementary filter
GATE_SIGMA = {"gps_pos": 5.0, "gps_vel": 5.0, "baro": 5.0, "compass": 3.0}  # EKF3: EK3_*_I_GATE / 100
GPS_RETRY_S = 10.0  # GPS position rejected this long -> reset to it
VEL_RETRY_S = 5.0  # GPS velocity rejected this long -> the filter is lost: reset to GPS and admit it (widen P)
# How fast the filter lets the gyro bias wander (rad/s per sqrt(s)). A tuning choice: bigger = learns a sudden bias
# change faster, but the estimate wanders more when there's no GPS to pin it (experiment 12's outage gets worse).
# EKF3's default works out at about 5e-5 (EK3_GBIAS_P_NSE = 1e-3, applied to the delta angle each step): tuned for
# biases that drift slowly with temperature. See docs/results/ekf-compare.md for all three on ArduPilot data.
GYRO_BIAS_WALK_RAD_S = 2e-4
BIAS_WALKS = {"ekf3_like": 5e-5, "default": GYRO_BIAS_WALK_RAD_S, "fast": 1e-3}
LOST_SIGMA = {"att": math.radians(5.0), "bias": math.radians(1.0)}  # after a reset: "I may be this far off"
ZERO_VEL_SIGMA_M_S = 0.02  # on the ground, disarmed: "I'm not moving" is a (very good) velocity measurement
INITIAL_SIGMA = {"pos": 0.05, "vel": 0.05, "att": math.radians(1.0), "bias": math.radians(2.0)}
SENSORS = ("gps_pos", "gps_vel", "baro", "compass")
_I3 = np.eye(3)


def skew(v: np.ndarray) -> np.ndarray:
    """The matrix [v]x with [v]x @ w = v x w."""
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def quat_from_rotvec(r: np.ndarray) -> np.ndarray:
    """A rotation of |r| radians about the axis r/|r|, as a quaternion."""
    angle = float(np.linalg.norm(r))
    if angle < 1e-12:
        return np.array([1.0, 0.5 * r[0], 0.5 * r[1], 0.5 * r[2]])
    s = math.sin(angle / 2.0) / angle
    return np.array([math.cos(angle / 2.0), r[0] * s, r[1] * s, r[2] * s])


class EKF:
    """Drop-in replacement for estimator.Estimator (same state / update / set_stationary / settled interface)."""

    kind = "ekf"

    def __init__(self, cfg: SensorConfig, initial: State, physics_dt_s: float):
        self.cfg = cfg
        self.est = initial.copy()
        self.imu_dt_s = physics_dt_s * IMU_EVERY_N_STEPS
        self.baro_dt_s = physics_dt_s * BARO_EVERY_N_STEPS
        self.gps_dt_s = physics_dt_s * GPS_EVERY_N_STEPS
        self.gyro_bias = np.zeros(3)
        k = cfg.noise_scale
        g = k * cfg.gps_noise_scale
        # What the filter believes about its sensors (the datasheet numbers). Small floors keep it well-behaved
        # when a noise is set to zero: no real sensor is perfect, and P must never become exactly zero.
        self.sigma_accel = max(ACCEL_NOISE_M_S2 * k, 0.05)
        self.sigma_gyro = max(GYRO_NOISE_RAD_S * k, 0.002)
        self.r_gps_pos = (GPS_NOISE_M * g) ** 2 + (cfg.gps_drift_m * k) ** 2 + 0.05**2
        self.r_gps_vel = max(GPS_VEL_NOISE_M_S * g, 0.02) ** 2
        self.r_baro = max(BARO_NOISE_M * k, 0.02) ** 2
        self.r_compass = max(COMPASS_NOISE_RAD * k, math.radians(0.5)) ** 2
        s = INITIAL_SIGMA
        self.P = np.diag([s["pos"] ** 2] * 3 + [s["vel"] ** 2] * 3 + [s["att"] ** 2] * 3 + [s["bias"] ** 2] * 3)
        self._q_dt = -1.0  # the process noise Q is rebuilt whenever the IMU step changes (see _process_noise)
        self._phi = np.eye(12)
        # GPS readings describe where the drone was gps_lag_s ago (receivers are ~0.1-0.2 s late). 0 = on time.
        # With a lag, each fix is compared with the estimate from that moment (kept in a short history).
        self.gps_lag_s = 0.0
        self.bias_walk_rad_s = GYRO_BIAS_WALK_RAD_S
        self._history: deque = deque(maxlen=200)
        self.stationary = False
        self.settled = False
        self._gps_n = 0
        self.time_s = initial.time_s
        # Health, like EKF3's XKF4 log: the last test ratio per sensor (innovation / gate; > 1 = rejected).
        self.test_ratio = {s: 0.0 for s in SENSORS}
        self.rejected = {s: 0 for s in SENSORS}
        self.accepted = {s: 0 for s in SENSORS}
        self.gps_resets = 0
        self.vel_resets = 0
        self._gps_last_ok_s = initial.time_s
        self._vel_last_ok_s = initial.time_s

    # -- interface shared with the complementary filter ---------------------------------------------------------
    @property
    def state(self) -> State:
        return self.est

    @property
    def settle_left_s(self) -> float:
        return 0.0 if self.settled else max(0.0, GPS_SETTLE_S - self._gps_n / GPS_HZ)

    def set_stationary(self, stationary: bool) -> None:
        if stationary != self.stationary:
            self.stationary = stationary
            self._gps_n = 0

    def update(self, r: Readings, time_s: float) -> State:
        self.est.time_s = self.time_s = time_s
        if r.new_imu:
            self._predict(r)
        if self.stationary and r.new_baro:
            self.fuse_zero_velocity()
        if r.new_baro:
            self.fuse_baro(r.baro_m)
        if r.new_gps:
            if self.stationary:
                self._gps_n += 1
                if self._gps_n >= GPS_SETTLE_S * GPS_HZ:
                    self.settled = True
            self.fuse_gps(r.gps_xy_m, r.gps_vel_m_s)
        if r.new_compass:
            self._fuse_compass(r.compass_rad)
        return self.est

    # -- the filter -----------------------------------------------------------------------------------------------
    def sigmas(self) -> dict[str, list[float]]:
        """One standard deviation of each part of the estimate: what the filter thinks its own error is."""
        d = np.sqrt(np.maximum(np.diag(self.P), 0.0))
        return {"pos_m": d[0:3].tolist(), "vel_m_s": d[3:6].tolist(), "att_rad": d[6:9].tolist(),
                "gyro_bias_rad_s": d[9:12].tolist()}

    def set_noise(self, accel_m_s2: float, gyro_rad_s: float, gps_pos_m: float, gps_vel_m_s: float, baro_m: float,
                  compass_rad: float) -> None:
        """Tell the filter how noisy its sensors are (1 standard deviation each), instead of the simulator's values."""
        self.sigma_accel, self.sigma_gyro = accel_m_s2, gyro_rad_s
        self.r_gps_pos, self.r_gps_vel = gps_pos_m**2, gps_vel_m_s**2
        self.r_baro, self.r_compass = baro_m**2, compass_rad**2
        self._q_dt = -1.0

    def _process_noise(self, dt: float) -> np.ndarray:
        """Per-step process noise: white noise on each IMU sample becomes a random walk in velocity and angle."""
        if dt != self._q_dt:
            self._q_dt = dt
            self.Q = np.diag([0.0] * 3 + [(self.sigma_accel * dt) ** 2] * 3 + [(self.sigma_gyro * dt) ** 2] * 3
                             + [self.bias_walk_rad_s**2 * dt] * 3)
            self._phi[0:3, 3:6] = _I3 * dt
        return self.Q

    def _predict(self, r: Readings) -> None:
        self.predict(r.gyro_rad_s, r.accel_m_s2, self.imu_dt_s)

    def predict(self, gyro_rad_s: np.ndarray, accel_m_s2: np.ndarray, dt: float) -> None:
        """Dead-reckon one IMU step (gyro and accelerometer in the body frame), and grow the uncertainty."""
        e = self.est
        Q = self._process_noise(dt)
        rate = gyro_rad_s - self.gyro_bias
        e.rate_rad_s = rate  # the controller gets the bias-corrected gyro
        rot = quat_to_rotation_matrix(e.quat)
        accel_world = rot @ accel_m_s2
        e.quat = integrate_quat(e.quat, rate, dt)
        accel_world[2] -= GRAVITY_M_S2
        e.pos_m = e.pos_m + e.vel_m_s * dt + 0.5 * accel_world * dt * dt
        e.vel_m_s = e.vel_m_s + accel_world * dt

        # How the errors grow and couple over one step (linearised):  dp += dv dt;
        # dv += -(R f) x dtheta dt  (a tilt error turns the measured push the wrong way);  dtheta += -R d_bias dt.
        phi = self._phi
        phi[3:6, 6:9] = -skew(accel_world + np.array([0.0, 0.0, GRAVITY_M_S2])) * dt
        phi[6:9, 9:12] = -rot * dt
        P = phi @ self.P @ phi.T + Q
        self.P = 0.5 * (P + P.T)
        if self.gps_lag_s > 0:
            self._history.append((self.time_s, e.pos_m.copy(), e.vel_m_s.copy()))

    def _past(self, lag_s: float) -> tuple[np.ndarray, np.ndarray]:
        """The estimated position and velocity lag_s ago (the newest if the history is too short)."""
        e = self.est
        if lag_s <= 0 or not self._history:
            return e.pos_m, e.vel_m_s
        target = self.time_s - lag_s
        for t, p, v in reversed(self._history):
            if t <= target:
                # The error then is (nearly) the error now: correct the CURRENT state by it.
                return p, v
        return self._history[0][1], self._history[0][2]

    def fuse_gps(self, xy_m: np.ndarray, vel_m_s: np.ndarray | None) -> None:
        """A GPS fix: horizontal position and (if the receiver gives it) 3D velocity, both world ENU."""
        past_p, past_v = self._past(self.gps_lag_s)
        e = self.est
        shift_p = e.pos_m[:2] - past_p[:2]
        self._fuse_gps_pos(np.asarray(xy_m) + shift_p)
        if vel_m_s is not None:
            shift_v = e.vel_m_s - past_v
            v = np.asarray(vel_m_s) + shift_v
            if self._fuse("gps_vel", v, e.vel_m_s.copy(), slice(3, 6), self.r_gps_vel):
                self._vel_last_ok_s = self.time_s
            elif self.time_s - self._vel_last_ok_s > VEL_RETRY_S:
                self._reset_to_gps(np.asarray(xy_m) + shift_p, v)

    def _reset_to_gps(self, xy: np.ndarray, vel: np.ndarray) -> None:
        """Lost: GPS velocity has disagreed for VEL_RETRY_S. Jump to the GPS and widen the uncertainty, so the next
        readings are trusted again (a filter that is too sure of itself stops listening, and never recovers)."""
        e = self.est
        e.pos_m = np.array([xy[0], xy[1], e.pos_m[2]])
        e.vel_m_s = np.asarray(vel, dtype=float).copy()
        keep_z = self.P[2, 2]
        self.P = np.zeros((12, 12))
        self.P[[0, 1], [0, 1]] = self.r_gps_pos
        self.P[2, 2] = keep_z
        self.P[[3, 4, 5], [3, 4, 5]] = self.r_gps_vel
        self.P[[6, 7, 8], [6, 7, 8]] = LOST_SIGMA["att"] ** 2
        self.P[[9, 10, 11], [9, 10, 11]] = LOST_SIGMA["bias"] ** 2
        self.vel_resets += 1
        self._vel_last_ok_s = self._gps_last_ok_s = self.time_s

    def fuse_zero_velocity(self) -> None:
        """On the ground, disarmed: 'I'm not moving' is a (very good) velocity measurement."""
        self._fuse("zero_vel", np.zeros(3), self.est.vel_m_s.copy(), slice(3, 6), ZERO_VEL_SIGMA_M_S**2)

    def fuse_baro(self, height_m: float) -> None:
        self._fuse("baro", np.array([height_m]), self.est.pos_m[2:3].copy(), slice(2, 3), self.r_baro)

    def fuse_compass(self, heading_rad: float) -> None:
        self._fuse_compass(heading_rad)

    def _gate(self, name: str, innov: np.ndarray, S_diag: np.ndarray) -> bool:
        """EKF3-style innovation test: ratio = innovation^2 / (gate^2 * predicted variance); pass if <= 1."""
        ratio = float(np.max(innov * innov / (GATE_SIGMA[name] ** 2 * S_diag)))
        self.test_ratio[name] = ratio
        ok = ratio <= 1.0
        (self.accepted if ok else self.rejected)[name] += 1
        return ok

    def _correct(self, H: np.ndarray, innov: np.ndarray, R: np.ndarray, S: np.ndarray | None = None) -> None:
        """The Kalman update: gain K = P H^T S^-1, move the state by K * innovation, shrink P (Joseph form)."""
        PHt = self.P @ H.T
        if S is None:
            S = H @ PHt + R
        K = np.linalg.solve(S, PHt.T).T
        dx = K @ innov
        IKH = np.eye(12) - K @ H
        P = IKH @ self.P @ IKH.T + K @ R @ K.T
        self.P = 0.5 * (P + P.T)
        self._inject(dx)

    def _inject(self, dx: np.ndarray) -> None:
        """Fold the estimated error into the nominal state (then the error is zero again)."""
        e = self.est
        e.pos_m = e.pos_m + dx[0:3]
        e.vel_m_s = e.vel_m_s + dx[3:6]
        e.quat = quat_normalize(quat_multiply(quat_from_rotvec(dx[6:9]), e.quat))
        self.gyro_bias = self.gyro_bias + dx[9:12]

    def _fuse(self, name: str, measured: np.ndarray, predicted: np.ndarray, rows: slice, r_var: float) -> bool:
        """A direct measurement of some of the error states (position, velocity or height)."""
        n = len(measured)
        H = np.zeros((n, 12))
        H[np.arange(n), np.arange(rows.start, rows.stop)] = 1.0
        innov = measured - predicted
        R = np.eye(n) * r_var
        S = H @ self.P @ H.T + R
        if name != "zero_vel" and not self._gate(name, innov, np.diag(S)):
            return False
        self._correct(H, innov, R, S)
        return True

    def _fuse_gps_pos(self, xy: np.ndarray) -> None:
        e = self.est
        if self._fuse("gps_pos", xy, e.pos_m[:2].copy(), slice(0, 2), self.r_gps_pos):
            self._gps_last_ok_s = self.time_s
        elif self.time_s - self._gps_last_ok_s > GPS_RETRY_S:
            # GPS has disagreed for too long: maybe WE are wrong (after a long dead-reckoning). Jump to it.
            e.pos_m = np.array([xy[0], xy[1], e.pos_m[2]])
            self.P[0:2, :] = 0.0
            self.P[:, 0:2] = 0.0
            self.P[0, 0] = self.P[1, 1] = self.r_gps_pos
            self.gps_resets += 1
            self._gps_last_ok_s = self.time_s

    def _fuse_compass(self, heading_rad: float) -> None:
        """Yaw measurement. yaw = atan2(R10, R00); a small world rotation dtheta changes it by H @ dtheta."""
        rot = quat_to_rotation_matrix(self.est.quat)
        r00, r10, r20 = rot[0, 0], rot[1, 0], rot[2, 0]
        den = r00 * r00 + r10 * r10
        if den < 1e-6:
            return  # pointing straight up or down: heading undefined
        H = np.zeros((1, 12))
        H[0, 6:9] = (-r00 * r20 / den, -r10 * r20 / den, 1.0)
        innov = np.array([math.remainder(heading_rad - quat_to_euler(self.est.quat)[2], 2 * math.pi)])
        R = np.array([[self.r_compass]])
        S = H @ self.P @ H.T + R
        if self._gate("compass", innov, np.diag(S)):
            self._correct(H, innov, R, S)

    def health(self) -> dict:
        """A compact report: uncertainty, gyro bias it has learnt, and how each sensor's readings were judged."""
        return {"sigma": {k: [round(x, 4) for x in v] for k, v in self.sigmas().items()},
                "gyro_bias_rad_s": [round(float(x), 5) for x in self.gyro_bias],
                "test_ratio": {k: round(v, 3) for k, v in self.test_ratio.items()},
                "rejected": dict(self.rejected), "accepted": dict(self.accepted), "gps_resets": self.gps_resets,
                "vel_resets": self.vel_resets}
