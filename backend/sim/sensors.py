"""Noisy sensors: what the flight controller can actually 'see'.

A real drone never knows its true position or tilt. It only has sensors, each imperfect:

  gyroscope      (250 Hz): spin rates. Little noise, but may have a constant BIAS (offset).
  accelerometer  (250 Hz): "specific force" = acceleration + gravity, felt in the body frame.
                           At rest it reads 9.81 m/s² straight UP, which shows which way is up.
  altimeter/baro  (50 Hz): height, quite noisy (± tens of cm).
  compass         (50 Hz): heading (yaw), a few degrees of noise.
  position/GPS    (10 Hz): horizontal position, noisy (± tens of cm) and slow.

All noise comes from the run's random generator, so the same seed gives the same noise.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field

import numpy as np

from .config import GRAVITY_M_S2
from .frames import quat_to_euler, quat_to_rotation_matrix
from .physics import State

IMU_EVERY_N_STEPS = 2  # 250 Hz (physics is 500 Hz)
BARO_EVERY_N_STEPS = 10  # 50 Hz
COMPASS_EVERY_N_STEPS = 10  # 50 Hz
GPS_EVERY_N_STEPS = 50  # 10 Hz

# Noise sizes (1 standard deviation) at noise_scale = 1. Roughly a hobby-grade flight controller.
GYRO_NOISE_RAD_S = 0.02
ACCEL_NOISE_M_S2 = 0.3
BARO_NOISE_M = 0.15
COMPASS_NOISE_RAD = math.radians(2.0)
GPS_NOISE_M = 0.3

SENSOR_LIMITS = {
    "noise_scale": (0.0, 5.0),
    "gyro_bias_deg_s": (-5.0, 5.0),
    "baro_trust": (0.01, 1.0),
    "gps_trust": (0.01, 1.0),
    "gps_noise_scale": (0.0, 5.0),
    "tilt_correction": (0.0, 5.0),
}


@dataclass(frozen=True)
class SensorConfig:
    """enabled=False means 'perfect sensors': the controller sees the true state (Phase 1 behaviour)."""

    enabled: bool = False
    noise_scale: float = 1.0  # multiplies every noise size above
    gyro_bias_deg_s: float = 0.0  # constant offset on the roll-axis gyro (shows estimator drift)
    baro_trust: float = 0.1  # estimator: how much each altimeter reading corrects the height (0–1)
    gps_trust: float = 0.2  # estimator: same for horizontal position
    gps_noise_scale: float = 1.0  # extra multiplier for GPS noise only (0 = perfect GPS, other sensors still noisy)
    # Estimator: how strongly the accelerometer corrects the tilt (1/s). Kept gentle on purpose: while a
    # multirotor accelerates, its accelerometer mostly feels its own thrust and "lies" about which way is down.
    tilt_correction: float = 0.2

    def to_dict(self) -> dict:
        return asdict(self)


def validate_sensors(s: SensorConfig) -> list[str]:
    problems = []
    for name, (lo, hi) in SENSOR_LIMITS.items():
        v = getattr(s, name)
        if not isinstance(v, (int, float)) or not math.isfinite(v) or not lo <= v <= hi:
            problems.append(f"sensors.{name} = {v!r} is outside the allowed range {lo}–{hi}.")
    return problems


@dataclass
class Readings:
    """The most recent reading from each sensor, and which ones are NEW this step."""

    gyro_rad_s: np.ndarray = field(default_factory=lambda: np.zeros(3))
    accel_m_s2: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, GRAVITY_M_S2]))
    baro_m: float = 0.0
    compass_rad: float = 0.0
    gps_xy_m: np.ndarray = field(default_factory=lambda: np.zeros(2))
    new_imu: bool = False
    new_baro: bool = False
    new_compass: bool = False
    new_gps: bool = False


class Sensors:
    def __init__(self, cfg: SensorConfig, rng: np.random.Generator):
        self.cfg = cfg
        self.rng = rng
        k = cfg.noise_scale
        self.gyro_sigma = GYRO_NOISE_RAD_S * k
        self.accel_sigma = ACCEL_NOISE_M_S2 * k
        self.baro_sigma = BARO_NOISE_M * k
        self.compass_sigma = COMPASS_NOISE_RAD * k
        self.gps_sigma = GPS_NOISE_M * k * cfg.gps_noise_scale
        self.gyro_bias = np.array([math.radians(cfg.gyro_bias_deg_s), 0.0, 0.0])
        self.readings = Readings()
        self.step_count = 0

    def sample(self, truth: State, accel_world_m_s2: np.ndarray) -> Readings:
        """Take whichever readings are due this physics step (each sensor has its own rate)."""
        r = self.readings
        n = self.step_count
        self.step_count += 1
        r.new_imu = n % IMU_EVERY_N_STEPS == 0
        r.new_baro = n % BARO_EVERY_N_STEPS == 0
        r.new_compass = n % COMPASS_EVERY_N_STEPS == 0
        r.new_gps = n % GPS_EVERY_N_STEPS == 0

        if r.new_imu:
            r.gyro_rad_s = truth.rate_rad_s + self.gyro_bias + self.gyro_sigma * self.rng.standard_normal(3)
            # Specific force in the body frame: R^T (a + g·up). At rest this is (0, 0, +9.81).
            rot = quat_to_rotation_matrix(truth.quat)
            specific_force_world = accel_world_m_s2 + np.array([0.0, 0.0, GRAVITY_M_S2])
            r.accel_m_s2 = rot.T @ specific_force_world + self.accel_sigma * self.rng.standard_normal(3)
        if r.new_baro:
            r.baro_m = float(truth.pos_m[2] + self.baro_sigma * self.rng.standard_normal())
        if r.new_compass:
            yaw = quat_to_euler(truth.quat)[2]
            r.compass_rad = math.remainder(yaw + self.compass_sigma * self.rng.standard_normal(), 2 * math.pi)
        if r.new_gps:
            r.gps_xy_m = truth.pos_m[:2] + self.gps_sigma * self.rng.standard_normal(2)
        return r
