"""The air around the drone: steady wind plus random gusts.

Wind matters because drag depends on the drone's speed RELATIVE TO THE AIR. A drone
hovering still in a 5 m/s wind feels the same drag as one flying at 5 m/s in calm air.

Gusts are "smoothly random": each gust component wanders around zero, with a typical
size (gust_std_m_s) and a typical duration (gust_time_constant_s). The randomness comes
from the run's seed, so the same seed always gives the same gusts.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass

import numpy as np

GUST_UPDATE_EVERY_N_STEPS = 10  # gusts change at 50 Hz, plenty for slow air movements
VERTICAL_GUST_SHARE = 0.3  # vertical gusts are weaker than horizontal ones

ENV_LIMITS = {
    "wind_speed_m_s": (0.0, 15.0),
    "wind_direction_deg": (-360.0, 360.0),
    "gust_std_m_s": (0.0, 5.0),
    "gust_time_constant_s": (0.2, 10.0),
    "wind_start_s": (0.0, 120.0),
    "unmodeled_payload_kg": (0.0, 1.0),
}


@dataclass(frozen=True)
class EnvConfig:
    """Wind settings. Direction is where the wind blows TOWARD: 0° = east, 90° = north."""

    wind_speed_m_s: float = 0.0
    wind_direction_deg: float = 0.0
    gust_std_m_s: float = 0.0
    gust_time_constant_s: float = 2.0
    wind_start_s: float = 0.0  # wind (and gusts) switch on at this time
    unmodeled_payload_kg: float = 0.0  # extra mass the CONTROLLER doesn't know about

    @property
    def active(self) -> bool:
        return self.wind_speed_m_s > 0 or self.gust_std_m_s > 0

    def steady_wind_m_s(self) -> np.ndarray:
        d = math.radians(self.wind_direction_deg)
        return np.array([math.cos(d), math.sin(d), 0.0]) * self.wind_speed_m_s

    def to_dict(self) -> dict:
        return asdict(self)


def validate_env(env: EnvConfig) -> list[str]:
    problems = []
    for name, (lo, hi) in ENV_LIMITS.items():
        v = getattr(env, name)
        if not isinstance(v, (int, float)) or not math.isfinite(v) or not lo <= v <= hi:
            problems.append(f"{name} = {v!r} is outside the allowed range {lo}–{hi}.")
    return problems


class Wind:
    """Produces the wind velocity (world frame, m/s) at each physics step."""

    def __init__(self, env: EnvConfig, rng: np.random.Generator, dt_s: float):
        self.env = env
        self.rng = rng
        self.steady = env.steady_wind_m_s()
        self.gust = np.zeros(3)
        self.step_count = 0
        update_dt = dt_s * GUST_UPDATE_EVERY_N_STEPS
        # Each update keeps part of the old gust and adds fresh randomness ("first-order filtered noise").
        self.keep = math.exp(-update_dt / env.gust_time_constant_s)
        self.fresh = math.sqrt(1.0 - self.keep**2)
        self.gust_scale = env.gust_std_m_s * np.array([1.0, 1.0, VERTICAL_GUST_SHARE])
        self.current = np.zeros(3)

    def step(self, time_s: float) -> np.ndarray:
        """Wind velocity for this step (zero before wind_start_s)."""
        if self.env.gust_std_m_s > 0 and self.step_count % GUST_UPDATE_EVERY_N_STEPS == 0:
            self.gust = self.keep * self.gust + self.fresh * self.gust_scale * self.rng.standard_normal(3)
        self.step_count += 1
        self.current = self.steady + self.gust if time_s >= self.env.wind_start_s else np.zeros(3)
        return self.current
