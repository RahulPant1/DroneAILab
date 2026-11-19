"""Building blocks for guided experiments.

An experiment = objective + a 3-line concept + a few sliders + a headless run + plots
+ a rule-based "what happened" (no AI) + "try next" suggestions.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field
from typing import Callable

import numpy as np

from backend.sim.config import ConfigError
from backend.sim.runner import RunResult

RAD_TO_DEG = 180.0 / math.pi


class ExperimentError(ValueError):
    """Bad experiment input. The message is plain English."""


@dataclass(frozen=True)
class Param:
    """A slider (or a choice when `options` is given)."""

    id: str
    label: str
    unit: str
    default: float | str
    help: str
    min: float = 0.0
    max: float = 1.0
    step: float = 0.1
    options: tuple[tuple[str, str], ...] = ()  # (value, label) pairs for a choice

    def check(self, value) -> float | str:
        if self.options:
            allowed = [v for v, _ in self.options]
            if value not in allowed:
                raise ExperimentError(f"{self.label}: choose one of {', '.join(allowed)}.")
            return value
        try:
            number = float(value)
        except (TypeError, ValueError):
            raise ExperimentError(f"{self.label} must be a number.") from None
        if not math.isfinite(number) or not self.min <= number <= self.max:
            raise ExperimentError(f"{self.label} = {value} is outside the allowed range {self.min}–{self.max} {self.unit}.")
        return number


@dataclass(frozen=True)
class Series:
    channel: str
    label: str
    scale: float = 1.0  # e.g. 57.3 to show radians in degrees
    style: str = "line"  # line | dashed | points


@dataclass(frozen=True)
class Plot:
    id: str
    title: str
    y_label: str  # includes the unit, e.g. "altitude (m)"
    how_to_read: str
    series: tuple[Series, ...]
    kind: str = "time"  # "time" = signals vs time; "map" = top-down x/y view (series[0] = x, series[1] = y)
    extra: dict = field(default_factory=dict)  # map data: obstacles, waypoints, planned path


@dataclass(frozen=True)
class Metric:
    key: str
    label: str
    unit: str = ""
    digits: int = 2


@dataclass(frozen=True)
class Experiment:
    id: str
    number: int
    title: str
    objective: str
    concept: tuple[str, str, str]
    params: tuple[Param, ...]
    key_metrics: tuple[Metric, ...]
    learn_slug: str
    runner: Callable[[dict, int], RunResult] = field(repr=False, compare=False)

    def check_params(self, raw: dict | None) -> dict:
        """Fill in defaults and reject unknown or out-of-range values."""
        raw = dict(raw or {})
        known = {p.id for p in self.params}
        unknown = set(raw) - known
        if unknown:
            raise ExperimentError(f"Unknown setting(s): {', '.join(sorted(unknown))}.")
        return {p.id: p.check(raw.get(p.id, p.default)) for p in self.params}

    def run(self, raw_params: dict | None = None, seed: int = 0) -> RunResult:
        params = self.check_params(raw_params)
        if not isinstance(seed, int) or not 0 <= seed <= 2**31:
            raise ExperimentError("seed must be a whole number between 0 and 2147483648.")
        try:
            result = self.runner(params, seed)
        except ConfigError as exc:
            raise ExperimentError(str(exc)) from None
        result.run_type = self.id
        result.scenario = {"experiment_id": self.id, "params": params, **result.scenario}
        return result

    def to_dict(self) -> dict:
        d = asdict(self)
        d.pop("runner", None)
        return d


# ---- helpers used by the experiment modules ----


def add_channel(result: RunResult, name: str, values) -> None:
    """Add a derived channel (same length as t_s), rounded like the rest of the telemetry."""
    n = len(result.telemetry["t_s"])
    arr = np.broadcast_to(np.asarray(values, dtype=float), (n,))
    result.telemetry[name] = [round(float(v), 5) for v in arr]


def ch(result: RunResult, name: str) -> np.ndarray:
    return np.asarray(result.telemetry[name], dtype=float)


def plots_to_dicts(*plots: Plot) -> list[dict]:
    return [asdict(p) for p in plots]


def try_next(text: str, **params) -> dict:
    """A suggestion; if params are given, the UI offers a one-click 'apply'."""
    return {"text": text, "params": params}


def motor_series() -> tuple[Series, ...]:
    return (
        Series("motor1_pct", "M1 front-right"),
        Series("motor2_pct", "M2 rear-left"),
        Series("motor3_pct", "M3 front-left"),
        Series("motor4_pct", "M4 rear-right"),
    )


def fmt(value, digits: int = 2) -> str:
    return "n/a" if value is None else f"{value:.{digits}f}"
