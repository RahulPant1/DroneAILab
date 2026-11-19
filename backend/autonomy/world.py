"""The mission world: box-shaped obstacles, and how close the drone is to them.

Everything is in the ENU world frame (x east, y north, z up), in metres. Obstacles are upright boxes
standing on the ground. The drone is treated as a flat disc of radius DRONE_RADIUS_M (arms + props).
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass

import numpy as np

DRONE_RADIUS_M = 0.3  # centre to propeller tip, roughly, for the default drone
SAFETY_MARGIN_M = 0.4  # extra gap the planner keeps between the drone's edge and any obstacle


@dataclass(frozen=True)
class Box:
    """An upright box on the ground: x_min..x_max (east), y_min..y_max (north), 0..height_m (up)."""

    x_min: float
    x_max: float
    y_min: float
    y_max: float
    height_m: float = 4.0

    def distance_2d(self, x: float, y: float) -> float:
        """Horizontal distance from point (x, y) to the box's outline (0 if the point is inside)."""
        dx = max(self.x_min - x, 0.0, x - self.x_max)
        dy = max(self.y_min - y, 0.0, y - self.y_max)
        return math.hypot(dx, dy)

    def closest_point(self, x: float, y: float) -> tuple[float, float]:
        return min(max(x, self.x_min), self.x_max), min(max(y, self.y_min), self.y_max)

    def blocks_altitude(self, z_m: float) -> bool:
        """A box only matters if the drone flies lower than its top."""
        return z_m < self.height_m

    def to_dict(self) -> dict:
        return asdict(self)


def clearance_m(boxes: list[Box], x: float, y: float, z: float) -> float:
    """Gap between the drone's EDGE and the nearest obstacle it could hit (negative = touching it).
    Returns +inf if no obstacle is tall enough to matter at this height."""
    gaps = [b.distance_2d(x, y) - DRONE_RADIUS_M for b in boxes if b.blocks_altitude(z)]
    return min(gaps) if gaps else math.inf


def clearance_series(boxes: list[Box], x: np.ndarray, y: np.ndarray, z: np.ndarray) -> np.ndarray:
    """Same as clearance_m, for whole arrays at once (used for metrics after a run)."""
    out = np.full(len(x), np.inf)
    for b in boxes:
        dx = np.maximum.reduce([b.x_min - x, np.zeros_like(x), x - b.x_max])
        dy = np.maximum.reduce([b.y_min - y, np.zeros_like(y), y - b.y_max])
        gap = np.hypot(dx, dy) - DRONE_RADIUS_M
        out = np.where(z < b.height_m, np.minimum(out, gap), out)
    return out
