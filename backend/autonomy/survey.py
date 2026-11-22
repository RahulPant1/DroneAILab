"""Survey missions: photograph a whole field from above, like a mapping or crop-scouting drone.

1. Camera footprint. A camera looking straight down from height h sees a rectangle on the ground:
     across = 2 h tan(HFOV / 2),   along = across / aspect ratio.
   Ground sample distance (GSD) = across / image width in pixels: how many cm of ground one pixel covers.
2. Lawnmower pattern. Fly parallel lines across the field. Neighbouring photos must OVERLAP (side overlap
   between lines, front overlap along a line) so mapping software can stitch them and nothing is missed:
     line spacing  = across × (1 − side overlap),   photo spacing = along × (1 − front overlap).
3. Camera trigger. Like ArduPilot's CAM_TRIGG_DIST: on a survey line, take a photo every `photo spacing`
   metres travelled, measured by the drone's OWN position estimate. The picture lands where the drone REALLY
   is, so GPS errors and wind push photos off their planned spots.
4. Coverage. Split the field into small cells; a cell is covered if at least one photo's footprint contains it.

Assumption: the camera sits on a gimbal that keeps it pointing straight down and lined up with the survey lines.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .site import polygon_area

MAX_POLYGON_POINTS = 12
MAX_LINES = 40
MAX_PHOTOS = 2000
MAX_COVERAGE_CELLS = 40_000
GAP_SAMPLE = 400  # at most this many uncovered cells are sent for drawing

Point = tuple[float, float]


class SurveyError(ValueError):
    """A survey that can't be planned. The message is plain English."""


@dataclass(frozen=True)
class Camera:
    """A typical small-drone mapping camera: 70° across, 4:3 picture, 4000 pixels wide."""

    hfov_deg: float = 70.0
    aspect: float = 4.0 / 3.0  # long side across the flight line
    width_px: int = 4000

    def footprint_m(self, alt_m: float) -> tuple[float, float]:
        """(across, along) size of the patch of ground one photo covers, from height alt_m."""
        across = 2.0 * max(alt_m, 0.0) * math.tan(math.radians(self.hfov_deg) / 2.0)
        return across, across / self.aspect

    def gsd_cm(self, alt_m: float) -> float:
        return self.footprint_m(alt_m)[0] / self.width_px * 100.0


CAMERA = Camera()


def check_polygon(poly) -> list[Point]:
    try:
        pts = [(float(x), float(y)) for x, y in poly]
    except (TypeError, ValueError):
        raise SurveyError("A field is a list of [x, y] corners.") from None
    if not 3 <= len(pts) <= MAX_POLYGON_POINTS:
        raise SurveyError(f"A field needs 3 to {MAX_POLYGON_POINTS} corners.")
    if any(not math.isfinite(v) for p in pts for v in p):
        raise SurveyError("Field corners must be finite numbers.")
    signs = set()
    n = len(pts)
    for i in range(n):
        (ax, ay), (bx, by), (cx, cy) = pts[i], pts[(i + 1) % n], pts[(i + 2) % n]
        cross = (bx - ax) * (cy - by) - (by - ay) * (cx - bx)
        if abs(cross) > 1e-9:
            signs.add(cross > 0)
    if len(signs) != 1:
        raise SurveyError("The field must be convex (no inward corners) for the lawnmower planner.")
    if polygon_area(pts) < 4.0:
        raise SurveyError("The field is too small to survey.")
    return pts


def longest_edge_angle(poly: list[Point]) -> float:
    """Direction (rad) of the field's longest side: flying along it means fewer, longer lines (fewer turns)."""
    n = len(poly)
    i = max(range(n), key=lambda k: math.dist(poly[k], poly[(k + 1) % n]))
    (ax, ay), (bx, by) = poly[i], poly[(i + 1) % n]
    return math.atan2(by - ay, bx - ax)


def _chord(poly: list[Point], u: np.ndarray, v: np.ndarray, c: float) -> tuple[float, float] | None:
    """Where the line {p : p·v = c} crosses the polygon, as a range of p·u (None if it misses)."""
    hits = []
    n = len(poly)
    for i in range(n):
        p, q = np.array(poly[i]), np.array(poly[(i + 1) % n])
        vp, vq = float(p @ v), float(q @ v)
        if (vp - c) * (vq - c) <= 0 and abs(vq - vp) > 1e-12:
            f = (c - vp) / (vq - vp)
            hits.append(float((p + f * (q - p)) @ u))
        elif abs(vp - c) < 1e-12:
            hits.append(float(p @ u))
    return (min(hits), max(hits)) if hits else None


@dataclass
class SurveyPlan:
    lines: list[tuple[Point, Point]]  # each survey line, in flying order and direction
    angle_rad: float  # direction of the lines
    line_spacing_m: float
    photo_spacing_m: float
    footprint_m: tuple[float, float]  # (across, along)
    gsd_cm: float
    alt_m: float
    side_overlap: float
    front_overlap: float

    @property
    def line_length_m(self) -> float:
        return sum(math.dist(a, b) for a, b in self.lines)

    def photos_estimate(self) -> int:
        return sum(int(math.dist(a, b) // self.photo_spacing_m) + 2 for a, b in self.lines)

    def to_dict(self) -> dict:
        return {
            "lines": [[[round(a[0], 2), round(a[1], 2)], [round(b[0], 2), round(b[1], 2)]] for a, b in self.lines],
            "angle_deg": round(math.degrees(self.angle_rad), 1),
            "line_spacing_m": round(self.line_spacing_m, 2),
            "photo_spacing_m": round(self.photo_spacing_m, 2),
            "footprint_m": [round(v, 2) for v in self.footprint_m],
            "gsd_cm": round(self.gsd_cm, 2),
            "alt_m": self.alt_m,
            "side_overlap_pct": round(self.side_overlap * 100),
            "front_overlap_pct": round(self.front_overlap * 100),
            "photos_estimate": self.photos_estimate(),
        }


def plan_lawnmower(poly, alt_m: float, side_overlap: float = 0.7, front_overlap: float = 0.75,
                   angle_rad: float | None = None, start: Point = (0.0, 0.0), camera: Camera = CAMERA) -> SurveyPlan:
    """Parallel lines covering a convex field, flown back and forth, starting from the corner nearest `start`."""
    poly = check_polygon(poly)
    if not 0.0 <= side_overlap <= 0.9 or not 0.0 <= front_overlap <= 0.9:
        raise SurveyError("Overlaps must be between 0 and 90 %.")
    if alt_m <= 0:
        raise SurveyError("Survey altitude must be above the ground.")
    across, along = camera.footprint_m(alt_m)
    angle = longest_edge_angle(poly) if angle_rad is None else angle_rad
    u = np.array([math.cos(angle), math.sin(angle)])  # along the lines
    v = np.array([-math.sin(angle), math.cos(angle)])  # across them
    vs = [float(np.array(p) @ v) for p in poly]
    vmin, vmax = min(vs), max(vs)
    width = vmax - vmin
    spacing = across * (1 - side_overlap)  # exactly the overlap asked for
    n = max(1, math.ceil(width / spacing - 1e-9))
    if n > MAX_LINES:
        raise SurveyError(f"That would need {n} survey lines (max {MAX_LINES}): fly higher or use less overlap.")
    vmid = (vmin + vmax) / 2
    lines = []
    for i in range(n):
        c = vmid + (i - (n - 1) / 2) * spacing  # centred on the field; the outer photos hang over its edges a little
        # The photos are `across` wide, so the line must run as long as the field is anywhere in its strip.
        lo, hi = math.inf, -math.inf
        for dc in (-spacing / 2, 0.0, spacing / 2):
            ch = _chord(poly, u, v, min(max(c + dc, vmin), vmax))
            if ch:
                lo, hi = min(lo, ch[0]), max(hi, ch[1])
        a, b = lo * u + c * v, hi * u + c * v
        lines.append(((float(a[0]), float(a[1])), (float(b[0]), float(b[1]))))

    # Fly them back and forth (boustrophedon); pick the order and direction that start nearest `start`.
    best = None
    for order in (lines, lines[::-1]):
        for flip_first in (False, True):
            seq = []
            for k, (a, b) in enumerate(order):
                seq.append((b, a) if (k % 2 == 0) == flip_first else (a, b))
            d = math.dist(start, seq[0][0])
            if best is None or d < best[0]:
                best = (d, seq)
    return SurveyPlan(best[1], angle, spacing, along * (1 - front_overlap), (across, along), camera.gsd_cm(alt_m),
                      alt_m, side_overlap, front_overlap)


class SurveyCamera:
    """Takes the photos during the flight, CAM_TRIGG_DIST style.

    update() is called every step with which survey line the drone is flying (None = not on a line). On a line:
    one photo when the line starts, one every `spacing` metres the drone THINKS it has moved, and a last one
    at the end of the line. Each photo is recorded where the camera REALLY was, at its real height."""

    def __init__(self, photo_spacing_m: float, angle_by_line: dict[int, float]):
        self.spacing = photo_spacing_m
        self.angle_by_line = angle_by_line
        self.photos: list[dict] = []
        self.line: int | None = None
        self._last_est: tuple[float, float] | None = None

    def _shoot(self, t: float, est_xy, truth_xyz) -> None:
        if len(self.photos) >= MAX_PHOTOS:
            return
        self.photos.append({"t_s": round(t, 2), "x_m": round(float(truth_xyz[0]), 3), "y_m": round(float(truth_xyz[1]), 3),
                            "alt_m": round(float(truth_xyz[2]), 3), "angle_rad": self.angle_by_line.get(self.line, 0.0),
                            "line": self.line,
                            # the photo's GEOTAG: where the drone THOUGHT it was (what mapping software gets)
                            "est_x_m": round(float(est_xy[0]), 3), "est_y_m": round(float(est_xy[1]), 3)})
        self._last_est = (float(est_xy[0]), float(est_xy[1]))

    def update(self, t: float, line: int | None, est_xy, truth_xyz) -> None:
        moved = math.dist(self._last_est, (float(est_xy[0]), float(est_xy[1]))) if self._last_est else math.inf
        if line != self.line:
            if self.line is not None and moved > 0.3 * self.spacing:
                self._shoot(t, est_xy, truth_xyz)  # end of the line
            self.line = line
            if line is not None:
                self._shoot(t, est_xy, truth_xyz)  # start of the line
            return
        if line is not None and moved >= self.spacing:
            self._shoot(t, est_xy, truth_xyz)


def _inside_convex(poly: list[Point], pts: np.ndarray) -> np.ndarray:
    sign = None
    inside = np.ones(len(pts), dtype=bool)
    n = len(poly)
    area2 = sum(poly[i][0] * poly[(i + 1) % n][1] - poly[(i + 1) % n][0] * poly[i][1] for i in range(n))
    sign = 1.0 if area2 > 0 else -1.0
    for i in range(n):
        (ax, ay), (bx, by) = poly[i], poly[(i + 1) % n]
        cross = (bx - ax) * (pts[:, 1] - ay) - (by - ay) * (pts[:, 0] - ax)
        inside &= sign * cross >= -1e-9
    return inside


def coverage(poly, photos: list[dict], camera: Camera = CAMERA) -> dict:
    """How much of the field the photos really cover. Each photo covers its footprint at the height it was
    taken from, centred where the camera really was."""
    poly = check_polygon(poly)
    xs, ys = [p[0] for p in poly], [p[1] for p in poly]
    area = polygon_area(poly)
    cell = max(0.25, math.sqrt(area / MAX_COVERAGE_CELLS))
    gx = np.arange(min(xs) + cell / 2, max(xs), cell)
    gy = np.arange(min(ys) + cell / 2, max(ys), cell)
    X, Y = np.meshgrid(gx, gy)
    pts = np.column_stack([X.ravel(), Y.ravel()])
    pts = pts[_inside_convex(poly, pts)]
    count = np.zeros(len(pts), dtype=np.int32)
    for ph in photos:
        across, along = camera.footprint_m(ph["alt_m"])
        a = ph["angle_rad"]
        rel = pts - np.array([ph["x_m"], ph["y_m"]])
        du = rel[:, 0] * math.cos(a) + rel[:, 1] * math.sin(a)
        dv = -rel[:, 0] * math.sin(a) + rel[:, 1] * math.cos(a)
        count += (np.abs(du) <= along / 2) & (np.abs(dv) <= across / 2)
    n = max(len(pts), 1)
    gaps = pts[count == 0]
    if len(gaps) > GAP_SAMPLE:
        gaps = gaps[np.linspace(0, len(gaps) - 1, GAP_SAMPLE).astype(int)]
    return {
        "coverage_pct": round(100.0 * float(np.sum(count >= 1)) / n, 2),
        "seen_twice_pct": round(100.0 * float(np.sum(count >= 2)) / n, 2),  # stitching needs every spot in 2+ photos
        "cell_m": round(cell, 3),
        "cells": int(len(pts)),
        "gaps": [[round(float(x), 2), round(float(y), 2)] for x, y in gaps],
    }
