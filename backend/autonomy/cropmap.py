"""Crop-health map: turn a survey's photos into one map of the field and find the sick patches.

This is the job farm-survey drones are paid for: fly → map → finding → action.

1. NDVI camera. Healthy plants reflect a lot of near-infrared (NIR) light and absorb red light. A multispectral
   camera measures both; NDVI = (NIR − red) / (NIR + red). Healthy crop ≈ 0.8, stressed ≈ 0.3-0.5, bare soil ≈ 0.15.
   Each field here hides a few stressed patches (too dry, waterlogged, a pest), which is the "ground truth".
2. Photos. Each survey photo samples NDVI on a 0.25 m grid over its footprint, where the camera REALLY was,
   plus sensor noise.
3. Stitching (an orthomosaic). Mapping software only knows each photo's GEOTAG: where the drone THOUGHT it was.
   Every photo is pasted onto the map at its geotag. With GPS errors the photos land a metre or so off, so the
   map (and every patch on it) is shifted and a little smeared. Professional surveys fix that with RTK GPS
   (centimetre accuracy) or ground control points measured on the ground, plus image matching between photos.
4. Findings. Smooth the mosaic, mark cells with NDVI below 0.6, group touching cells into patches, and report
   each patch bigger than 2 m²: where, how big, how bad.
"""

from __future__ import annotations

import base64
import math
from collections import deque
from dataclasses import dataclass

import numpy as np

from .survey import CAMERA, Camera, check_polygon

CELL_M = 0.25  # map resolution (and the photos' processed resolution)
MARGIN_M = 4.0
PIXEL_NOISE = 0.02
STRESS_NDVI = 0.6  # below this a cell counts as stressed
MIN_PATCH_M2 = 2.0
HEADLAND_M = 2.0  # ignore this strip along the field's edge (tractor tracks; and a shifted map puts soil there)
MATCH_RADIUS_M = 4.0  # a finding within this distance of a true patch counts as finding it
SOIL_NDVI = 0.15
HEALTHY_NDVI = 0.82


@dataclass(frozen=True)
class Patch:
    """A stressed area: an irregular blob around (x, y), `radius_m` on average; NDVI drops by `severity` in its middle."""

    x_m: float
    y_m: float
    radius_m: float
    severity: float
    cause: str
    wobble: float = 0.25  # how irregular the outline is
    phase: float = 0.0


PATCHES: dict[str, tuple[Patch, ...]] = {
    "north field": (Patch(41.0, 33.0, 4.0, 0.45, "dry patch (irrigation not reaching it)", phase=0.7),
                    Patch(26.0, 21.0, 2.3, 0.35, "pest damage", phase=2.1)),
    "east field": (Patch(63.0, -33.0, 3.5, 0.5, "waterlogged dip", phase=1.3),),
    "orchard": (Patch(-33.0, 25.0, 2.2, 0.4, "diseased trees", phase=0.2),),
}


def _inside(poly: list[tuple[float, float]], x: np.ndarray, y: np.ndarray) -> np.ndarray:
    n = len(poly)
    area2 = sum(poly[i][0] * poly[(i + 1) % n][1] - poly[(i + 1) % n][0] * poly[i][1] for i in range(n))
    sign = 1.0 if area2 > 0 else -1.0
    inside = np.ones(np.shape(x), dtype=bool)
    for i in range(n):
        (ax, ay), (bx, by) = poly[i], poly[(i + 1) % n]
        inside &= sign * ((bx - ax) * (y - ay) - (by - ay) * (x - ax)) >= 0
    return inside


def ndvi_truth(x: np.ndarray, y: np.ndarray, poly: list[tuple[float, float]], patches) -> np.ndarray:
    """The real NDVI of the ground at (x, y): crop inside the field (with natural variation), soil outside."""
    crop = (HEALTHY_NDVI + 0.03 * np.sin(2 * np.pi * (0.8 * x + 0.6 * y) / 2.3) + 0.025 * np.sin(2 * np.pi * (-0.5 * x + 0.87 * y) / 3.7)
            + 0.03 * np.sin(2 * np.pi * (0.3 * x - 0.95 * y) / 5.9) + 0.02 * np.sin(2 * np.pi * x / 9.1 + 1.0))
    for p in patches:
        dx, dy = x - p.x_m, y - p.y_m
        ang = np.arctan2(dy, dx)
        r = p.radius_m * (1 + p.wobble * np.sin(3 * ang + p.phase) + 0.5 * p.wobble * np.sin(5 * ang + 2 * p.phase))
        d = np.hypot(dx, dy) / np.maximum(r, 0.1)
        crop = crop - p.severity * np.clip(1.3 - d, 0.0, 1.0)  # full strength in the middle, fading out to 1.3 radii
    soil = SOIL_NDVI + 0.03 * np.sin(1.3 * x) * np.cos(0.9 * y)
    return np.where(_inside(poly, x, y), np.clip(crop, 0.0, 0.95), soil)


def truth_area_m2(poly, patch: Patch) -> float:
    """Area of a true patch below the stress threshold (counted on the map grid)."""
    g = np.arange(-2.0 * patch.radius_m, 2.0 * patch.radius_m, CELL_M / 2)
    X, Y = np.meshgrid(patch.x_m + g, patch.y_m + g)
    v = ndvi_truth(X, Y, poly, [patch])
    return float(np.sum((v < STRESS_NDVI) & _inside(poly, X, Y))) * (CELL_M / 2) ** 2


def photo_pixels(ph: dict, camera: Camera = CAMERA) -> tuple[np.ndarray, np.ndarray]:
    """Pixel offsets (along, across) of a processed photo, on the CELL_M grid, from its centre."""
    across, along = camera.footprint_m(ph["alt_m"])
    u = np.arange(-along / 2 + CELL_M / 2, along / 2, CELL_M)
    v = np.arange(-across / 2 + CELL_M / 2, across / 2, CELL_M)
    U, V = np.meshgrid(u, v)
    return U.ravel(), V.ravel()


def _to_world(cx: float, cy: float, angle: float, u: np.ndarray, v: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    c, s = math.cos(angle), math.sin(angle)
    return cx + u * c - v * s, cy + u * s + v * c


def describe_location(poly, x: float, y: float) -> str:
    """'north-east corner', 'middle', ... of the field's bounding box."""
    xs, ys = [p[0] for p in poly], [p[1] for p in poly]
    fx = (x - min(xs)) / max(max(xs) - min(xs), 1e-6)
    fy = (y - min(ys)) / max(max(ys) - min(ys), 1e-6)
    ns = "north" if fy > 0.66 else "south" if fy < 0.34 else ""
    ew = "east" if fx > 0.66 else "west" if fx < 0.34 else ""
    if ns and ew:
        return f"{ns}-{ew} corner"
    if ns or ew:
        return f"{ns or ew} side"
    return "middle"


def _edge_distance(poly, x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """Distance from each point inside a convex polygon to its nearest edge (negative outside)."""
    n = len(poly)
    area2 = sum(poly[i][0] * poly[(i + 1) % n][1] - poly[(i + 1) % n][0] * poly[i][1] for i in range(n))
    sign = 1.0 if area2 > 0 else -1.0
    d = np.full(np.shape(x), np.inf)
    for i in range(n):
        (ax, ay), (bx, by) = poly[i], poly[(i + 1) % n]
        L = math.hypot(bx - ax, by - ay)
        d = np.minimum(d, sign * ((bx - ax) * (y - ay) - (by - ay) * (x - ax)) / L)
    return d


def _label(mask: np.ndarray) -> list[list[tuple[int, int]]]:
    """Groups of touching True cells (4-neighbours)."""
    seen = np.zeros_like(mask)
    groups = []
    for i, j in zip(*np.nonzero(mask)):
        if seen[i, j]:
            continue
        q, cells = deque([(i, j)]), []
        seen[i, j] = True
        while q:
            a, b = q.popleft()
            cells.append((a, b))
            for da, db in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                na, nb = a + da, b + db
                if 0 <= na < mask.shape[0] and 0 <= nb < mask.shape[1] and mask[na, nb] and not seen[na, nb]:
                    seen[na, nb] = True
                    q.append((na, nb))
        groups.append(cells)
    return groups


def build_map(field_name: str, polygon, photos: list[dict], seed: int = 0,
              camera: Camera = CAMERA) -> dict:
    """Stitch a field's survey photos into an NDVI map and find the stressed patches. Returns a JSON-friendly dict."""
    poly = check_polygon(polygon)
    patches = PATCHES.get(field_name, ())
    rng = np.random.default_rng(seed + 777)
    xs, ys = [p[0] for p in poly], [p[1] for p in poly]
    x0, y0 = min(xs) - MARGIN_M, min(ys) - MARGIN_M
    nx = int(math.ceil((max(xs) + MARGIN_M - x0) / CELL_M))
    ny = int(math.ceil((max(ys) + MARGIN_M - y0) / CELL_M))
    total = np.zeros((ny, nx))
    count = np.zeros((ny, nx))
    tag_err = []
    for ph in photos:
        u, v = photo_pixels(ph, camera)
        # What the camera saw: the ground around where it REALLY was.
        wx, wy = _to_world(ph["x_m"], ph["y_m"], ph["angle_rad"], u, v)
        values = ndvi_truth(wx, wy, poly, patches) + PIXEL_NOISE * rng.standard_normal(len(u))
        # Where mapping software puts it: around its GEOTAG.
        tx, ty = ph.get("est_x_m", ph["x_m"]), ph.get("est_y_m", ph["y_m"])
        tag_err.append(math.hypot(tx - ph["x_m"], ty - ph["y_m"]))
        gx, gy = _to_world(tx, ty, ph["angle_rad"], u, v)
        ix = np.round((gx - x0) / CELL_M - 0.5).astype(int)
        iy = np.round((gy - y0) / CELL_M - 0.5).astype(int)
        ok = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
        np.add.at(total, (iy[ok], ix[ok]), values[ok])
        np.add.at(count, (iy[ok], ix[ok]), 1)

    have = count > 0
    mosaic = np.divide(total, count, out=np.full_like(total, np.nan), where=have)
    # Smooth (3 × 3 mean of the cells that have data) so single noisy cells don't count as patches.
    filled = np.nan_to_num(mosaic)
    pad_v, pad_c = np.pad(filled, 1), np.pad(have.astype(float), 1)
    acc_v = sum(pad_v[1 + a:1 + a + ny, 1 + b:1 + b + nx] for a in (-1, 0, 1) for b in (-1, 0, 1))
    acc_c = sum(pad_c[1 + a:1 + a + ny, 1 + b:1 + b + nx] for a in (-1, 0, 1) for b in (-1, 0, 1))
    smooth = np.divide(acc_v, acc_c, out=np.full_like(acc_v, np.nan), where=acc_c > 0)
    cx_grid = x0 + (np.arange(nx) + 0.5) * CELL_M
    cy_grid = y0 + (np.arange(ny) + 0.5) * CELL_M
    x_lo, y_lo = x0, y0
    GX, GY = np.meshgrid(cx_grid, cy_grid)
    in_field = _inside(poly, GX, GY)
    core = _edge_distance(poly, GX, GY) > HEADLAND_M
    stressed = core & have & (np.nan_to_num(smooth, nan=1.0) < STRESS_NDVI)
    findings = []
    for cells in _label(stressed):
        area = len(cells) * CELL_M * CELL_M
        if area < MIN_PATCH_M2:
            continue
        rows, cols = np.array([c[0] for c in cells]), np.array([c[1] for c in cells])
        fx, fy = float(cx_grid[cols].mean()), float(cy_grid[rows].mean())
        findings.append({"x_m": round(fx, 2), "y_m": round(fy, 2), "area_m2": round(area, 1),
                         "mean_ndvi": round(float(np.nanmean(mosaic[rows, cols])), 2),
                         "min_ndvi": round(float(np.nanmin(smooth[rows, cols])), 2),
                         "where": describe_location(poly, fx, fy),
                         "box": [int(cols.min()), ny - 1 - int(rows.max()), int(cols.max()), ny - 1 - int(rows.min())]})  # image: north up
    findings.sort(key=lambda f: -f["area_m2"])
    # Compare with the truth.
    truth = []
    for p in patches:
        near = [f for f in findings if math.hypot(f["x_m"] - p.x_m, f["y_m"] - p.y_m) < MATCH_RADIUS_M]
        f = min(near, key=lambda f: math.hypot(f["x_m"] - p.x_m, f["y_m"] - p.y_m)) if near else None
        truth.append({"x_m": p.x_m, "y_m": p.y_m, "cause": p.cause, "area_m2": round(truth_area_m2(poly, p), 1),
                      "found": f is not None, "position_error_m": round(math.hypot(f["x_m"] - p.x_m, f["y_m"] - p.y_m), 2) if f else None,
                      "found_area_m2": f["area_m2"] if f else None})
    matched = {id(f) for p in truth for f in findings if p["found"] and math.hypot(f["x_m"] - p["x_m"], f["y_m"] - p["y_m"]) < MATCH_RADIUS_M}
    covered = int(np.sum(in_field & have))
    field_cells = int(np.sum(in_field))
    field_ndvi = float(np.nanmean(np.where(in_field & have, mosaic, np.nan))) if covered else None
    img = np.where(have, np.clip(np.nan_to_num(mosaic), 0, 1), 0.0)[::-1]  # north up
    return {
        "field": field_name,
        "cell_m": CELL_M,
        "bounds": [round(x_lo, 3), round(x_lo + nx * CELL_M, 3), round(y_lo, 3), round(y_lo + ny * CELL_M, 3)],
        "w": nx, "h": ny,
        "image": base64.b64encode((img * 255).astype(np.uint8).tobytes()).decode("ascii"),  # NDVI × 255, north up
        "mapped_pct": round(100.0 * covered / max(field_cells, 1), 1),
        "field_ndvi": round(field_ndvi, 3) if field_ndvi is not None else None,
        "stressed_m2": round(float(np.sum(stressed)) * CELL_M * CELL_M, 1),
        "findings": findings,
        "truth": truth,
        "found": sum(t["found"] for t in truth),
        "false_alarms": sum(1 for f in findings if id(f) not in matched),
        "photos": len(photos),
        "geotag_error_m": round(float(np.mean(tag_err)), 2) if tag_err else None,
    }


def rule_report(m: dict) -> list[str]:
    """A plain report without the AI (always available)."""
    lines = [f"{m['field'].capitalize()}: mapped {m['mapped_pct']} % from {m['photos']} photos; average NDVI "
             f"{m['field_ndvi']} (healthy crop is about 0.8)."]
    if not m["findings"]:
        lines.append("No stressed patches found.")
    for i, f in enumerate(m["findings"], 1):
        lines.append(f"Patch {i}: {f['where']}, about {f['area_m2']:.0f} m², NDVI {f['mean_ndvi']} (lowest {f['min_ndvi']}). "
                     f"Walk to ({f['x_m']:.0f}, {f['y_m']:.0f}) and look: dry soil, pests, standing water?")
    return lines


# ---- recent maps, so the AI report can be asked for by id (the client never sends map data itself) ----

_RECENT: dict[str, dict] = {}
MAX_RECENT = 20


def remember(m: dict) -> str:
    """Keep a map's summary (no picture) under a short id; the oldest are forgotten."""
    import uuid

    key = uuid.uuid4().hex[:12]
    _RECENT[key] = {k: v for k, v in m.items() if k != "image"}
    while len(_RECENT) > MAX_RECENT:
        _RECENT.pop(next(iter(_RECENT)))
    return key


def recall(key: str) -> dict | None:
    return _RECENT.get(key)
