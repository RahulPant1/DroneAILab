"""🏔 Terrain: the ground is no longer a flat plane z = 0.

A Terrain is a grid of ground heights (metres, ENU: x east, y north, z up) around the home point, plus an optional
aerial image to paint on it. Heights are RELATIVE TO HOME: the take-off spot is z = 0, exactly like a real
drone's barometer altitude. So "hold 10 m" means 10 m above home, and a hill 15 m high ahead is a real danger:
fly at 10 m toward it and you hit the hillside. That is one of the most common real-world crashes.

Two sources:
  * built-in "hills": made from a few smooth bumps (no download, used by the tests);
  * a real place: `python -m backend.terrain_fetch --lat .. --lon .. --name myfield` downloads open elevation
    tiles (and optionally open satellite imagery) once into data/terrain/<name>/; afterwards it works offline.

The grid is bilinearly interpolated. Around home (HOME_FLAT_M) the ground is blended to exactly 0, so the drone
starts on level ground at z = 0, like on a take-off mat.
"""

from __future__ import annotations

import base64
import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

TERRAIN_DIR = Path(__file__).resolve().parents[2] / "data" / "terrain"
HOME_FLAT_M = 3.0      # radius of the flat take-off spot (exactly z = 0)
HOME_BLEND_M = 6.0     # from here outwards the real ground takes over
HILLS_ID = "hills"


@dataclass
class Terrain:
    """Ground heights on a regular grid. heights_m[row, col]: row 0 = south edge (y_min), col 0 = west (x_min)."""

    id: str
    title: str
    heights_m: np.ndarray
    x_min_m: float
    y_min_m: float
    cell_m: float
    image: dict | None = None          # {"tiles": [...], ...} for the 3D view (served by the API)
    attribution: str = ""
    info: dict = field(default_factory=dict)

    @property
    def size_m(self) -> tuple[float, float]:
        rows, cols = self.heights_m.shape
        return (cols - 1) * self.cell_m, (rows - 1) * self.cell_m

    def raw_height(self, x_m: float, y_m: float) -> float:
        """Bilinear interpolation of the grid (outside the grid: the nearest edge value)."""
        rows, cols = self.heights_m.shape
        fx = min(max((x_m - self.x_min_m) / self.cell_m, 0.0), cols - 1.000001)
        fy = min(max((y_m - self.y_min_m) / self.cell_m, 0.0), rows - 1.000001)
        c, r = int(fx), int(fy)
        tx, ty = fx - c, fy - r
        h = self.heights_m
        return float((h[r, c] * (1 - tx) + h[r, c + 1] * tx) * (1 - ty) + (h[r + 1, c] * (1 - tx) + h[r + 1, c + 1] * tx) * ty)

    def height(self, x_m: float, y_m: float) -> float:
        """Ground height (m above home) under a point. Exactly 0 on the take-off spot, real ground further out."""
        d = math.hypot(x_m, y_m)
        if d <= HOME_FLAT_M:
            return 0.0
        h = self.raw_height(x_m, y_m)
        if d >= HOME_BLEND_M:
            return h
        s = (d - HOME_FLAT_M) / (HOME_BLEND_M - HOME_FLAT_M)
        return h * s * s * (3 - 2 * s)  # smoothstep: no kink at the mat's edge

    def grid_for_view(self) -> np.ndarray:
        """The grid with the take-off spot flattened, exactly as the physics feels it (for drawing)."""
        rows, cols = self.heights_m.shape
        out = np.empty_like(self.heights_m, dtype=np.float32)
        for r in range(rows):
            y = self.y_min_m + r * self.cell_m
            for c in range(cols):
                out[r, c] = self.height(self.x_min_m + c * self.cell_m, y)
        return out

    def to_dict(self) -> dict:
        """What the 3D view needs: the grid as base64 float32 (little-endian), rows south -> north."""
        g = self.grid_for_view()
        w, d = self.size_m
        return {
            "id": self.id, "title": self.title,
            "rows": int(g.shape[0]), "cols": int(g.shape[1]),
            "x_min_m": self.x_min_m, "y_min_m": self.y_min_m, "cell_m": self.cell_m,
            "width_m": w, "depth_m": d,
            "min_m": round(float(g.min()), 2), "max_m": round(float(g.max()), 2),
            "heights_b64": base64.b64encode(g.astype("<f4").tobytes()).decode(),
            "image": self.image, "attribution": self.attribution, "info": self.info,
        }


def hills() -> Terrain:
    """A made-up valley with hills: a ridge to the north, a knoll to the east, a dip to the south-west.
    Built from smooth Gaussian bumps so it's the same every time and needs no download."""
    cell, half = 4.0, 200.0
    n = int(2 * half / cell) + 1
    xs = np.linspace(-half, half, n)
    X, Y = np.meshgrid(xs, xs)  # rows = y
    h = (
        22.0 * np.exp(-(((Y - 75.0) / 28.0) ** 2)) * (0.7 + 0.3 * np.cos(X / 37.0))   # the ridge, north
        + 14.0 * np.exp(-(((X - 45.0) ** 2 + (Y + 5.0) ** 2) / (2 * 14.0 ** 2)))       # the knoll, east
        - 4.0 * np.exp(-(((X + 50.0) ** 2 + (Y + 45.0) ** 2) / (2 * 25.0 ** 2)))       # the dip, south-west
        + 1.2 * np.sin(X / 9.0) * np.cos(Y / 11.0)                                    # gentle bumps
        + 0.012 * X                                                                   # the valley tilts
    )
    return Terrain(HILLS_ID, "Hill valley (made up)", h.astype(np.float32), -half, -half, cell,
                   info={"note": "A ridge 22 m high 75 m north, a 14 m knoll 45 m east. No download needed."})


def list_terrains() -> list[dict]:
    """Built-in + downloaded places (data/terrain/<id>/meta.json)."""
    out = [{"id": HILLS_ID, "title": hills().title, "downloaded": False}]
    if TERRAIN_DIR.exists():
        for meta in sorted(TERRAIN_DIR.glob("*/meta.json")):
            try:
                m = json.loads(meta.read_text())
                out.append({"id": meta.parent.name, "title": m.get("title", meta.parent.name), "downloaded": True})
            except (OSError, ValueError):
                continue
    return out


def load(terrain_id: str) -> Terrain:
    """Load a terrain by id: 'hills' or a folder in data/terrain/ written by backend.terrain_fetch."""
    if terrain_id == HILLS_ID:
        return hills()
    if not terrain_id.replace("-", "").replace("_", "").isalnum():
        raise ValueError("Terrain names are letters, digits, - and _ only.")
    folder = TERRAIN_DIR / terrain_id
    meta_file = folder / "meta.json"
    if not meta_file.exists():
        names = ", ".join(t["id"] for t in list_terrains())
        raise ValueError(f"Unknown terrain '{terrain_id}'. Choose one of: {names}.")
    m = json.loads(meta_file.read_text())
    heights = np.load(folder / "heights.npy").astype(np.float32)
    return Terrain(terrain_id, m.get("title", terrain_id), heights, float(m["x_min_m"]), float(m["y_min_m"]),
                   float(m["cell_m"]), image=m.get("image"), attribution=m.get("attribution", ""),
                   info=m.get("info", {}))
