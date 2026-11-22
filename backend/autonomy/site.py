"""The farm: one shared map for surveys, AI-planned missions and precision landing.

Everything is in the ENU world frame (x east, y north), in metres from the home pad at (0, 0).
  fields     : convex areas to survey (photograph from above)
  buildings  : boxes the drone flies OVER if it is higher than their roof, otherwise AROUND
  no_fly     : areas it must never enter, at any height (people, animals, a neighbour's garden)
  pads       : landing pads with a printed marker the down camera can see (precision landing)
  places     : named points (to fly to, or hover over)
  geofence   : the drone may not go further than this from home, or higher than the ceiling

Names are what the AI mission planner may use: it picks from these, it never invents coordinates.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .world import Box

NO_FLY_HEIGHT_M = 1000.0  # a no-fly zone blocks every altitude


@dataclass(frozen=True)
class Field:
    name: str
    polygon: tuple[tuple[float, float], ...]  # corners in order (convex)
    crop: str = ""


@dataclass(frozen=True)
class Building:
    name: str
    box: Box


@dataclass(frozen=True)
class NoFly:
    name: str
    box: Box  # x/y extent; the height is ignored (it blocks every altitude)
    why: str = ""


@dataclass(frozen=True)
class Pad:
    name: str
    x_m: float
    y_m: float
    size_m: float = 1.0  # side of the printed marker


@dataclass(frozen=True)
class Place:
    name: str
    x_m: float
    y_m: float


@dataclass(frozen=True)
class Site:
    title: str
    fields: tuple[Field, ...]
    buildings: tuple[Building, ...]
    no_fly: tuple[NoFly, ...]
    pads: tuple[Pad, ...]
    places: tuple[Place, ...]
    geofence_radius_m: float = 90.0
    ceiling_m: float = 60.0
    notes: tuple[str, ...] = field(default=())

    def field(self, name: str) -> Field | None:
        return next((f for f in self.fields if f.name == name), None)

    def pad(self, name: str) -> Pad | None:
        return next((p for p in self.pads if p.name == name), None)

    def point(self, name: str) -> tuple[float, float] | None:
        """Where a named thing is: a pad, a place, or the centre of a field / building / no-fly zone."""
        for p in self.pads:
            if p.name == name:
                return (p.x_m, p.y_m)
        for p in self.places:
            if p.name == name:
                return (p.x_m, p.y_m)
        f = self.field(name)
        if f:
            return polygon_centroid(f.polygon)
        for b in self.buildings:
            if b.name == name:
                return ((b.box.x_min + b.box.x_max) / 2, (b.box.y_min + b.box.y_max) / 2)
        return None

    def destination_names(self) -> list[str]:
        return [p.name for p in self.pads] + [p.name for p in self.places] + [f.name for f in self.fields]

    def obstacles(self, alt_m: float) -> list[Box]:
        """What the route planner must keep clear of at this altitude: no-fly zones (always) and buildings
        taller than the drone flies."""
        out = [Box(n.box.x_min, n.box.x_max, n.box.y_min, n.box.y_max, NO_FLY_HEIGHT_M) for n in self.no_fly]
        return out + [b.box for b in self.buildings if b.box.blocks_altitude(alt_m)]

    def to_dict(self) -> dict:
        return {
            "title": self.title,
            "fields": [{"name": f.name, "polygon": [list(p) for p in f.polygon], "crop": f.crop} for f in self.fields],
            "buildings": [{"name": b.name, **b.box.to_dict()} for b in self.buildings],
            "no_fly": [{"name": n.name, "why": n.why, **n.box.to_dict()} for n in self.no_fly],
            "pads": [{"name": p.name, "x_m": p.x_m, "y_m": p.y_m, "size_m": p.size_m} for p in self.pads],
            "places": [{"name": p.name, "x_m": p.x_m, "y_m": p.y_m} for p in self.places],
            "geofence_radius_m": self.geofence_radius_m,
            "ceiling_m": self.ceiling_m,
        }


def polygon_centroid(poly) -> tuple[float, float]:
    """Area-weighted centre of a polygon."""
    a = cx = cy = 0.0
    n = len(poly)
    for i in range(n):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % n]
        c = x0 * y1 - x1 * y0
        a += c
        cx += (x0 + x1) * c
        cy += (y0 + y1) * c
    if abs(a) < 1e-12:
        return (sum(p[0] for p in poly) / n, sum(p[1] for p in poly) / n)
    return (cx / (3 * a), cy / (3 * a))


def polygon_area(poly) -> float:
    return abs(sum(poly[i][0] * poly[(i + 1) % len(poly)][1] - poly[(i + 1) % len(poly)][0] * poly[i][1]
                   for i in range(len(poly)))) / 2.0


FARM = Site(
    title="Hillside farm",
    fields=(
        Field("north field", ((8.0, 12.0), (48.0, 12.0), (48.0, 40.0), (8.0, 40.0)), "wheat"),
        Field("east field", ((48.0, -10.0), (76.0, -16.0), (78.0, -42.0), (52.0, -40.0)), "potatoes"),
        Field("orchard", ((-46.0, 10.0), (-26.0, 10.0), (-26.0, 32.0), (-46.0, 32.0)), "apple trees"),
    ),
    buildings=(
        Building("barn", Box(-18.0, -8.0, -6.0, 4.0, height_m=9.0)),
        Building("silo", Box(20.0, 24.0, -4.0, 0.0, height_m=16.0)),
        Building("farmhouse", Box(-22.0, -12.0, -32.0, -22.0, height_m=8.0)),
    ),
    no_fly=(
        NoFly("neighbour's garden", Box(24.0, 42.0, -36.0, -16.0), "people relax there: never fly over people"),
        NoFly("horse paddock", Box(-52.0, -32.0, -48.0, -30.0), "drones frighten horses"),
    ),
    pads=(Pad("home", 0.0, 0.0), Pad("pad B", -12.0, 30.0)),
    places=(Place("pond", 26.0, 56.0), Place("gate", 62.0, 6.0), Place("water trough", -40.0, -10.0)),
    notes=("Home pad is at (0, 0). x = metres east, y = metres north.",),
)


def distance_from_home(x: float, y: float) -> float:
    return math.hypot(x, y)
