"""Autonomous exploration: a drone dropped into an unknown place maps it by itself, with its lidar only.

Three pieces, each a classic of robot navigation (the ideas behind ROS 2's Nav2 and slam_toolbox, written here in
plain NumPy):

1. MAPPING + FRONTIER EXPLORATION. Every lidar beam says two things: the cell where it hit is OCCUPIED, and every
   cell it passed through is FREE. A log-odds grid adds up that evidence, so each cell is free, occupied or still
   UNKNOWN. A "frontier" is a free cell next to an unknown one: the edge of what the drone has seen. The explorer
   flies to the best frontier (near, and long), looks, and repeats until there are no frontiers it can reach.

2. LOCAL PLANNER. The global path (Dijkstra through known-free cells) says where to go; a local planner decides how
   to move right now. Two of them:
     pursuit  chase a point 1.5 m along the path; brake if the lidar sees something close ahead.
     dwa      the Dynamic Window Approach: 10 times a second, try ~100 velocities the drone can reach within half a
              second, imagine flying each for 1.5 s, throw away any that come too close to what the lidar sees right
              now, and pick the best mix of progress, clearance and speed. Moving obstacles are handled for free:
              it only ever looks at the latest scan.

3. LOCALISATION WITHOUT GPS. Indoors there is no GPS: the EKF can only dead-reckon, and drifts by metres. Scan
   matching fixes that: try small shifts and turns of the latest scan against the map built so far, keep the one that
   fits best, and give that position + heading to the EKF as a measurement (like a GPS fix, but from the walls).
   Map and position are estimated together: SLAM (simultaneous localisation and mapping).

The lidar is honest: beams are measured from the TRUE pose in the drone's BODY frame (they turn with it); the map is
drawn at the ESTIMATED pose. A box hit by the real drone ends the flight (a crash).
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field

import numpy as np

from backend.sim.control import Setpoint
from backend.sim.frames import quat_to_euler
from backend.sim.physics import State

from .world import DRONE_RADIUS_M, Box

# ---- the lidar and the map ----------------------------------------------------------------------------------------
BEAMS = 72
RANGE_M = 8.0
SCAN_PERIOD_S = 0.1
LIDAR_NOISE_M = 0.02
BODY_ANGLES = np.linspace(0.0, 2 * math.pi, BEAMS, endpoint=False)  # 0 = the drone's nose, anticlockwise
RES_M = 0.2
L_FREE, L_OCC, L_MIN, L_MAX = -0.7, 0.9, -2.0, 3.0  # log-odds evidence per beam, and the clamp
OCC_ABOVE, FREE_BELOW = 0.6, -0.6
KEEP_OUT_M = DRONE_RADIUS_M + 0.2  # plan through cells at least this far from anything occupied (indoors: tight)

# ---- exploration -------------------------------------------------------------------------------------------------
FLY_ALT_M = 1.5
CRUISE_M_S = 1.5
MIN_FRONTIER_CELLS = 4  # smaller frontier clusters are noise (a corner the lidar just grazed)
GAIN_WEIGHT = 0.5  # metres of travel worth one metre of frontier length
REPLAN_S = 2.0
SWITCH_MARGIN_M = 1.5  # keep the current goal unless another frontier is this much better
GOAL_REACHED_M = 0.6
LOOKAHEAD_M = 1.5
DONE_HOLD_S = 3.0  # no frontier left for this long = finished

# ---- local planners ------------------------------------------------------------------------------------------------
ACCEL_M_S2 = 2.0
WINDOW_S = 0.5  # DWA: velocities reachable within this long
HORIZON_S = 1.5  # DWA: imagine each velocity for this long
DWA_SAMPLES = 11  # per axis
MIN_CLEAR_M = 0.25  # gap between the drone's edge and an obstacle a trajectory must keep
MIN_CLEAR_MOVING_M = 0.6  # ...and from something moving (it may not keep going the way it was)
ESCAPE_S = 0.5  # nothing safe: pick the velocity with the most room this far ahead
BRAKE_M = 1.0  # pursuit: something closer than this ahead -> stop
W_PROGRESS, W_CLEAR, W_SPEED = 1.0, 0.6, 0.2
LOCAL_PERIOD_S = 0.1  # both local planners decide 10 times a second (with each new scan)

DYN_SEEN_FREE = 3  # a hit in a cell seen through at least this often...
DYN_FREE_RATIO = 2.0  # ...and far more often than it was hit (a wall grazed now and then isn't moving) = moved there
DYN_MIN_POINTS = 3  # smaller groups of 'moving' hits are noise
DYN_CLUSTER_M = 0.8
DYN_MATCH_M = 0.6  # a cluster this close to last scan's is the same thing
DYN_SMOOTH = 0.5  # velocity filter (0 = keep old, 1 = newest only)
DYN_MAX_SPEED_M_S = 3.0

# ---- scan matching -------------------------------------------------------------------------------------------------
MATCH_XY_M = 0.3
MATCH_YAW_RAD = math.radians(3.0)
MATCH_SIGMA_M = 0.05  # how good the EKF is told a scan-matched position is
MATCH_YAW_SIGMA_RAD = math.radians(1.0)
MATCH_MIN_HITS = 15
MATCH_MIN_MAP_CELLS = 40
MATCH_SIGMA_FIT_M = 0.1  # scoring: how close (m) a beam's end must be to a mapped surface to count as on it
MATCH_ON_WALL = 0.3  # a beam 'lands on a mapped wall' if its score is above this (within ~0.15 m)

LOCAL_PLANNERS = {"pursuit": "Chase the path, brake when blocked", "dwa": "Dynamic Window Approach (DWA)"}
LOCALISATION = {"gps": "GPS + EKF", "dead_reckoning": "No GPS: EKF on motion sensors only",
                "slam": "No GPS: EKF + lidar scan matching (SLAM)"}


# ---- worlds ------------------------------------------------------------------------------------------------------
@dataclass
class Mover:
    """A box that walks back and forth between two points (a person, a robot)."""

    a: tuple[float, float]
    b: tuple[float, float]
    speed_m_s: float = 0.8
    size_m: float = 0.6

    def where(self, t: float) -> tuple[float, float]:
        d = math.dist(self.a, self.b)
        s = (self.speed_m_s * t) % (2 * d)
        f = (s if s <= d else 2 * d - s) / d
        return self.a[0] + (self.b[0] - self.a[0]) * f, self.a[1] + (self.b[1] - self.a[1]) * f

    def box(self, t: float) -> Box:
        x, y = self.where(t)
        h = self.size_m / 2
        return Box(x - h, x + h, y - h, y + h, 2.0)


@dataclass
class World:
    title: str
    walls: list[Box]
    start: tuple[float, float]
    bounds: tuple[float, float, float, float]  # inside the outer walls: x0, x1, y0, y1
    movers: list[Mover] = field(default_factory=list)

    def boxes(self, t: float, movers: bool = True) -> list[Box]:
        return self.walls + ([m.box(t) for m in self.movers] if movers else [])


def _fence(x0: float, x1: float, y0: float, y1: float, t: float = 0.2) -> list[Box]:
    return [Box(x0 - t, x1 + t, y0 - t, y0), Box(x0 - t, x1 + t, y1, y1 + t), Box(x0 - t, x0, y0, y1), Box(x1, x1 + t, y0, y1)]


def _rooms() -> World:
    w = 0.2
    walls = _fence(0, 16, 0, 12) + [
        Box(6, 6 + w, 0, 2), Box(6, 6 + w, 3.8, 8), Box(6, 6 + w, 9.8, 12),  # left rooms | hall: two 1.8 m doors
        Box(11, 11 + w, 0, 5), Box(11, 11 + w, 6.8, 12),  # hall | right rooms, one door
        Box(0, 2, 6, 6 + w), Box(3.8, 6, 6, 6 + w),  # between the two left rooms
        Box(11, 13, 6, 6 + w), Box(14.8, 16, 6, 6 + w),  # between the two right rooms
        Box(8, 9, 7.5, 8.5),  # a cupboard in the hall
    ]
    return World("A building: 4 rooms and a hall (16 × 12 m)", walls, (3.0, 3.0), (0, 16, 0, 12),
                 [Mover((8.5, 1.0), (8.5, 11.0), 0.8)])


def _forest() -> World:
    rng = np.random.default_rng(7)
    trees: list[Box] = []
    while len(trees) < 22:
        x, y = rng.uniform(1.5, 18.5, 2)
        if math.hypot(x - 2, y - 2) < 3 or any(math.hypot(x - (b.x_min + 0.25), y - (b.y_min + 0.25)) < 2.6 for b in trees):
            continue
        trees.append(Box(x - 0.25, x + 0.25, y - 0.25, y + 0.25, 6.0))
    return World("A fenced forest: 22 trees (20 × 20 m)", _fence(0, 20, 0, 20) + trees, (2.0, 2.0), (0, 20, 0, 20),
                 [Mover((4.0, 12.0), (16.0, 12.0), 1.0)])


WORLDS = {"rooms": _rooms(), "forest": _forest()}


# ---- the map -----------------------------------------------------------------------------------------------------
class Grid:
    """Log-odds occupancy grid: > OCC_ABOVE occupied, < FREE_BELOW free, otherwise unknown."""

    def __init__(self, bounds: tuple[float, float, float, float], margin_m: float = 2.0, res: float = RES_M):
        x0, x1, y0, y1 = bounds
        self.res = res
        self.x0, self.y0 = x0 - margin_m, y0 - margin_m
        self.nx = int(math.ceil((x1 - x0 + 2 * margin_m) / res))
        self.ny = int(math.ceil((y1 - y0 + 2 * margin_m) / res))
        self.L = np.zeros((self.nx, self.ny), np.float32)
        # Where exactly the hits were: the sum of hit positions per cell and how many. Their mean is the wall surface
        # in that cell, to a few cm, much better than "somewhere in this 0.2 m cell". Used by scan matching.
        self.hx = np.zeros((self.nx, self.ny))
        self.hy = np.zeros((self.nx, self.ny))
        self.hn = np.zeros((self.nx, self.ny))
        self.seen_free = np.zeros((self.nx, self.ny), np.int32)  # how many scans saw through this cell
        self.hit_count = np.zeros((self.nx, self.ny), np.int32)  # how many scans hit something in it (never reset)
        self.version = 0
        self._steps = np.arange(0.0, RANGE_M, res / 2)
        r = int(math.ceil(KEEP_OUT_M / res))
        self._disc = [(dx, dy) for dx in range(-r, r + 1) for dy in range(-r, r + 1) if math.hypot(dx, dy) * res <= KEEP_OUT_M]

    def index(self, x, y):
        return (np.floor((np.asarray(x) - self.x0) / self.res).astype(int),
                np.floor((np.asarray(y) - self.y0) / self.res).astype(int))

    def center(self, i, j) -> tuple[float, float]:
        return self.x0 + (i + 0.5) * self.res, self.y0 + (j + 0.5) * self.res

    def _inside(self, i, j):
        return (i >= 0) & (i < self.nx) & (j >= 0) & (j < self.ny)

    def integrate(self, x: float, y: float, world_angles: np.ndarray, ranges: np.ndarray) -> None:
        """Add one scan seen from (x, y): free along each beam, occupied where it hit."""
        c, s = np.cos(world_angles), np.sin(world_angles)
        px = x + c[:, None] * self._steps
        py = y + s[:, None] * self._steps
        along = self._steps[None, :] < (ranges[:, None] - self.res * 0.7)
        i, j = self.index(px[along], py[along])
        ok = self._inside(i, j)
        flat = np.unique(i[ok] * self.ny + j[ok])
        self.L.flat[flat] += L_FREE
        self.seen_free.flat[flat] += 1
        hit = ranges < RANGE_M - 1e-6
        i, j = self.index(x + ranges[hit] * c[hit], y + ranges[hit] * s[hit])
        ok = self._inside(i, j)
        flat = np.unique(i[ok] * self.ny + j[ok])
        self.L.flat[flat] += L_OCC
        np.clip(self.L, L_MIN, L_MAX, out=self.L)
        hx, hy = (x + ranges[hit] * c[hit])[ok], (y + ranges[hit] * s[hit])[ok]
        np.add.at(self.hx, (i[ok], j[ok]), hx)
        np.add.at(self.hy, (i[ok], j[ok]), hy)
        np.add.at(self.hn, (i[ok], j[ok]), 1.0)
        self.hit_count.flat[flat] += 1
        gone = self.L < FREE_BELOW  # seen empty since (someone walked away): forget where the hits were
        self.hn[gone] = self.hx[gone] = self.hy[gone] = 0.0
        self.version += 1

    def occupied(self) -> np.ndarray:
        return self.L > OCC_ABOVE

    def free(self) -> np.ndarray:
        return self.L < FREE_BELOW

    def blocked(self) -> np.ndarray:
        """Occupied cells grown by the drone's size + margin (the configuration space)."""
        occ = self.occupied()
        out = np.zeros_like(occ)
        for dx, dy in self._disc:
            src = occ[max(0, -dx):self.nx - max(0, dx), max(0, -dy):self.ny - max(0, dy)]
            out[max(0, dx):self.nx - max(0, -dx), max(0, dy):self.ny - max(0, -dy)] |= src
        return out

    def surface_points(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Mean hit position of every cell (NaN where there is none), padded by one cell for neighbour lookups."""
        have = (self.hn > 0) & (self.L > 0)
        n = np.where(have, self.hn, 1.0)
        mx = np.pad(np.where(have, self.hx / n, np.nan), 1, constant_values=np.nan)
        my = np.pad(np.where(have, self.hy / n, np.nan), 1, constant_values=np.nan)
        return mx, my, have

    def distance_to_surface(self, surf, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """For each point: distance to the nearest mapped surface point in its cell or the 8 around it (inf if none)."""
        mx, my, _ = surf
        i, j = self.index(x, y)
        i = np.clip(i, -1, self.nx) + 1  # +1: the padding
        j = np.clip(j, -1, self.ny) + 1
        best = np.full(np.shape(x), np.inf)
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                a, b = np.clip(i + di, 0, self.nx + 1), np.clip(j + dj, 0, self.ny + 1)
                d = np.hypot(mx[a, b] - x, my[a, b] - y)
                best = np.fmin(best, d)
        return best


# ---- global planning: distances over known-free cells, frontiers, paths --------------------------------------------
_MOVES = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0)] + [(dx, dy, math.sqrt(2)) for dx in (1, -1) for dy in (1, -1)]


def dijkstra(passable: np.ndarray, start: tuple[int, int]) -> tuple[np.ndarray, dict]:
    """Distance (cells) from start to every passable cell, 8-connected without cutting corners, and the parents."""
    nx, ny = passable.shape
    dist = np.full((nx, ny), np.inf)
    parent: dict = {}
    dist[start] = 0.0
    heap = [(0.0, start)]
    while heap:
        d, (i, j) = heapq.heappop(heap)
        if d > dist[i, j]:
            continue
        for dx, dy, step in _MOVES:
            a, b = i + dx, j + dy
            if not (0 <= a < nx and 0 <= b < ny) or not passable[a, b]:
                continue
            if dx and dy and not (passable[i + dx, j] and passable[i, j + dy]):
                continue
            nd = d + step
            if nd < dist[a, b]:
                dist[a, b] = nd
                parent[(a, b)] = (i, j)
                heapq.heappush(heap, (nd, (a, b)))
    return dist, parent


def frontier_clusters(free: np.ndarray, unknown: np.ndarray, passable: np.ndarray) -> list[list[tuple[int, int]]]:
    """Passable free cells with an unknown neighbour, grouped into connected clusters."""
    nb = np.zeros_like(unknown)
    nb[1:, :] |= unknown[:-1, :]
    nb[:-1, :] |= unknown[1:, :]
    nb[:, 1:] |= unknown[:, :-1]
    nb[:, :-1] |= unknown[:, 1:]
    cells = set(zip(*np.nonzero(free & passable & nb)))
    clusters = []
    while cells:
        seed = cells.pop()
        group, todo = [seed], [seed]
        while todo:
            i, j = todo.pop()
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    n = (i + dx, j + dy)
                    if n in cells:
                        cells.remove(n)
                        group.append(n)
                        todo.append(n)
        clusters.append([(int(a), int(b)) for a, b in group])
    return clusters


def line_free(passable: np.ndarray, a: tuple[int, int], b: tuple[int, int]) -> bool:
    n = max(abs(b[0] - a[0]), abs(b[1] - a[1])) * 2 + 1
    ii = np.round(np.linspace(a[0], b[0], n)).astype(int)
    jj = np.round(np.linspace(a[1], b[1], n)).astype(int)
    return bool(passable[ii, jj].all())


# ---- the explorer ------------------------------------------------------------------------------------------------
class Explorer:
    """Runs the lidar, the map, scan matching and the exploration logic. Use with run_headless:
    `before_step=explorer.before_step` (lidar + localisation) and the explorer as the setpoint source."""

    def __init__(self, world: World, local: str = "dwa", localisation: str = "gps", movers: bool = False, seed: int = 0):
        self.w, self.local, self.loc, self.movers_on = world, local, localisation, movers
        self.grid = Grid(world.bounds)
        self.rng = np.random.default_rng([seed, 77])
        self._next_scan = 0.0
        self._next_plan = 0.0
        self.scan_pts: np.ndarray = np.zeros((0, 2))  # latest hits, world frame at the ESTIMATED pose
        self.static_pts: np.ndarray = np.zeros((0, 2))
        self.movers_seen: list[dict] = []  # tracked moving things: centroid, velocity, points
        self.path: list[tuple[float, float]] = []
        self.goal: tuple[int, int] | None = None
        self.goal_cost = math.inf
        self.goals: list[tuple[float, float]] = []
        self.v_cmd = np.zeros(2)
        self._next_local = 0.0
        self.phase = "explore"  # explore | done | crashed
        self.done = False
        self._no_frontier_since: float | None = None
        self.events: list[dict] = []
        self.matches = 0
        self.match_scores: list[float] = []
        self.match_rejects = 0
        self.brakes = 0
        self.min_clear_m = math.inf
        self.clear_now_m = RANGE_M
        self.last_scan: tuple | None = None
        self.t = 0.0
        self.coverage_log: list[tuple[float, float]] = []
        self._interior, self._interior_n = self._truth_interior()

    # ---- truth-side bookkeeping (scoring only: the explorer never uses it to decide) ----
    def _truth_interior(self) -> tuple[np.ndarray, int]:
        g = self.grid
        i, j = np.meshgrid(np.arange(g.nx), np.arange(g.ny), indexing="ij")
        x, y = g.center(i, j)
        x0, x1, y0, y1 = self.w.bounds
        inside = (x > x0) & (x < x1) & (y > y0) & (y < y1)
        for b in self.w.walls:
            inside &= ~((x >= b.x_min) & (x <= b.x_max) & (y >= b.y_min) & (y <= b.y_max))
        return inside, int(inside.sum())

    def coverage(self) -> float:
        """Fraction of the real free floor area that the map knows (free or occupied)."""
        known = (self.grid.L > OCC_ABOVE) | (self.grid.L < FREE_BELOW)
        return float((known & self._interior).sum()) / max(1, self._interior_n)

    def _event(self, t: float, kind: str, text: str) -> None:
        self.events.append({"t_s": round(t, 2), "type": kind, "message": text})

    # ---- called before every physics step: lidar, localisation, map ----
    def before_step(self, sim) -> None:
        t = sim.state.time_s
        self.t = t
        truth = sim.state
        boxes = self.w.boxes(t, self.movers_on)
        tx, ty = float(truth.pos_m[0]), float(truth.pos_m[1])
        clear = min(b.distance_2d(tx, ty) for b in boxes) - DRONE_RADIUS_M
        self.clear_now_m = clear
        self.min_clear_m = min(self.min_clear_m, clear)
        if clear < 0 and self.phase != "crashed":
            self.phase, self.done = "crashed", True
            self._event(t, "crash", f"Hit something at ({tx:.1f}, {ty:.1f}): the real drone was where its map said free.")
        if sim.sensors is not None and self.loc != "gps":
            sim.sensors.gps_ok = False
        if t < self._next_scan:
            return
        self._next_scan = t + SCAN_PERIOD_S
        from backend.autonomy.lidar import scan as cast

        yaw_true = quat_to_euler(truth.quat)[2]
        ranges = cast(boxes, tx, ty, float(truth.pos_m[2]), RANGE_M, BODY_ANGLES + yaw_true)
        hit = ranges < RANGE_M
        ranges = np.where(hit, np.clip(ranges + self.rng.normal(0, LIDAR_NOISE_M, BEAMS), 0.0, RANGE_M), ranges)
        self.last_scan = ((tx, ty, float(truth.pos_m[2])), ranges, yaw_true)  # for drawing the beams
        est = sim.estimate
        if self.loc == "slam" and sim.estimator is not None:
            self._scan_match(sim.estimator, ranges)
            est = sim.estimate
        ex, ey = float(est.pos_m[0]), float(est.pos_m[1])
        yaw_est = quat_to_euler(est.quat)[2]
        ang = BODY_ANGLES + yaw_est
        pts = np.column_stack([ex + ranges[hit] * np.cos(ang[hit]), ey + ranges[hit] * np.sin(ang[hit])])
        self._track_movers(pts)  # before the map learns this scan: "was this spot seen empty before?"
        self.grid.integrate(ex, ey, ang, ranges)
        self.scan_pts = pts
        if int(t * 10) % 10 == 0:
            self.coverage_log.append((round(t, 1), round(self.coverage(), 4)))

    def _track_movers(self, pts: np.ndarray) -> None:
        """Split the scan into static hits and moving things, and estimate each moving thing's velocity.

        A wall is never seen through; the spot where a person now stands was seen empty before. So a hit in a cell
        that several earlier beams passed through is something that moved there. Group those hits, match each group
        with the nearest one from the last scan, and the shift / 0.1 s is its velocity (smoothed)."""
        g = self.grid
        i, j = g.index(pts[:, 0], pts[:, 1])
        i, j = np.clip(i, 0, g.nx - 1), np.clip(j, 0, g.ny - 1)
        sf, hc = g.seen_free[i, j], g.hit_count[i, j]
        dyn = (sf >= DYN_SEEN_FREE) & (sf > DYN_FREE_RATIO * hc)
        self.static_pts = pts[~dyn]
        groups: list[np.ndarray] = []
        left = [tuple(q) for q in pts[dyn]]
        while left:
            grp = [left.pop()]
            grew = True
            while grew:
                grew = False
                for q in list(left):
                    if any(math.dist(q, m) < DYN_CLUSTER_M for m in grp):
                        grp.append(q)
                        left.remove(q)
                        grew = True
            if len(grp) >= DYN_MIN_POINTS:
                groups.append(np.array(grp))
            else:
                self.static_pts = np.vstack([self.static_pts, np.array(grp)])
        tracked = []
        for grp in groups:
            c = grp.mean(axis=0)
            prev = min(self.movers_seen, key=lambda m: math.dist(m["c"], c), default=None)
            v = np.zeros(2)
            if prev is not None and math.dist(prev["c"], c) < DYN_MATCH_M:
                raw = (c - prev["c"]) / SCAN_PERIOD_S
                v = (1 - DYN_SMOOTH) * prev["v"] + DYN_SMOOTH * raw
                sp = float(np.linalg.norm(v))
                if sp > DYN_MAX_SPEED_M_S:
                    v *= DYN_MAX_SPEED_M_S / sp
            tracked.append({"c": c, "v": v, "pts": grp})
        self.movers_seen = tracked

    def _scan_match(self, ekf, ranges: np.ndarray) -> None:
        """Find the small shift + turn that makes this scan fit the map best; tell the EKF."""
        g = self.grid
        if int(g.occupied().sum()) < MATCH_MIN_MAP_CELLS:
            return
        hit = ranges < RANGE_M
        if hit.sum() < MATCH_MIN_HITS:
            return
        surf = g.surface_points()
        est = ekf.state
        ex, ey = float(est.pos_m[0]), float(est.pos_m[1])
        yaw = quat_to_euler(est.quat)[2]
        r, a = ranges[hit], BODY_ANGLES[hit]

        def best(dxs, dys, dths, base=(0.0, 0.0, 0.0)):
            DX, DY, DT = np.meshgrid(dxs + base[0], dys + base[1], dths + base[2], indexing="ij")
            DX, DY, DT = DX.ravel(), DY.ravel(), DT.ravel()
            ang = a[None, :] + yaw + DT[:, None]
            px = ex + DX[:, None] + r[None, :] * np.cos(ang)
            py = ey + DY[:, None] + r[None, :] * np.sin(ang)
            d = g.distance_to_surface(surf, px, py)
            vals = np.exp(-0.5 * (np.minimum(d, 1.0) / MATCH_SIGMA_FIT_M) ** 2)  # 1 on a mapped surface, 0 far off
            k = int(np.argmax(vals.sum(axis=1)))
            return (float(DX[k]), float(DY[k]), float(DT[k])), vals[k]

        coarse = np.linspace(-MATCH_XY_M, MATCH_XY_M, 13)
        (dx, dy, dth), _ = best(coarse, coarse, np.linspace(-MATCH_YAW_RAD, MATCH_YAW_RAD, 7))
        fine = np.linspace(-0.04, 0.04, 9)
        (dx, dy, dth), vals = best(fine, fine, np.radians(np.linspace(-0.5, 0.5, 5)), (dx, dy, dth))
        on_wall = int((vals > MATCH_ON_WALL).sum())
        self.match_scores.append(on_wall / len(vals))
        # Trust it only if enough beams land on walls already in the map. Beams into a room it hasn't mapped yet
        # don't count against it: they simply can't help.
        if on_wall < MATCH_MIN_HITS:
            self.match_rejects += 1
            return
        saved = ekf.r_gps_pos, ekf.r_compass
        ekf.r_gps_pos, ekf.r_compass = MATCH_SIGMA_M**2, MATCH_YAW_SIGMA_RAD**2
        try:
            ekf.fuse_gps(np.array([ex + dx, ey + dy]), None)
            ekf.fuse_compass(math.remainder(yaw + dth, 2 * math.pi))
        finally:
            ekf.r_gps_pos, ekf.r_compass = saved
        self.matches += 1

    # ---- global: choose a frontier, plan a path ----
    def _plan(self, x: float, y: float, t: float) -> None:
        g = self.grid
        free, occ = g.free(), g.occupied()
        unknown = ~free & ~occ
        passable = free & ~g.blocked()
        si, sj = (int(v) for v in g.index(x, y))
        si, sj = min(max(si, 0), g.nx - 1), min(max(sj, 0), g.ny - 1)
        for dx in (-1, 0, 1):  # it may stand inside a margin (close to a wall): let it leave
            for dy in (-1, 0, 1):
                if 0 <= si + dx < g.nx and 0 <= sj + dy < g.ny and free[si + dx, sj + dy]:
                    passable[si + dx, sj + dy] = True
        passable[si, sj] = True
        dist, parent = dijkstra(passable, (si, sj))
        best, best_cost = None, math.inf
        for cl in frontier_clusters(free, unknown, passable):
            if len(cl) < MIN_FRONTIER_CELLS:
                continue
            d = np.array([dist[c] for c in cl])
            if not np.isfinite(d).any():
                continue
            k = int(np.argmin(d))
            cost = d[k] * g.res - GAIN_WEIGHT * len(cl) * g.res
            if cost < best_cost:
                best, best_cost = cl[k], cost
        if self.goal is not None and self.goal in parent and best is not None:
            cur = dist[self.goal] * g.res - GAIN_WEIGHT * g.res * MIN_FRONTIER_CELLS
            still = unknown[max(0, self.goal[0] - 1):self.goal[0] + 2, max(0, self.goal[1] - 1):self.goal[1] + 2].any()
            if still and best_cost > cur - SWITCH_MARGIN_M:
                best = self.goal  # hysteresis: don't flip between two similar frontiers
        if best is None:
            self.path = []
            self.goal = None
            if self._no_frontier_since is None:
                self._no_frontier_since = t
            elif t - self._no_frontier_since > DONE_HOLD_S and self.phase == "explore":
                self.phase, self.done = "done", True
                self._event(t, "done", f"No frontier left that it can reach: explored {100 * self.coverage():.0f} % of the floor.")
            return
        self._no_frontier_since = None
        if best != self.goal:
            self.goals.append(g.center(*best))
        self.goal = best
        cells = [best]
        while cells[-1] != (si, sj):
            cells.append(parent[cells[-1]])
        cells.reverse()
        keep, i = [cells[0]], 0
        while i < len(cells) - 1:  # straighten: jump to the farthest cell in line of sight
            j = len(cells) - 1
            while j > i + 1 and not line_free(passable, cells[i], cells[j]):
                j -= 1
            keep.append(cells[j])
            i = j
        self.path = [(x, y)] + [g.center(*c) for c in keep[1:]]

    def _carrot(self, x: float, y: float) -> tuple[float, float]:
        """The point LOOKAHEAD_M further along the path than the closest point to (x, y)."""
        p = self.path
        if len(p) < 2:
            return p[-1] if p else (x, y)
        best_s, best_d, cum = 0.0, math.inf, [0.0]
        for a, b in zip(p, p[1:]):
            cum.append(cum[-1] + math.dist(a, b))
        for k, (a, b) in enumerate(zip(p, p[1:])):
            L = max(1e-9, math.dist(a, b))
            u = max(0.0, min(1.0, ((x - a[0]) * (b[0] - a[0]) + (y - a[1]) * (b[1] - a[1])) / (L * L)))
            d = math.hypot(a[0] + u * (b[0] - a[0]) - x, a[1] + u * (b[1] - a[1]) - y)
            if d < best_d:
                best_d, best_s = d, cum[k] + u * L
        s = min(cum[-1], best_s + LOOKAHEAD_M)
        for k, (a, b) in enumerate(zip(p, p[1:])):
            if s <= cum[k + 1] or k == len(p) - 2:
                f = (s - cum[k]) / max(1e-9, cum[k + 1] - cum[k])
                return a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1])
        return p[-1]

    # ---- local: choose a velocity ----
    def _pursuit(self, pos: np.ndarray, carrot: tuple[float, float]) -> np.ndarray:
        d = np.array(carrot) - pos
        n = float(np.linalg.norm(d))
        v = d / n * min(CRUISE_M_S, n) if n > 1e-6 else np.zeros(2)
        if len(self.scan_pts) and np.linalg.norm(v) > 0.05:
            rel = self.scan_pts - pos
            ahead = rel @ (v / np.linalg.norm(v))
            side = np.abs(rel @ np.array([-v[1], v[0]]) / np.linalg.norm(v))
            if np.any((ahead > 0) & (ahead < BRAKE_M + DRONE_RADIUS_M) & (side < DRONE_RADIUS_M + 0.2)):
                self.brakes += 1
                self._next_plan = 0.0  # replan now
                return np.zeros(2)
        return v

    def _dwa(self, pos: np.ndarray, carrot: tuple[float, float]) -> np.ndarray:
        lo, hi = self.v_cmd - ACCEL_M_S2 * WINDOW_S, self.v_cmd + ACCEL_M_S2 * WINDOW_S
        vx, vy = np.meshgrid(np.linspace(lo[0], hi[0], DWA_SAMPLES), np.linspace(lo[1], hi[1], DWA_SAMPLES))
        V = np.column_stack([vx.ravel(), vy.ravel(), ])
        V = V[np.linalg.norm(V, axis=1) <= CRUISE_M_S + 1e-9]
        V = np.vstack([V, np.zeros((1, 2))])  # stopping is always an option
        ts = np.linspace(0.1, HORIZON_S, 15)
        traj = pos[None, None, :] + V[:, None, :] * ts[None, :, None]  # (n, k, 2)
        d_static = np.full((len(V), len(ts)), RANGE_M)  # distance at each moment of each imagined trajectory
        if len(self.static_pts):
            d_static = np.linalg.norm(traj[:, :, None, :] - self.static_pts[None, None, :, :], axis=3).min(axis=2)
        d_moving = np.full((len(V), len(ts)), RANGE_M)
        for m in self.movers_seen:  # moving things: compare with where they WILL be at each moment
            future = m["pts"][None, :, :] + ts[:, None, None] * m["v"][None, None, :]  # (k, p, 2)
            d_moving = np.minimum(d_moving, np.linalg.norm(traj[:, :, None, :] - future[None], axis=3).min(axis=2))
        gap_static = d_static.min(axis=1) - DRONE_RADIUS_M
        gap_moving = d_moving.min(axis=1) - DRONE_RADIUS_M
        clear = np.minimum(gap_static, gap_moving - (MIN_CLEAR_MOVING_M - MIN_CLEAR_M))
        ok = clear > MIN_CLEAR_M
        c = np.array(carrot)
        progress = np.linalg.norm(c - pos) - np.linalg.norm(c - traj[:, -1, :], axis=1)
        score = W_PROGRESS * progress + W_CLEAR * np.minimum(clear, 1.5) + W_SPEED * np.linalg.norm(V, axis=1)
        if not ok.any():
            # Nothing is safe (something is coming at it): don't freeze, get out of the way. Every option starts
            # from the same spot, so judge them by the room they leave ESCAPE_S from now.
            self.brakes += 1
            k = int(np.searchsorted(ts, ESCAPE_S))
            room = np.minimum(d_static[:, k], d_moving[:, k])
            return V[int(np.argmax(room))]
        score[~ok] = -np.inf
        return V[int(np.argmax(score))]

    def setpoint_for(self, est: State, t: float, truth: State | None = None) -> Setpoint:
        x, y = float(est.pos_m[0]), float(est.pos_m[1])
        pos = np.array([x, y])
        if self.phase == "crashed":
            self.v_cmd = np.zeros(2)
            return Setpoint(armed=True, alt_m=FLY_ALT_M, position_control=True, x_m=x, y_m=y)
        if self.phase == "done":
            self.path = []
        at_goal = self.goal is not None and math.dist(pos, self.grid.center(*self.goal)) < GOAL_REACHED_M
        if self.phase == "explore" and (t >= self._next_plan or not self.path or at_goal):
            self._next_plan = t + REPLAN_S
            self._plan(x, y, t)
        if t >= self._next_local:
            # Even with no path (holding still while it thinks), keep the local planner running: something may be
            # walking towards it. With no path the 'carrot' is where it is now.
            self._next_local = t + LOCAL_PERIOD_S
            carrot = self._carrot(x, y) if self.path else (x, y)
            self.v_cmd = self._dwa(pos, carrot) if self.local == "dwa" else self._pursuit(pos, carrot)
        v = self.v_cmd
        return Setpoint(armed=True, alt_m=FLY_ALT_M, position_control=True, x_m=x + 0.1 * v[0], y_m=y + 0.1 * v[1],
                        vel_ff_m_s=(float(v[0]), float(v[1]), 0.0))


def run_explore(world_id: str = "rooms", local: str = "dwa", localisation: str = "gps", movers: bool = False,
                seed: int = 0, max_s: float = 240.0, config=None):
    """Explore one world headless. Returns (RunResult, Explorer)."""
    from backend.sim.config import get_preset
    from backend.sim.runner import run_headless
    from backend.sim.sensors import SensorConfig

    cfg = config or get_preset("basic")
    w = WORLDS[world_id]
    ex = Explorer(w, local, localisation, movers, seed)
    init = State.hovering(cfg, FLY_ALT_M)
    init.pos_m[:2] = w.start

    def extra(sim) -> dict[str, float]:
        return {"coverage": ex.coverage(), "clear_m": min(ex.clear_now_m, RANGE_M),
                "speed_cmd_m_s": float(np.linalg.norm(ex.v_cmd)),
                "est_err_m": float(np.hypot(*(sim.estimate.pos_m[:2] - sim.state.pos_m[:2])))}

    stop_at: list[float] = []

    def stop() -> bool:
        if ex.done and not stop_at:
            stop_at.append(ex.t + 1.0)
        return bool(stop_at) and ex.t >= stop_at[0]

    r = run_headless(cfg, ex, max_s, seed=seed, initial_state=init, run_type="explore",
                     sensors=SensorConfig(enabled=True, estimator="ekf"), before_step=ex.before_step,
                     extra_channels=extra, stop_when=stop)
    r.events = sorted(r.events + ex.events, key=lambda e: e["t_s"])
    return r, ex


def explore_metrics(r, ex: Explorer) -> dict:
    T = r.telemetry
    t = np.asarray(T["t_s"])
    cov = np.asarray(T["coverage"])
    err = np.hypot(np.subtract(T["x_est_m"], T["x_m"]), np.subtract(T["y_est_m"], T["y_m"]))
    xy = np.column_stack([T["x_m"], T["y_m"]])
    k90 = np.nonzero(cov >= 0.9)[0]
    return {"outcome": ex.phase, "coverage_pct": round(100 * float(cov[-1]), 1),
            "time_to_90_s": round(float(t[k90[0]]), 1) if len(k90) else None, "flight_time_s": round(float(t[-1]), 1),
            "path_length_m": round(float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum()), 1),
            "min_clearance_m": round(float(ex.min_clear_m), 2), "brakes": ex.brakes, "frontier_goals": len(ex.goals),
            "est_err_mean_m": round(float(err.mean()), 3), "est_err_max_m": round(float(err.max()), 3),
            "scan_matches": ex.matches, "map_cells": int(ex.grid.occupied().sum())}
