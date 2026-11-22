"""Path planning: how to get from waypoint to waypoint without hitting anything.

Three approaches, from simplest to smartest:
  straight  : just connect the waypoints with straight lines. Ignores obstacles completely.
  potential : no plan at all. While flying, the goal PULLS and nearby obstacles PUSH (see follower.py).
              Simple and reactive, but it only "sees" what's close, so it can get stuck.
  astar     : before take-off, cover the map with a grid, mark cells too close to obstacles as blocked,
              and search for the shortest chain of free cells (A* search). Then straighten the zig-zags.
"""

from __future__ import annotations

import heapq
import math
import time
from dataclasses import dataclass

from .world import DRONE_RADIUS_M, SAFETY_MARGIN_M, Box

GRID_RESOLUTION_M = 0.2
MAP_BORDER_M = 3.0  # how far beyond the waypoints/obstacles the search may go
MAX_GRID_CELLS = 250_000  # keeps planning fast and memory small

Point = tuple[float, float]


class PlanningError(ValueError):
    """No safe path could be found. The message is plain English."""


@dataclass
class PlanInfo:
    planner: str
    cells_expanded: int = 0
    planning_time_ms: float = 0.0
    grid_resolution_m: float = 0.0


def path_length(path: list[Point]) -> float:
    return sum(math.dist(a, b) for a, b in zip(path, path[1:]))


def _free(boxes: list[Box], x: float, y: float, keep_out_m: float) -> bool:
    return all(b.distance_2d(x, y) >= keep_out_m for b in boxes)


def _segment_free(boxes: list[Box], a: Point, b: Point, keep_out_m: float, step_m: float) -> bool:
    """Is the straight line a→b at least keep_out_m away from every obstacle? (checked every step_m)"""
    n = max(1, math.ceil(math.dist(a, b) / step_m))
    return all(_free(boxes, a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n, keep_out_m) for i in range(n + 1))


def astar_leg(start: Point, goal: Point, boxes: list[Box], cruise_alt_m: float, info: PlanInfo,
              resolution_m: float = GRID_RESOLUTION_M) -> list[Point]:
    """Shortest safe route between two points on a grid, then shortened with straight-line shortcuts.
    A coarser grid (resolution_m) lets it plan across a whole farm."""
    tall = [b for b in boxes if b.blocks_altitude(cruise_alt_m)]
    keep_out = DRONE_RADIUS_M + SAFETY_MARGIN_M
    for name, p in (("start", start), ("goal", goal)):
        if not _free(tall, *p, keep_out - 1e-9):
            raise PlanningError(f"The {name} point ({p[0]:.1f}, {p[1]:.1f}) is too close to an obstacle (needs {keep_out:.1f} m of space).")

    res = resolution_m
    xs = [start[0], goal[0]] + [v for b in tall for v in (b.x_min, b.x_max)]
    ys = [start[1], goal[1]] + [v for b in tall for v in (b.y_min, b.y_max)]
    x0, y0 = min(xs) - MAP_BORDER_M, min(ys) - MAP_BORDER_M
    nx = int(math.ceil((max(xs) + MAP_BORDER_M - x0) / res)) + 1
    ny = int(math.ceil((max(ys) + MAP_BORDER_M - y0) / res)) + 1
    if nx * ny > MAX_GRID_CELLS:
        raise PlanningError("The mission area is too big for the planner's grid. Keep waypoints closer together.")

    def to_cell(p: Point) -> tuple[int, int]:
        return round((p[0] - x0) / res), round((p[1] - y0) / res)

    def to_point(c: tuple[int, int]) -> Point:
        return x0 + c[0] * res, y0 + c[1] * res

    blocked_cache: dict[tuple[int, int], bool] = {}

    def blocked(c: tuple[int, int]) -> bool:
        if c not in blocked_cache:
            inside = 0 <= c[0] < nx and 0 <= c[1] < ny
            blocked_cache[c] = not inside or not _free(tall, *to_point(c), keep_out)
        return blocked_cache[c]

    s, g = to_cell(start), to_cell(goal)
    blocked_cache[s] = blocked_cache[g] = False  # exact start/goal were checked above
    moves = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0)] + [(dx, dy, math.sqrt(2)) for dx in (1, -1) for dy in (1, -1)]
    h = lambda c: math.hypot(c[0] - g[0], c[1] - g[1])  # noqa: E731  (straight-line guess of the remaining distance)
    open_heap = [(h(s), 0.0, s)]
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    cost = {s: 0.0}
    while open_heap:
        _, c_cost, c = heapq.heappop(open_heap)
        if c == g:
            break
        if c_cost > cost[c]:
            continue  # an outdated queue entry
        info.cells_expanded += 1
        for dx, dy, step in moves:
            n = (c[0] + dx, c[1] + dy)
            if blocked(n) or (dx and dy and (blocked((c[0] + dx, c[1])) or blocked((c[0], c[1] + dy)))):
                continue  # no squeezing diagonally past a corner
            new_cost = c_cost + step
            if new_cost < cost.get(n, math.inf):
                cost[n] = new_cost
                came_from[n] = c
                heapq.heappush(open_heap, (new_cost + h(n), new_cost, n))
    else:
        raise PlanningError(f"No safe path from ({start[0]:.1f}, {start[1]:.1f}) to ({goal[0]:.1f}, {goal[1]:.1f}): obstacles block every route.")

    cells = [g]
    while cells[-1] != s:
        cells.append(came_from[cells[-1]])
    raw = [start] + [to_point(c) for c in reversed(cells[1:-1])] + [goal]

    # Straighten: from each point, jump to the farthest later point that is in clear line of sight.
    short = [raw[0]]
    i = 0
    while i < len(raw) - 1:
        j = len(raw) - 1
        while j > i + 1 and not _segment_free(tall, raw[i], raw[j], keep_out - 1e-6, res / 2):
            j -= 1
        short.append(raw[j])
        i = j
    return short


def plan_route(waypoints: list[Point], boxes: list[Box], planner: str, cruise_alt_m: float,
               resolution_m: float = GRID_RESOLUTION_M) -> tuple[list[Point], PlanInfo]:
    """The full planned route through all waypoints (for 'potential', just the waypoints themselves)."""
    info = PlanInfo(planner=planner)
    if planner in ("straight", "potential"):
        return list(waypoints), info
    if planner != "astar":
        raise PlanningError(f"Unknown planner '{planner}'.")
    started = time.perf_counter()
    route = [waypoints[0]]
    for a, b in zip(waypoints, waypoints[1:]):
        route += astar_leg(a, b, boxes, cruise_alt_m, info, resolution_m)[1:]
    info.planning_time_ms = (time.perf_counter() - started) * 1000.0
    info.grid_resolution_m = resolution_m
    return route, info
