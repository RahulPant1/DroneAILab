"""Autonomous exploration (backend/autonomy/explore.py): mapping, frontiers, DWA, scan matching, experiment 13."""

import math

import numpy as np
import pytest

from backend.autonomy import explore as E
from backend.autonomy.world import Box
from backend.experiments import get_experiment


def test_grid_marks_free_along_beams_and_occupied_at_hits():
    g = E.Grid((0, 10, 0, 10))
    ranges = np.full(E.BEAMS, E.RANGE_M)
    ranges[0] = 3.0  # beam 0 (east) hits something 3 m away
    g.integrate(5.0, 5.0, E.BODY_ANGLES, ranges)
    i, j = g.index(5.0 + 3.0, 5.0)
    assert g.occupied()[i, j]
    i, j = g.index(6.5, 5.0)
    assert g.free()[i, j]  # passed through
    i, j = g.index(9.0, 5.0)
    assert not g.free()[i, j] and not g.occupied()[i, j]  # behind the hit: still unknown


def test_moving_obstacle_trail_is_cleared_by_later_beams():
    g = E.Grid((0, 10, 0, 10))
    hit = np.full(E.BEAMS, E.RANGE_M)
    hit[0] = 3.0
    g.integrate(5.0, 5.0, E.BODY_ANGLES, hit)
    i, j = g.index(8.0, 5.0)
    assert g.occupied()[i, j]
    for _ in range(4):  # it walked away: the beam now passes through
        g.integrate(5.0, 5.0, E.BODY_ANGLES, np.full(E.BEAMS, 4.5))
    assert g.free()[i, j]


def test_frontiers_are_free_cells_next_to_unknown():
    free = np.zeros((10, 10), bool)
    free[2:5, 2:5] = True
    unknown = ~free
    clusters = E.frontier_clusters(free, unknown, free)
    assert len(clusters) == 1 and len(clusters[0]) == 8  # the ring around the centre cell


def test_dwa_steers_round_an_obstacle_and_pursuit_brakes():
    w = E.World("test", [], (0.0, 0.0), (-10, 10, -10, 10))
    wall = np.column_stack([np.full(21, 1.0), np.linspace(-1.0, 1.0, 21)])  # a short wall 1 m ahead (east)
    for local in ("dwa", "pursuit"):
        ex = E.Explorer(w, local)
        ex.scan_pts = ex.static_pts = wall
        ex.v_cmd = np.array([1.0, 0.0])
        v = ex._dwa(np.zeros(2), (5.0, 0.0)) if local == "dwa" else ex._pursuit(np.zeros(2), (5.0, 0.0))
        if local == "dwa":
            assert abs(v[1]) > 0.3 or np.linalg.norm(v) < 0.6  # turns aside or slows: never straight on
            traj = np.array([v * t for t in np.linspace(0.1, E.HORIZON_S, 15)])
            assert np.linalg.norm(traj[:, None] - wall[None], axis=2).min() - E.DRONE_RADIUS_M > E.MIN_CLEAR_M
        else:
            assert np.allclose(v, 0.0) and ex.brakes == 1


def test_a_person_walking_through_seen_free_space_is_tracked_with_its_velocity():
    from backend.autonomy.lidar import scan

    w = E.World("test", [Box(-3.1, -3.0, -1.0, 1.0)], (0.0, 0.0), (-10, 10, -10, 10))  # a wall 3 m west
    ex = E.Explorer(w, "dwa")
    for sx in np.linspace(-0.5, 0.5, 5):  # from several spots, so beams cross every cell ahead several times
        for sy in np.linspace(-0.5, 0.5, 5):
            ex.grid.integrate(sx, sy, E.BODY_ANGLES, scan(w.walls, sx, sy, 1.5, E.RANGE_M, E.BODY_ANGLES))
    body = lambda cx: np.array([[cx + dx, 3.0 + dy] for dx in (-0.2, 0.0, 0.2) for dy in (-0.2, 0.2)])  # noqa: E731
    wall = np.array([[-3.1 + 0.05, y] for y in np.linspace(-0.8, 0.8, 5)])
    ex._track_movers(np.vstack([body(1.0), wall]))
    ex._track_movers(np.vstack([body(1.1), wall]))  # 0.1 m in 0.1 s
    assert len(ex.movers_seen) == 1
    assert ex.movers_seen[0]["v"] == pytest.approx([0.5, 0.0], abs=0.05)  # smoothed: half of 1 m/s after one step
    assert len(ex.static_pts) == len(wall)


def test_dwa_keeps_out_of_the_way_of_something_coming_at_it():
    w = E.World("test", [], (0.0, 0.0), (-10, 10, -10, 10))
    ex = E.Explorer(w, "dwa")
    person = np.array([[2.0 + dx, dy] for dx in (-0.2, 0.2) for dy in (-0.2, 0.2)])
    ex.movers_seen = [{"c": person.mean(axis=0), "v": np.array([-1.0, 0.0]), "pts": person}]  # walking at it
    v = ex._dwa(np.zeros(2), (0.0, 0.0))  # it wants to stay where it is
    ts = np.linspace(0.1, E.HORIZON_S, 15)
    gap = min(np.linalg.norm(v * t - (person + np.array([-1.0, 0.0]) * t), axis=1).min() for t in ts) - E.DRONE_RADIUS_M
    assert np.linalg.norm(v) > 0.3 and gap > 0.3  # steps aside instead of being walked into


def test_scan_matching_finds_a_known_offset():
    """Build a map of a room, then nudge the estimate 0.25 m: matching the next scan must recover it, to a
    few cm (the map keeps where each hit really was, not just its 0.2 m cell). Walls off the grid lines."""
    w = E.World("box", E._fence(0.07, 8.07, 0.13, 6.13) + [Box(3.07, 3.57, 2.13, 4.13)], (2.0, 1.5), (0, 8, 0, 6))
    ex = E.Explorer(w, "dwa", "slam")
    from backend.autonomy.lidar import scan

    x, y = 2.0, 1.5
    for _ in range(5):
        ex.grid.integrate(x, y, E.BODY_ANGLES, scan(w.walls, x, y, 1.5, E.RANGE_M, E.BODY_ANGLES))

    class FakeEKF:
        def __init__(self):
            from backend.sim.physics import State

            self.state = State.at(1.5)
            self.state.pos_m[:2] = (x + 0.2, y - 0.15)  # wrong by (0.2, -0.15)
            self.r_gps_pos = self.r_compass = 1.0
            self.fixes = []

        def fuse_gps(self, xy, vel):
            self.fixes.append(tuple(xy))

        def fuse_compass(self, h):
            self.heading = h

    ekf = FakeEKF()
    ex._scan_match(ekf, scan(w.walls, x, y, 1.5, E.RANGE_M, E.BODY_ANGLES))
    assert ekf.fixes and math.dist(ekf.fixes[0], (x, y)) < 0.04
    assert abs(ekf.heading) < math.radians(0.6) and ekf.r_gps_pos == 1.0  # its own noise settings put back


@pytest.mark.parametrize("world", ["rooms", "forest"])
def test_dwa_explores_each_world_without_touching_anything(world):
    r, ex = E.run_explore(world, "dwa", "gps")
    m = E.explore_metrics(r, ex)
    assert m["outcome"] == "done" and m["coverage_pct"] > 90 and m["min_clearance_m"] > 0.1


def test_no_gps_dead_reckoning_crashes_and_slam_does_not():
    r, ex = E.run_explore("rooms", "dwa", "dead_reckoning")
    assert ex.phase == "crashed" and E.explore_metrics(r, ex)["est_err_max_m"] > 1.0
    r, ex = E.run_explore("rooms", "dwa", "slam")
    m = E.explore_metrics(r, ex)
    assert m["outcome"] == "done" and m["coverage_pct"] > 90 and m["est_err_max_m"] < 0.4 and m["scan_matches"] > 100


def test_experiment_13_runs():
    r = get_experiment("exploration").runner({"world": "rooms", "local_planner": "dwa", "localisation": "gps",
                                              "movers": "on"}, 0)
    assert r.metrics["outcome"] == "done" and r.summary and {p["id"] for p in r.plots} >= {"map", "coverage", "clear"}
    extra = next(p for p in r.plots if p["id"] == "map")["extra"]
    assert extra["discovered"] and extra["waypoints"] and len(extra["estimate"][0]) == len(r.telemetry["t_s"])


# ---- live Fly view ----


def test_live_view_explores_and_streams_its_map():
    import base64

    from backend.autonomy.live import MissionSim
    from backend.sim.config import get_preset

    rt = MissionSim(get_preset("basic"), seed=0)
    assert "Exploring" in rt.command("explore", world="rooms", local="dwa", localisation="slam", movers=True)
    info = rt.course_info()
    assert info["explore"] and len(info["obstacles"]) == len(E.WORLDS["rooms"].walls)
    assert rt.sim.estimator.kind == "ekf"
    snap = rt.snapshot()["explore"]
    m = snap["map"]
    cells = np.frombuffer(base64.b64decode(m["cells"]), np.uint8).reshape(m["ny"], m["nx"])
    assert cells.max() <= 2 and snap["people"] and snap["phase"] == "takeoff"
    while rt.state.time_s < 90 and rt.explore_phase in ("takeoff", "explore"):
        rt.advance(0.05)
    snap = rt.snapshot()
    assert rt.explore_phase == "done" and snap["explore"]["coverage"] > 0.8 and snap["explore"]["est_err_m"] < 0.4
    assert any("frontier" in n for n in rt.notices) and rt.pilot.mode == "hover"
    rt.reset()
    assert rt.snapshot()["explore"] is None and rt.course_info()["explore"]  # Reset keeps the world, clears the run


def test_live_explore_hands_over_to_a_flying_button():
    from backend.autonomy.live import MissionSim
    from backend.sim.config import get_preset

    rt = MissionSim(get_preset("basic"), seed=0)
    rt.command("explore", world="forest")
    for _ in range(100):
        rt.advance(0.05)
    assert rt.explore_phase == "explore" and rt.snapshot()["lidar"]["ranges"]
    rt.command("hover")
    assert rt.explore_phase == "stopped" and rt.snapshot()["explore"]["phase"] == "stopped"
    with pytest.raises(ValueError):
        rt.command("explore", world="moon")
    rt.command("course", scenario=None)
    assert rt.course_info() is None
