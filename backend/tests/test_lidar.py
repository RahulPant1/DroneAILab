"""📡 Sense-and-avoid: the lidar's ray casting, the map and its planner, and missions flown knowing no obstacles."""

import math

import numpy as np
import pytest

from backend.autonomy.lidar import ANGLES, BEAMS, KEEP_OUT_M, RANGE_M, OccupancyMap, scan
from backend.autonomy.live import MissionSim
from backend.autonomy.mission import run_mission, scenario_mission
from backend.autonomy.world import Box
from backend.sim.config import get_preset


def beam(angle_deg: float) -> int:
    return int(round(math.radians(angle_deg) / (2 * math.pi) * BEAMS)) % BEAMS


def test_beams_measure_the_distance_to_the_first_box_they_hit():
    wall = Box(3.0, 3.5, -1.0, 1.0, height_m=4.0)
    r = scan([wall], 0.0, 0.0, 2.0)
    assert r[beam(0)] == pytest.approx(3.0)  # straight east: 3 m to the wall's face
    assert r[beam(90)] == RANGE_M and r[beam(180)] == RANGE_M  # nothing north or west
    assert r[beam(15)] == pytest.approx(3.0 / math.cos(math.radians(15)), rel=1e-6)
    assert np.all(scan([wall], 0.0, 0.0, 5.0) == RANGE_M)  # flying above it: it sees nothing
    near, far = Box(2.0, 2.2, -1.0, 1.0), Box(5.0, 5.2, -1.0, 1.0)
    assert scan([far, near], 0.0, 0.0, 1.0)[beam(0)] == pytest.approx(2.0)  # the nearer one hides the farther
    assert len(ANGLES) == BEAMS == 72


def test_the_map_blocks_a_margin_around_hits_and_plans_around_them():
    m = OccupancyMap()
    wall = Box(3.0, 3.4, -3.0, 3.0)
    for y in np.linspace(-2.5, 2.5, 11):  # look at the wall from a few places
        m.add_hits(0.0, y, scan([wall], 0.0, y, 1.0))
    assert m.is_blocked(3.0, 0.0) and m.is_blocked(3.0 - KEEP_OUT_M + 0.1, 0.0) and not m.is_blocked(1.5, 0.0)
    path = m.plan((0.0, 0.0), (6.0, 0.0))
    assert path is not None and path[0] == (0.0, 0.0) and path[-1] == (6.0, 0.0)
    assert all(m.segment_free(a, b) for a, b in zip(path[1:], path[2:]))
    assert max(abs(p[1]) for p in path) > 3.0  # it goes round the end of the wall
    version = m.version
    m.add_hits(0.0, 0.0, scan([wall], 0.0, 0.0, 1.0))
    assert m.version == version  # nothing new seen: no new map version


def test_no_path_when_the_goal_is_walled_in():
    m = OccupancyMap()
    ring = [Box(4, 4.3, -3, 3), Box(7.7, 8, -3, 3), Box(4, 8, 2.7, 3), Box(4, 8, -3, -2.7)]
    for x in (0.0, 6.0):
        for y in (-5.0, 0.0, 5.0):
            m.add_hits(x if x == 0 else 2.0, y, scan(ring, x if x == 0 else 2.0, y, 1.0))
            m.add_hits(10.0, y, scan(ring, 10.0, y, 1.0))
    m.add_hits(6.0, 6.0, scan(ring, 6.0, 6.0, 1.0))
    m.add_hits(6.0, -6.0, scan(ring, 6.0, -6.0, 1.0))
    assert m.plan((0.0, 0.0), (6.0, 0.0)) is None


def test_on_hidden_walls_it_discovers_the_second_wall_and_still_gets_there():
    lidar, _, _ = run_mission(scenario_mission("hidden_walls"), "lidar")
    astar, _, _ = run_mission(scenario_mission("hidden_walls"), "astar")
    m = lidar.metrics
    assert m["completed"] and m["collisions"] == 0
    assert m["replans"] >= 2 and m["map_cells"] > 100
    assert astar.metrics["flown_length_m"] <= m["flown_length_m"] < 1.3 * astar.metrics["flown_length_m"]
    replans = [e["t_s"] for e in lidar.events if e["type"] == "replan"]
    assert replans[-1] - replans[0] > 3.0  # the second wall was found well after the first
    assert lidar.scenario["discovered"] and len(lidar.scenario["plans"]) == m["replans"] + 1


def test_live_lidar_mission_and_map_updates_are_sent_only_when_they_change():
    rt = MissionSim(get_preset("basic"))
    rt.command("course", scenario="hidden_walls")
    rt.command("mission", planner="lidar")
    frames_with_map = frames = 0
    for _ in range(int(60 / 0.05)):
        rt.advance(0.05)
        snap = rt.snapshot()
        frames += 1
        frames_with_map += "map" in (snap["lidar"] or {})
        if rt.follower is None:
            break
    assert "Mission complete" in rt.pilot.message
    assert 0 < frames_with_map < frames / 2
    assert "Lidar on" in rt.command("lidar", enabled=True)
    assert rt.command("lidar", enabled=False) == "Lidar off." and rt.snapshot()["lidar"] is None
