"""Phase 4: missions, planners, obstacle avoidance and mission metrics."""

import math

import pytest
from fastapi.testclient import TestClient

from backend import api
from backend.autonomy.mission import SCENARIOS, MissionError, mission_from_dict, run_mission, scenario_mission
from backend.autonomy.planner import PlanningError, astar_leg, PlanInfo, path_length
from backend.autonomy.world import DRONE_RADIUS_M, SAFETY_MARGIN_M, Box, clearance_m
from backend.experiments import get_experiment
from backend.sim.environment import EnvConfig


def fly(scenario, planner, **kw):
    return run_mission(scenario_mission(scenario, **kw), planner)[0].metrics


# ---- the lesson matrix: which planner succeeds where ----

EXPECTED = {
    "around_block": {"straight": False, "potential": True, "astar": True},
    "slalom": {"straight": False, "potential": False, "astar": True},
    "box_canyon": {"straight": False, "potential": False, "astar": True},
    "patrol": {"straight": True, "potential": True, "astar": True},
}


@pytest.mark.parametrize("scenario", list(SCENARIOS))
@pytest.mark.parametrize("planner", ["straight", "potential", "astar"])
def test_who_succeeds_where(scenario, planner):
    m = fly(scenario, planner)
    assert m["completed"] == EXPECTED[scenario][planner]
    if m["completed"]:
        assert m["collisions"] == 0 and m["waypoints_reached"] == m["waypoints_total"]


def test_straight_lines_crash_and_potential_field_gets_stuck_in_the_trap():
    assert fly("box_canyon", "straight")["collisions"] == 1
    m = fly("box_canyon", "potential")
    assert m["got_stuck"] and m["collisions"] == 0


def test_astar_keeps_its_safety_margin_in_still_air():
    for sc in ("around_block", "slalom", "box_canyon"):
        assert fly(sc, "astar")["min_clearance_m"] > SAFETY_MARGIN_M - 0.1


def test_faster_flight_cuts_corners_more():
    slow, fast = fly("patrol", "astar", cruise_speed_m_s=0.8), fly("patrol", "astar", cruise_speed_m_s=3.0)
    assert fast["mission_time_s"] < slow["mission_time_s"]
    assert fast["cross_track_max_m"] > slow["cross_track_max_m"] + 0.1
    assert fast["max_waypoint_miss_m"] > slow["max_waypoint_miss_m"]


def test_crosswind_pushes_the_drone_toward_obstacles_but_astar_still_completes():
    calm = fly("slalom", "astar")
    windy = run_mission(scenario_mission("slalom"), "astar", env=EnvConfig(wind_speed_m_s=6, wind_direction_deg=90))[0].metrics
    assert windy["completed"] and windy["min_clearance_m"] < calm["min_clearance_m"]


def test_missions_are_deterministic():
    a = run_mission(scenario_mission("slalom"), "astar", env=EnvConfig(wind_speed_m_s=3, gust_std_m_s=1), seed=5)[0]
    b = run_mission(scenario_mission("slalom"), "astar", env=EnvConfig(wind_speed_m_s=3, gust_std_m_s=1), seed=5)[0]
    assert a.telemetry == b.telemetry


# ---- planner and world ----


def test_astar_route_is_short_safe_and_simplified():
    box = Box(4, 6, -1, 3)
    info = PlanInfo("astar")
    route = astar_leg((0, 0), (10, -2), [box], 2.0, info)
    assert len(route) <= 6  # zig-zags straightened
    assert path_length(route) < math.dist((0, 0), (10, -2)) * 1.25
    for (ax, ay), (bx, by) in zip(route, route[1:]):
        for i in range(51):
            px, py = ax + (bx - ax) * i / 50, ay + (by - ay) * i / 50
            assert box.distance_2d(px, py) >= DRONE_RADIUS_M + SAFETY_MARGIN_M - 0.01


def test_astar_reports_impossible_routes_readably():
    walled_in = [Box(-3, 3, 2, 3), Box(-3, 3, -3, -2), Box(2, 3, -3, 3), Box(-3, -2, -3, 3)]
    with pytest.raises(PlanningError, match="No safe path"):
        astar_leg((0, 0), (10, 0), walled_in, 2.0, PlanInfo("astar"))


def test_low_boxes_can_be_flown_over():
    low = Box(4, 6, -1, 1, height_m=1.0)
    assert clearance_m([low], 5, 0, 2.0) == math.inf  # cruising at 2 m, a 1 m box doesn't matter
    m = run_mission(mission_from_dict({"waypoints": [[0, 0], [10, 0]], "obstacles": [low.to_dict()]}), "straight")[0].metrics
    assert m["completed"]


@pytest.mark.parametrize(
    "mission, reason",
    [
        ({"waypoints": [[0, 0]]}, "2 to 20 waypoints"),
        ({"waypoints": [[0, 0], [5, 0]], "obstacles": [{"x_min": 4, "x_max": 6, "y_min": -1, "y_max": 1}]}, "inside or touching"),
        ({"waypoints": [[0, 0], [500, 0]]}, "within"),
        ({"waypoints": [[0, 0], [5, 0]], "obstacles": [{"x_min": 3, "x_max": 2, "y_min": 1, "y_max": 2}]}, "zero or negative size"),
        ({"waypoints": [[0, 0], [5, 0]], "cruise_speed_m_s": 9}, "Cruise speed"),
        ({"waypoints": "nope"}, "pairs"),
    ],
)
def test_bad_missions_are_rejected_readably(mission, reason):
    with pytest.raises(MissionError, match=reason):
        mission_from_dict(mission)


# ---- Lab experiment + API ----


def test_mission_experiment_has_a_map_plot_with_everything_needed():
    r = get_experiment("missions").run({}, 0)
    m = next(p for p in r.plots if p["kind"] == "map")
    assert {"obstacles", "waypoints", "planned_route", "safety_zone_m"} <= set(m["extra"])
    assert any("mission complete" in line for line in r.summary)


def test_api_custom_mission_and_scenarios():
    c = TestClient(api.app)
    assert set(c.get("/api/missions/scenarios").json()["scenarios"]) == set(SCENARIOS)
    body = {"mission": {"waypoints": [[0, 0], [8, 0]], "obstacles": [{"x_min": 3, "x_max": 5, "y_min": -1, "y_max": 1}]}, "planner": "astar"}
    res = c.post("/api/missions/run", json=body).json()
    assert res["metrics"]["completed"] and res["metrics"]["collisions"] == 0
    bad = c.post("/api/missions/run", json={"mission": {"waypoints": [[0, 0]]}})
    assert bad.status_code == 400 and "waypoints" in bad.json()["detail"]
    assert c.post("/api/missions/run", json={**body, "planner": "teleport"}).status_code == 400
