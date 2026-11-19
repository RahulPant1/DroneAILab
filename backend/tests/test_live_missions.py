"""Missions in the live Fly view (MissionSim): same outcomes as the Lab, take-over, crashes, WebSocket."""

import pytest
from fastapi.testclient import TestClient

from backend import api
from backend.autonomy.live import MissionSim
from backend.autonomy.world import clearance_m
from backend.sim.config import get_preset

from .test_phase4 import EXPECTED

BASIC = get_preset("basic")


def run_live(rt: MissionSim, max_s: float = 120.0) -> list[str]:
    """Advance in 50 ms chunks (like the WebSocket loop) until the autopilot hands back control."""
    notes: list[str] = []
    for _ in range(int(max_s / 0.05)):
        rt.advance(0.05)
        notes += rt.notices
        rt.notices.clear()
        if not rt.follower:
            break
    return notes


@pytest.mark.parametrize("scenario", list(EXPECTED))
@pytest.mark.parametrize("planner", ["straight", "potential", "astar"])
def test_live_missions_match_the_lab(scenario, planner):
    rt = MissionSim(BASIC)
    rt.load_course(scenario)
    rt.start_mission(planner)
    notes = run_live(rt)
    assert rt.follower is None, "the autopilot should have finished"
    assert ("Landed at the final waypoint." in notes) == EXPECTED[scenario][planner]
    if any("Stuck" in n for n in notes):
        assert rt.pilot.mode == "hover"  # stuck: it hovers, and the controls come back to you
    else:
        assert rt.pilot.mode == "idle"


def test_course_puts_the_drone_on_the_start_and_free_flight_clears_it():
    rt = MissionSim(BASIC)
    rt.load_course("patrol")
    info = rt.course_info()
    assert info["id"] == "patrol" and info["route"] is None and len(info["obstacles"]) == 1
    assert list(rt.state.pos_m[:2]) == list(info["waypoints"][0])
    rt.start_mission("astar", 2.0)
    assert rt.course_info()["route"][0] == [0.0, 0.0] and rt.course_info()["planner"] == "astar"
    rt.load_course(None)
    assert rt.course_info() is None and rt.follower is None


def test_flying_into_an_obstacle_by_hand_stops_the_motors_beside_it():
    rt = MissionSim(BASIC)
    rt.load_course("around_block")
    rt.command("takeoff", alt_m=2.0)
    for _ in range(80):
        rt.advance(0.05)
    rt.command("target", x_m=10.0, y_m=1.0)  # straight through the block
    for _ in range(200):
        rt.advance(0.05)
    assert rt.pilot.mode == "idle" and not rt.pilot.sp.armed
    assert any("Hit an obstacle" in n for n in rt.notices)
    assert rt.state.on_ground and clearance_m(rt.mission.obstacles, *rt.state.pos_m) > -0.1  # beside, not inside


def test_hover_takes_over_from_the_autopilot_and_takeoff_is_refused_meanwhile():
    rt = MissionSim(BASIC)
    rt.load_course("patrol")
    rt.start_mission("astar")
    for _ in range(100):
        rt.advance(0.05)
    assert rt.snapshot()["mode"] == "mission" and rt.snapshot()["mission"]["phase"] == "cruise"
    assert "mission is flying" in rt.command("takeoff")
    rt.command("hover")
    assert rt.follower is None and rt.pilot.mode == "hover" and rt.snapshot()["mission"] is None


def test_bad_mission_requests_are_plain_english_errors():
    rt = MissionSim(BASIC)
    with pytest.raises(ValueError, match="Load a course"):
        rt.start_mission("astar")
    rt.load_course("slalom")
    with pytest.raises(ValueError, match="Planner"):
        rt.start_mission("teleport")
    with pytest.raises(ValueError, match="speed"):
        rt.start_mission("astar", 9.0)
    with pytest.raises(ValueError, match="Unknown course"):
        rt.load_course("moon")


def test_websocket_loads_a_course_and_flies_a_mission():
    with TestClient(api.app).websocket_connect("/ws/sim") as ws:
        assert ws.receive_json()["course"] is None
        ws.send_json({"cmd": "course", "scenario": "box_canyon"})
        msg = next(m for m in (ws.receive_json() for _ in range(10)) if m["type"] == "course")
        assert len(msg["course"]["obstacles"]) == 3 and msg["course"]["route"] is None
        ws.send_json({"cmd": "mission", "planner": "astar", "cruise_speed_m_s": 1.5})
        msg = next(m for m in (ws.receive_json() for _ in range(10)) if m["type"] == "course")
        assert len(msg["course"]["route"]) > 2  # A* goes around, so more than start + goal
        state = next(m for m in (ws.receive_json() for _ in range(10)) if m["type"] == "state")
        assert state["mode"] == "mission" and state["mission"]["waypoints_total"] == 1
        ws.send_json({"cmd": "mission", "planner": "nope"})
        assert any(m["type"] == "error" for m in (ws.receive_json() for _ in range(10)))
