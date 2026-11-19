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


# ---- wind in the live view ----


def test_live_wind_pushes_the_drone_and_the_controller_pulls_it_back():
    rt = MissionSim(BASIC)
    rt.command("takeoff", alt_m=2.0)
    for _ in range(100):
        rt.advance(0.05)
    assert "6.0 m/s" in rt.command("wind", speed_m_s=6.0, direction_deg=0.0, gust_m_s=0.0)
    xs = []
    for _ in range(200):
        rt.advance(0.05)
        xs.append(float(rt.state.pos_m[0]))
    assert max(xs) > 0.2  # blown downwind (east) at first...
    assert abs(xs[-1]) < 0.15  # ...then the speed I-term learns the push and brings it back
    assert rt.snapshot()["wind_m_s"][0] == pytest.approx(6.0)


def test_wind_survives_reset_and_calm_turns_it_off():
    rt = MissionSim(BASIC)
    rt.command("wind", speed_m_s=4.0, direction_deg=90.0, gust_m_s=1.0)
    rt.reset()
    assert rt.env.wind_speed_m_s == 4.0 and rt.sim.wind is not None
    assert "calm" in rt.command("wind", speed_m_s=0, direction_deg=0, gust_m_s=0)
    assert rt.sim.wind is None and rt.snapshot()["wind_m_s"] == [0.0, 0.0, 0.0]
    with pytest.raises(ValueError, match="wind_speed_m_s"):
        rt.command("wind", speed_m_s=99)


def test_crosswind_shrinks_the_gap_in_a_live_mission_like_in_the_lab():
    def min_gap(wind):
        rt = MissionSim(BASIC)
        rt.command("wind", speed_m_s=wind, direction_deg=90.0, gust_m_s=0.0)
        rt.load_course("slalom")
        rt.start_mission("astar")
        gap = 9.0
        while rt.follower:
            rt.advance(0.05)
            if rt.state.pos_m[2] > 0.05:
                gap = min(gap, clearance_m(rt.mission.obstacles, *rt.state.pos_m))
        return gap

    calm, windy = min_gap(0.0), min_gap(6.0)
    assert windy < calm - 0.08 and windy > 0  # closer to the boxes, but A*'s margin still saves it


# ---- sensor noise in the live view ----


def settle_gps(rt: MissionSim) -> None:
    """With sensor noise on, take-off is refused until the GPS has settled (5 s on the ground)."""
    for _ in range(200):
        if rt.gps_settle_left_s() == 0:
            return
        rt.advance(0.05)
    raise AssertionError("GPS never settled")


def hover_spread(rt: MissionSim) -> tuple[float, float]:
    """Fly 15 s at 2 m; return (horizontal wander std, mean estimate error)."""
    import numpy as np

    settle_gps(rt)
    rt.command("takeoff", alt_m=2.0)
    for _ in range(100):
        rt.advance(0.05)
    pos, err = [], []
    for _ in range(300):
        rt.advance(0.05)
        snap = rt.snapshot()
        pos.append(snap["pos_m"][:2])
        if snap["est_pos_m"]:
            err.append(float(np.linalg.norm(np.subtract(snap["est_pos_m"], snap["pos_m"]))))
    return float(np.std(pos, axis=0).max()), float(np.mean(err)) if err else 0.0


def test_perfect_sensors_by_default_so_the_live_view_is_unchanged():
    rt = MissionSim(BASIC)
    assert rt.sim.estimator is None and rt.snapshot()["est_pos_m"] is None
    wander, err = hover_spread(rt)
    assert wander < 0.01 and err == 0.0


def test_more_sensor_noise_means_more_wander_and_a_worse_estimate():
    results = {}
    for k in (1.0, 3.0):
        rt = MissionSim(BASIC)
        assert "estimate" in rt.command("sensors", enabled=True, noise_scale=k)
        results[k] = hover_spread(rt)
    assert 0.03 < results[1.0][0] < results[3.0][0] < 1.0
    assert 0.03 < results[1.0][1] < results[3.0][1]


def test_sensor_setting_survives_reset_and_can_be_switched_off_mid_flight():
    rt = MissionSim(BASIC)
    rt.command("sensors", enabled=True, noise_scale=2.0)
    rt.reset()
    assert rt.sim.estimator is not None
    settle_gps(rt)
    rt.command("takeoff", alt_m=2.0)
    for _ in range(60):
        rt.advance(0.05)
    assert "Perfect" in rt.command("sensors", enabled=False)
    assert rt.sim.estimator is None and rt.snapshot()["est_pos_m"] is None
    for _ in range(60):
        rt.advance(0.05)
    assert abs(rt.state.pos_m[2] - 2.0) < 0.3  # still flying fine
    with pytest.raises(ValueError, match="noise_scale"):
        rt.command("sensors", enabled=True, noise_scale=50)


# ---- the mission autopilot navigates on the estimate ----


def fly_noisy(scenario: str, noise: float, seed: int) -> list[str]:
    rt = MissionSim(BASIC, seed=seed)
    if noise:
        rt.command("sensors", enabled=True, noise_scale=noise)
    rt.load_course(scenario)
    rt.start_mission("astar")
    return run_live(rt)


def test_noisy_autopilot_still_lands_properly_and_reports_the_real_miss():
    notes = fly_noisy("slalom", 1.0, seed=0)
    assert notes[-1] == "Landed at the final waypoint."  # ground contact comes from the truth, so no early motor cut
    wp = next(n for n in notes if "waypoint 1" in n)
    assert "by its estimate. Really" in wp


def test_very_noisy_sensors_can_fool_the_autopilot_into_a_box():
    # 3x noise: the estimate is often wrong by more than A*'s 0.4 m margin, so some slalom runs hit a box.
    outcomes = [fly_noisy("slalom", 3.0, seed)[-1] for seed in range(5)]
    assert any("Hit an obstacle" in o for o in outcomes)
    assert all("Hit an obstacle" not in fly_noisy("slalom", 0.0, seed)[-1] for seed in range(2))


# ---- the buttons (take-off / hover / fly-to / land) also act on the estimate ----


def test_buttons_use_the_estimate_and_still_land_safely_with_noise():
    import numpy as np

    rt = MissionSim(BASIC, seed=0)
    rt.command("sensors", enabled=True, noise_scale=3.0)
    settle_gps(rt)
    est = rt.sim.estimate.pos_m.copy()
    rt.command("takeoff", alt_m=2.0)
    assert (rt.pilot.sp.x_m, rt.pilot.sp.y_m) == (float(est[0]), float(est[1]))  # target = where it THINKS it is
    for _ in range(100):
        rt.advance(0.05)
    assert rt.pilot.mode == "hover"
    rt.command("target", x_m=4.0, y_m=3.0, alt_m=2.0)
    for _ in range(200):
        rt.advance(0.05)
    assert 0.05 < np.hypot(rt.state.pos_m[0] - 4.0, rt.state.pos_m[1] - 3.0) < 1.5  # off by the estimate error
    rt.command("land")
    for _ in range(400):
        rt.advance(0.05)
        if rt.pilot.mode == "idle":
            break
    assert rt.pilot.mode == "idle" and rt.state.on_ground  # disarmed on real ground contact, not mid-air


# ---- GPS settle before arming ----


def test_take_off_waits_for_gps_and_averaging_shrinks_the_error():
    import numpy as np

    rt = MissionSim(BASIC, seed=0)
    assert "GPS settling for 5 s" in rt.command("sensors", enabled=True, noise_scale=3.0)
    assert "GPS settling" in rt.command("takeoff") and rt.pilot.mode == "idle"
    rt.advance(0.05)
    rt.advance(0.05)
    early = float(np.linalg.norm(rt.sim.estimate.pos_m[:2] - rt.state.pos_m[:2]))
    assert rt.snapshot()["gps_settle_left_s"] > 4.5
    settle_gps(rt)
    late = float(np.linalg.norm(rt.sim.estimate.pos_m[:2] - rt.state.pos_m[:2]))
    assert late < early and late < 0.3  # 50 averaged fixes: about 1/7 of a single fix's error
    assert np.allclose(rt.sim.estimate.vel_m_s, 0.0)  # it knows it is standing still
    assert "Taking off" in rt.command("takeoff")


def test_no_settle_with_perfect_sensors_mid_flight_switch_or_after_landing():
    rt = MissionSim(BASIC)
    assert rt.gps_settle_left_s() == 0 and rt.snapshot()["gps_settle_left_s"] == 0
    rt.command("takeoff", alt_m=2.0)
    for _ in range(60):
        rt.advance(0.05)
    rt.command("sensors", enabled=True, noise_scale=1.0)  # switched on in the air: can't average, no wait
    assert rt.gps_settle_left_s() == 0
    rt.command("land")
    for _ in range(200):
        rt.advance(0.05)
    assert rt.pilot.mode == "idle" and "Taking off" in rt.command("takeoff")  # already settled once


def test_mission_waits_on_the_ground_for_gps_then_flies():
    rt = MissionSim(BASIC)
    rt.command("sensors", enabled=True, noise_scale=1.0)
    rt.load_course("patrol")
    assert "settle" in rt.start_mission("astar")
    for _ in range(40):
        rt.advance(0.05)
    snap = rt.snapshot()
    assert snap["mission"]["phase"] == "gps" and snap["pos_m"][2] < 0.01
    assert run_live(rt)[-1] == "Landed at the final waypoint."


# ---- GPS noise on/off (other sensors still noisy) ----


def test_gps_noise_is_most_of_the_sideways_wander():
    on = MissionSim(BASIC, seed=0)
    on.command("sensors", enabled=True, noise_scale=3.0, gps_noise=True)
    off = MissionSim(BASIC, seed=0)
    assert "GPS is perfect" in off.command("sensors", enabled=True, noise_scale=3.0, gps_noise=False)
    assert off.sim.sensors.gps_sigma == 0 and off.sim.sensors.gyro_sigma > 0  # only the GPS is clean
    wander_on, err_on = hover_spread(on)
    wander_off, err_off = hover_spread(off)
    assert wander_off < 0.5 * wander_on and err_off < 0.3 * err_on


def test_gps_noise_defaults_to_on_so_lab_and_older_clients_are_unchanged():
    from backend.sim.sensors import SensorConfig

    assert SensorConfig().gps_noise_scale == 1.0
    rt = MissionSim(BASIC)
    rt.command("sensors", enabled=True, noise_scale=1.0)  # no gps_noise field
    assert rt.sim.sensors.gps_sigma > 0
