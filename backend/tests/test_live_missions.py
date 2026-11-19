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


# ---- GPS outage and the GPS-loss failsafe ----


def outage_drift(failsafe: bool, seed: int = 0, seconds: float = 20.0) -> tuple[MissionSim, float]:
    """Hover with 1x noise, lose GPS for `seconds`; return the sim and how far the drone really drifted."""
    import numpy as np

    rt = MissionSim(BASIC, seed=seed)
    rt.command("sensors", enabled=True, noise_scale=1.0)
    settle_gps(rt)
    rt.command("takeoff", alt_m=2.0)
    for _ in range(100):
        rt.advance(0.05)
    assert "dead reckoning" in rt.command("gps", outage=True, failsafe=failsafe)
    for _ in range(int(seconds / 0.05)):
        rt.advance(0.05)
    return rt, float(np.hypot(*rt.state.pos_m[:2]))


def test_without_gps_the_estimate_drifts_and_the_failsafe_limits_the_damage():
    rt_fs, with_fs = outage_drift(failsafe=True)
    rt_no, without = outage_drift(failsafe=False)
    assert without > 5.0  # dead reckoning on noisy motion sensors runs away quickly
    assert with_fs < 0.5 * without  # level + altitude hold drifts far less
    assert rt_fs.snapshot()["failsafe"] and not rt_no.snapshot()["failsafe"]
    assert abs(rt_fs.state.pos_m[2] - 2.0) < 0.3  # the altimeter still holds the height
    assert rt_fs.snapshot()["gps_lost_s"] > 19


def test_gps_return_brings_the_drone_back_and_ends_the_failsafe():
    import numpy as np

    rt, _ = outage_drift(failsafe=True, seconds=10.0)
    assert "GPS back" in rt.command("gps", outage=False)
    for _ in range(300):
        rt.advance(0.05)
    assert not rt.snapshot()["failsafe"] and rt.snapshot()["gps_lost_s"] == 0
    assert np.hypot(*rt.state.pos_m[:2]) < 1.0  # back near the hover point (0, 0)


def test_mission_aborts_on_gps_loss_and_take_off_needs_a_fix():
    rt = MissionSim(BASIC)
    rt.command("sensors", enabled=True, noise_scale=1.0)
    rt.load_course("patrol")
    rt.start_mission("astar")
    for _ in range(200):
        rt.advance(0.05)
    rt.command("gps", outage=True)
    notes = run_live(rt, max_s=3.0)
    assert rt.follower is None and "Mission aborted: GPS lost." in notes
    rt.reset()  # the outage is kept
    assert "No GPS fix" in rt.command("takeoff")


def test_gps_outage_does_nothing_with_perfect_sensors():
    rt = MissionSim(BASIC)
    assert "only matters with sensor noise on" in rt.command("gps", outage=True)
    assert rt.gps_lost_s() == 0 and "Taking off" in rt.command("takeoff")


# ---- failsafe: W/A/S/D tilt the drone (attitude mode) ----


def press(rt: MissionSim, seconds: float = 1.5, coast_s: float = 2.0, **sticks) -> "np.ndarray":
    import numpy as np

    p0 = rt.state.pos_m[:2].copy()
    rt.command("sticks", **sticks)
    for _ in range(int(seconds / 0.05)):
        rt.advance(0.05)
    rt.command("sticks")
    for _ in range(int(coast_s / 0.05)):
        rt.advance(0.05)
    return np.asarray(rt.state.pos_m[:2] - p0)


@pytest.mark.parametrize("yaw_deg", [0.0, 90.0])
def test_in_failsafe_w_and_a_tilt_the_drone_forward_and_left_for_its_heading(yaw_deg):
    import math

    import numpy as np

    rt, _ = outage_drift(failsafe=True, seconds=1.5)
    rt.pilot.sp.yaw_rad = math.radians(yaw_deg)  # face east (0) or north (90)
    for _ in range(60):
        rt.advance(0.05)
    target = (rt.pilot.sp.x_m, rt.pilot.sp.y_m)
    fwd = np.array([math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))])
    left = np.array([-fwd[1], fwd[0]])
    moved = press(rt, forward=1.0)
    assert moved @ fwd > 2.0  # W: nose down, forward
    rt2, _ = outage_drift(failsafe=True, seconds=1.5)
    rt2.pilot.sp.yaw_rad = math.radians(yaw_deg)
    for _ in range(60):
        rt2.advance(0.05)
    assert press(rt2, left=1.0) @ left > 2.0  # A: left side down, left
    assert (rt.pilot.sp.x_m, rt.pilot.sp.y_m) == target  # the position target stays frozen


def test_after_steering_in_failsafe_gps_return_brings_it_back_to_the_frozen_target():
    import numpy as np

    rt, _ = outage_drift(failsafe=True, seconds=1.5)
    target = np.array([rt.pilot.sp.x_m, rt.pilot.sp.y_m])
    press(rt, forward=1.0)
    press(rt, left=1.0)
    rt.command("sticks", up=1.0)  # R still climbs in failsafe
    for _ in range(20):
        rt.advance(0.05)
    rt.command("sticks")
    assert rt.pilot.sp.alt_m > 2.5 and np.hypot(*(rt.state.pos_m[:2] - target)) > 3.0
    rt.command("gps", outage=False)
    for _ in range(300):
        rt.advance(0.05)
    assert np.hypot(*(rt.state.pos_m[:2] - target)) < 1.0


def test_failsafe_extra_drag_lets_the_drone_coast_to_a_stop_and_switches_off_with_gps():
    import numpy as np

    rt, _ = outage_drift(failsafe=True, seconds=1.5)
    assert rt.sim.physics.extra_drag_n_s_m > 0  # on in failsafe
    press(rt, seconds=1.5, coast_s=6.0, forward=1.0)
    assert np.hypot(*rt.state.vel_m_s[:2]) < 0.2  # coasted (nearly) to a stop by itself
    rt.command("gps", outage=False)
    rt.advance(0.05)
    assert rt.sim.physics.extra_drag_n_s_m == 0.0  # real physics again once GPS is back


def test_failsafe_drag_setting_zero_means_real_physics_and_it_keeps_coasting():
    import numpy as np
    import pytest as _pytest

    rt, _ = outage_drift(failsafe=True, seconds=1.5)
    msg = rt.command("gps", outage=True, failsafe=True, drag_n_s_m=0.0)
    assert "real physics" in msg
    press(rt, seconds=1.5, coast_s=6.0, forward=1.0)
    assert rt.sim.physics.extra_drag_n_s_m == 0.0
    assert np.hypot(*rt.state.vel_m_s[:2]) > 1.0  # no helper: still coasting 6 s later
    rt.reset()
    assert rt.failsafe_drag_n_s_m == 0.0  # the setting survives a reset, like wind and sensors
    with _pytest.raises(ValueError):
        rt.command("gps", outage=True, drag_n_s_m=5.0)
