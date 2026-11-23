"""The Kalman filter (backend/sim/ekf.py), experiment 12, and the ArduPilot EKF3 replay (backend/sitl/ekf_replay.py).

The replay helpers are checked offline. The real flight (ArduCopter SITL with LOG_REPLAY, about 3 minutes) only runs
when asked:  DRONELAB_SITL=1 .venv/bin/pytest -q backend/tests/test_ekf.py
"""

import math
import os

import numpy as np
import pytest

from backend.autonomy.agile import run_track
from backend.autonomy.live import MissionSim
from backend.experiments import get_experiment
from backend.experiments.estimators import _fly
from backend.sim.config import ConfigError, get_preset
from backend.sim.ekf import EKF, GPS_RETRY_S
from backend.sim.physics import State
from backend.sim.runner import LightweightSim
from backend.sim.sensors import SensorConfig, validate_sensors
from backend.sitl import ekf_replay as R
from backend.sitl import launch

BASIC = get_preset("basic")


def test_unknown_estimator_is_refused():
    assert any("estimator" in p for p in validate_sensors(SensorConfig(enabled=True, estimator="magic")))
    with pytest.raises(ConfigError):
        LightweightSim().reset(BASIC, 0, None, None, SensorConfig(enabled=True, estimator="magic"))


def test_gps_velocity_has_its_own_noise_so_other_sensors_are_unchanged():
    """Adding GPS velocity must not change any other reading (old runs with the same seed stay the same)."""
    runs = []
    for est in ("complementary", "ekf"):
        sim = LightweightSim()
        sim.reset(BASIC, 3, State.hovering(BASIC, 2.0), None, SensorConfig(enabled=True, estimator=est))
        r = sim.sensors.sample(sim.state, np.zeros(3))
        runs.append((r.gyro_rad_s.copy(), r.accel_m_s2.copy(), r.gps_xy_m.copy(), r.baro_m))
    for a, b in zip(*runs):
        assert np.allclose(a, b)


def test_ekf_flies_the_figure_eight_where_the_complementary_filter_misses_rings():
    """The banked-turn problem from experiment 11: GPS velocity keeps the EKF's tilt right."""
    ekf = run_track("figure_eight", "geometric", 12.0, sensors=SensorConfig(enabled=True, estimator="ekf"), tilt_deg=30)
    cf = run_track("figure_eight", "geometric", 12.0, sensors=SensorConfig(enabled=True), tilt_deg=30)
    assert ekf.metrics["rings_passed"] == 8 and ekf.metrics["path_err_mean_m"] < 0.15
    assert cf.metrics["rings_passed"] < 8


def test_ekf_learns_the_gyro_bias():
    r = _fly("gyro_bias", "ekf", 5.0, 0)
    assert r.telemetry["bias_deg_s"][-1] == pytest.approx(3.0, abs=0.3)
    assert np.mean(r.telemetry["tilt_err_deg"]) < 1.0


def test_innovation_gate_rejects_a_short_glitch_and_gives_in_to_a_long_one():
    short_ekf, short_cf = _fly("gps_glitch", "ekf", 5.0, 0), _fly("gps_glitch", "complementary", 5.0, 0)
    assert max(short_ekf.telemetry["off_m"]) < 0.6  # the drone stayed put
    assert max(short_cf.telemetry["off_m"]) > 3.0  # the complementary filter followed the glitch
    assert short_ekf.metrics["gps_resets"] == 0
    long_ekf = _fly("gps_glitch", "ekf", GPS_RETRY_S + 5.0, 0)
    assert long_ekf.metrics["gps_resets"] >= 1  # after 10 s of disagreement it resets to the GPS


def test_ekf_knows_how_lost_it_is_in_an_outage():
    r = _fly("gps_outage", "ekf", 10.0, 0)
    t = np.asarray(r.telemetry["t_s"])
    i = int(np.searchsorted(t, 19.9))
    err, sigma2 = r.telemetry["est_err_m"][i], r.telemetry["sigma2_m"][i]
    assert err > 1.0 and sigma2 > err  # really drifted, and its own 2-sigma covers it
    assert r.telemetry["sigma2_m"][int(np.searchsorted(t, 9.0))] < 0.5  # small before the outage


def test_experiment_12_runs_and_compares_both_filters():
    r = get_experiment("estimators").runner({"scenario": "gps_glitch", "event_s": 5.0}, 0)
    m = r.metrics
    assert m["ekf_off_max_m"] < 0.6 < m["cf_off_max_m"] and m["gps_rejected_s"] >= 4.5
    assert {p["id"] for p in r.plots} >= {"est_err", "tilt_err", "off", "gate"}
    assert len(r.telemetry["cf_est_err_m"]) == len(r.telemetry["t_s"]) and r.summary


def test_live_view_can_fly_on_the_ekf_and_reports_its_health():
    rt = MissionSim(BASIC, seed=0)
    assert "Kalman" in rt.command("sensors", enabled=True, noise_scale=1.0, estimator="ekf")
    assert rt.sim.estimator.kind == "ekf"
    for _ in range(110):
        rt.advance(0.05)
    assert rt.gps_settle_left_s() == 0  # the same 5 s ground settle, done by zero-velocity updates
    assert "Taking off" in rt.command("takeoff", alt_m=3.0)
    for _ in range(100):
        rt.advance(0.05)
    snap = rt.snapshot()
    assert snap["ekf"]["sigma"]["pos_m"][0] < 0.3 and abs(rt.state.pos_m[2] - 3.0) < 0.5
    assert "reject" in rt.command("gps_glitch", on=True)
    for _ in range(40):
        rt.advance(0.05)
    assert rt.snapshot()["ekf"]["rejected"]["gps_pos"] > 0 and np.linalg.norm(rt.state.pos_m[:2]) < 0.6
    rt.command("gps_glitch", on=False)
    rt.command("sensors", enabled=True, noise_scale=1.0)
    assert rt.sim.estimator.kind == "complementary" and rt.snapshot()["ekf"] is None


# ---- ArduPilot replay helpers (offline) ----


def test_ardupilot_axes_convert_to_ours():
    assert np.allclose(R.frd_to_flu((1, 2, 3)), (1, -2, -3))
    assert np.allclose(R.ned_to_enu((1, 2, 3)), (2, 1, -3))
    rot = R.rot_from_ned_euler(0, 0, 0)  # level, nose north
    assert math.degrees(math.atan2(rot[1, 0], rot[0, 0])) == pytest.approx(90.0)  # our yaw: 90° = north
    rot = R.rot_from_ned_euler(10, 0, 90)  # nose east, right wing down
    assert rot[:, 0] == pytest.approx([1, 0, 0], abs=1e-9)
    assert rot[2, 1] > 0  # left wing (our +y) goes up


def test_heading_from_a_magnetometer_reading():
    dec = math.radians(10.0)  # magnetic north is 10° east of true north
    for yaw_deg in (0.0, 90.0, 200.0):  # our yaw: from east, anticlockwise
        mag_angle = math.radians(90.0 - 10.0)  # the field's horizontal direction in the world (from east)
        field_world = np.array([math.cos(mag_angle), math.sin(mag_angle), -1.5]) * 300.0
        rot = R.quat_to_rotation_matrix(R.quat_from_euler(0.0, 0.0, math.radians(yaw_deg)))
        field_body = rot.T @ field_world
        h = R.heading_from_mag(field_body, rot, dec)
        assert math.degrees(math.remainder(h - math.radians(yaw_deg), 2 * math.pi)) == pytest.approx(0.0, abs=1e-6)


def test_attitude_errors_are_tilt_and_heading():
    a = R.rot_from_ned_euler(0, 0, 0)
    assert R.attitude_errors_deg(a, R.rot_from_ned_euler(3, 4, 0)) == pytest.approx((5.0, 0.0), abs=0.02)
    assert R.attitude_errors_deg(a, R.rot_from_ned_euler(0, 0, 7)) == pytest.approx((0.0, 7.0), abs=1e-6)


def test_ekf_variable_step_predict_matches_the_fixed_one():
    init = State.hovering(BASIC, 2.0)
    a, b = EKF(SensorConfig(enabled=True), init, 0.002), EKF(SensorConfig(enabled=True), init, 0.002)
    gyro, accel = np.array([0.1, -0.05, 0.2]), np.array([0.3, 0.1, 9.9])
    from backend.sim.sensors import Readings

    r = Readings(gyro_rad_s=gyro, accel_m_s2=accel, new_imu=True)
    for _ in range(50):
        a.update(r, 0.0)
        b.predict(gyro, accel, 0.004)
    assert np.allclose(a.est.pos_m, b.est.pos_m) and np.allclose(a.P, b.P)


@pytest.mark.skipif(os.environ.get("DRONELAB_SITL") != "1", reason="real SITL flight: set DRONELAB_SITL=1")
@pytest.mark.skipif(not launch.is_built(), reason="ArduCopter SITL not built (scripts/sitl_setup.sh)")
def test_real_ardupilot_log_replays_through_our_ekf():
    res = R.compare(say=lambda _t: None)
    parts = res["parts"]
    glitch = next(v for k, v in parts.items() if k.startswith("GPS glitch"))
    for who in ("dronelab", "ekf3"):
        assert glitch[who]["gps_rejected_s"] >= 4.0 and glitch[who]["horiz_err_max_m"] < 1.0
    lap = parts["figure eight, 30° tilt"]
    assert lap["dronelab"]["horiz_err_mean_m"] < 0.5 and lap["ekf3"]["horiz_err_mean_m"] < 1.0
    assert parts["hover, gyro bias on"]["dronelab_fast"]["bias_end_deg_s"] == pytest.approx(2.0, abs=0.3)
