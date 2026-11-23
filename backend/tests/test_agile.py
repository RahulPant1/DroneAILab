"""Agile flight: minimum-snap trajectories, feedforward + geometric control, experiment 11, the live race autopilot."""

import math

import numpy as np
import pytest

from backend.autonomy.agile import plan_track, run_track, track_points
from backend.autonomy.live import MissionSim
from backend.autonomy.trajectory import (
    Limits, Trajectory, TrajectoryError, fit_to_limits, initial_times, plan_trajectory, solve_min_snap,
)
from backend.games import race as race_mod
from backend.sim.config import get_preset
from backend.sim.control import CascadedController, Setpoint
from backend.sim.frames import quat_from_euler, quat_from_rotation_matrix, quat_to_rotation_matrix
from backend.sim.physics import State

PTS = np.array([[0, 0, 2], [4, 0, 2], [7, 3.5, 2.5], [4, 7, 2], [0.5, 3.5, 2.5], [0.5, 1.5, 2.5]], dtype=float)
TIMES = np.array([2.0, 1.4, 1.3, 1.5, 1.5])


@pytest.fixture(autouse=True)
def tmp_best_file(tmp_path, monkeypatch):
    """Never touch the real data/race_best.json in tests."""
    monkeypatch.setattr(race_mod, "BEST_TIMES_FILE", tmp_path / "race_best.json")


# ---- trajectory maths ----

def test_min_snap_passes_every_waypoint_starts_and_ends_at_rest_and_is_smooth():
    tr = Trajectory(PTS, TIMES, solve_min_snap(PTS, TIMES), {})
    s = tr.sample(tr.knots_s())
    assert np.allclose(s["pos"], PTS, atol=1e-6)
    for k in ("vel", "acc", "jerk"):
        assert np.allclose(s[k][[0, -1]], 0.0, atol=1e-6)
    for t in tr.knots_s()[1:-1]:  # velocity, acceleration, jerk and snap match across each join
        a, b = tr.sample(t - 1e-7), tr.sample(t + 1e-7)
        for k in ("vel", "acc", "jerk", "snap"):
            assert np.allclose(a[k], b[k], atol=1e-3), (t, k)


def test_gate_directions_make_the_velocity_point_through_each_ring():
    dirs = {1: [1, 0, 0], 2: [0, 1, 0], 3: [-1, 0, 0], 4: [0, -1, 0]}
    tr = Trajectory(PTS, TIMES, solve_min_snap(PTS, TIMES, {k: np.array(v, float) for k, v in dirs.items()}), {})
    vel = tr.sample(tr.knots_s())["vel"]
    for i, d in dirs.items():
        v = vel[i]
        assert float(v @ np.array(d)) > 1.0  # going the right way, quickly
        assert np.linalg.norm(v - (v @ np.array(d)) * np.array(d)) < 1e-6  # and not sideways


def test_matches_the_independent_minsnap_trajectories_library():
    """Cross-check against minsnap-trajectories (MIT, Bry & Roy's closed form): same waypoints and times, same curve."""
    ms = pytest.importorskip("minsnap_trajectories")
    knots = np.concatenate([[0.0], np.cumsum(TIMES)])
    z = np.zeros(3)
    refs = [ms.Waypoint(time=0.0, position=PTS[0], velocity=z, acceleration=z, jerk=z)]
    refs += [ms.Waypoint(time=float(knots[i]), position=PTS[i]) for i in range(1, len(PTS) - 1)]
    refs += [ms.Waypoint(time=float(knots[-1]), position=PTS[-1], velocity=z, acceleration=z, jerk=z)]
    poly = ms.generate_trajectory(refs, degree=7, idx_minimized_orders=4, num_continuous_orders=4, algorithm="closed-form")
    t = np.linspace(0.0, knots[-1], 300)
    theirs = ms.compute_trajectory_derivatives(poly, t, 4)
    ours = Trajectory(PTS, TIMES, solve_min_snap(PTS, TIMES), {}).sample(t)
    for i, k in enumerate(("pos", "vel", "acc", "jerk")):
        assert np.abs(theirs[i] - ours[k]).max() < 1e-6, k


def test_fast_sampler_matches_the_full_one():
    tr = plan_track("figure_eight", "geometric", 12.0).trajectory
    for t in np.linspace(0.0, tr.duration_s, 17):
        full = tr.sample(t)
        for got, k in zip(tr.sample_one(t), ("pos", "vel", "acc", "jerk")):
            assert np.allclose(got, full[k][0], atol=1e-9)


def test_timing_fits_the_limits_and_one_limit_is_tight():
    lim = Limits(speed_m_s=4.0, accel_m_s2=5.0, tilt_deg=30.0)
    tr = plan_trajectory(PTS, lim)
    p = tr.peaks()
    assert p["speed_m_s"] <= lim.speed_m_s + 1e-6 and p["accel_m_s2"] <= lim.accel_m_s2 + 1e-6
    assert p["tilt_deg"] <= lim.tilt_deg + 1e-3
    assert max(p["speed_m_s"] / lim.speed_m_s, p["accel_m_s2"] / lim.accel_m_s2, p["tilt_deg"] / lim.tilt_deg) > 0.99
    # Stretching time by k: speeds / k, accelerations / k^2 (the shape doesn't change)
    slow = tr.scaled(2.0).peaks()
    assert slow["speed_m_s"] == pytest.approx(p["speed_m_s"] / 2, rel=1e-6)
    assert slow["accel_m_s2"] == pytest.approx(p["accel_m_s2"] / 4, rel=1e-6)


def test_moving_time_between_pieces_lowers_the_snap_cost_for_the_same_total():
    from backend.autonomy.trajectory import optimise_times, snap_cost

    t0 = initial_times(PTS, 4.0, 5.0)
    t1 = optimise_times(PTS, t0, {})
    assert t1.sum() == pytest.approx(t0.sum())
    assert snap_cost(solve_min_snap(PTS, t1), t1) < snap_cost(solve_min_snap(PTS, t0), t0)


def test_bad_waypoints_are_explained():
    with pytest.raises(TrajectoryError, match="same place"):
        plan_trajectory([[0, 0, 1], [0, 0, 1], [1, 0, 1]])
    with pytest.raises(TrajectoryError, match="two 3D"):
        plan_trajectory([[0, 0, 1]])
    with pytest.raises(TrajectoryError, match="middle"):
        solve_min_snap(PTS[:3], TIMES[:2], {0: np.array([1.0, 0, 0])})
    tr = Trajectory(PTS, TIMES, solve_min_snap(PTS, TIMES), {})
    with pytest.raises(TrajectoryError):
        fit_to_limits(tr, Limits(thrust_per_kg=5.0))  # less thrust than it takes to hover: impossible at any speed


def test_quaternion_from_rotation_matrix_round_trips():
    for r, p, y in [(0.1, -0.2, 0.3), (1.2, 0.4, -2.9), (-0.7, 1.4, 3.1), (0.0, 0.0, math.pi)]:
        q = quat_from_euler(r, p, y)
        q2 = quat_from_rotation_matrix(quat_to_rotation_matrix(q))
        assert min(np.abs(q - q2).max(), np.abs(q + q2).max()) < 1e-9


# ---- control ----

def test_setpoint_without_feedforward_behaves_exactly_as_before():
    cfg = get_preset("basic")
    a, b = CascadedController(cfg), CascadedController(cfg)
    s = State.hovering(cfg, 2.0)
    s.vel_m_s[:] = (0.4, -0.2, 0.1)
    old = Setpoint(alt_m=3.0, position_control=True, x_m=1.0, y_m=-1.0)
    same = Setpoint(alt_m=3.0, position_control=True, x_m=1.0, y_m=-1.0, vel_ff_m_s=None, law="cascade")
    for _ in range(20):
        assert np.array_equal(a.update(s, old, 0.004), b.update(s, same, 0.004))


def test_geometric_controller_tilts_the_thrust_toward_the_wanted_acceleration():
    cfg = get_preset("basic")
    c = CascadedController(cfg)
    s = State.hovering(cfg, 2.0)
    sp = Setpoint(alt_m=2.0, position_control=True, law="geometric", vel_ff_m_s=(0, 0, 0), acc_ff_m_s2=(3.0, 0, 0))
    thrust, q_des, _ = c.geometric.update(s, sp, 0.004)
    up = quat_to_rotation_matrix(q_des)[:, 2]
    assert math.degrees(math.atan2(up[0], up[2])) == pytest.approx(math.degrees(math.atan2(3.0, 9.81)), abs=0.5)
    assert up[1] == pytest.approx(0.0, abs=1e-9)
    assert thrust == pytest.approx(cfg.mass_kg * 9.81, rel=0.01)  # projected on the (still level) body axis


@pytest.mark.parametrize("method", ["waypoints", "pid_ff", "geometric"])
def test_every_method_flies_all_rings_of_the_square_loop(method):
    m = run_track("square_loop", method, 12.0, tilt_deg=30.0).metrics
    assert m["rings_passed"] == m["rings_total"] == 4
    assert m["lap_time_s"] is not None and m["motor_saturated_pct"] < 5


def test_feedforward_and_geometric_control_beat_waypoints_where_expected():
    r = {m: run_track("square_loop", m, 12.0, tilt_deg=30.0).metrics for m in ("waypoints", "pid_ff", "geometric")}
    assert r["pid_ff"]["lap_time_s"] < 0.8 * r["waypoints"]["lap_time_s"]  # a planned curve is much faster
    assert r["geometric"]["path_err_mean_m"] < 0.5 * r["pid_ff"]["path_err_mean_m"]  # and the geometric one is tighter
    assert r["geometric"]["track_err_mean_m"] < 0.1
    assert r["waypoints"]["track_err_mean_m"] > 1.0  # it trails the carrot


def test_track_points_add_a_hover_start_and_an_exit_past_the_last_ring():
    rings, pts, dirs = track_points("square_loop")
    assert len(pts) == len(rings) + 2 and tuple(pts[0][:2]) == (0.0, 0.0) and pts[0][2] == rings[0].z_m
    assert np.allclose(pts[-1], rings[-1].centre + rings[-1].normal * 2.0)
    assert set(dirs) == set(range(1, len(rings) + 1))


# ---- experiment 11 ----

def test_experiment_11_runs_and_draws_the_rings():
    from backend.experiments import get_experiment

    e = get_experiment("agile_flight")
    r = e.runner(e.check_params({"track": "square_loop"}), 0)
    assert r.metrics["rings_passed"] == 4 and r.summary and r.try_next
    plot = next(p for p in r.plots if p["kind"] == "map")
    assert len(plot["extra"]["gates"]) == 4 and all(g[4] == 1 for g in plot["extra"]["gates"])
    assert len(plot["extra"]["planned_route"]) > 50  # the smooth curve, not just the ring centres
    assert {"ref_x_m", "track_err_m", "speed_m_s"} <= set(r.telemetry)


def test_real_sensors_fool_the_tilt_estimate_in_turns_and_gyro_only_tilt_fixes_it():
    from backend.experiments import get_experiment

    e = get_experiment("agile_flight")
    plain = e.runner(e.check_params({"conditions": "sensors"}), 0).metrics
    gyro = e.runner(e.check_params({"conditions": "sensors_gyro"}), 0).metrics
    assert gyro["path_err_mean_m"] < 0.5 * plain["path_err_mean_m"]
    assert gyro["rings_passed"] == 8 and plain["rings_passed"] < 8


# ---- the live race autopilot ----

def _fly(rt: MissionSim, seconds: float) -> None:
    for _ in range(int(seconds / 0.05)):
        rt.advance(0.05)
        if rt.agile_phase is None:
            return


def test_live_autopilot_flies_a_lap_and_never_saves_a_best_time(tmp_path):
    rt = MissionSim(get_preset("basic"))
    rt.load_race("first_flight")
    msg = rt.command("agile", method="geometric", tilt_deg=30)
    assert "trajectory" in msg and rt.race_info()["autopilot"]["path"]
    _fly(rt, 40.0)
    assert rt.agile_phase is None and rt.pilot.mode == "hover"
    assert rt.snapshot()["race"]["finished"]
    assert any("Autopilot finished" in n for n in rt.notices)
    assert race_mod.load_best_times() == {}  # an autopilot lap is not a best time


def test_live_autopilot_needs_a_race_and_any_button_takes_over():
    rt = MissionSim(get_preset("basic"))
    with pytest.raises(ValueError, match="race track"):
        rt.command("agile", method="geometric")
    rt.load_race("square_loop")
    rt.command("agile", method="pid_ff", tilt_deg=25)
    for _ in range(int(6 / 0.05)):
        rt.advance(0.05)
    assert rt.snapshot()["agile"]["phase"] in ("climb", "fly")
    rt.command("hover")
    assert rt.agile_phase is None and rt.snapshot()["agile"] is None
    assert any("you have the controls" in n for n in rt.notices)
