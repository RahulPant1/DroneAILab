"""Phase 2: Engineering Lab. Acceptance criteria from PRD §5, plus sensors, wind, storage and API."""

import json

import numpy as np
import pytest
from fastapi.testclient import TestClient

from backend import api
from backend.experiments import EXPERIMENTS, ExperimentError, get_experiment
from backend.sim.config import get_preset
from backend.sim.control import Setpoint
from backend.sim.environment import EnvConfig
from backend.sim.physics import State
from backend.sim.runner import run_headless
from backend.sim.sensors import SensorConfig
from backend.storage import RunStore, StorageError

BASIC = get_preset("basic")


def run(exp_id, **params):
    return get_experiment(exp_id).run(params, seed=0)


# ---- PRD acceptance ----


def test_too_high_kp_visibly_oscillates():
    m = run("pid_tuning", kp=25.0).metrics
    assert m["oscillations"] >= 3 and m["overshoot_pct"] > 30
    assert any("oscillates" in line for line in run("pid_tuning", kp=25.0).summary)


def test_kd_zero_gives_more_overshoot_than_default():
    assert run("pid_tuning", kd=0.0).metrics["overshoot_pct"] > run("pid_tuning").metrics["overshoot_pct"] + 20


def test_wind_causes_measurable_drift_that_the_controller_recovers_from():
    m = run("wind", wind_speed_m_s=5.0).metrics
    assert m["max_drift_m"] > 0.2  # measurable push
    assert m["recovery_time_s"] is not None and m["steady_offset_m"] < 0.05  # and back on the spot


def test_without_velocity_integral_the_wind_leaves_a_steady_offset():
    assert run("wind", wind_speed_m_s=5.0, vel_ki=0.0).metrics["steady_offset_m"] > 0.3


def test_noise_creates_a_visible_gap_between_truth_and_estimate():
    r = run("sensor_noise")
    gap = np.abs(np.asarray(r.telemetry["z_est_m"]) - np.asarray(r.telemetry["z_m"]))
    assert gap.max() > 0.05  # visible on a plot
    assert r.metrics["est_altitude_error_m"] < r.metrics["raw_altimeter_error_m"]  # but the filter helps


def test_save_reload_compare(tmp_path):
    store = RunStore(tmp_path)
    a = store.save(run("pid_tuning"), "default")
    b = store.save(run("pid_tuning", kd=0.0), "no damping")
    reloaded = store.get(a["id"])
    assert reloaded["result"]["telemetry"]["z_m"] == run("pid_tuning").telemetry["z_m"]
    from backend.storage import compare

    c = compare(store.get(a["id"]), store.get(b["id"]))
    row = next(r for r in c["metrics"] if r["metric"] == "overshoot_pct")
    assert row["diff"] > 20
    assert [e["id"] for e in store.list()] == [b["id"], a["id"]]  # newest first


# ---- Ki lesson ----


def test_integral_term_removes_the_error_from_a_hidden_payload():
    without_i = run("pid_tuning", payload_kg=0.3, ki=0.0).metrics["steady_state_error_m"]
    with_i = run("pid_tuning", payload_kg=0.3, ki=1.5).metrics["steady_state_error_m"]
    assert without_i < -0.5  # stuck well below the target
    assert abs(with_i) < 0.1


# ---- sensors and estimator ----


def test_noise_free_sensors_give_an_estimate_equal_to_the_truth():
    r = run_headless(BASIC, Setpoint(alt_m=2, position_control=True), 5.0, initial_state=State.hovering(BASIC, 2.0),
                     sensors=SensorConfig(enabled=True, noise_scale=0.0))  # fmt: skip
    for est, truth in (("z_est_m", "z_m"), ("x_est_m", "x_m"), ("roll_est_rad", "roll_rad")):
        assert np.allclose(r.telemetry[est], r.telemetry[truth], atol=1e-3)


def test_sensor_noise_is_deterministic_per_seed():
    s = SensorConfig(enabled=True)
    a = run_headless(BASIC, Setpoint(alt_m=2), 3.0, seed=7, sensors=s).telemetry
    b = run_headless(BASIC, Setpoint(alt_m=2), 3.0, seed=7, sensors=s).telemetry
    c = run_headless(BASIC, Setpoint(alt_m=2), 3.0, seed=8, sensors=s).telemetry
    assert a == b and a["baro_m"] != c["baro_m"]


def test_gyro_bias_error_follows_bias_over_correction_gain():
    m = run("sensor_noise", noise_scale=0.0, gyro_bias_deg_s=1.0, tilt_correction=0.5).metrics
    assert abs(m["mean_roll_error_deg"]) == pytest.approx(2.0, rel=0.25)  # theory: 1.0 / 0.5 = 2°


# ---- wind model ----


def test_wind_only_starts_at_wind_start_and_gusts_are_seeded():
    env = EnvConfig(wind_speed_m_s=4.0, gust_std_m_s=1.0, wind_start_s=1.0)
    r = run_headless(BASIC, Setpoint(alt_m=2, position_control=True), 3.0, initial_state=State.hovering(BASIC, 2.0), env=env)
    t, wx = np.asarray(r.telemetry["t_s"]), np.asarray(r.telemetry["wind_x_m_s"])
    assert np.all(wx[t < 0.99] == 0) and wx[t > 1.1].mean() > 2.0
    again = run_headless(BASIC, Setpoint(alt_m=2, position_control=True), 3.0, initial_state=State.hovering(BASIC, 2.0), env=env)
    assert again.telemetry["wind_x_m_s"] == r.telemetry["wind_x_m_s"]


def test_wind_pushes_downwind():
    for direction, (axis, sign) in ((0.0, ("x_m", 1)), (90.0, ("y_m", 1)), (180.0, ("x_m", -1))):
        m = run("wind", wind_speed_m_s=6.0, wind_direction_deg=direction, vel_ki=0.0)
        assert sign * m.telemetry[axis][-1] > 0.3


# ---- experiment framework ----


@pytest.mark.parametrize("exp_id", list(EXPERIMENTS))
def test_every_experiment_runs_and_its_plots_use_real_channels(exp_id):
    exp = EXPERIMENTS[exp_id]
    r = exp.run({}, 0)
    assert r.summary and r.try_next and r.plots
    for plot in r.plots:
        for s in plot["series"]:
            assert s["channel"] in r.telemetry, f"{exp_id}: plot '{plot['id']}' needs missing channel {s['channel']}"
    for metric in exp.key_metrics:
        assert metric.key in r.metrics
    json.dumps(r.to_dict(), allow_nan=False)  # browsers can't parse NaN
    for suggestion in r.try_next:  # every one-click suggestion must itself be valid
        exp.check_params(suggestion["params"])
    assert len(exp.concept) == 3 and 1 <= len(exp.params) <= 4


def test_out_of_range_and_unknown_params_are_rejected_readably():
    with pytest.raises(ExperimentError, match="outside the allowed range"):
        run("pid_tuning", kp=500.0)
    with pytest.raises(ExperimentError, match="Unknown setting"):
        run("pid_tuning", warp=1)
    with pytest.raises(ExperimentError, match="choose one of"):
        run("attitude", axis="sideways")


def test_attitude_experiment_names_the_right_motors():
    roll = run("attitude", axis="roll").metrics
    assert roll["harder_group_pct"] > roll["other_group_pct"] + 10
    assert run("attitude", axis="yaw").metrics["horizontal_distance_m"] < 0.05


# ---- storage ----


def test_storage_cap_delete_and_bad_ids(tmp_path):
    store = RunStore(tmp_path, max_runs=3)
    result = run("thrust_takeoff")
    ids = [store.save(result, f"run {i}")["id"] for i in range(3)]
    with pytest.raises(StorageError, match="limit"):
        store.save(result)
    assert store.delete_old(keep_newest=1) == 2
    assert [e["id"] for e in store.list()] == [ids[2]]
    store.delete(ids[2])
    assert store.list() == []
    for bad in ("../../CLAUDE", "x", ""):
        with pytest.raises(StorageError):
            store.get(bad)


def test_index_is_rebuilt_if_lost(tmp_path):
    store = RunStore(tmp_path)
    saved = store.save(run("mass_thrust"), "keep me")
    (tmp_path / "index.json").unlink()
    assert [e["id"] for e in RunStore(tmp_path).list()] == [saved["id"]]


# ---- API ----


@pytest.fixture
def client(tmp_path, monkeypatch):
    monkeypatch.setattr(api, "store", RunStore(tmp_path))
    return TestClient(api.app)


def test_api_lab_flow(client):
    exps = client.get("/api/experiments").json()
    assert [e["number"] for e in exps] == list(range(1, 12))
    res = client.post("/api/experiments/pid_tuning/run", json={"params": {"kp": 25}}).json()
    assert res["metrics"]["oscillations"] >= 3 and res["plots"]
    bad = client.post("/api/experiments/pid_tuning/run", json={"params": {"kp": 999}})
    assert bad.status_code == 400 and "outside" in bad.json()["detail"]
    assert client.post("/api/experiments/nope/run", json={}).status_code == 400

    a = client.post("/api/runs", json={"experiment_id": "pid_tuning", "params": {}, "seed": 0, "label": "A"}).json()
    b = client.post("/api/runs", json={"experiment_id": "pid_tuning", "params": {"kd": 0}, "seed": 0, "label": "B"}).json()
    assert [r["label"] for r in client.get("/api/runs").json()] == ["B", "A"]
    cmp = client.get(f"/api/runs/compare?a={a['id']}&b={b['id']}").json()
    assert cmp["a"]["label"] == "A" and any(r["metric"] == "overshoot_pct" for r in cmp["metrics"])
    assert client.get(f"/api/runs/{a['id']}").json()["result"]["scenario"]["params"]["kd"] == 3.5
    assert client.delete(f"/api/runs/{a['id']}").status_code == 200
    assert client.get(f"/api/runs/{a['id']}").status_code == 404
    assert client.post("/api/runs/delete-old", json={"keep_newest": 0}).json() == {"deleted": 1}


def test_every_experiment_links_to_an_existing_lesson():
    from backend.api import LEARN_DIR

    for exp in EXPERIMENTS.values():
        assert (LEARN_DIR / f"{exp.learn_slug}.md").is_file(), exp.learn_slug
