"""Phase 1a acceptance tests (PRD §5). Each test name says what it proves."""

import math
import time

import numpy as np
import pytest

from backend.sim.config import GRAVITY_M_S2, get_preset
from backend.sim.control import Setpoint
from backend.sim.physics import State
from backend.sim.runner import run_free_fall, run_headless, run_hover

BASIC = get_preset("basic")


def test_default_drone_climbs_to_2m_and_holds_within_5cm_from_5s_to_10s():
    r = run_hover(BASIC, alt_m=2.0, duration_s=10.0)
    t, z = r.channel("t_s"), r.channel("z_m")
    window = (t >= 5.0) & (t <= 10.0)
    assert np.all(np.abs(z[window] - 2.0) <= 0.05), f"max error {np.abs(z[window] - 2.0).max():.3f} m"


def test_too_heavy_drone_never_leaves_the_ground_and_says_why():
    heavy = BASIC.with_changes(mass_kg=2.5)  # same motors, T/W = 0.8
    assert heavy.thrust_to_weight < 1.0
    r = run_hover(heavy)
    assert not r.metrics["left_ground"]
    assert np.all(r.channel("z_m") == 0.0)
    explanation = " ".join(r.summary)
    assert "never left the ground" in explanation and "T/W < 1" in explanation


def test_doubling_the_mass_raises_hover_motor_percent():
    light = BASIC.with_changes(mass_kg=0.6)
    heavy = BASIC.with_changes(mass_kg=1.2)
    light_pct = run_hover(light).metrics["hover_motor_pct"]
    heavy_pct = run_hover(heavy).metrics["hover_motor_pct"]
    assert heavy_pct > light_pct
    assert heavy_pct / light_pct == pytest.approx(2.0, rel=0.02)  # thrust needed is proportional to mass


def test_measured_hover_throttle_matches_100_over_tw():
    for preset in ("basic", "lightweight", "heavy", "high_thrust"):
        cfg = get_preset(preset)
        assert run_hover(cfg).metrics["hover_motor_pct"] == pytest.approx(cfg.hover_throttle_pct, rel=0.01)


def test_same_seed_gives_identical_telemetry():
    a = run_hover(BASIC, seed=42).telemetry
    b = run_hover(BASIC, seed=42).telemetry
    assert a == b


def test_free_fall_matches_half_g_t_squared_within_1_percent():
    """Semi-implicit Euler runs slightly ahead of the exact answer, by ½·g·t·dt
    (relative error dt/t). So: within 1 % once t ≥ 0.25 s, and never more than 2 cm off."""
    no_drag = BASIC.with_changes(drag_linear_n_s_m=0.0, drag_quadratic_n_s2_m2=0.0)
    r = run_free_fall(no_drag, start_alt_m=50.0, duration_s=2.0)
    t, z = r.channel("t_s"), r.channel("z_m")
    fallen_sim = 50.0 - z
    fallen_theory = 0.5 * GRAVITY_M_S2 * t**2
    later = t >= 0.25
    assert np.allclose(fallen_sim[later], fallen_theory[later], rtol=0.01)
    assert np.max(np.abs(fallen_sim - fallen_theory)) < 0.02


def test_free_fall_stops_at_the_ground_without_bouncing():
    r = run_free_fall(BASIC, start_alt_m=5.0, duration_s=3.0)
    z = r.channel("z_m")
    assert z.min() >= 0.0 and z[-1] == 0.0
    assert [e["type"] for e in r.events] == ["touchdown"]


def test_20s_headless_run_finishes_in_under_3s():
    start = time.perf_counter()
    run_hover(BASIC, duration_s=20.0)
    assert time.perf_counter() - start < 3.0


def test_attitude_controller_levels_a_tilted_drone():
    tilted = State.at(5.0, roll_rad=math.radians(15), pitch_rad=math.radians(-10), yaw_rad=0.3)
    r = run_headless(BASIC, Setpoint(alt_m=5.0, yaw_rad=0.3), 2.0, initial_state=tilted)
    t = r.channel("t_s")
    after_1s = t >= 1.0
    assert np.all(np.abs(r.channel("roll_rad")[after_1s]) < math.radians(1))
    assert np.all(np.abs(r.channel("pitch_rad")[after_1s]) < math.radians(1))
    assert np.allclose(r.channel("yaw_rad")[after_1s], 0.3, atol=math.radians(1))


def test_disarmed_drone_on_the_ground_stays_put():
    r = run_headless(BASIC, Setpoint(armed=False), 2.0)
    assert np.all(r.channel("z_m") == 0.0) and not r.events


def test_run_result_is_json_serialisable():
    import json

    data = json.loads(run_hover(BASIC, duration_s=2.0).to_json())
    assert {"config", "telemetry", "metrics", "events", "summary", "seed"} <= set(data)
    assert len(data["telemetry"]["t_s"]) == 101  # 2 s at 50 Hz, plus t = 0
