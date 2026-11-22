"""Phase 1b: position control, pilot modes, real-time pacing, and the API."""

import math

import numpy as np
import pytest
from fastapi.testclient import TestClient

from backend import api
from backend.sim.config import get_preset
from backend.sim.control import Setpoint
from backend.sim.physics import State
from backend.sim.pilot import Sticks
from backend.sim.runner import PHYSICS_DT_S, RealtimeSim, run_headless

BASIC = get_preset("basic")


def fly(rt: RealtimeSim, seconds: float) -> None:
    """Advance a real-time sim by `seconds` of sim time without actually waiting."""
    for _ in range(int(seconds / 0.05)):
        rt.advance(0.05)


@pytest.mark.parametrize("preset", ["basic", "lightweight", "heavy", "high_thrust"])
def test_position_control_flies_to_a_target_and_stays_there(preset):
    sp = Setpoint(alt_m=2.0, position_control=True, x_m=3.0, y_m=-2.0)
    r = run_headless(get_preset(preset), sp, 10.0, initial_state=State.at(2.0))
    t, x, y = r.channel("t_s"), r.channel("x_m"), r.channel("y_m")
    late = t >= 6.0
    assert np.all(np.hypot(x[late] - 3.0, y[late] + 2.0) < 0.1)
    tilt = np.hypot(r.channel("roll_rad"), r.channel("pitch_rad"))
    assert np.degrees(tilt.max()) < 30.0  # the tilt limit (25°) is respected, give or take the transient


def test_position_control_works_when_facing_north():
    sp = Setpoint(alt_m=2.0, position_control=True, x_m=2.0, y_m=1.0, yaw_rad=math.pi / 2)
    r = run_headless(BASIC, sp, 10.0, initial_state=State.at(2.0, yaw_rad=math.pi / 2))
    assert math.hypot(r.channel("x_m")[-1] - 2.0, r.channel("y_m")[-1] - 1.0) < 0.05


def test_tilting_creates_horizontal_motion_in_the_expected_direction():
    """Fixed attitude, no position control: nose-down pitch -> forward (east); right-down roll -> right (south)."""
    pitch = run_headless(BASIC, Setpoint(alt_m=5, pitch_rad=math.radians(10)), 2.0, initial_state=State.at(5.0))
    roll = run_headless(BASIC, Setpoint(alt_m=5, roll_rad=math.radians(10)), 2.0, initial_state=State.at(5.0))
    assert pitch.channel("x_m")[-1] > 0.5 and abs(pitch.channel("y_m")[-1]) < 0.05
    assert roll.channel("y_m")[-1] < -0.5 and abs(roll.channel("x_m")[-1]) < 0.05


def test_takeoff_hover_land_cycle():
    rt = RealtimeSim(BASIC)
    assert rt.pilot.mode == "idle"
    rt.command("takeoff")
    fly(rt, 5.0)
    assert rt.pilot.mode == "hover" and abs(rt.state.pos_m[2] - 2.0) < 0.05
    rt.command("land")
    fly(rt, 6.0)
    assert rt.pilot.mode == "idle" and rt.state.on_ground and not rt.pilot.sp.armed


def test_keyboard_nudge_moves_the_drone_forward_relative_to_its_heading():
    rt = RealtimeSim(BASIC)
    rt.command("takeoff")
    fly(rt, 4.0)
    rt.command("sticks", forward=1.0)
    fly(rt, 2.0)
    rt.command("sticks")  # release all keys
    fly(rt, 4.0)
    x, y, _ = rt.state.pos_m
    assert x > 2.0 and abs(y) < 0.1  # facing east, so "forward" = +x


def test_pause_freezes_time_and_resume_continues():
    rt = RealtimeSim(BASIC)
    rt.command("takeoff")
    fly(rt, 1.0)
    rt.command("pause")
    t_paused = rt.state.time_s
    fly(rt, 1.0)
    assert rt.state.time_s == t_paused
    rt.command("resume")
    fly(rt, 1.0)
    assert rt.state.time_s > t_paused


def test_realtime_pacing_and_catch_up_limit():
    rt = RealtimeSim(BASIC)
    assert rt.advance(0.1) == round(0.1 / PHYSICS_DT_S)
    assert rt.advance(5.0) == round(0.1 / PHYSICS_DT_S)  # a 5 s stall only simulates 0.1 s


def test_unknown_command_is_an_error_not_a_crash():
    with pytest.raises(ValueError):
        RealtimeSim(BASIC).command("barrel_roll")


def test_sticks_are_clamped():
    assert Sticks(forward=5, left=-9).clamped() == Sticks(forward=1, left=-1)


# ---- API ----


@pytest.fixture
def client(monkeypatch):
    monkeypatch.setenv("OLLAMA_URL", "http://127.0.0.1:9")  # nothing listens here -> "AI offline"
    api._health_cache.update(at=-1e9, value=None)
    return TestClient(api.app)


def test_health_reports_llm_offline_and_sim_ok_when_ollama_is_unreachable(client):
    body = client.get("/api/health").json()
    assert body["sim"] == "ok" and body["llm"] == "offline"


def test_presets_and_limits(client):
    body = client.get("/api/presets").json()
    assert set(body["presets"]) == {"basic", "lightweight", "heavy", "high_thrust"}
    assert body["limits"]["mass_kg"] == [0.1, 5.0]  # 0.1: the Drone Builder's micro quad is ~0.18 kg


def test_validate_accepts_good_and_rejects_bad_configs(client):
    good = client.post("/api/config/validate", json={"config": BASIC.to_dict()}).json()
    assert good["valid"] and good["derived"]["thrust_to_weight"] == pytest.approx(2.0)
    bad = client.post("/api/config/validate", json={"config": {**BASIC.to_dict(), "mass_kg": 50}}).json()
    assert not bad["valid"] and "mass_kg" in bad["problems"][0]
    junk = client.post("/api/config/validate", json={"config": {"hyperdrive": 1}}).json()
    assert not junk["valid"]


def test_learn_page_is_served_and_paths_are_locked_down(client):
    assert "Thrust" in client.get("/api/learn/01-thrust-and-hover").json()["markdown"]
    assert client.get("/api/learn/..%2F..%2FCLAUDE").status_code == 404


def test_websocket_streams_state_and_takes_off(client):
    with client.websocket_connect("/ws/sim") as ws:
        assert ws.receive_json()["type"] == "config"
        ws.send_json({"cmd": "takeoff"})
        alt, mode = 0.0, None
        for _ in range(60):  # ~2 s of real time at 30 Hz
            msg = ws.receive_json()
            if msg["type"] == "state":
                alt, mode = msg["pos_m"][2], msg["mode"]
        assert mode in ("takeoff", "hover") and alt > 0.3
        ws.send_json({"cmd": "reset", "config": {**BASIC.to_dict(), "mass_kg": 99}})
        errors = [m for m in (ws.receive_json() for _ in range(5)) if m["type"] == "error"]
        assert errors and "mass_kg" in errors[0]["message"]


def test_lite_mode_serves_the_page_without_hiding_the_api(tmp_path, monkeypatch):
    monkeypatch.setenv("OLLAMA_URL", "http://127.0.0.1:9")
    api._health_cache.update(at=-1e9, value=None)
    (tmp_path / "index.html").write_text("<!doctype html><title>DroneLab</title>")
    (tmp_path / "assets").mkdir()
    (tmp_path / "assets" / "app.js").write_text("console.log(1)")
    n_routes = len(api.app.router.routes)
    assert api.mount_frontend(api.app, tmp_path)
    try:
        c = TestClient(api.app)
        assert "DroneLab" in c.get("/").text
        assert c.get("/assets/app.js").status_code == 200
        assert c.get("/api/health").json()["sim"] == "ok"  # API still wins
        with c.websocket_connect("/ws/sim") as ws:
            assert ws.receive_json()["type"] == "config"
    finally:
        del api.app.router.routes[n_routes:]  # undo the mount for other tests
    assert not api.mount_frontend(api.app, tmp_path / "missing")
