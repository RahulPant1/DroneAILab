"""🎯 Precision landing: the camera picture, marker detection, and landings that beat a drifting GPS."""

import numpy as np
import pytest

from backend.autonomy.live import MissionSim
from backend.autonomy.plan import compile_plan, run_plan
from backend.autonomy.precland import IMAGE_H, IMAGE_W, detect, pixel_to_offset, render
from backend.experiments import get_experiment
from backend.sim.config import get_preset
from backend.sim.frames import quat_from_euler
from backend.sim.physics import State
from backend.sim.sensors import SensorConfig

PAD = [(0.0, 0.0, 1.0)]


def test_the_camera_sees_the_marker_and_measures_where_it_is():
    rng = np.random.default_rng(0)
    s = State.at(8.0)
    s.pos_m[:2] = [1.3, -0.8]
    s.quat = quat_from_euler(0.05, -0.08, 0.7)  # tilted and turned: the maths must undo that
    img = render(s, PAD, rng)
    assert img.shape == (IMAGE_H, IMAGE_W)
    d = detect(img)
    assert d is not None
    dx, dy = pixel_to_offset(d["u"], d["v"], s)
    assert s.pos_m[0] + dx == pytest.approx(0.0, abs=0.1) and s.pos_m[1] + dy == pytest.approx(0.0, abs=0.1)


def test_no_false_marker_on_grass_and_none_when_cut_off():
    rng = np.random.default_rng(1)
    s = State.at(8.0)
    false = 0
    for i in range(100):
        s.pos_m[:2] = [20.0 + 3 * i, 7.0]
        false += detect(render(s, PAD, rng)) is not None
    assert false == 0
    s.pos_m[:] = [3.6, 0.0, 6.0]  # marker at the picture's edge: only part of it is visible
    assert detect(render(s, PAD, rng)) is None


def test_camera_landing_beats_gps_landing_with_drifting_gps():
    plan_cam = compile_plan({"tasks": [{"type": "land", "place": "pad B"}], "altitude_m": 10})
    errs = {"gps": [], "vision": []}
    for seed in range(4):
        for mode in errs:
            r = run_plan(plan_cam, sensors=SensorConfig(enabled=True, gps_drift_m=1.5), seed=seed, lander_mode=mode)
            errs[mode].append(r.metrics["landing_error_m"])
    assert np.mean(errs["vision"]) < 0.5
    assert np.mean(errs["gps"]) > 2 * np.mean(errs["vision"])


def test_precision_landing_experiment_shows_camera_frames():
    r = get_experiment("precision_landing").run({}, 0)
    assert r.metrics["landing_error_m"] < 0.6 and r.metrics["marker_sightings"] > 10
    frames = next(p for p in r.plots if p["kind"] == "images")["extra"]["frames"]
    assert 2 <= len(frames) <= 4 and frames[0]["detection"] and len(frames[0]["image"]) > 1000
    assert any("RELATIVE" in line for line in r.summary)


def test_live_precision_landing_button_and_down_camera():
    rt = MissionSim(get_preset("basic"))
    with pytest.raises(ValueError, match="farm"):
        rt.command("precland")
    rt.command("course", scenario="farm")
    with pytest.raises(ValueError, match="Take off"):
        rt.command("precland")
    assert "camera on" in rt.command("camera", enabled=True)
    rt.command("takeoff", alt_m=6)
    for _ in range(100):
        rt.advance(0.05)
    assert rt.snapshot()["camera"]["image"] is not None or rt.snapshot()["camera"] is not None
    rt.command("target", x_m=-10, y_m=27)
    for _ in range(200):
        rt.advance(0.05)
    assert "pad B" in rt.command("precland", mode="vision")
    for _ in range(int(60 / 0.05)):
        rt.advance(0.05)
        if rt.prec_lander is None:
            break
    assert "Landed" in rt.pilot.message and rt.state.on_ground
    assert abs(rt.state.pos_m[0] + 12) < 0.3 and abs(rt.state.pos_m[1] - 30) < 0.3
