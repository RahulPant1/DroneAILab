"""🌾 Surveys and task plans: lawnmower geometry, real coverage, safety checks, and plans flown in the simulator."""

import math

import pytest

from backend.autonomy.live import MissionSim
from backend.autonomy.plan import PlanError, compile_plan, run_plan
from backend.autonomy.site import FARM
from backend.autonomy.survey import CAMERA, SurveyError, coverage, plan_lawnmower
from backend.experiments import get_experiment
from backend.sim.battery import BatteryConfig
from backend.sim.config import get_preset
from backend.sim.environment import EnvConfig
from backend.sim.sensors import SensorConfig


def status(plan, cid):
    return next(c["status"] for c in plan.checks if c["id"] == cid)


# ---- geometry ----

def test_camera_footprint_and_lawnmower_spacing():
    across, along = CAMERA.footprint_m(20.0)
    assert across == pytest.approx(2 * 20 * math.tan(math.radians(35)))  # 28.0 m
    assert along == pytest.approx(across * 3 / 4)
    assert CAMERA.gsd_cm(20.0) == pytest.approx(across / 4000 * 100)
    rect = [(0, 0), (60, 0), (60, 30), (0, 30)]
    p = plan_lawnmower(rect, 20.0, side_overlap=0.7, front_overlap=0.75)
    assert p.line_spacing_m == pytest.approx(across * 0.3)
    assert p.photo_spacing_m == pytest.approx(along * 0.25)
    assert len(p.lines) == math.ceil(30 / p.line_spacing_m)
    assert all(abs(a[1] - b[1]) < 1e-9 for a, b in p.lines)  # along the long (60 m) side
    assert p.lines[0][0] == pytest.approx((0, p.lines[0][0][1])) or p.lines[0][0][0] == pytest.approx(60)
    for (a1, b1), (a2, b2) in zip(p.lines, p.lines[1:]):
        assert math.dist(b1, a2) < math.dist(b1, b2)  # back and forth: each line starts where the last ended
    assert len(plan_lawnmower(rect, 40.0).lines) < len(p.lines)  # higher = wider photos = fewer lines


def test_bad_fields_are_refused():
    with pytest.raises(SurveyError, match="convex"):
        plan_lawnmower([(0, 0), (10, 0), (5, 2), (10, 10), (0, 10)], 20)
    with pytest.raises(SurveyError, match="corners"):
        plan_lawnmower([(0, 0), (1, 1)], 20)
    with pytest.raises(SurveyError, match="lines"):
        plan_lawnmower([(0, 0), (80, 0), (80, 80), (0, 80)], 5.0, side_overlap=0.9)


def test_coverage_counts_what_the_photos_really_saw():
    rect = [(0, 0), (40, 0), (40, 20), (0, 20)]
    across, along = CAMERA.footprint_m(10.0)
    one = {"x_m": 20.0, "y_m": 10.0, "alt_m": 10.0, "angle_rad": 0.0}
    c = coverage(rect, [one])
    assert c["coverage_pct"] == pytest.approx(100 * along * min(across, 20) / 800, abs=1.5)
    assert coverage(rect, [])["coverage_pct"] == 0 and len(coverage(rect, [])["gaps"]) > 0
    lower = {**one, "alt_m": 5.0}
    assert coverage(rect, [lower])["coverage_pct"] < c["coverage_pct"]  # lower = smaller footprint


# ---- compiling plans: checks ----

def test_a_simple_survey_plan_passes_every_check():
    p = compile_plan({"tasks": [{"type": "survey", "field": "north field"}]})
    assert p.ok and all(c["status"] == "pass" for c in p.checks)
    assert p.waypoints[0] == (0.0, 0.0) and p.waypoints[-1] == (0.0, 0.0)  # home and back
    assert p.land == {"place": "home", "x_m": 0.0, "y_m": 0.0, "precision": True, "pad_size_m": 1.0}
    assert len(p.camera_lines) == len(p.surveys[0]["lines"])
    assert p.estimate["photos"] > 20 and 0 < p.estimate["battery_used_pct"] < 40


def test_unsafe_values_fail_their_checks():
    p = compile_plan({"tasks": [{"type": "goto", "place": "pond"}], "altitude_m": 100, "speed_m_s": 9})
    assert not p.ok and status(p, "altitude") == "fail" and status(p, "speed") == "fail"
    tiny = compile_plan({"tasks": [{"type": "survey", "field": "east field"}] * 3}, battery=BatteryConfig(cells=3, capacity_mah=1000))
    assert status(tiny, "battery") == "fail" and not tiny.ok
    pad = compile_plan({"tasks": [{"type": "land", "place": "pond"}]})
    assert status(pad, "landing") == "fail"
    hover = compile_plan({"tasks": [{"type": "hover", "place": "gate", "seconds": 300}]})
    assert status(hover, "hover") == "fail"


def test_routes_go_around_tall_buildings_and_no_fly_zones_but_over_low_ones():
    low = compile_plan({"tasks": [{"type": "goto", "place": "gate"}], "altitude_m": 20})
    high_silo = compile_plan({"tasks": [{"type": "survey", "field": "east field"}], "altitude_m": 10})
    low_txt = next(c for c in low.checks if c["id"] == "obstacles")["detail"]
    assert low_txt.startswith("Nothing tall") and "flies over the barn, silo" in low_txt
    assert "silo (16 m tall)" in next(c for c in high_silo.checks if c["id"] == "obstacles")["detail"]
    nfz = compile_plan({"tasks": [{"type": "goto", "place": "gate"}, {"type": "survey", "field": "east field"}]})
    route = nfz.route
    garden = next(z for z in FARM.no_fly if z.name == "neighbour's garden").box
    for a, b in zip(route, route[1:]):
        for k in range(21):
            x, y = a[0] + (b[0] - a[0]) * k / 20, a[1] + (b[1] - a[1]) * k / 20
            assert garden.distance_2d(x, y) > 0.5  # never inside the neighbour's garden


def test_structure_errors_are_plain_english():
    with pytest.raises(PlanError, match="no field called"):
        compile_plan({"tasks": [{"type": "survey", "field": "moon"}]})
    with pytest.raises(PlanError, match="no place called"):
        compile_plan({"tasks": [{"type": "goto", "place": "school"}]})
    with pytest.raises(PlanError, match="last task"):
        compile_plan({"tasks": [{"type": "land", "place": "home"}, {"type": "goto", "place": "pond"}]})
    with pytest.raises(PlanError, match="1 to"):
        compile_plan({"tasks": []})


# ---- flying them ----

def test_survey_flight_covers_the_field_and_the_estimates_hold():
    p = compile_plan({"tasks": [{"type": "survey", "field": "north field"}], "altitude_m": 20})
    r = run_plan(p, battery=BatteryConfig())
    m = r.metrics
    assert m["completed"] and m["collisions"] == 0 and m["no_fly_entered"] == 0
    assert m["coverage_pct"] == 100.0 and m["photos"] >= p.estimate["photos"] - 3
    assert m["flight_time_s"] == pytest.approx(p.estimate["time_s"], rel=0.15)
    used_pct = 100 * m["battery_used_mah"] / 3000
    assert used_pct == pytest.approx(p.estimate["battery_used_pct"], rel=0.15)
    assert m["landing_error_m"] < 0.2


def test_no_overlap_plus_wind_and_gps_error_leaves_gaps_that_overlap_prevents():
    def cov(overlap):
        p = compile_plan({"tasks": [{"type": "survey", "field": "north field"}], "altitude_m": 20, "side_overlap_pct": overlap})
        r = run_plan(p, env=EnvConfig(wind_speed_m_s=6, wind_direction_deg=90, gust_std_m_s=1.2),
                     sensors=SensorConfig(enabled=True, gps_drift_m=1.5), seed=0)
        return r.metrics["coverage_pct"]

    assert cov(0) < 99.0 < cov(60) == 100.0


def test_hover_and_land_on_another_pad():
    p = compile_plan({"tasks": [{"type": "hover", "place": "gate", "seconds": 4}, {"type": "land", "place": "pad B"}],
                      "altitude_m": 12})
    r = run_plan(p)
    hovers = [e for e in r.events if e["type"] == "hover"]
    assert len(hovers) == 1 and r.metrics["completed"]
    assert r.metrics["landing_error_m"] < 0.2
    assert r.metrics["flight_time_s"] == pytest.approx(p.estimate["time_s"], rel=0.2)


def test_survey_experiment_runs_and_explains():
    exp = get_experiment("survey")
    r = exp.run({"field": "orchard", "altitude_m": 15}, 0)
    assert r.metrics["coverage_pct"] == 100.0 and r.metrics["gsd_cm"] == pytest.approx(CAMERA.gsd_cm(15), rel=0.01)
    assert any("photos" in line for line in r.summary) and r.plots[0]["kind"] == "map" and r.plots[0]["extra"]["photos"]


# ---- live ----

def test_live_plan_on_the_farm_with_time_speed_up():
    rt = MissionSim(get_preset("basic"))
    assert "farm" in rt.command("course", scenario="farm").lower()
    assert rt.course_info()["site"]["no_fly"]
    with pytest.raises(ValueError, match="ceiling"):
        rt.command("plan", spec={"tasks": [{"type": "goto", "place": "pond"}], "altitude_m": 90})
    rt.command("speed", factor=8)
    rt.command("plan", spec={"tasks": [{"type": "survey", "field": "orchard"}, {"type": "land", "place": "pad B"}], "altitude_m": 15})
    photos_seen = 0
    for _ in range(int(60 / 0.05)):  # 60 s of wall time = 480 s of sim time at 8x
        rt.advance(0.05)
        snap = rt.snapshot()
        if snap["survey"] and "photos" in snap["survey"]:
            photos_seen = len(snap["survey"]["photos"])
        if rt.follower is None:
            break
    assert "Plan complete" in rt.pilot.message and "pad B" in rt.pilot.message
    assert photos_seen > 10 and abs(rt.state.pos_m[0] + 12) < 0.3 and abs(rt.state.pos_m[1] - 30) < 0.3
    with pytest.raises(ValueError):
        rt.command("speed", factor=3)


# ---- 🌱 crop-health maps ----

def test_crop_map_finds_every_planted_patch_and_gps_error_shifts_it():
    from backend.autonomy.cropmap import PATCHES, build_map, recall, remember, rule_report

    p = compile_plan({"tasks": [{"type": "survey", "field": "north field"}], "altitude_m": 20})
    perfect = run_plan(p, seed=0)
    m = build_map("north field", p.surveys[0]["polygon"], perfect.scenario["photos"], seed=0)
    assert m["found"] == len(PATCHES["north field"]) == 2 and m["false_alarms"] == 0
    assert all(t["position_error_m"] < 0.6 for t in m["truth"])
    big = max(m["truth"], key=lambda t: t["area_m2"])
    assert big["found_area_m2"] == pytest.approx(big["area_m2"], rel=0.2)
    assert m["mapped_pct"] == 100.0 and 0.75 < m["field_ndvi"] < 0.85
    assert m["findings"][0]["where"] == "north-east corner"
    drift = run_plan(p, sensors=SensorConfig(enabled=True, gps_drift_m=1.5), seed=0)
    md = build_map("north field", p.surveys[0]["polygon"], drift.scenario["photos"], seed=0)
    assert md["found"] == 2 and md["geotag_error_m"] > 0.5
    assert max(t["position_error_m"] for t in md["truth"]) > max(t["position_error_m"] for t in m["truth"])
    key = remember(m)
    assert recall(key)["field"] == "north field" and "image" not in recall(key)
    assert any("Patch 1" in line for line in rule_report(m))


def test_crop_report_api_with_a_fake_llm(monkeypatch, tmp_path):
    import json as _json

    import httpx
    from fastapi.testclient import TestClient

    from backend import api
    from backend.ai import assistant
    from backend.ai.ollama_client import OllamaClient

    monkeypatch.setattr(assistant, "LOG_PATH", tmp_path / "log.jsonl")
    r = get_experiment("crop_health").run({"field": "orchard"}, 0)
    key = r.scenario["cropmap"]["id"]
    seen = []

    def handler(request):
        seen.append(_json.loads(request.content))
        return httpx.Response(200, json={"message": {"content": "- Orchard average NDVI 0.81; one patch of 999 m²."}})

    monkeypatch.setattr(api, "get_ai_client", lambda: OllamaClient("http://fake", "m", transport=httpx.MockTransport(handler)))
    client = TestClient(api.app)
    out = client.post("/api/crop/report", json={"id": key}).json()
    assert "999" in out["unverified_numbers"] and "0.81" not in out["unverified_numbers"]
    assert '"patches"' in seen[0]["messages"][1]["content"]
    assert client.post("/api/crop/report", json={"id": "nope"}).status_code == 404


def test_live_survey_makes_a_crop_map_after_landing():
    import time

    rt = MissionSim(get_preset("basic"))
    rt.command("speed", factor=8)
    rt.command("plan", spec={"tasks": [{"type": "survey", "field": "orchard"}], "altitude_m": 20})
    for _ in range(int(60 / 0.05)):
        rt.advance(0.05)
        rt.snapshot()
        if rt.follower is None:
            break
    deadline = time.monotonic() + 20
    while rt.cropmap_busy and time.monotonic() < deadline:
        time.sleep(0.05)
    snap = rt.snapshot()
    assert snap["cropmaps"] and snap["cropmaps"][0]["field"] == "orchard" and snap["cropmaps"][0]["found"] == 1
    assert "cropmaps" not in rt.snapshot()  # sent once
