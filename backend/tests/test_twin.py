"""🛠 -> 🛰 Digital twin and starting parameters (backend/sim/apparams.py, backend/sitl/twin.py).

Offline: the maths and the two file formats. The real checks (ArduCopter SITL boots with the file: every name exists,
every value and type reads back; then the twin flies its hover + failsafe tests, ~3 min) only run when asked:
    DRONELAB_SITL=1 .venv/bin/pytest -q backend/tests/test_twin.py
"""

import json
import math
import os
import time

import pytest
from fastapi.testclient import TestClient

from backend import api
from backend.sim import apparams as A
from backend.sim import builder as B
from backend.sitl import launch

SIM_FRAME_FIELDS = {"mass", "diagonal_size", "refSpd", "refAngle", "refVoltage", "refCurrent", "refAlt", "refTempC",
                    "maxVoltage", "battCapacityAh", "refBatRes", "propExpo", "refRotRate", "hoverThrOut", "pwmMin",
                    "pwmMax", "spin_min", "spin_max", "slew_max", "disc_area", "mdrag_coef", "num_motors"}  # SIM_Frame.cpp


def test_hover_throttle_inverts_the_thrust_curve():
    for tw in (1.5, 2.0, 3.79, 8.0):
        t = A.hover_throttle(tw)
        thrust = (1 - A.PROP_EXPO) * t + A.PROP_EXPO * t * t  # MOT_THST_EXPO curve
        assert thrust == pytest.approx(1.0 / tw, abs=1e-9)
    assert A.hover_throttle(2.0, expo=0.0) == pytest.approx(0.5)


@pytest.mark.parametrize("prop,gyro,acc", [(5, 80, 1100), (10, 40, 1100), (15, 25, 800), (20, 20, 500), (30, 20, 200)])
def test_prop_size_rules_follow_ardupilots_tuning_table(prop, gyro, acc):
    assert A.gyro_filter_hz(prop) == gyro and A.accel_max_degss(prop) == acc


def test_parameters_are_sane_for_a_build():
    d = B.design(B.DEFAULT_PARTS)
    p = {n: v for n, v, _ in A.params(d)}
    assert len(p) == len(A.params(d))  # no name twice
    assert p["BATT_CAPACITY"] == 3000 and p["BATT_LOW_VOLT"] == pytest.approx(3 * 3.5) and p["BATT_FS_LOW_ACT"] == 2
    assert p["MOT_BAT_VOLT_MAX"] == pytest.approx(12.6) and 0.3 < p["MOT_THST_HOVER"] < 0.5
    assert p["FS_THR_ENABLE"] == 1 and p["FENCE_ENABLE"] == 1 and p["RTL_ALT_M"] > 5 and p["ARMING_SKIPCHK"] == 0
    assert "RTL_ALT" not in p and "ARMING_CHECK" not in p  # renamed in 4.7: the old names would be ignored
    assert p["ATC_RAT_RLL_FLTD"] == p["INS_GYRO_FILTER"] / 2


def test_both_file_formats_parse():
    d = B.design(B.DEFAULT_PARTS)
    names = [n for n, _, _ in A.params(d)]
    mp = [ln for ln in A.param_file(d).splitlines() if ln and not ln.startswith("#")]
    assert [ln.split(",")[0] for ln in mp] == names and all(len(ln.split(",")) == 2 for ln in mp)
    qgc = [ln.split("\t") for ln in A.qgc_param_file(d).splitlines() if ln and not ln.startswith("#")]
    assert all(len(c) == 5 and c[0] == "1" and c[1] == "1" and c[4] in {"2", "4", "6", "9"} for c in qgc)
    assert [c[2] for c in qgc] == names
    for c in qgc:  # integer-typed parameters must be written as integers
        if c[4] != "9":
            assert float(c[3]).is_integer(), c


def test_sitl_model_uses_only_sim_frame_fields_and_the_builders_numbers():
    d = B.design(B.DEFAULT_PARTS)
    m = A.sitl_model(d)
    assert set(m) <= SIM_FRAME_FIELDS
    assert m["mass"] == pytest.approx(d["config"].mass_kg) and m["refCurrent"] == d["report"]["hover_current_a"]
    assert m["hoverThrOut"] == pytest.approx(A.hover_throttle(d["report"]["thrust_to_weight"]), abs=1e-3)
    assert m["disc_area"] == pytest.approx(4 * math.pi * (10 * 0.0254 / 2) ** 2, rel=1e-3)


def test_param_download_api():
    c = TestClient(api.app)
    r = c.post("/api/builder/params", json={"parts": {}, "format": "qgc"})
    assert r.status_code == 200 and r.json()["filename"] == "dronelab.params" and "\tMOT_THST_HOVER\t" in r.json()["text"]
    assert c.post("/api/builder/params", json={"parts": {}, "format": "xml"}).status_code == 400
    bad = c.post("/api/builder/params", json={"parts": {"prop": "15in", "frame": "racer_250"}})
    assert bad.status_code == 400  # props that don't fit the frame


@pytest.mark.skipif(os.environ.get("DRONELAB_SITL") != "1", reason="real SITL: set DRONELAB_SITL=1")
@pytest.mark.skipif(not launch.is_built(), reason="ArduCopter SITL not built (scripts/sitl_setup.sh)")
def test_real_ardupilot_accepts_every_parameter_with_its_type(tmp_path):
    from pymavlink import mavutil

    from backend.sitl.twin import build_files, twin_boot_params

    files = build_files({}, tmp_path)
    d = files["design"]
    sitl = launch.start(extra_params=twin_boot_params(d), model_json=files["model"])
    try:
        assert "Loaded model params" in sitl.log_path.read_text()
        m = mavutil.mavlink_connection(sitl.url)
        m.wait_heartbeat(timeout=30)
        m.mav.param_request_list_send(m.target_system, m.target_component)
        got, t0 = {}, time.time()
        while time.time() - t0 < 60:
            msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=5)
            if msg is None:
                break
            got[msg.param_id] = (msg.param_value, msg.param_type)
            if len(got) >= msg.param_count:
                break
        m.close()
    finally:
        sitl.stop()
    for name, value, _ in A.params(d):
        assert name in got, f"{name} doesn't exist in ArduCopter 4.7"
        assert got[name][0] == pytest.approx(float(value), rel=1e-3, abs=1e-4), name
        assert got[name][1] == A.PARAM_TYPES.get(name, 9), f"{name}: type {got[name][1]}"


@pytest.mark.skipif(os.environ.get("DRONELAB_SITL") != "1", reason="real SITL flight: set DRONELAB_SITL=1")
@pytest.mark.skipif(not launch.is_built(), reason="ArduCopter SITL not built (scripts/sitl_setup.sh)")
def test_real_ardupilot_flies_the_twin_and_its_failsafes_work(tmp_path):
    from backend.sitl.twin import build_files, fly

    files = build_files({}, tmp_path)
    res = fly(files, speedup=4, say=lambda _t: None)
    h, t = res["tests"]["hover"], res["tests"]
    assert abs(h["throttle_pct"] - 100 * A.hover_throttle(res["builder"]["thrust_to_weight"])) < 6
    assert t["radio_lost"]["ok"] and t["radio_lost"]["back_home_m"] < 3
    assert t["battery_low"]["ok"]
    json.dumps(res)  # results are plain JSON
