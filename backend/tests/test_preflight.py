"""✈ Pre-flight parameter check and the log doctor's real-hardware checks (compass, yaw fight, wobble, cells)."""

import numpy as np

from backend.logdoctor import preflight
from backend.logdoctor.analyze import analyze_messages
from backend.sim import apparams, builder
from backend.tests.test_logdoctor import by_id, flight
from fastapi.testclient import TestClient

from backend import api


def items(r):
    return {i["id"]: i for i in r["items"]}


def full_dump(**over):
    """A board's full parameter dump that is ready to fly (calibrated, failsafes on)."""
    p = {"FRAME_CLASS": 1, "FRAME_TYPE": 1, "INS_ACCOFFS_X": 0.02, "INS_ACCOFFS_Y": -0.01, "INS_ACCOFFS_Z": 0.05,
         "INS_ACCSCAL_X": 1.001, "INS_ACCSCAL_Y": 0.998, "INS_ACCSCAL_Z": 1.0, "COMPASS_USE": 1,
         "COMPASS_OFS_X": 45, "COMPASS_OFS_Y": -80, "COMPASS_OFS_Z": 120, "FS_THR_ENABLE": 1, "BATT_MONITOR": 4,
         "BATT_LOW_VOLT": 10.5, "BATT_FS_LOW_ACT": 2, "FS_EKF_ACTION": 1, "FENCE_ENABLE": 1, "FENCE_ALT_MAX": 60,
         "FENCE_RADIUS": 100, "RTL_ALT_M": 15, "FLTMODE1": 0, "FLTMODE2": 2, "FLTMODE3": 5, "FLTMODE6": 6,
         "ARMING_SKIPCHK": 0, "MOT_THST_HOVER": 0.35, "MOT_SPIN_ARM": 0.1, "MOT_SPIN_MIN": 0.15,
         "INS_LOG_BAT_MASK": 1}
    for i in (1, 2, 3, 4):
        p[f"RC{i}_MIN"], p[f"RC{i}_MAX"] = 982 + i, 2006 - i
    p.update(over)
    return p


def test_ready_board_passes():
    r = preflight.check(full_dump())
    assert r["verdict"] == "ok", [(i["title"], i["found"]) for i in r["items"] if i["status"] != "ok"]


def test_each_preflight_problem():
    assert items(preflight.check(full_dump(INS_ACCOFFS_X=0, INS_ACCOFFS_Y=0, INS_ACCOFFS_Z=0, INS_ACCSCAL_X=1,
                                           INS_ACCSCAL_Y=1)))["accel_cal"]["status"] == "bad"
    assert items(preflight.check(full_dump(COMPASS_OFS_X=0, COMPASS_OFS_Y=0, COMPASS_OFS_Z=0)))["compass_cal"]["status"] == "bad"
    assert items(preflight.check(full_dump(COMPASS_OFS_Z=900)))["compass_ofs"]["status"] == "warn"
    assert items(preflight.check(full_dump(FS_THR_ENABLE=0)))["fs_radio"]["status"] == "bad"
    assert items(preflight.check(full_dump(BATT_FS_LOW_ACT=0)))["fs_batt_low"]["status"] == "bad"
    assert items(preflight.check(full_dump(FLTMODE6=5)))["rtl_switch"]["status"] == "warn"
    assert items(preflight.check(full_dump(FLTMODE6=5, RC7_OPTION=4)))["rtl_switch"]["status"] == "ok"
    assert items(preflight.check(full_dump(ARMING_SKIPCHK=8)))["arming"]["status"] == "bad"
    assert items(preflight.check(full_dump(MOT_THST_HOVER=0.7)))["hover"]["status"] == "warn"
    assert items(preflight.check(full_dump(RTL_ALT_M=5)))["rtl_alt"]["status"] == "warn"
    radio = {f"RC{i}_{e}": v for i in (1, 2, 3, 4) for e, v in (("MIN", 1100), ("MAX", 1900))}
    assert items(preflight.check(full_dump(**radio)))["radio_cal"]["status"] == "warn"
    r = preflight.check(full_dump(RTL_ALT=1500))
    assert items(r)["renamed"]["status"] == "warn" and "RTL_ALT_M" in items(r)["renamed"]["todo"]
    assert r["fix_first"][0]["id"] == "renamed"


def test_builder_file_round_trips_through_both_formats():
    d = builder.design({})
    for text in (apparams.qgc_param_file(d), apparams.param_file(d)):
        p = preflight.parse_param_text(text)
        assert p["FRAME_CLASS"] == 1 and p["FS_THR_ENABLE"] == 1 and len(p) == len(apparams.params(d))
        r = preflight.check(p, partial=True)
        # what DroneLab can set is all fine; only the calibrations are left, as 'to do on the board'
        assert r["verdict"] == "ok", [(i["title"], i["found"]) for i in r["items"] if i["status"] in ("bad", "warn")]
        assert items(r)["accel_cal"]["status"] == "info" and items(r)["compass_cal"]["status"] == "info"


def test_api():
    c = TestClient(api.app)
    r = c.post("/api/preflight", content=b"FRAME_CLASS,1\nFS_THR_ENABLE,0\n")
    assert r.status_code == 200 and items(r.json())["fs_radio"]["status"] == "bad" and r.json()["partial"]
    assert c.post("/api/preflight", content=b"hello").status_code == 400
    r = c.post("/api/preflight/builder", json={"parts": {}})
    assert r.status_code == 200 and r.json()["verdict"] == "ok"


# ---- the log doctor's real-hardware checks ----

def with_(log, **extra):
    log = dict(log)
    log.update(extra)
    return log


def mag_rows(log, pct):
    """MAG field that grows `pct` % from idle to full throttle; throttle swings 30-70 %."""
    ctun, mag = [], []
    for r in log["CTUN"]:
        thr = 0.5 + 0.2 * np.sin(r["t"] / 3)
        ctun.append(dict(r, ThO=float(thr)))
        f = 400.0 * (1 + pct / 100 * thr)
        mag.append({"t": r["t"], "MagX": f * 0.6, "MagY": f * 0.0, "MagZ": f * 0.8})
    return ctun, mag


def test_compass_interference():
    log = flight()
    for pct, status in ((10, "ok"), (45, "warn"), (90, "bad")):
        ctun, mag = mag_rows(log, pct)
        c = by_id(analyze_messages(with_(log, CTUN=ctun, MAG=mag)))["compass_motor"]
        assert c["status"] == status, (pct, c["value"])
    ctun = [dict(r, ThO=0.5) for r in log["CTUN"]]
    _, mag = mag_rows(log, 50)
    assert by_id(analyze_messages(with_(log, CTUN=ctun, MAG=mag)))["compass_motor"]["status"] == "info"


def test_motors_fighting_on_yaw():
    assert by_id(analyze_messages(flight()))["motor_yaw"]["status"] == "ok"
    c = by_id(analyze_messages(flight(motors=(1640, 1640, 1540, 1540))))["motor_yaw"]
    assert c["status"] == "warn" and "1 and 2" in c["value"]
    r = analyze_messages(flight(motors=(1500, 1500, 1660, 1660)))
    assert by_id(r)["motor_yaw"]["status"] == "bad"
    assert "motor_yaw" in [f["id"] for f in r["fix_first"]]  # after 'motors' (also unbalanced here)


def test_wobble():
    log = flight()
    assert by_id(analyze_messages(log))["wobble"]["status"] == "ok"
    att = [dict(r, Roll=0.5 + 4.0 * np.sin(2 * np.pi * 3.0 * r["t"])) for r in log["ATT"]]  # 3 Hz ringing
    c = by_id(analyze_messages(with_(log, ATT=att)))["wobble"]
    assert c["status"] == "bad" and "3.0 Hz" in c["value"]


def test_battery_per_cell_and_resistance():
    log = flight()
    assert by_id(analyze_messages(log))["battery_cells"]["status"] == "ok"
    # 4S pack, 20 mOhm per cell: V = 16.3 - 0.08 * I with the current swinging 10-30 A
    bat = [{"t": r["t"], "Volt": 16.3 - 0.08 * (20 + 10 * np.sin(r["t"])), "Curr": 20 + 10 * np.sin(r["t"])} for r in log["BAT"]]
    c = by_id(analyze_messages(with_(log, BAT=bat)))["battery_cells"]
    assert "4S" in c["value"] and "20 mΩ" in c["value"] and c["status"] == "warn"  # 3.5 V/cell at 30 A: just under
    low = [{"t": r["t"], "Volt": 12.6 - 0.04 * r["t"]} for r in log["BAT"]]  # 3S sagging to ~3.4 V/cell
    assert by_id(analyze_messages(with_(log, BAT=low)))["battery_cells"]["status"] in ("warn", "bad")


def test_report_has_preflight_when_the_log_has_all_parameters():
    log = flight()
    assert analyze_messages(log)["preflight"] is None  # 1 parameter: not a full dump
    parm = [{"t": 0.0, "Name": k, "Value": v} for k, v in full_dump(FS_THR_ENABLE=0).items()]
    parm += [{"t": 0.0, "Name": f"X_{i}", "Value": 0} for i in range(100)]
    r = analyze_messages(with_(log, PARM=parm))
    assert r["preflight"]["verdict"] == "bad" and items(r["preflight"])["fs_radio"]["status"] == "bad"


# ---- opt-in: real ArduCopter SITL flights with the problem built in (DRONELAB_SITL=1) ----
import os  # noqa: E402

import pytest  # noqa: E402


@pytest.mark.skipif(os.environ.get("DRONELAB_SITL") != "1", reason="real SITL flights: set DRONELAB_SITL=1")
def test_sitl_compass_and_wobble_are_caught():
    from backend.sitl.hardware_check import run

    res = run(["compass", "wobble"], say=lambda *_: None)
    assert res["compass"]["checks"]["compass_motor"]["status"] == "bad"
    assert res["compass"]["checks"]["wobble"]["status"] == "ok"
    assert res["wobble"]["checks"]["wobble"]["status"] == "bad"
    assert res["wobble"]["checks"]["compass_motor"]["status"] in ("ok", "info")
