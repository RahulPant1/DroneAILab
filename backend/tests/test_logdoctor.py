"""🩺 Log doctor: checks on synthetic logs, the log store's safety, the AI debrief with a fake Ollama, and (opt-in)
a real ArduCopter SITL log."""

import asyncio
import json
import os

import numpy as np
import pytest

from backend.ai import assistant
from backend.logdoctor import store
from backend.logdoctor.analyze import analyze_messages
from backend.logdoctor.explain import explain
from backend.tests.test_phase3 import client_with


def flight(seconds=60.0, hz=10.0, vibe=2.0, motors=(1590, 1590, 1590, 1590), sats=12, hdop=0.9, errors=(), msgs=(),
           sim_offset_m=0.0):
    """A synthetic ArduCopter log: armed at 1 s, hovering at 10 m, disarmed at the end."""
    t = np.arange(0, seconds, 1 / hz)
    rows = lambda f: [dict(t=float(x), **f(float(x))) for x in t]  # noqa: E731
    lat0, lon0 = -35.36, 149.16
    off = sim_offset_m / 6378137.0 * 180 / np.pi
    return {
        "EV": [{"t": 1.0, "Id": 10}, {"t": 2.0, "Id": 28}, {"t": seconds - 0.5, "Id": 11}],
        "MODE": [{"t": 1.0, "ModeNum": 4}],
        "CTUN": rows(lambda x: {"DAlt": 10.0, "Alt": 10.0 if x > 2 else 0.0}),
        "ATT": rows(lambda x: {"DesRoll": 0.0, "Roll": 0.5, "DesPitch": 0.0, "Pitch": -0.3}),
        "VIBE": rows(lambda x: {"VibeX": vibe, "VibeY": vibe * 0.8, "VibeZ": vibe * 1.2, "Clip": 0}),
        "XKF4": rows(lambda x: {"SV": 0.1, "SP": 0.05, "SH": 0.02, "SM": 0.1}),
        "GPS": rows(lambda x: {"Status": 3, "NSats": sats, "HDop": hdop, "Spd": 0.2}),
        "RCOU": rows(lambda x: {f"C{i + 1}": m for i, m in enumerate(motors)}),
        "BAT": rows(lambda x: {"Volt": 12.6 - 0.01 * x}),
        "POS": rows(lambda x: {"Lat": lat0, "Lng": lon0}),
        "SIM": rows(lambda x: {"Lat": lat0 + (off if x > 30 else 0.0), "Lng": lon0}),
        "PSCN": rows(lambda x: {"TPN": 0.0, "PN": 0.1}),
        "PSCE": rows(lambda x: {"TPE": 0.0, "PE": -0.1}),
        "ERR": [dict(t=t_, Subsys=s, ECode=c) for t_, s, c in errors],
        "MSG": [dict(t=t_, Message=m) for t_, m in msgs],
        "PARM": [{"t": 0.0, "Name": "FRAME_CLASS", "Value": 1}],
    }


def by_id(report):
    return {c["id"]: c for c in report["checks"]}


def test_a_healthy_hover_passes_every_check():
    r = analyze_messages(flight(msgs=[(0.0, "ArduCopter V4.7.3 (d9c5097c)"), (1.0, "Arming motors")]), "healthy.BIN")
    assert r["verdict"] == "ok", [(c["title"], c["value"]) for c in r["checks"] if c["status"] != "ok"]
    assert r["overview"]["modes_used"] == ["GUIDED"] and r["overview"]["frame"] == "quad"
    assert r["overview"]["max_altitude_m"] == 10.0 and r["overview"]["armed_time_s"] == pytest.approx(58.5)
    texts = [e["text"] for e in r["timeline"]]
    assert "Arming motors" in texts and not any(t.startswith("ArduCopter V") for t in texts)  # boot chatter hidden
    assert set(r["charts"]) >= {"altitude", "vibration", "motors", "ekf", "truth"}
    assert all(len(col) <= 400 for ch in r["charts"].values() for col in ch["data"])


def test_each_problem_is_caught_by_its_own_check():
    assert by_id(analyze_messages(flight(vibe=45)))["vibration"]["status"] == "warn"
    assert by_id(analyze_messages(flight(vibe=70)))["vibration"]["status"] == "bad"
    assert by_id(analyze_messages(flight(sats=7, hdop=1.6)))["gps"]["status"] == "warn"
    motors = by_id(analyze_messages(flight(motors=(1800, 1500, 1500, 1500))))["motors"]
    assert motors["status"] == "bad" and "motor 1" in motors["meaning"]  # one motor works much harder
    truth = by_id(analyze_messages(flight(sim_offset_m=20)))["truth"]
    assert truth["status"] == "bad" and "20." in truth["value"]


def test_failsafes_and_crashes_are_decoded_and_flagged():
    r = analyze_messages(flight(errors=[(32.0, 17, 1), (40.0, 12, 1), (50.0, 11, 2)]))
    fs = by_id(r)["failsafes"]
    assert fs["status"] == "bad"
    assert "EKF failsafe: TRIGGERED" in fs["value"] and "crash detected" in fs["value"]
    assert "GPS: GPS glitch" in [e["text"] for e in r["timeline"]]
    assert r["verdict"] == "bad"


def test_store_only_maps_known_ids_and_rejects_non_logs(tmp_path, monkeypatch):
    monkeypatch.setattr(store, "UPLOAD_DIR", tmp_path)
    monkeypatch.setattr(store, "SITL_LOG_DIR", tmp_path / "sitl")
    for bad in ("../etc/passwd", "upload-../../x", "sitl-12", "nope"):
        with pytest.raises(store.LogNotFound):
            store.path_for(bad)
    with pytest.raises(ValueError, match="DataFlash"):
        store.save_upload("x.bin", b"not a log")
    with pytest.raises(ValueError, match=".BIN"):
        store.save_upload("flight.tlog", b"\xa3\x95....")
    log_id = store.save_upload("my flight (1).BIN", b"\xa3\x95" + b"\0" * 2000)
    assert log_id.startswith("upload-") and store.path_for(log_id).parent == tmp_path
    assert [d["id"] for d in store.list_logs()] == [log_id]


def test_ai_debrief_sees_only_the_summary_and_checks_numbers(tmp_path, monkeypatch):
    monkeypatch.setattr(assistant, "LOG_PATH", tmp_path / "log.jsonl")
    r = analyze_messages(flight(errors=[(32.0, 17, 1)]), "gps.BIN")
    seen: list = []
    reply = ("What happened:\n- [Measured] EKF failsafe at 32 s.\nMost likely cause:\n- [Hypothesis] GPS.\n"
             "What to check next:\n- [Hypothesis] Antenna placement; it drifted 57.3 m.")
    out = asyncio.run(explain(client_with(reply=reply, seen=seen), r))
    sent = seen[0]["messages"][1]["content"]
    assert "LOG ANALYSIS" in sent and '"charts"' not in sent and len(sent) < 6000  # summary only, no raw data
    assert out["unverified_numbers"] == ["57.3"]  # invented number flagged; 32 is in the data
    logged = json.loads((tmp_path / "log.jsonl").read_text().splitlines()[-1])
    assert logged["kind"] == "log_explain" and logged["log"] == "gps.BIN"


@pytest.mark.skipif(os.environ.get("DRONELAB_SITL") != "1", reason="real SITL log: set DRONELAB_SITL=1")
def test_a_real_sitl_gps_loss_log_tells_the_story():
    from backend.logdoctor.analyze import analyze
    from backend.sitl import faults, launch

    if not launch.is_built():
        pytest.skip("ArduCopter SITL not built")
    result = faults.run_fault("gps_loss", speedup=3, say=lambda *_: None)
    assert result.log, "the fault run should point at its flight log"
    r = analyze(store.path_for(result.log))
    checks = by_id(r)
    assert checks["gps"]["status"] == "bad" and checks["failsafes"]["status"] == "bad"
    assert "EKF failsafe: TRIGGERED" in checks["failsafes"]["value"]
    assert "LAND" in r["overview"]["modes_used"] and r["overview"]["simulated"]
