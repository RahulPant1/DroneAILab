"""Fault-injection lab: judges, report and panel rules without SITL, plus every scenario on real ArduCopter.

The real runs take ~35 s each (11 scenarios, ~7 min), so they only run when asked:
    DRONELAB_SITL=1 .venv/bin/pytest -q backend/tests/test_faults.py
"""

import os
import xml.etree.ElementTree as ET

import pytest

from backend.sitl import faults, launch
from backend.sitl.bridge import SitlBridge
from backend.sitl.faults import FAULTS, HOLD_ENU, FaultResult


def fake_result(fault_id: str, path: list[tuple[float, float]], events=(), alt=10.0, mode="GUIDED") -> FaultResult:
    """A result as if the drone flew `path` (true = believed) after the fault, sampled every 0.2 s."""
    f = faults.BY_ID[fault_id]
    r = FaultResult(f.id, f.title, f.frame, f.injected, f.expected, kind=f.kind)
    for i, (e, n) in enumerate(path):
        s = r.samples
        s["t_s"].append(round(i * 0.2, 2))
        s["est_e"].append(e)
        s["est_n"].append(n)
        s["alt"].append(alt)
        s["true_e"].append(e)
        s["true_n"].append(n)
        s["mode"].append(mode)
    r.events = list(events)
    landed = any(text == "Disarming motors" for _, text in r.events)
    r.metrics = faults._metrics(r, None, HOLD_ENU[0], HOLD_ENU[1], r.events[-1][0] if landed else None)
    return r


def test_every_scenario_has_a_clear_story():
    assert len({f.id for f in FAULTS}) == len(FAULTS)
    for f in FAULTS:
        assert f.kind in ("safety", "limitation") and f.frame in launch.FRAMES
        assert f.injected and f.expected and f.observe_s > 0
        assert f.inject or f.keep_doing  # it must actually break something


def test_rtl_judge_needs_rtl_and_a_landing_at_home():
    home_run = [(0.0, 20.0 - k) for k in range(21)] + [(0.0, 0.0)] * 5
    ok = fake_result("rc_loss", home_run, [(1.0, "mode RTL"), (30.0, "Disarming motors")])
    assert faults.judge_rtl(ok)[0]
    stayed = fake_result("rc_loss", [(0.0, 20.0)] * 20, [(1.0, "mode RTL"), (30.0, "Disarming motors")])
    assert not faults.judge_rtl(stayed)[0]  # landed 20 m from home: not a return to launch
    no_rtl = fake_result("rc_loss", home_run, [(30.0, "Disarming motors")])
    assert not faults.judge_rtl(no_rtl)[0]


def test_hold_judge_uses_the_true_position_and_rejects_failsafes():
    calm = fake_result("strong_wind", [(0.3, 20.2)] * 20)
    assert faults.judge_hold(3.0)(calm)[0]
    blown = fake_result("strong_wind", [(0.0, 20.0), (4.0, 20.0)])
    assert not faults.judge_hold(3.0)(blown)[0]
    landed = fake_result("strong_wind", [(0.0, 20.0)] * 5, [(2.0, "mode LAND")])
    assert not faults.judge_hold(3.0)(landed)[0]


def test_motor_judges_read_the_impact_message():
    fell = fake_result("motor_quad", [(0.0, 20.0)] * 5, [(1.7, "SIM Hit ground at 10.7 m/s")], alt=0.0)
    passed, text = faults.judge_quad_motor(fell)
    assert passed and "10.7 m/s" in text
    soft = fake_result("motor_quad", [(0.0, 20.0)] * 5, [(9.0, "SIM Hit ground at 1.2 m/s")])
    assert not faults.judge_quad_motor(soft)[0]  # landing softly later is not "fell out of the sky"


def test_report_has_both_tables_and_valid_charts(tmp_path):
    results = [
        fake_result("rc_loss", [(0.0, 20.0 - k) for k in range(21)], [(1.0, "mode RTL"), (30.0, "Disarming motors")]),
        fake_result("gps_spoof", [(0.0, 20.0 - k) for k in range(21)]),
    ]
    results[0].verdict, results[1].verdict = "PASS", "CONFIRMED"
    path = faults.write_report(results, tmp_path)
    text = path.read_text()
    assert "Safety checks: 1 of 1 passed" in text and "Known limitations: 1 of 1 reproduced" in text
    assert "## Safety checks" in text and "## Known limitations" in text
    for r in results:
        ET.parse(tmp_path / "faults" / f"{r.id}.svg")  # well-formed SVG
    assert [r.id for r in faults.load_results(tmp_path / "faults" / "results.json")] == ["rc_loss", "gps_spoof"]


def test_panel_rejects_unknown_faults():
    with pytest.raises(ValueError, match="Unknown fault"):
        SitlBridge().handle({"action": "fault", "fault": "lightning"})


@pytest.mark.skipif(os.environ.get("DRONELAB_SITL") != "1", reason="real SITL flight: set DRONELAB_SITL=1")
@pytest.mark.skipif(not launch.is_built(), reason="ArduCopter SITL not built (scripts/sitl_setup.sh)")
@pytest.mark.parametrize("fault_id", [f.id for f in FAULTS])
def test_real_arducopter_handles_the_fault(fault_id):
    r = faults.run_fault(fault_id, speedup=3, say=lambda *_: None)
    assert r.verdict in ("PASS", "CONFIRMED"), f"{r.verdict}: {r.observed} | events: {r.events[:8]}"
