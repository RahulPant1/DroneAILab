"""Gyro noise spectrum + notch recommendation (backend/logdoctor/spectrum.py) and the SITL filter comparison.

Offline: synthetic batch-sampler messages. The real flights (three fresh ArduCopter SITLs, about 4 minutes) only run
when asked:  DRONELAB_SITL=1 .venv/bin/pytest -q backend/tests/test_vibration.py
"""

import math
import os

import numpy as np
import pytest

from backend.logdoctor import spectrum as S
from backend.logdoctor.analyze import analyze_messages
from backend.sitl import launch

RATE_HZ, MUL = 1000.0, 938.0  # 1 kHz, int16 per rad/s (as ArduPilot logs it)


def batch_messages(motor_hz: float, amp_deg_s: float, n_blocks: int = 6, instance: int = 0, t0: float = 10.0,
                   seed: int = 0) -> tuple[list[dict], list[dict]]:
    """ISBH headers + ISBD data like the batch sampler writes: 1024 gyro samples per block, 32 per message."""
    rng = np.random.default_rng(seed)
    isbh, isbd = [], []
    for k in range(n_blocks):
        n = k * 10 + instance
        t = np.arange(1024) / RATE_HZ + k
        sig = np.radians(amp_deg_s) * np.sin(2 * np.pi * motor_hz * t)[:, None] + np.radians(0.05) * rng.standard_normal((1024, 3))
        raw = np.round(sig * MUL).astype(int)
        isbh.append({"t": t0 + k, "N": n, "type": 1, "instance": instance, "mul": MUL, "smp_cnt": 1024, "smp_rate": RATE_HZ})
        for seq in range(32):
            part = raw[seq * 32:(seq + 1) * 32]
            isbd.append({"t": t0 + k, "N": n, "seqno": seq, "x": list(part[:, 0]), "y": list(part[:, 1]), "z": list(part[:, 2])})
    return isbh, isbd


def test_fft_finds_the_motor_peak_and_its_size():
    isbh, isbd = batch_messages(187.0, 4.0)
    blocks = S.batches(isbh, isbd)[0]
    freqs, amp = S.average_spectrum(blocks)
    peak = S.find_peak(freqs, amp)
    assert peak["freq_hz"] == pytest.approx(187.0, abs=1.5)
    assert peak["amp_deg_s"] == pytest.approx(4.0 * math.sqrt(3), rel=0.15)  # same sine on 3 axes, combined
    assert peak["above_floor_db"] > 20


def test_no_peak_in_plain_noise():
    isbh, isbd = batch_messages(187.0, 0.0)
    assert S.find_peak(*S.average_spectrum(S.batches(isbh, isbd)[0])) is None


def test_notch_recommendation_follows_the_parameter_docs():
    rec = S.notch_recommendation(192.4, 0.37)
    assert rec == {"INS_HNTCH_ENABLE": 1, "INS_HNTCH_MODE": 1, "INS_HNTCH_FREQ": 192, "INS_HNTCH_BW": 96,
                   "INS_HNTCH_ATT": 40, "INS_HNTCH_HMNCS": 1, "INS_HNTCH_REF": 0.37}


def test_filters_trade_delay_for_noise():
    notch = S.notch_recommendation(192.0, 0.37)
    lp20, lp80, both = S.filter_cost(20, None, 10, 192), S.filter_cost(80, None, 10, 192), S.filter_cost(80, notch, 10, 192)
    assert lp20["lag_ms"] > 3 * lp80["lag_ms"]  # a low cutoff delays the gyro a lot
    assert lp80["noise_left"] > 10 * lp20["noise_left"]  # ...but a high one lets the motor noise through
    assert both["noise_left"] < lp20["noise_left"] and both["lag_ms"] < lp20["lag_ms"] / 2  # the notch: both
    depth = abs(S.notch_response(192, 96, 40, [192])[0])
    assert 20 * math.log10(depth) == pytest.approx(-40.0, abs=0.01)
    assert abs(S.notch_response(192, 96, 40, [5])[0]) == pytest.approx(1.0, abs=0.01)  # leaves slow motion alone


def test_log_doctor_reports_gyro_noise_and_recommends_a_notch():
    pre_h, pre_d = batch_messages(190.0, 6.0, instance=0)
    post_h, post_d = batch_messages(190.0, 1.5, instance=2, seed=1)
    L = {
        "EV": [{"t": 1.0, "Id": 10}, {"t": 30.0, "Id": 11}],
        "PARM": [{"t": 0.0, "Name": "INS_LOG_BAT_OPT", "Value": 4}, {"t": 0.0, "Name": "INS_GYRO_FILTER", "Value": 80},
                 {"t": 0.0, "Name": "MOT_THST_HOVER", "Value": 0.35}],
        "ISBH": pre_h + post_h,
        "ISBD": pre_d + post_d,
    }
    r = analyze_messages(L)
    c = next(c for c in r["checks"] if c["id"] == "gyro_noise")
    assert c["status"] == "warn" and "190 Hz" in c["value"] and "after the filters" in c["value"]
    assert "INS_HNTCH_FREQ = 190" in c["meaning"] and "INS_HNTCH_REF = 0.35" in c["meaning"]
    chart = r["charts"]["gyro_spectrum"]
    assert chart["x_label"] == "frequency (Hz)" and len(chart["labels"]) == 2


def test_logs_without_the_batch_sampler_have_no_gyro_check():
    r = analyze_messages({"EV": [{"t": 1.0, "Id": 10}, {"t": 30.0, "Id": 11}]})
    assert not any(c["id"] == "gyro_noise" for c in r["checks"]) and "gyro_spectrum" not in r["charts"]


@pytest.mark.skipif(os.environ.get("DRONELAB_SITL") != "1", reason="real SITL flight: set DRONELAB_SITL=1")
@pytest.mark.skipif(not launch.is_built(), reason="ArduCopter SITL not built (scripts/sitl_setup.sh)")
def test_real_ardupilot_notch_removes_the_motor_noise():
    from backend.sitl.vibration import compare

    res = compare(say=lambda _t: None)
    runs = res["runs"]
    assert 100 < res["notch"]["INS_HNTCH_FREQ"] < 400
    left = {k: (r["spectrum"].get("post_peak") or {"amp_deg_s": 0.0})["amp_deg_s"] for k, r in runs.items()}
    assert left["lpf80_notch"] < left["lpf80"] / 5
    assert runs["lpf80_notch"]["filter_cost"]["lag_ms"] < runs["lpf20"]["filter_cost"]["lag_ms"] / 2
