"""Gyro noise spectrum from ArduPilot's batch sampler, and a harmonic-notch recommendation.

Motors and props shake the flight controller at the rotor's spin frequency (and its multiples, the harmonics). The
gyro feels that as fake rotation, the rate controller reacts to it, and the motors get a fast buzz on top of their
command: hot motors, wasted battery, and a twitchy drone. Two filters fight it:

  - a LOW-PASS filter (INS_GYRO_FILTER, default 20 Hz) removes everything fast, but also delays the real signal
    (phase lag), which limits how sharp the control can be;
  - a HARMONIC NOTCH (INS_HNTCH_*) removes a narrow band around the motor frequency only, and follows it as the
    throttle changes. With a notch the low-pass can be raised (e.g. 80 Hz) for less lag.

The batch sampler (INS_LOG_BAT_MASK = 1) logs blocks of raw gyro samples at the sensor rate (typically 1 kHz).
INS_LOG_BAT_OPT = 4 logs them both before and after the filters, so you can see what the filters removed.
We average the FFT of every block taken while flying (Welch's method) and look for the biggest peak.

The recommendation follows the notch parameter descriptions in ArduPilot's source (libraries/Filter/
HarmonicNotchFilter.cpp): FREQ = the peak at the reference throttle, BW = FREQ / 2, MODE 1 = follow the throttle,
REF = the hover throttle.
"""

from __future__ import annotations

import math
from collections import defaultdict

import numpy as np

MIN_PEAK_HZ = 30.0  # below this it's the drone really moving (or the frame's own modes), not motor noise
PEAK_OVER_FLOOR_DB = 10.0  # a peak this far above the median noise floor counts as a motor-noise peak
MAX_POINTS = 256  # chart points
GYRO = 1  # ISBH.type: 0 = accelerometer, 1 = gyro


def batches(isbh: list[dict], isbd: list[dict], sensor_type: int = GYRO) -> dict[int, list[dict]]:
    """Group the batch sampler's blocks by instance: {instance: [{t, rate_hz, xyz (n x 3, rad/s)}]}.

    Each block is announced by an ISBH header (its number N, sensor type, instance, scale and rate) and carried by
    ISBD messages of 32 int16 samples per axis, in order of seqno.
    """
    headers = {(int(h["N"])): h for h in isbh if int(h["type"]) == sensor_type}
    data: dict[int, list[tuple[int, list, list, list]]] = defaultdict(list)
    for d in isbd:
        n = int(d["N"])
        if n in headers:
            data[n].append((int(d["seqno"]), list(d["x"]), list(d["y"]), list(d["z"])))
    out: dict[int, list[dict]] = defaultdict(list)
    for n, h in headers.items():
        parts = sorted(data.get(n, []))
        if not parts:
            continue
        xyz = np.array([[v for p in parts for v in p[k]] for k in (1, 2, 3)], dtype=float).T / float(h["mul"])
        count = int(h["smp_cnt"])
        if len(xyz) < count:
            continue  # incomplete block (log cut off)
        out[int(h["instance"])].append({"t": float(h["t"]), "rate_hz": float(h["smp_rate"]), "xyz": xyz[:count]})
    return out


def average_spectrum(blocks: list[dict]) -> tuple[np.ndarray, np.ndarray] | None:
    """Mean amplitude spectrum (deg/s per bin, x+y+z combined) of equal-length blocks, Hann-windowed."""
    if not blocks:
        return None
    n = min(len(b["xyz"]) for b in blocks)
    rate = float(np.mean([b["rate_hz"] for b in blocks]))
    win = np.hanning(n)
    gain = win.sum() / 2.0  # amplitude correction: a sine of amplitude A shows as A
    acc = np.zeros(n // 2 + 1)
    for b in blocks:
        x = np.degrees(b["xyz"][:n]) - np.degrees(b["xyz"][:n]).mean(axis=0)
        amp = np.abs(np.fft.rfft(x * win[:, None], axis=0)) / gain
        acc += np.sqrt((amp**2).sum(axis=1))  # combine the three axes
    return np.fft.rfftfreq(n, 1.0 / rate), acc / len(blocks)


def find_peak(freqs: np.ndarray, amp: np.ndarray) -> dict | None:
    """The biggest peak above MIN_PEAK_HZ, if it stands PEAK_OVER_FLOOR_DB above the median noise floor."""
    sel = freqs >= MIN_PEAK_HZ
    if not sel.any():
        return None
    f, a = freqs[sel], amp[sel]
    i = int(np.argmax(a))
    floor = float(np.median(a))
    db = 20 * math.log10(max(a[i], 1e-9) / max(floor, 1e-9))
    if db < PEAK_OVER_FLOOR_DB:
        return None
    return {"freq_hz": round(float(f[i]), 1), "amp_deg_s": round(float(a[i]), 3), "above_floor_db": round(db, 1),
            "floor_deg_s": round(floor, 4)}


def to_db(amp: np.ndarray) -> np.ndarray:
    """Amplitude (deg/s) in dB relative to 1 deg/s."""
    return 20 * np.log10(np.maximum(amp, 1e-6))


def notch_recommendation(peak_hz: float, hover_throttle: float | None) -> dict[str, float]:
    """Throttle-based harmonic notch at the measured peak (see the module docstring)."""
    rec = {"INS_HNTCH_ENABLE": 1, "INS_HNTCH_MODE": 1, "INS_HNTCH_FREQ": round(peak_hz), "INS_HNTCH_BW": round(peak_hz / 2),
           "INS_HNTCH_ATT": 40, "INS_HNTCH_HMNCS": 1}
    if hover_throttle:
        rec["INS_HNTCH_REF"] = round(hover_throttle, 2)
    return rec


def analyze(L: dict[str, list[dict]], flying: callable | None = None) -> dict | None:
    """Spectrum before (and, if logged, after) the filters, the motor-noise peak and what to set.

    `flying(t) -> bool` picks the blocks taken in flight (on the ground the motors are idle and quiet).
    Returns None when the log has no gyro batch data.
    """
    by_inst = batches(L.get("ISBH", []), L.get("ISBD", []))
    if not by_inst:
        return None
    if flying is not None:
        by_inst = {k: [b for b in v if flying(b["t"])] for k, v in by_inst.items()}
        by_inst = {k: v for k, v in by_inst.items() if v}
        if not by_inst:
            return None
    params = {p["Name"]: float(p["Value"]) for p in L.get("PARM", [])}
    inst = sorted(by_inst)
    post_logged = int(params.get("INS_LOG_BAT_OPT", 0)) & 0b110 != 0
    pre = average_spectrum(by_inst[inst[0]])
    post = average_spectrum(by_inst[inst[-1]]) if post_logged and len(inst) > 1 else None
    if int(params.get("INS_LOG_BAT_OPT", 0)) & 0b010 and len(inst) == 1:
        pre, post = None, average_spectrum(by_inst[inst[0]])  # only post-filter data was logged
    hover = params.get("MOT_THST_HOVER")
    out: dict = {"blocks": len(by_inst[inst[0]]), "rate_hz": round(by_inst[inst[0]][0]["rate_hz"]),
                 "gyro_filter_hz": params.get("INS_GYRO_FILTER"), "notch_on": bool(params.get("INS_HNTCH_ENABLE", 0)),
                 "hover_throttle": round(hover, 3) if hover else None}
    ref = pre or post
    freqs = ref[0]
    idx = np.unique(np.linspace(0, len(freqs) - 1, min(len(freqs), MAX_POINTS)).astype(int))
    out["chart"] = {"labels": [], "data": [[round(float(f), 1) for f in freqs[idx]]], "x_label": "frequency (Hz)"}
    for name, spec in (("before filters", pre), ("after filters", post)):
        if spec is None:
            continue
        out["chart"]["labels"].append(f"{name} (dB)")
        out["chart"]["data"].append([round(float(v), 1) for v in to_db(spec[1])[idx]])
        key = "pre" if spec is pre else "post"
        out[f"{key}_peak"] = find_peak(*spec)
        out[f"{key}_rms_deg_s"] = round(float(np.sqrt(np.mean(spec[1][spec[0] >= MIN_PEAK_HZ] ** 2))), 4)
    peak = out.get("pre_peak") or out.get("post_peak")
    out["recommendation"] = notch_recommendation(peak["freq_hz"], hover) if peak and not out["notch_on"] else None
    return out


# ---- what the filters cost: delay (phase lag) at the frequencies the controller cares about ----------------------
# Standard biquads from the public "Audio EQ Cookbook" (R. Bristow-Johnson): a 2-pole low-pass (Q = 0.707, the
# shape of a gyro low-pass) and a notch. The notch's poles are set by its bandwidth (Q = centre / bandwidth); its
# zeros are pulled off the unit circle so the depth at the centre is exactly the attenuation setting, not infinite.
def _biquad_response(b: np.ndarray, a: np.ndarray, freqs_hz: np.ndarray, fs_hz: float) -> np.ndarray:
    z = np.exp(-1j * 2 * np.pi * np.asarray(freqs_hz) / fs_hz)
    return (b[0] + b[1] * z + b[2] * z * z) / (a[0] + a[1] * z + a[2] * z * z)


def lowpass_response(cutoff_hz: float, freqs_hz, fs_hz: float = 1000.0) -> np.ndarray:
    w = 2 * np.pi * cutoff_hz / fs_hz
    alpha = np.sin(w) / (2 * 0.7071)
    c = np.cos(w)
    b = np.array([(1 - c) / 2, 1 - c, (1 - c) / 2])
    a = np.array([1 + alpha, -2 * c, 1 - alpha])
    return _biquad_response(b, a, freqs_hz, fs_hz)


def notch_response(center_hz: float, bandwidth_hz: float, attenuation_db: float, freqs_hz, fs_hz: float = 1000.0):
    w = 2 * np.pi * center_hz / fs_hz
    q = center_hz / bandwidth_hz
    alpha = np.sin(w) / (2 * q)
    depth = 10 ** (-attenuation_db / 20.0)  # gain left at the centre frequency
    c = np.cos(w)
    b = np.array([1 + alpha * depth, -2 * c, 1 - alpha * depth])
    a = np.array([1 + alpha, -2 * c, 1 - alpha])
    return _biquad_response(b, a, freqs_hz, fs_hz)


def filter_cost(gyro_filter_hz: float, notch: dict | None, at_hz: float = 10.0, noise_hz: float | None = None) -> dict:
    """Phase lag (degrees and milliseconds) at `at_hz` (a typical rate-loop frequency), and how much is left of a
    vibration at `noise_hz` (fraction of its amplitude)."""
    h = lowpass_response(gyro_filter_hz, [at_hz, noise_hz or at_hz])
    if notch:
        h = h * notch_response(notch["INS_HNTCH_FREQ"], notch["INS_HNTCH_BW"], notch["INS_HNTCH_ATT"],
                               [at_hz, noise_hz or at_hz])
    lag_deg = -math.degrees(float(np.angle(h[0])))
    out = {"lag_deg": round(lag_deg, 1), "lag_ms": round(lag_deg / 360.0 / at_hz * 1000.0, 2), "at_hz": at_hz}
    if noise_hz:
        out["noise_left"] = round(float(abs(h[1])), 4)
    return out
