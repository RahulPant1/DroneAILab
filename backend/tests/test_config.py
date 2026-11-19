"""Config: presets, derived numbers, validation, JSON round trip."""

import json

import pytest

from backend.sim.config import PRESETS, ConfigError, DroneConfig, get_preset, require_valid, validate_config


def test_presets_are_valid_and_have_the_promised_thrust_to_weight():
    for cfg in PRESETS.values():
        assert validate_config(cfg) == []
    assert get_preset("basic").thrust_to_weight == pytest.approx(2.0)
    assert get_preset("heavy").thrust_to_weight == pytest.approx(1.3)


def test_hover_throttle_is_100_over_tw():
    assert get_preset("basic").hover_throttle_pct == pytest.approx(50.0)


def test_out_of_range_values_are_rejected_with_a_readable_reason():
    bad = DroneConfig().with_changes(mass_kg=9.0, gains={"alt_kp": -1.0})
    problems = validate_config(bad)
    assert any("mass_kg" in p and "0.2–5.0" in p for p in problems)
    assert any("gains.alt_kp" in p for p in problems)
    with pytest.raises(ConfigError):
        require_valid(bad)


def test_nan_is_rejected():
    assert validate_config(DroneConfig().with_changes(mass_kg=float("nan")))


def test_json_round_trip_and_unknown_fields():
    cfg = get_preset("lightweight")
    again = DroneConfig.from_dict(json.loads(json.dumps(cfg.to_dict())))
    assert again == cfg
    with pytest.raises(ConfigError):
        DroneConfig.from_dict({"mass_kg": 1.0, "warp_drive": True})
