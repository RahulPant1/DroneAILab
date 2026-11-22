"""🔋 Battery and 🛠 Drone Builder: the LiPo model, the failsafe, and builds whose numbers match a real simulated flight."""

import itertools

import pytest

from backend.autonomy.live import MissionSim
from backend.sim import builder
from backend.sim.battery import Battery, BatteryConfig, cell_ocv
from backend.sim.config import get_preset
from backend.sim.control import Setpoint
from backend.sim.physics import State
from backend.sim.runner import PHYSICS_DT_S, LightweightSim


def hover_sim(cfg, battery, alt=3.0):
    sim = LightweightSim()
    sim.reset(cfg, 0, State.hovering(cfg, alt), battery=battery)
    return sim, Setpoint(alt_m=alt, position_control=True)


def run(sim, sp, seconds):
    for _ in range(int(seconds / PHYSICS_DT_S)):
        sim.step(sp)


# ---- battery ----

def test_lipo_voltage_follows_charge_and_sags_under_load():
    assert cell_ocv(1.0) == pytest.approx(4.2) and cell_ocv(0.0) == pytest.approx(3.27)
    assert all(cell_ocv(a) < cell_ocv(b) for a, b in zip([0, 0.1, 0.3, 0.5, 0.8], [0.1, 0.3, 0.5, 0.8, 1.0]))
    light, heavy = Battery(BatteryConfig(cells=4)), Battery(BatteryConfig(cells=4))
    light.step(50.0, 0.01)
    heavy.step(800.0, 0.01)
    assert heavy.voltage_v < light.voltage_v < 4 * 4.2  # more current -> more sag
    b = Battery(BatteryConfig(cells=3, capacity_mah=3000, avionics_w=0.0, efficiency=1.0))
    for _ in range(3600):
        b.step(10.0 * 12.0, 0.1)  # ~10 A for 6 minutes = ~1000 mAh
    assert b.used_mah == pytest.approx(1000, rel=0.1) and b.soc == pytest.approx(2 / 3, abs=0.05)


def test_no_battery_means_the_old_unlimited_physics():
    sim, _ = hover_sim(get_preset("basic"), None)
    assert sim.battery is None and sim.physics.voltage_scale == 1.0
    rt = MissionSim(get_preset("basic"))
    assert rt.snapshot()["battery"] is None


def test_a_draining_battery_costs_thrust_until_it_can_no_longer_fly():
    cfg = get_preset("basic")
    sim, sp = hover_sim(cfg, BatteryConfig(cells=3, capacity_mah=3000, drain_speedup=10))
    run(sim, sp, 5)
    early_motor, early_v = sim.motor_pct.mean(), sim.battery.voltage_v
    run(sim, sp, 50)
    late_motor, late_v = sim.motor_pct.mean(), sim.battery.voltage_v
    assert late_v < early_v - 1.0 and late_motor > early_motor + 5  # lower voltage: works the motors harder
    run(sim, sp, 40)
    assert sim.battery.status == "empty" and sim.state.pos_m[2] < 0.5  # empty: it comes down


def test_live_low_battery_failsafe_flies_home_and_lands():
    rt = MissionSim(get_preset("basic"))
    assert "Battery on" in rt.command("battery", enabled=True, cells=3, capacity_mah=3000, drain_speedup=10)
    rt.command("takeoff", alt_m=3)
    notes = []
    for i in range(int(200 / 0.05)):
        rt.advance(0.05)
        notes += rt.notices
        rt.notices.clear()
        if i == int(4 / 0.05):
            rt.command("target", x_m=8, y_m=0)
        if rt.state.on_ground and rt.snapshot()["battery"]["failsafe"]:
            break
    assert any("Battery low" in n and "flying home" in n for n in notes)
    b = rt.snapshot()["battery"]
    assert abs(rt.state.pos_m[0]) < 1.0 and abs(rt.state.pos_m[1]) < 1.0  # back home
    assert b["soc_pct"] > 5 and b["status"] != "empty"
    with pytest.raises(ValueError):
        rt.command("battery", enabled=True, drain_speedup=50)  # too fast to make it home: refused


# ---- builder ----

def every_compatible_design():
    for (fk, f), (mk, m), (pk, p) in itertools.product(builder.FRAMES.items(), builder.MOTORS.items(), builder.PROPS.items()):
        if p.diameter_in <= f.max_prop_in and p.diameter_in in m.thrust_g:
            for cells in builder.CELL_OPTIONS:
                if cells <= m.max_cells:
                    for cap in (450, 3000, 10000):
                        yield {"frame": fk, "motor": mk, "prop": pk, "cells": cells, "capacity_mah": cap, "payload_kg": 0.0}


def test_every_compatible_combination_gives_a_valid_report():
    n = 0
    for parts in every_compatible_design():
        d = builder.design(parts)
        r = d["report"]
        assert r["mass_g"] > 0 and r["thrust_to_weight"] > 0
        assert (r["verdict"] == "won't fly") == (r["thrust_to_weight"] < 1.0)
        n += 1
    assert n > 20


def test_impossible_combinations_are_refused_with_a_reason():
    with pytest.raises(builder.DesignError, match="don't fit"):
        builder.design({"frame": "racer_250", "prop": "10in"})
    with pytest.raises(builder.DesignError, match="burn it out"):
        builder.design({"motor": "2212_920kv", "cells": 6})
    with pytest.raises(builder.DesignError, match="made for"):
        builder.design({"motor": "2306_2450kv", "prop": "10in"})
    weak = builder.design({"cells": 2, "capacity_mah": 10000, "payload_kg": 1.0})  # ~2.2 kg on ~1.5 kg of thrust
    assert weak["report"]["verdict"] == "won't fly" and weak["report"]["flight_time_min"] is None


def test_classic_450_numbers_are_realistic():
    r = builder.design({})["report"]
    assert 800 <= r["mass_g"] <= 1100
    assert 2.5 <= r["thrust_to_weight"] <= 4.5
    assert 12 <= r["flight_time_min"] <= 22  # an F450 on 3S 3000 mAh hovers roughly 12-18 min
    heavier = builder.design({"payload_kg": 0.5})["report"]
    assert heavier["flight_time_min"] < r["flight_time_min"] and heavier["thrust_to_weight"] < r["thrust_to_weight"]


@pytest.mark.parametrize("parts", [
    {},
    {"frame": "racer_250", "motor": "2306_2450kv", "prop": "5in", "cells": 4, "capacity_mah": 1300},
    {"frame": "big_650", "motor": "3510_360kv", "prop": "15in", "cells": 6, "capacity_mah": 10000, "payload_kg": 1.0},
])
def test_the_simulator_burns_the_estimated_hover_power_and_flies_the_design(parts):
    d = builder.design(parts)
    sim, sp = hover_sim(d["config"], d["battery"])
    run(sim, sp, 6)
    assert sim.battery.power_w == pytest.approx(d["report"]["hover_power_w"], rel=0.03)
    go = Setpoint(alt_m=3.0, position_control=True, x_m=5.0)
    run(sim, go, 8)
    assert sim.state.pos_m[0] == pytest.approx(5.0, abs=0.2) and sim.state.pos_m[2] == pytest.approx(3.0, abs=0.2)


def test_estimated_flight_time_matches_a_simulated_hover_to_the_low_battery_point():
    d = builder.design({})
    speed = 10.0
    sim, sp = hover_sim(d["config"], BatteryConfig(**{**d["battery"].__dict__, "drain_speedup": speed}))
    t = 0.0
    while sim.battery.status == "ok" and t < 200:
        run(sim, sp, 1.0)
        t += 1.0
    simulated_min = t * speed / 60.0
    assert simulated_min == pytest.approx(d["report"]["flight_time_min"], rel=0.2)


def test_flying_a_design_over_the_websocket_and_presets_switch_the_battery_off():
    from fastapi.testclient import TestClient

    from backend import api

    with TestClient(api.app).websocket_connect("/ws/sim") as ws:
        ws.receive_json()

        def reply():
            m = ws.receive_json()
            while m.get("type") == "state":
                m = ws.receive_json()
            return m

        def state():
            m = ws.receive_json()
            while m.get("type") != "state":
                m = ws.receive_json()
            return m

        ws.send_json({"cmd": "design", "parts": {"cells": 4}, "drain_speedup": 5})
        m = reply()
        assert m["type"] == "config" and "My build" in m["config"]["name"] and "take off" in m["message"]
        assert state()["battery"]["cells"] == 4
        ws.send_json({"cmd": "reset", "preset": "basic"})
        assert "Battery off" in reply()["message"]
        assert state()["battery"] is None
