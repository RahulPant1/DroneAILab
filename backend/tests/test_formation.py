"""Multi-drone formation: slots, following the leader, avoidance (between drones and around obstacles), crashes."""

import math

import numpy as np
import pytest

from backend.autonomy.formation import MAX_WINGMEN, slot_offsets, slot_world
from backend.autonomy.live import MissionSim
from backend.sim.config import get_preset


def flying(n=4, shape="v", spacing=1.5, avoid=True, settle_s=5.0) -> MissionSim:
    rt = MissionSim(get_preset("basic"))
    rt.command("formation", count=n, shape=shape, spacing_m=spacing, avoid=avoid)
    rt.command("takeoff")
    run(rt, settle_s)
    return rt


def run(rt: MissionSim, seconds: float) -> list[str]:
    notes: list[str] = []
    for _ in range(int(seconds / 0.05)):
        rt.advance(0.05)
        notes += rt.notices
        rt.notices.clear()
    return notes


def test_slots_for_each_shape_alternate_left_and_right():
    assert slot_offsets("v", 4, 1.5) == [(-1.5, 1.5), (-1.5, -1.5), (-3.0, 3.0), (-3.0, -3.0)]
    assert slot_offsets("line", 2, 2.0) == [(0.0, 2.0), (0.0, -2.0)]
    assert slot_offsets("column", 3, 1.5) == [(-1.5, 0.0), (-3.0, 0.0), (-4.5, 0.0)]
    # A slot "1.5 m behind" a leader facing north is 1.5 m south of it.
    assert np.allclose(slot_world(np.array([0.0, 0.0]), math.pi / 2, (-1.5, 0.0)), [0.0, -1.5])


def test_wingmen_take_off_hold_their_slots_turn_with_the_leader_and_land_with_it():
    rt = flying(n=4, shape="v")
    snap = rt.snapshot()["formation"]
    expected = [(-1.5, 1.5), (-1.5, -1.5), (-3.0, 3.0), (-3.0, -3.0)]
    for w, (x, y) in zip(snap["wingmen"], expected):
        assert w["pos_m"][0] == pytest.approx(x, abs=0.1) and w["pos_m"][1] == pytest.approx(y, abs=0.1)
        assert w["pos_m"][2] == pytest.approx(2.0, abs=0.1)
    assert snap["min_gap_m"] == pytest.approx(math.hypot(1.5, 1.5), abs=0.1)
    rt.command("sticks", yaw=1.0)  # Q: turn left 90° (60°/s)
    run(rt, 1.5)
    rt.command("sticks")
    run(rt, 4.0)
    w1 = rt.formation.wingmen[0].state.pos_m  # behind-left of a north-facing leader = south-west of it
    assert w1[0] == pytest.approx(-1.5, abs=0.2) and w1[1] == pytest.approx(-1.5, abs=0.2)
    rt.command("land")
    run(rt, 8.0)
    assert all(w.state.on_ground and not w.pilot.sp.armed for w in rt.formation.wingmen)


@pytest.mark.parametrize("avoid", [False, True])
def test_backing_into_your_own_column_crashes_without_avoidance_and_not_with_it(avoid):
    rt = flying(n=4, shape="column", avoid=avoid)
    rt.command("sticks", forward=-1.0)  # S: fly backwards into the wingmen behind you
    notes = run(rt, 4.0)
    crashes = [n for n in notes if "💥" in n]
    if avoid:
        assert crashes == [] and rt.pilot.mode == "hover"
    else:
        assert "Your drone and wingman 1 collided" in crashes[0]
        assert rt.pilot.mode == "idle" and not rt.pilot.sp.armed  # your motors stopped too
        assert rt.formation.wingmen[0].crashed


def test_on_a_mission_wingmen_break_formation_to_get_past_obstacles():
    rt = MissionSim(get_preset("basic"))
    rt.load_course("slalom")
    rt.command("formation", count=2, shape="v", spacing_m=1.5, avoid=True)
    rt.start_mission("astar")
    notes, broke = [], False
    for _ in range(int(90 / 0.05)):
        rt.advance(0.05)
        notes += rt.notices
        rt.notices.clear()
        broke |= any(w.breaking for w in rt.formation.wingmen)
        if not rt.follower:
            break
    assert "complete" in rt.pilot.message.lower()
    assert broke  # they left their slots to follow the leader's path through the slalom
    assert [n for n in notes if "💥" in n] == []


def test_formation_settings_validation_wind_and_changing_the_number_mid_flight():
    rt = flying(n=2, shape="line")
    with pytest.raises(ValueError):
        rt.command("formation", count=MAX_WINGMEN + 1)
    with pytest.raises(ValueError):
        rt.command("formation", count=2, shape="star")
    with pytest.raises(ValueError):
        rt.command("formation", count=2, spacing_m=0.5)
    first = rt.formation.wingmen[0]
    rt.command("formation", count=3, shape="line", spacing_m=1.5, avoid=True)
    assert rt.formation.wingmen[0] is first  # existing wingmen keep flying; the new one starts on the ground
    assert rt.formation.wingmen[2].state.on_ground
    run(rt, 5.0)
    assert rt.formation.wingmen[2].state.pos_m[2] == pytest.approx(2.0, abs=0.15)
    rt.command("wind", speed_m_s=5.0, direction_deg=90, gust_m_s=0.0)
    assert all(w.sim.env.wind_speed_m_s == 5.0 for w in rt.formation.wingmen)
    assert "Formation off" in rt.command("formation", count=0)
    assert rt.snapshot()["formation"] is None


def test_formation_over_the_websocket():
    from fastapi.testclient import TestClient

    from backend import api

    with TestClient(api.app).websocket_connect("/ws/sim") as ws:
        ws.receive_json()  # Connected.
        ws.send_json({"cmd": "formation", "count": 3, "shape": "column", "spacing_m": 2.0, "avoid": False})
        msg = ws.receive_json()
        while msg.get("type") == "state":
            msg = ws.receive_json()
        assert "3 wingmen in a column" in msg["message"] and "NO collision avoidance" in msg["message"]
        state = ws.receive_json()
        while state.get("type") != "state":
            state = ws.receive_json()
        f = state["formation"]
        assert f["count"] == 3 and len(f["wingmen"]) == 3 and f["avoid"] is False
