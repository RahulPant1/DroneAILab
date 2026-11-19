"""Rings race: ring passing, order, clock, best times, and every track flown end to end in the live sim."""

import numpy as np
import pytest

from backend.autonomy.live import MissionSim
from backend.games import race as race_mod
from backend.games.race import TRACKS, Race, Ring
from backend.sim.config import get_preset


@pytest.fixture(autouse=True)
def tmp_best_file(tmp_path, monkeypatch):
    """Never touch the real data/race_best.json in tests."""
    path = tmp_path / "race_best.json"
    monkeypatch.setattr(race_mod, "BEST_TIMES_FILE", path)
    return path


P = lambda *v: np.array(v, dtype=float)  # noqa: E731


def test_a_ring_counts_only_through_the_hoop_and_in_the_arrow_direction():
    east = Ring(3, 0, 2, heading_deg=0)
    assert east.passed(P(2.9, 0, 2), P(3.1, 0, 2))  # straight through the middle
    assert east.passed(P(2.9, 0.5, 2.5), P(3.1, 0.5, 2.5))  # off-centre but inside the 0.9 m radius
    assert not east.passed(P(3.1, 0, 2), P(2.9, 0, 2))  # backwards
    assert not east.passed(P(2.9, 1.2, 2), P(3.1, 1.2, 2))  # beside the ring
    assert not east.passed(P(2.9, 0, 3.0), P(3.1, 0, 3.0))  # over the top
    north = Ring(0, 5, 2, heading_deg=90)
    assert north.passed(P(0, 4.9, 2), P(0, 5.1, 2))
    assert not north.passed(P(-0.1, 5.2, 2), P(0.1, 5.2, 2))  # sideways, just past the ring's plane


def test_rings_count_in_order_and_the_clock_starts_at_take_off(tmp_best_file):
    r = Race("first_flight")
    assert r.update(P(2.9, 0, 2), P(3.1, 0, 2), 1.0, armed=False) == []  # not armed: no race yet
    assert r.update(P(0, 0, 0), P(0, 0, 0), 2.0, armed=True) == ["Go! The clock is running."]
    r.update(P(5.9, 0, 2), P(6.1, 0, 2), 3.0, armed=True)  # ring 2 first: doesn't count
    assert r.next_ring == 0
    r.update(P(2.9, 0, 2), P(3.1, 0, 2), 4.0, armed=True)
    r.update(P(5.9, 0, 2), P(6.1, 0, 2), 5.0, armed=True)
    msgs = r.update(P(8.9, 0, 2), P(9.1, 0, 2), 7.5, armed=True)
    assert r.finish_s == pytest.approx(5.5) and "New best" in msgs[0]
    assert race_mod.load_best_times(tmp_best_file) == {"first_flight": 5.5}
    slower = Race("first_flight")
    slower.update(P(0, 0, 0), P(0, 0, 0), 0.0, armed=True)
    for x in (3, 6):
        slower.update(P(x - 0.1, 0, 2), P(x + 0.1, 0, 2), 1.0, armed=True)
    msgs = slower.update(P(8.9, 0, 2), P(9.1, 0, 2), 9.0, armed=True)
    assert "best: 5.5 s" in msgs[0] and not slower.new_best
    assert race_mod.load_best_times(tmp_best_file) == {"first_flight": 5.5}  # a slower time doesn't replace it


def fly_track(rt: MissionSim, max_s: float = 120.0) -> list[str]:
    """A simple autopilot, flying like a person would: aim at each ring (a point just past its centre)."""
    notes: list[str] = []
    rt.command("takeoff", alt_m=2.0)
    for ring in rt.race.rings:
        goal = ring.centre + ring.normal * 0.5
        rt.command("target", x_m=float(goal[0]), y_m=float(goal[1]), alt_m=float(goal[2]))
        for _ in range(int(max_s / 0.05)):
            rt.advance(0.05)
            notes += rt.notices
            rt.notices.clear()
            if np.linalg.norm(rt.state.pos_m - goal) < 0.3:
                break
    return notes


@pytest.mark.parametrize("track", list(TRACKS))
def test_every_track_can_be_finished_in_the_live_sim(track):
    rt = MissionSim(get_preset("basic"))
    rt.load_race(track)
    assert list(rt.state.pos_m[:2]) == list(TRACKS[track]["start"])
    notes = fly_track(rt)
    status = rt.snapshot()["race"]
    assert status["finished"], notes
    assert status["next_ring"] == status["total"] == len(TRACKS[track]["rings"])
    assert 3.0 < status["time_s"] < 60.0
    assert any(n.startswith("🏁 Finished") for n in notes)


def test_reset_restarts_the_race_and_a_course_replaces_it():
    rt = MissionSim(get_preset("basic"))
    rt.load_race("first_flight")
    fly_track(rt)
    assert rt.snapshot()["race"]["finished"]
    rt.reset()
    st = rt.snapshot()["race"]
    assert st["next_ring"] == 0 and not st["running"] and st["time_s"] == 0
    assert st["best_s"] is not None  # the best time survives
    rt.load_course("patrol")
    assert rt.race is None and rt.snapshot()["race"] is None
    rt.load_race("zig_zag")
    assert rt.mission is None and rt.course_info() is None
    with pytest.raises(ValueError):
        rt.load_race("moon")


def test_race_over_the_websocket():
    from fastapi.testclient import TestClient

    from backend import api

    client = TestClient(api.app)
    tracks = client.get("/api/race/tracks").json()["tracks"]
    assert set(tracks) == set(TRACKS) and tracks["first_flight"]["rings"] == 3
    with client.websocket_connect("/ws/sim") as ws:
        ws.receive_json()  # Connected.
        ws.send_json({"cmd": "race", "track": "up_and_down"})
        for _ in range(100):
            msg = ws.receive_json()
            if msg.get("type") == "course":
                break
        assert msg["race"]["id"] == "up_and_down" and len(msg["race"]["rings"]) == 4 and msg["course"] is None
        assert "R (up)" in msg["message"]
        state = ws.receive_json()
        while state.get("type") != "state":
            state = ws.receive_json()
        assert state["race"]["total"] == 4
