"""🎙 Voice commands: the quick phrases, the AI path (fake Ollama), the safety rules, the WebSocket and the API.
Never loads Whisper or the real model: speech-to-text is tested only for 'not installed / no audio' behaviour."""

import asyncio
import json

import httpx
import pytest
from fastapi.testclient import TestClient

from backend import api
from backend.ai import assistant
from backend.ai.ollama_client import OllamaClient
from backend.autonomy.live import MissionSim
from backend.sim.config import get_preset
from backend.voice import commands as V


@pytest.fixture(autouse=True)
def _tmp_log(tmp_path, monkeypatch):
    monkeypatch.setattr(assistant, "LOG_PATH", tmp_path / "log.jsonl")


def fake_client(reply: dict | str, seen: list | None = None) -> OllamaClient:
    def handler(request: httpx.Request) -> httpx.Response:
        body = json.loads(request.content)
        if seen is not None:
            seen.append(body)
        content = reply if isinstance(reply, str) else json.dumps(reply)
        return httpx.Response(200, json={"message": {"content": content}})

    return OllamaClient(url="http://fake", model="gemma4:test", transport=httpx.MockTransport(handler))


def say(rt: MissionSim, text: str, client: OllamaClient | None = None, steps: int = 0) -> dict:
    out = asyncio.run(V.handle(rt, text, lambda m: api.handle_message(rt, m), lambda: client or fake_client({}), lambda: False))
    for _ in range(steps):
        rt.advance(0.05)
    return out[-1]


@pytest.mark.parametrize("text,expected", [
    ("Take off!", [{"action": "takeoff"}]),
    ("take off to five metres", [{"action": "takeoff", "height_m": 5.0}]),
    ("go left three metres", [{"action": "move", "direction": "left", "metres": 3.0}]),
    ("please go up", [{"action": "move", "direction": "up", "metres": 1.0}]),
    ("fly forwards 2.5 m", [{"action": "move", "direction": "forward", "metres": 2.5}]),
    ("turn right", [{"action": "turn", "direction": "right", "degrees": 90.0}]),
    ("go forward 4 m and then turn around", [{"action": "move", "direction": "forward", "metres": 4.0},
                                              {"action": "turn", "direction": "left", "degrees": 180.0}]),
    ("come back home", [{"action": "go_home"}]),
    ("hover", [{"action": "hover"}]),
    ("land now", [{"action": "land"}]),
    ("lidar on", [{"action": "lidar", "on": True}]),
    ("race the figure eight", None),  # not a fixed phrase: the AI's job
    ("Go left, left.", [{"action": "move", "direction": "left", "metres": 2.0}]),  # what Whisper really heard (2026-11-28)
    ("left", [{"action": "move", "direction": "left", "metres": 2.0}]),
])
def test_quick_phrases(text, expected):
    assert V.quick_parse(text) == expected


def test_quick_commands_fly_the_drone_without_any_ai():
    rt = MissionSim(get_preset("basic"))
    seen: list = []
    client = fake_client({}, seen)
    assert "Take off" in say(rt, "take off", client, steps=100)["say"]
    x0 = float(rt.state.pos_m[0])
    say(rt, "go forward 3 metres", client, steps=120)
    assert rt.state.pos_m[0] - x0 == pytest.approx(3.0, abs=0.4)  # yaw 0 = nose east
    say(rt, "turn left", client, steps=60)
    say(rt, "go forward 2 m", client, steps=120)  # now facing north
    assert rt.state.pos_m[1] == pytest.approx(2.0, abs=0.4)
    assert seen == []  # the model was never asked


def test_stop_always_wins_and_cancels_whatever_was_waiting():
    rt = MissionSim(get_preset("basic"))
    say(rt, "take off", steps=80)
    client = fake_client({"understood": "Loading the slalom.", "actions": [{"action": "course", "course": "slalom"}], "not_possible": []})
    assert say(rt, "load the slalom please", client)["pending"]  # flying: moving to another world waits for go
    out = say(rt, "no wait STOP the drone")
    assert out["say"].startswith("Stopping") and rt.pilot.mode == "hover" and rt.voice_pending is None
    assert "Nothing is waiting" in say(rt, "go")["say"]


def test_ai_actions_wait_for_go_and_are_checked_in_order():
    rt = MissionSim(get_preset("basic"))
    client = fake_client({"understood": "I'll race the figure eight on autopilot.",
                          "actions": [{"action": "race", "track": "figure_eight"},
                                      {"action": "race_autopilot", "method": "geometric"}], "not_possible": []})
    out = say(rt, "race the figure eight with the autopilot", client)
    assert out["pending"] and out["source"] == "ai" and len(out["actions"]) == 2  # autopilot checked AFTER the race loads
    assert rt.race is None  # nothing happened yet
    out = say(rt, "go!", steps=20)
    assert out["say"].startswith("Going") and rt.race is not None and rt.agile_phase is not None


def test_names_and_numbers_from_the_model_are_checked_by_code():
    rt = MissionSim(get_preset("basic"))
    bad_track = fake_client({"understood": "x", "actions": [{"action": "race", "track": "moon_track"}], "not_possible": []})
    assert "I know these race tracks" in say(rt, "race on the moon", bad_track)["say"]
    say(rt, "take off", steps=80)
    too_far = fake_client({"understood": "x", "actions": [{"action": "move", "direction": "left", "metres": 500}], "not_possible": []})
    assert "between" in say(rt, "go left a lot", too_far)["say"]
    needs_race = fake_client({"understood": "x", "actions": [{"action": "race_autopilot"}], "not_possible": []})
    assert "Load a race track first" in say(rt, "autopilot please", needs_race)["say"]
    nothing = fake_client({"understood": "", "actions": [], "not_possible": ["do a flip"]})
    assert "can't do that (do a flip)" in say(rt, "do a flip", nothing)["say"]


def test_word_rules_fix_what_a_small_model_gets_wrong_for_exploration():
    rt = MissionSim(get_preset("basic"))
    wrong = fake_client({"understood": "Exploring with dead reckoning.",
                         "actions": [{"action": "explore", "world": "rooms", "localisation": "dead_reckoning"}], "not_possible": []})
    out = say(rt, "explore the forest without GPS with someone walking around", wrong)
    assert out["pending"] and "forest" in out["say"] and "SLAM" in out["say"] and "walking" in out["say"]
    assert "dead reckoning" not in out["say"]  # its own (wrong) sentence was dropped
    say(rt, "go", steps=5)
    assert rt.explore_args == {"world": "forest", "local": "dwa", "localisation": "slam", "movers": True}


def test_the_prompt_lists_only_what_exists_and_the_schema_uses_enums():
    ctx = V.context(MissionSim(get_preset("basic")))
    p = V.system_prompt(ctx)
    assert "figure_eight" in p and "box_canyon" in p and "rooms" in p and "empty playground" in p
    props = V.json_schema()["properties"]["actions"]["items"]["properties"]
    assert "figure_eight" in props["track"]["enum"] and set(props["action"]["enum"]) == set(V.ACTIONS)


def test_ai_offline_still_leaves_simple_commands():
    rt = MissionSim(get_preset("basic"))

    def down(request):
        raise httpx.ConnectError("no ollama")

    client = OllamaClient(url="http://fake", model="m", transport=httpx.MockTransport(down))
    out = say(rt, "race the figure eight", client)
    assert "offline" in out["say"] and "take off" in out["say"]


def test_websocket_voice_command(monkeypatch):
    monkeypatch.setattr(api, "get_ai_client", lambda: fake_client({}))
    with TestClient(api.app).websocket_connect("/ws/sim") as ws:
        ws.send_json({"cmd": "voice", "text": "take off"})
        for _ in range(200):
            msg = ws.receive_json()
            if msg.get("type") == "voice":
                break
        assert msg["say"].startswith("Take off") and msg["source"] == "quick"


def test_transcribe_endpoint_rejects_empty_audio():
    r = TestClient(api.app).post("/api/voice/transcribe", content=b"")
    assert r.status_code in (400, 503)  # no audio (or voice not installed): a clear error, never a crash
    s = TestClient(api.app).get("/api/voice/status").json()
    assert set(s) == {"available", "detail", "loaded"}


def test_go_left_is_a_move_even_if_the_model_says_turn_and_repeats_are_one_action():
    """The real bug: Whisper heard 'Go left, left.' and the model answered 'turn left' twice (a 180° spin on the spot)."""
    rt = MissionSim(get_preset("basic"))
    say(rt, "take off", steps=80)
    model = fake_client({"understood": "I will turn the drone left twice.",
                         "actions": [{"action": "turn", "direction": "left", "degrees": 90},
                                     {"action": "turn", "direction": "left", "degrees": 90}], "not_possible": []})
    y0 = float(rt.state.pos_m[1])
    out = say(rt, "please go to the left, to the left", model, steps=120)  # not a fixed phrase: the AI path
    assert out["actions"] == ["go left 2 m"] and "turn" not in out["say"]
    assert rt.state.pos_m[1] - y0 == pytest.approx(2.0, abs=0.4)  # slid left (north, nose east)
    assert V.apply_hints({"action": "turn", "direction": "left"}, "turn left") == {"action": "turn", "direction": "left"}


def test_warmup_endpoint_reads_the_instructions_once(monkeypatch):
    seen: list = []
    monkeypatch.setattr(api, "get_ai_client", lambda: fake_client({"understood": "", "actions": [], "not_possible": []}, seen))
    monkeypatch.setattr(api, "_warm_stt", lambda: None)
    with TestClient(api.app) as client:
        assert client.post("/api/voice/warmup").status_code == 200
        for _ in range(50):
            if seen:
                break
            import time

            time.sleep(0.02)
    assert seen and seen[0]["messages"][0]["content"].rstrip().endswith("wingmen")  # what changes is LAST in the prompt
