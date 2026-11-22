"""🧭 AI mission planner, tested against a FAKE Ollama (no model is loaded by these tests)."""

import asyncio
import json

import httpx
import pytest
from fastapi.testclient import TestClient

from backend import api
from backend.ai import assistant
from backend.ai.mission_planner import interpret, plan_from_text, plan_json_schema
from backend.ai.ollama_client import OllamaClient
from backend.autonomy.plan import PlanError
from backend.autonomy.site import FARM


def fake(reply: str, seen: list | None = None) -> OllamaClient:
    def handler(request: httpx.Request) -> httpx.Response:
        if seen is not None:
            seen.append(json.loads(request.content))
        return httpx.Response(200, json={"message": {"role": "assistant", "content": reply}, "done": True})

    return OllamaClient(url="http://fake", model="gemma4:test", transport=httpx.MockTransport(handler))


@pytest.fixture(autouse=True)
def tmp_log(tmp_path, monkeypatch):
    path = tmp_path / "assistant_log.jsonl"
    monkeypatch.setattr(assistant, "LOG_PATH", path)
    return path


def reply(**kw) -> str:
    base = {"understood": "Survey the north field at 25 m, then land on pad B.", "altitude_m": 25, "speed_m_s": 3,
            "tasks": [{"type": "survey", "field": "north field"}, {"type": "land", "place": "pad B", "precision": True}],
            "not_on_map": [], "left_out": [], "assumptions": []}
    return json.dumps({**base, **kw})


def test_the_schema_only_allows_names_on_the_map():
    s = plan_json_schema(FARM)
    item = s["properties"]["tasks"]["items"]["properties"]
    assert set(item["field"]["enum"]) == {"north field", "east field", "orchard"}
    assert "pad B" in item["place"]["enum"] and "neighbour's garden" not in item["place"]["enum"]


def test_a_good_reply_becomes_a_checked_flyable_plan(tmp_log):
    seen = []
    out = asyncio.run(plan_from_text(fake(reply(), seen), "survey the north field at 25 m and land on pad B"))
    assert out["ok"] and out["plan"]["ok"] and out["spec"]["altitude_m"] == 25
    assert out["plan"]["land"]["place"] == "pad B" and out["plan"]["surveys"][0]["field"] == "north field"
    assert out["unverified_numbers"] == []
    body = seen[0]
    assert body["format"]["properties"]["tasks"] and body["think"] is False
    assert "NO-FLY ZONE" in body["messages"][0]["content"]
    log = [json.loads(line) for line in tmp_log.read_text().splitlines()]
    assert log[-1]["kind"] == "mission_plan" and log[-1]["flyable"] is True


def test_unsafe_requests_are_planned_faithfully_and_then_fail_the_checks():
    out = interpret(reply(altitude_m=150, understood="Survey the north field at 150 m."), "survey the north field at 150 m")
    assert not out["plan"]["ok"]
    assert next(c for c in out["plan"]["checks"] if c["id"] == "altitude")["status"] == "fail"


def test_places_not_on_the_map_block_the_plan_and_invented_numbers_are_flagged():
    out = interpret(reply(not_on_map=["the school"], understood="Fly to the school 731 m away."), "fly to the school")
    assert not out["plan"]["ok"] and out["plan"]["checks"][0]["id"] == "names"
    assert "731" in out["unverified_numbers"]
    left = interpret(reply(left_out=["neighbour's garden: it's a no-fly zone"]), "survey the garden and the north field")
    assert left["plan"]["checks"][0]["status"] == "warn"


def test_bad_replies_are_rejected_in_plain_english():
    with pytest.raises(PlanError, match="valid JSON"):
        interpret("not json", "x")
    with pytest.raises(PlanError, match="didn't plan any tasks"):
        interpret(reply(tasks=[]), "x")
    with pytest.raises(PlanError, match="no place called"):
        interpret(reply(tasks=[{"type": "goto", "place": "Mars"}]), "x")
    out = asyncio.run(plan_from_text(fake("{oops"), "survey"))
    assert out["ok"] is False and "JSON" in out["error"]
    fixed = interpret(reply(tasks=[{"type": "survey", "place": "orchard"}]), "survey the orchard")  # common slip
    assert fixed["plan"]["surveys"][0]["field"] == "orchard"


def test_api_plan_endpoints(monkeypatch):
    client = TestClient(api.app)
    assert {f["name"] for f in client.get("/api/site").json()["fields"]} == {"north field", "east field", "orchard"}
    r = client.post("/api/plan/compile", json={"spec": {"tasks": [{"type": "survey", "field": "orchard"}]}})
    assert r.status_code == 200 and r.json()["ok"]
    assert client.post("/api/plan/compile", json={"spec": {"tasks": [{"type": "goto", "place": "Mars"}]}}).status_code == 400
    monkeypatch.setattr(api, "get_ai_client", lambda: fake(reply()))
    r = client.post("/api/ai/plan", json={"request": "survey the north field at 25 m, land on pad B",
                                          "battery": {"cells": 4, "capacity_mah": 1500}})
    body = r.json()
    assert r.status_code == 200 and body["ok"] and "4S 1500 mAh" in next(c for c in body["plan"]["checks"] if c["id"] == "battery")["detail"]
    with client.websocket_connect("/ws/sim") as ws:
        ws.receive_json()
        ws.send_json({"cmd": "plan", "spec": body["spec"]})
        m = ws.receive_json()
        while m.get("type") == "state":
            m = ws.receive_json()
        assert m["type"] == "course" and m["course"]["id"] == "farm" and m["course"]["plan"]["ok"]
        assert "Plan started" in m["message"]
