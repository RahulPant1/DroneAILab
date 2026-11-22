"""Have the local LLM explain a log analysis like a flight-test engineer debriefing a learner.

The model sees only the compact analysis (overview, the checks with their measured values, the timeline). Never the
raw log, never charts. Same safety net as the DroneLab assistant: [Measured] vs [Hypothesis] labels, every number in
the answer is checked against the data, and every exchange is written to data/assistant_log.jsonl.
"""

from __future__ import annotations

import json
import time

from backend.ai.assistant import log_exchange, tidy_answer, unverified_numbers
from backend.ai.ollama_client import OllamaClient

MAX_TIMELINE_FOR_MODEL = 25

SYSTEM_PROMPT = """You are a drone flight-test engineer reviewing an ArduPilot (ArduCopter) flight log with a learner.

You get LOG ANALYSIS: JSON with an overview, rule-based checks (status ok/warn/bad/info, measured value, limit, meaning)
and a timeline of flight modes, events, errors and autopilot messages (t = seconds from the start of the log).

Write a short debrief with exactly these three headings, each followed by 1-4 bullets:
What happened:
Most likely cause:
What to check next:

Rules:
1. Start every bullet with [Measured] (directly from LOG ANALYSIS; quote the number and unit) or [Hypothesis] (your reasoning).
2. NEVER invent numbers. Use only numbers that appear in LOG ANALYSIS.
3. Follow the timeline: say which failsafe or mode change happened and when, if any.
4. If every check is ok and nothing unusual is in the timeline, say the flight looks healthy and suggest one thing worth watching.
5. If the overview says simulated: true, remember faults may have been injected on purpose in a simulator.
6. Plain text, at most about 180 words, no greeting, no LaTeX."""


def compact(analysis: dict) -> dict:
    """What the model sees: no chart data, a trimmed timeline."""
    return {
        "overview": analysis["overview"],
        "overall": analysis["verdict"],
        "checks": [{k: c[k] for k in ("title", "status", "value", "limit")} for c in analysis["checks"]],
        "timeline": analysis["timeline"][:MAX_TIMELINE_FOR_MODEL],
    }


async def explain(client: OllamaClient, analysis: dict, question: str = "") -> dict:
    data = json.dumps(compact(analysis), separators=(",", ":"), ensure_ascii=False)  # keep ° and × readable
    user = f"LOG ANALYSIS:\n{data}" + (f"\n\nTHE LEARNER ASKS: {question[:500]}" if question else "")
    messages = [{"role": "system", "content": SYSTEM_PROMPT}, {"role": "user", "content": user}]
    entry = {"kind": "log_explain", "model": client.model, "log": analysis["overview"].get("file"), "messages": messages}
    start = time.monotonic()
    try:
        answer = tidy_answer(await client.chat(messages))
    except Exception as exc:
        log_exchange({**entry, "ok": False, "error": str(exc)})
        raise
    duration = time.monotonic() - start
    flagged = unverified_numbers(answer, [data, question])
    log_exchange({**entry, "ok": True, "reply": answer, "duration_s": round(duration, 2), "unverified_numbers": flagged})
    return {"answer": answer, "model": client.model, "unverified_numbers": flagged, "duration_s": round(duration, 1),
            "context_chars": len(data)}
