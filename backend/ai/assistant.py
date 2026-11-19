"""The assistant's two jobs: answer a question about runs, and suggest a next experiment.

Safety net around the model:
  - It only sees a compact summary of real simulator runs (built here, server-side).
  - Every number in an answer is checked against that summary; unknown numbers are flagged to the user.
  - Suggestions are validated (schemas.py) and never run by themselves: the user must click Apply.
  - Every prompt and reply is appended to data/assistant_log.jsonl so the user can review them.
"""

from __future__ import annotations

import json
import re
import time
from datetime import datetime, timezone
from pathlib import Path

from .ollama_client import OllamaClient
from .prompts import ASK_SYSTEM_PROMPT, build_run_data, suggest_system_prompt
from .schemas import SuggestionRejected, suggestion_json_schema, validate_suggestion

PROJECT_ROOT = Path(__file__).resolve().parents[2]
LOG_PATH = PROJECT_ROOT / "data" / "assistant_log.jsonl"
LOG_MAX_BYTES = 5_000_000
MAX_QUESTION_CHARS = 1000
MAX_HISTORY_MESSAGES = 4
MAX_HISTORY_CHARS = 1500
ALWAYS_KNOWN_NUMBERS = (9.81, 100.0)  # g, and "100 %"

Run = tuple[str | None, dict]  # (label, RunResult dict)


# ---- logging ----


def log_exchange(entry: dict, path: Path | None = None) -> None:
    path = path or LOG_PATH
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and path.stat().st_size > LOG_MAX_BYTES:
        path.replace(path.with_name(path.stem + ".1.jsonl"))  # keep one old file
    entry = {"at": datetime.now(timezone.utc).isoformat(timespec="seconds"), **entry}
    with path.open("a", encoding="utf-8") as f:
        f.write(json.dumps(entry, ensure_ascii=False) + "\n")


# ---- number checking ----

_NUMBER_RE = re.compile(r"(?<![\w.])[-−]?\d+(?:[.,]\d+)?")


def _numbers_in(value) -> list[float]:
    """Every number inside a JSON-like value, including numbers written inside strings."""
    out: list[float] = []
    if isinstance(value, bool) or value is None:
        return out
    if isinstance(value, (int, float)):
        return [float(value)]
    if isinstance(value, str):
        for m in _NUMBER_RE.findall(value):
            try:
                out.append(float(m.replace("−", "-").replace(",", ".")))
            except ValueError:
                pass
        return out
    if isinstance(value, dict):
        for v in value.values():
            out += _numbers_in(v)
    elif isinstance(value, (list, tuple)):
        for v in value:
            out += _numbers_in(v)
    return out


def unverified_numbers(answer: str, known_sources: list) -> list[str]:
    """Numbers in the answer that don't match anything in the run data or the question.

    A match allows for rounding: '0.39' matches 0.3896, and '64' matches 64.3. Small whole
    numbers (0–10) are skipped: they're usually counts or list numbering.
    """
    known = [abs(v) for v in _numbers_in(known_sources)] + list(ALWAYS_KNOWN_NUMBERS)
    flagged: list[str] = []
    for token in _NUMBER_RE.findall(answer):
        text = token.replace("−", "-").replace(",", ".")
        try:
            x = abs(float(text))
        except ValueError:
            continue
        decimals = len(text.split(".")[1]) if "." in text else 0
        if decimals == 0 and x <= 10:
            continue
        tol_abs = 0.5 * 10 ** (-decimals)
        ok = any(abs(x - v) <= max(tol_abs, 0.015 * v) for v in known)
        # Also accept unit conversions the model might reasonably make (cm <-> m, fraction <-> %).
        ok = ok or any(abs(x - v * f) <= max(tol_abs, 0.015 * v * f) for v in known for f in (100.0, 0.01))
        if not ok and token not in flagged:
            flagged.append(token)
    return flagged


# ---- tidying ----

_LATEX_TEXT_RE = re.compile(r"\\(?:text|mathrm|textbf|mathbf)\{([^{}]*)\}")


def tidy_answer(text: str) -> str:
    """Remove LaTeX-style markup the panel can't display ($\\text{Kp}$ -> Kp, 64.3\\% -> 64.3 %)."""
    text = _LATEX_TEXT_RE.sub(r"\1", text)
    text = text.replace("\\%", "%").replace("\\_", "_").replace("\\times", "×").replace("\\cdot", "·")
    text = re.sub(r"\$([^$\n]{1,80})\$", r"\1", text)  # inline $...$
    return text.strip()


# ---- the two jobs ----


def _history_messages(history: list[dict] | None) -> list[dict]:
    msgs = []
    for m in (history or [])[-MAX_HISTORY_MESSAGES:]:
        role = m.get("role")
        content = str(m.get("content", ""))[:MAX_HISTORY_CHARS]
        if role in ("user", "assistant") and content:
            msgs.append({"role": role, "content": content})
    return msgs


async def ask(client: OllamaClient, question: str, runs: list[Run], history: list[dict] | None = None) -> dict:
    question = (question or "").strip()
    if not question:
        raise ValueError("Please type a question.")
    question = question[:MAX_QUESTION_CHARS]

    if runs:
        run_data, tokens = build_run_data(runs)
        user = f"RUN DATA:\n{run_data}\n\nQUESTION: {question}"
    else:
        run_data, tokens = "", 0
        user = f"(No run data attached.)\n\nQUESTION: {question}"
    messages = [{"role": "system", "content": ASK_SYSTEM_PROMPT}, *_history_messages(history), {"role": "user", "content": user}]

    start = time.monotonic()
    entry = {"kind": "ask", "model": client.model, "messages": messages}
    try:
        answer = tidy_answer(await client.chat(messages))
    except Exception as exc:
        log_exchange({**entry, "ok": False, "error": str(exc)})
        raise
    duration = time.monotonic() - start
    flagged = unverified_numbers(answer, [run_data, question])
    log_exchange({**entry, "ok": True, "reply": answer, "duration_s": round(duration, 2), "unverified_numbers": flagged})
    return {
        "answer": answer,
        "model": client.model,
        "runs": [label for label, _ in runs],
        "unverified_numbers": flagged,
        "context_tokens": tokens,
        "duration_s": round(duration, 1),
    }


async def suggest(client: OllamaClient, runs: list[Run], goal: str = "") -> dict:
    """Ask for ONE next experiment. Returns {ok: True, suggestion} or {ok: False, error, raw}."""
    goal = (goal or "").strip()[:MAX_QUESTION_CHARS]
    run_data, tokens = build_run_data(runs) if runs else ("", 0)
    parts = [f"RUN DATA:\n{run_data}" if run_data else "(No run data attached: suggest a good first experiment.)"]
    if goal:
        parts.append(f"GOAL: {goal}")
    messages = [{"role": "system", "content": suggest_system_prompt()}, {"role": "user", "content": "\n\n".join(parts)}]

    # Settings to change FROM: the discussed run's settings, per experiment.
    base_params: dict[str, dict] = {}
    for _, result in runs:
        sc = result.get("scenario", {})
        if sc.get("experiment_id"):
            base_params.setdefault(sc["experiment_id"], sc.get("params", {}))

    start = time.monotonic()
    entry = {"kind": "suggest", "model": client.model, "messages": messages}
    try:
        raw = await client.chat(messages, json_schema=suggestion_json_schema())
    except Exception as exc:
        log_exchange({**entry, "ok": False, "error": str(exc)})
        raise
    duration = round(time.monotonic() - start, 1)
    try:
        suggestion = validate_suggestion(raw, base_params)
    except SuggestionRejected as exc:
        log_exchange({**entry, "ok": False, "reply": raw, "rejected": str(exc), "duration_s": duration})
        return {"ok": False, "error": str(exc), "raw": raw[:2000], "duration_s": duration}
    suggestion["unverified_numbers"] = unverified_numbers(
        suggestion["reason"] + " " + suggestion["expected_effect"], [run_data, goal, suggestion["params"], base_params]
    )
    log_exchange({**entry, "ok": True, "reply": raw, "accepted": suggestion, "duration_s": duration})
    return {"ok": True, "suggestion": suggestion, "context_tokens": tokens, "duration_s": duration}
