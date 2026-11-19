"""What the model is told: system prompts and the compact JSON summary of runs ("grounding").

Grounding rule (PRD): send at most 2 runs, as a compact summary of settings, metrics, events and the
rule-based explanation. Raw telemetry is NEVER sent. The whole run summary stays under ~2k tokens.
"""

from __future__ import annotations

import json
import math

from backend.experiments import EXPERIMENTS

MAX_RUNS = 2
MAX_CONTEXT_TOKENS = 2000
CHARS_PER_TOKEN = 3.5  # rough rule of thumb for English + JSON
MAX_EVENTS = 8

ASK_SYSTEM_PROMPT = """You are the assistant inside DroneLab, a quadcopter simulator used by a BEGINNER to learn drone engineering.

You may be given RUN DATA: a JSON summary of one or two simulator runs (settings, measured metrics, events, and a rule-based explanation).

Rules (follow all of them):
1. Start each claim or bullet with a label:
   [Measured]   = it comes directly from RUN DATA. Quote the number with its unit.
   [Hypothesis] = your explanation, reasoning, or general knowledge.
2. NEVER invent numbers. Only use numbers that appear in RUN DATA or in the question.
3. If the data doesn't show something, say "I don't know: the run data doesn't show that." Don't guess values.
4. If there is no RUN DATA, explain the idea in general terms and label everything [Hypothesis].
5. Be brief: at most about 150 words. No greeting. Write each point as a bullet starting "- [Measured]" or "- [Hypothesis]".
   Plain text only: no LaTeX, no $ signs, no backslashes. Write "Kp", "64.3 %", and use everyday words instead of metric ids where possible.
6. You can't see plots, run code, or change settings. You only see the summary. To try something, the user can click "Suggest an experiment".
Units: SI (m, s, kg, N). Names ending in _deg are degrees, _pct are percent, _m_s are metres per second."""

SUGGEST_SYSTEM_PROMPT = """You are the assistant inside DroneLab, a quadcopter simulator used by a BEGINNER.
Suggest ONE next experiment that would teach them something, based on the RUN DATA (and their GOAL, if given).

Available experiments, their settings, and allowed ranges:
{catalogue}

Reply ONLY with JSON:
{{"experiment_id": "...", "changes": {{"<setting id>": <new value>}}, "reason": "...", "expected_effect": "..."}}
- Change 1 to 3 settings. Keep values inside the allowed ranges. Use the exact setting ids above.
- Prefer the same experiment as the RUN DATA unless the goal asks for something else.
- "reason": one or two sentences, quoting measured numbers from RUN DATA when relevant.
- "expected_effect": a prediction of what the plots or metrics will show. It's a hypothesis, so don't state invented numbers as fact."""


def experiment_catalogue() -> str:
    lines = []
    for e in EXPERIMENTS.values():
        settings = []
        for p in e.params:
            if p.options:
                settings.append(f"{p.id} (one of: {', '.join(v for v, _ in p.options)})")
            else:
                settings.append(f"{p.id} [{p.min}–{p.max} {p.unit}, default {p.default}]")
        lines.append(f"- {e.id}: {e.title}. Settings: " + "; ".join(settings))
    return "\n".join(lines)


def suggest_system_prompt() -> str:
    return SUGGEST_SYSTEM_PROMPT.format(catalogue=experiment_catalogue())


def _round(v, sig: int = 4):
    """Round to 4 significant figures, so numbers stay readable and short."""
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        return v
    if v == 0 or not math.isfinite(v):
        return 0 if v == 0 else None
    return round(v, max(0, sig - 1 - int(math.floor(math.log10(abs(v))))))


def compact_run(result: dict, label: str | None = None) -> dict:
    """A small, telemetry-free summary of one run."""
    scenario = result.get("scenario", {})
    exp_id = scenario.get("experiment_id", result.get("run_type"))
    exp = EXPERIMENTS.get(exp_id)
    cfg = result.get("config", {})
    derived = cfg.get("derived", {})
    env = result.get("env", {})
    wind, sensors = env.get("wind", {}), env.get("sensors", {})

    environment = {}
    if wind.get("wind_speed_m_s") or wind.get("gust_std_m_s"):
        environment["wind"] = {k: _round(wind[k]) for k in ("wind_speed_m_s", "wind_direction_deg", "gust_std_m_s", "wind_start_s")}
    if wind.get("unmodeled_payload_kg"):
        environment["hidden_payload_kg"] = _round(wind["unmodeled_payload_kg"])
    if sensors.get("enabled"):
        environment["sensors"] = {k: _round(v) for k, v in sensors.items() if k != "enabled"}
    if not environment:
        environment = "calm air, perfect sensors"

    return {
        "label": label or (exp.title if exp else exp_id),
        "experiment": f"{exp.number}. {exp.title}" if exp else exp_id,
        "settings": {k: _round(v) for k, v in scenario.get("params", {}).items()},
        "duration_s": _round(scenario.get("duration_s")),
        "seed": result.get("seed"),
        "drone": {
            "name": cfg.get("name"),
            "mass_kg": _round(cfg.get("mass_kg")),
            "thrust_to_weight": _round(derived.get("thrust_to_weight")),
            "hover_throttle_pct": _round(derived.get("hover_throttle_pct")),
            "gains": {k: _round(v) for k, v in (cfg.get("gains") or {}).items()},
        },
        "environment": environment,
        "metrics": {k: _round(v) for k, v in result.get("metrics", {}).items() if isinstance(v, (int, float, bool, str)) or v is None},
        "events": [f"t={e['t_s']:.2f} s: {e['message']}" for e in result.get("events", [])[:MAX_EVENTS]],
        "rule_based_explanation": result.get("summary", []),
    }


def estimate_tokens(text: str) -> int:
    return math.ceil(len(text) / CHARS_PER_TOKEN)


def build_run_data(runs: list[tuple[str | None, dict]]) -> tuple[str, int]:
    """JSON text for up to 2 runs, trimmed to fit the token budget. Returns (text, estimated tokens)."""
    if len(runs) > MAX_RUNS:
        raise ValueError(f"At most {MAX_RUNS} runs can be discussed at once.")
    compact = [compact_run(result, label) for label, result in runs]

    def dump() -> str:
        return json.dumps(compact if len(compact) > 1 else compact[0], separators=(",", ":"), ensure_ascii=False)

    text = dump()
    # Trim the least important parts first until it fits.
    for trim in ("rule_based_explanation", "gains", "events"):
        if estimate_tokens(text) <= MAX_CONTEXT_TOKENS:
            break
        for c in compact:
            if trim == "gains":
                c["drone"].pop("gains", None)
            else:
                c.pop(trim, None)
        text = dump()
    if estimate_tokens(text) > MAX_CONTEXT_TOKENS:
        for c in compact:
            c["metrics"] = dict(list(c["metrics"].items())[:25])
        text = dump()
    return text, estimate_tokens(text)
