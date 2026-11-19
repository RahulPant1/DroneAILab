"""The shape of an AI 'suggest an experiment' reply, and how we check it.

Two layers of checking, because a language model can produce anything:
  1. Structure: is it valid JSON with exactly the expected fields? (pydantic)
  2. Meaning: does the experiment exist, are the settings real, and are the values inside the
     same limits the UI uses? (Experiment.check_params)
Anything that fails gets a plain-English reason and is never run.
"""

from __future__ import annotations

import json

from pydantic import BaseModel, ConfigDict, Field, ValidationError

from backend.experiments import EXPERIMENTS, ExperimentError, get_experiment

MAX_CHANGES = 3


class Suggestion(BaseModel):
    model_config = ConfigDict(extra="forbid")

    experiment_id: str
    changes: dict[str, float | str]
    reason: str = Field(min_length=1, max_length=1200)
    expected_effect: str = Field(min_length=1, max_length=1200)


def suggestion_json_schema() -> dict:
    """JSON schema passed to Ollama's `format`, so the model is constrained to this shape."""
    return {
        "type": "object",
        "properties": {
            "experiment_id": {"type": "string", "enum": list(EXPERIMENTS)},
            "changes": {
                "type": "object",
                "description": "setting id -> new value (1 to 3 settings)",
                "additionalProperties": {"type": ["number", "string"]},
            },
            "reason": {"type": "string"},
            "expected_effect": {"type": "string"},
        },
        "required": ["experiment_id", "changes", "reason", "expected_effect"],
        "additionalProperties": False,
    }


class SuggestionRejected(ValueError):
    """The AI's suggestion can't be used. The message says why, in plain English."""


def _pydantic_reason(exc: ValidationError) -> str:
    parts = []
    for err in exc.errors()[:3]:
        where = ".".join(str(p) for p in err["loc"]) or "reply"
        parts.append(f"'{where}': {err['msg']}")
    return "The AI's reply had the wrong shape: " + "; ".join(parts) + "."


def validate_suggestion(raw_text: str, base_params: dict[str, dict]) -> dict:
    """Check the model's raw reply. Returns a ready-to-show suggestion with a diff, or raises SuggestionRejected.

    `base_params` maps experiment id -> the settings to change FROM (the run being discussed, or defaults).
    """
    try:
        data = json.loads(raw_text)
    except (TypeError, ValueError):
        raise SuggestionRejected("The AI's reply wasn't valid JSON, so it was ignored.") from None
    try:
        s = Suggestion.model_validate(data)
    except ValidationError as exc:
        raise SuggestionRejected(_pydantic_reason(exc)) from None

    try:
        exp = get_experiment(s.experiment_id)
    except ExperimentError:
        raise SuggestionRejected(f"The AI suggested an experiment that doesn't exist ('{s.experiment_id}').") from None
    if not 1 <= len(s.changes) <= MAX_CHANGES:
        raise SuggestionRejected(f"The AI should change 1 to {MAX_CHANGES} settings, but proposed {len(s.changes)}.")
    known = {p.id: p for p in exp.params}
    unknown = [k for k in s.changes if k not in known]
    if unknown:
        raise SuggestionRejected(
            f"The AI used setting(s) that '{exp.title}' doesn't have: {', '.join(unknown)}. "
            f"Valid settings: {', '.join(known)}."
        )

    base = exp.check_params(base_params.get(exp.id))  # defaults filled in
    proposed = {**base, **s.changes}
    try:
        proposed = exp.check_params(proposed)
    except ExperimentError as exc:
        raise SuggestionRejected(f"The AI suggested a value outside the allowed limits. {exc}") from None

    diff = [
        {"param": k, "label": known[k].label, "unit": known[k].unit, "from": base[k], "to": proposed[k]}
        for k in s.changes
        if proposed[k] != base[k]
    ]
    if not diff:
        raise SuggestionRejected("The AI's suggestion didn't actually change any setting.")
    return {
        "experiment_id": exp.id,
        "experiment_title": exp.title,
        "params": proposed,
        "diff": diff,
        "reason": s.reason.strip(),
        "expected_effect": s.expected_effect.strip(),
    }
