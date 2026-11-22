"""The farmer's report: the local LLM turns a crop-health map's findings into a short, practical note.

It sees only the compact findings (field, crop, patches: where, how big, how low the NDVI), never the picture.
Every number it writes is checked against those findings; the rule-based report (cropmap.rule_report) is always
shown next to it. Logged to data/assistant_log.jsonl as kind "crop_report".
"""

from __future__ import annotations

import json
import time

from backend.autonomy.site import FARM

from .assistant import log_exchange, tidy_answer, unverified_numbers
from .ollama_client import OllamaClient

SYSTEM_PROMPT = """You are an agronomist's assistant writing a short report for a farmer, from a drone survey of one field.
NDVI measures how green and healthy plants are: healthy crop is about 0.8; below 0.6 means the plants are stressed.

Write at most 120 words, plain text, as bullets starting "- ":
- one bullet with the overall picture (the field's average NDVI, how many problem patches);
- one bullet per patch: where it is (use the "where" words and the x/y position), how big, how bad;
- for each patch, label possible causes as [Hypothesis] (e.g. irrigation, drainage, pests, disease): a drone map
  can't tell the cause, someone must go and look;
- end with what to do first (walk to the biggest or worst patch).
Use ONLY numbers that appear in the FINDINGS. Never invent numbers. No greeting, no LaTeX."""


def compact(m: dict) -> dict:
    crop = next((f.crop for f in FARM.fields if f.name == m["field"]), "")
    return {
        "field": m["field"], "crop": crop, "photos": m["photos"], "mapped_pct": m["mapped_pct"],
        "field_average_ndvi": m["field_ndvi"], "stressed_area_m2": m["stressed_m2"],
        "patches": [{k: f[k] for k in ("where", "x_m", "y_m", "area_m2", "mean_ndvi", "min_ndvi")} for f in m["findings"][:6]],
        "map_position_uncertainty_m": m.get("geotag_error_m"),
    }


async def write_report(client: OllamaClient, m: dict) -> dict:
    data = json.dumps(compact(m), separators=(",", ":"), ensure_ascii=False)
    messages = [{"role": "system", "content": SYSTEM_PROMPT}, {"role": "user", "content": f"FINDINGS:\n{data}"}]
    entry = {"kind": "crop_report", "model": client.model, "field": m["field"], "messages": messages}
    start = time.monotonic()
    try:
        answer = tidy_answer(await client.chat(messages))
    except Exception as exc:
        log_exchange({**entry, "ok": False, "error": str(exc)})
        raise
    duration = round(time.monotonic() - start, 1)
    flagged = unverified_numbers(answer, [data])
    log_exchange({**entry, "ok": True, "reply": answer, "duration_s": duration, "unverified_numbers": flagged})
    return {"answer": answer, "model": client.model, "unverified_numbers": flagged, "duration_s": duration}
