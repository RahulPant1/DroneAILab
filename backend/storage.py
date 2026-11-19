"""Saved runs: one JSON file per run in data/experiments/, plus a small index for fast listing.

Saved runs are created by RE-RUNNING the experiment on the server (same params + seed give the
same result), so a saved run is always genuine simulator output, never data pasted in by a client.
"""

from __future__ import annotations

import json
import os
import re
import secrets
import threading
from datetime import datetime, timezone
from pathlib import Path

from backend.sim.runner import RunResult

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_DIR = PROJECT_ROOT / "data" / "experiments"
MAX_RUNS = 100
LABEL_MAX_CHARS = 80
RUN_ID_RE = re.compile(r"^[0-9]{8}-[0-9]{6}-[a-z_]+-[0-9a-f]{6}$")

_lock = threading.Lock()


class StorageError(ValueError):
    pass


class RunStore:
    def __init__(self, directory: Path = DEFAULT_DIR, max_runs: int = MAX_RUNS):
        self.dir = Path(directory)
        self.max_runs = max_runs
        self.index_path = self.dir / "index.json"

    # ---- index ----

    def _read_index(self) -> list[dict]:
        try:
            data = json.loads(self.index_path.read_text(encoding="utf-8"))
            if isinstance(data, list):
                return data
        except (OSError, ValueError):
            pass
        return self._rebuild_index()

    def _rebuild_index(self) -> list[dict]:
        """If the index is missing or damaged, rebuild it from the run files."""
        entries = []
        for path in sorted(self.dir.glob("*.json")):
            if path.name == "index.json" or not RUN_ID_RE.match(path.stem):
                continue
            try:
                entries.append(self._summary(json.loads(path.read_text(encoding="utf-8"))))
            except (OSError, ValueError, KeyError):
                continue
        entries.sort(key=lambda e: e["saved_at"])
        self._write_index(entries)
        return entries

    def _write_index(self, entries: list[dict]) -> None:
        self.dir.mkdir(parents=True, exist_ok=True)
        tmp = self.index_path.with_suffix(".tmp")
        tmp.write_text(json.dumps(entries, indent=1), encoding="utf-8")
        os.replace(tmp, self.index_path)

    @staticmethod
    def _summary(record: dict) -> dict:
        result = record["result"]
        return {
            "id": record["id"],
            "label": record["label"],
            "saved_at": record["saved_at"],
            "experiment_id": result["scenario"].get("experiment_id", result["run_type"]),
            "params": result["scenario"].get("params", {}),
            "seed": result["seed"],
            "metrics": {k: v for k, v in result["metrics"].items() if isinstance(v, (int, float, bool, str)) or v is None},
        }

    # ---- public API ----

    def list(self) -> list[dict]:
        with _lock:
            return list(reversed(self._read_index()))  # newest first

    def save(self, result: RunResult, label: str = "") -> dict:
        label = (label or "").strip()[:LABEL_MAX_CHARS]
        with _lock:
            entries = self._read_index()
            if len(entries) >= self.max_runs:
                raise StorageError(
                    f"You already have {self.max_runs} saved runs (the limit). Delete some, or use 'Delete old runs'."
                )
            now = datetime.now(timezone.utc)
            run_id = f"{now:%Y%m%d-%H%M%S}-{result.run_type}-{secrets.token_hex(3)}"
            record = {
                "id": run_id,
                "label": label or f"{result.run_type} {now:%H:%M:%S}",
                "saved_at": now.isoformat(timespec="seconds"),
                "result": result.to_dict(),
            }
            self.dir.mkdir(parents=True, exist_ok=True)
            (self.dir / f"{run_id}.json").write_text(json.dumps(record), encoding="utf-8")
            entries.append(self._summary(record))
            self._write_index(entries)
            return entries[-1]

    def get(self, run_id: str) -> dict:
        path = self._path(run_id)
        try:
            return json.loads(path.read_text(encoding="utf-8"))
        except FileNotFoundError:
            raise StorageError(f"No saved run '{run_id}'.") from None

    def delete(self, run_id: str) -> None:
        path = self._path(run_id)
        with _lock:
            entries = self._read_index()
            if not path.exists():
                raise StorageError(f"No saved run '{run_id}'.")
            path.unlink()
            self._write_index([e for e in entries if e["id"] != run_id])

    def delete_old(self, keep_newest: int) -> int:
        """Delete all but the newest `keep_newest` runs. Returns how many were deleted."""
        if keep_newest < 0:
            raise StorageError("keep_newest must be 0 or more.")
        with _lock:
            entries = self._read_index()
            doomed = entries[: max(0, len(entries) - keep_newest)]
            for e in doomed:
                (self.dir / f"{e['id']}.json").unlink(missing_ok=True)
            self._write_index(entries[len(doomed) :])
            return len(doomed)

    def _path(self, run_id: str) -> Path:
        if not RUN_ID_RE.match(run_id or ""):
            raise StorageError(f"'{run_id}' is not a valid run id.")
        return self.dir / f"{run_id}.json"


def compare(a: dict, b: dict) -> dict:
    """Side-by-side numeric metrics of two saved runs (b − a)."""
    ma, mb = a["result"]["metrics"], b["result"]["metrics"]
    rows = []
    for key in sorted(set(ma) | set(mb)):
        va, vb = ma.get(key), mb.get(key)
        numeric = all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in (va, vb))
        rows.append({"metric": key, "a": va, "b": vb, "diff": (vb - va) if numeric else None})
    pa = a["result"]["scenario"].get("params", {})
    pb = b["result"]["scenario"].get("params", {})
    param_rows = [{"param": k, "a": pa.get(k), "b": pb.get(k)} for k in sorted(set(pa) | set(pb))]
    return {"a": a, "b": b, "metrics": rows, "params": param_rows}
