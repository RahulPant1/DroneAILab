"""Where flight logs come from: ArduPilot SITL's own logs, and .BIN files the user uploads (e.g. from a real drone).

Log ids are "sitl-00000042" or "upload-<name>"; they map to files only through this module, so a request can never
reach a path outside these two folders.
"""

from __future__ import annotations

import json
import re
import time
from pathlib import Path

from backend.sitl import launch

SITL_LOG_DIR = launch.RUN_DIR / "logs"
UPLOAD_DIR = launch.ROOT / "data" / "logs"
CACHE_DIR = UPLOAD_DIR / "cache"
MAX_UPLOAD_BYTES = 100 * 1024 * 1024
DATAFLASH_MAGIC = b"\xa3\x95"  # every DataFlash message starts with these two bytes
MAX_LISTED = 60

_SITL_ID = re.compile(r"^sitl-(\d{8})$")
_UPLOAD_ID = re.compile(r"^upload-([A-Za-z0-9_.-]{1,80})$")


class LogNotFound(LookupError):
    pass


def path_for(log_id: str) -> Path:
    m = _SITL_ID.match(log_id)
    if m:
        p = SITL_LOG_DIR / f"{m.group(1)}.BIN"
    else:
        m = _UPLOAD_ID.match(log_id)
        if not m or ".." in log_id:
            raise LogNotFound(f"Unknown log id {log_id!r}")
        p = UPLOAD_DIR / f"{m.group(1)}.bin"
    if not p.is_file():
        raise LogNotFound(f"No such log: {log_id}")
    return p


def list_logs() -> list[dict]:
    items = []
    if SITL_LOG_DIR.is_dir():
        items += [("sitl-" + p.stem, p, "simulator") for p in SITL_LOG_DIR.glob("*.BIN") if re.fullmatch(r"\d{8}", p.stem)]
    if UPLOAD_DIR.is_dir():
        items += [("upload-" + p.stem, p, "uploaded") for p in UPLOAD_DIR.glob("*.bin")]
    out = []
    for log_id, p, source in items:
        st = p.stat()
        if st.st_size < 1024:
            continue  # empty logs (SITL started but never armed)
        out.append({"id": log_id, "name": p.name, "source": source, "size_mb": round(st.st_size / 1e6, 2),
                    "modified": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(st.st_mtime)), "_mtime": st.st_mtime})
    out.sort(key=lambda d: -d["_mtime"])
    for d in out:
        d.pop("_mtime")
    return out[:MAX_LISTED]


def save_upload(filename: str, data: bytes) -> str:
    """Store an uploaded .BIN log. Returns its log id."""
    if not filename.lower().endswith(".bin"):
        raise ValueError("Please upload an ArduPilot DataFlash log (.BIN). Telemetry logs (.tlog) aren't supported.")
    if len(data) > MAX_UPLOAD_BYTES:
        raise ValueError(f"That log is larger than {MAX_UPLOAD_BYTES // 2**20} MB.")
    if not data.startswith(DATAFLASH_MAGIC):
        raise ValueError("That file doesn't look like an ArduPilot DataFlash log.")
    stem = re.sub(r"[^A-Za-z0-9_.-]", "_", Path(filename).stem)[:60].strip(".") or "log"
    stem = f"{time.strftime('%Y%m%d-%H%M%S')}-{stem}"
    UPLOAD_DIR.mkdir(parents=True, exist_ok=True)
    (UPLOAD_DIR / f"{stem}.bin").write_bytes(data)
    return f"upload-{stem}"


ANALYSIS_VERSION = 2  # bump when the checks change, so old cached reports are redone (2 = real-hardware checks)


def cached_analysis(log_id: str, analyze) -> dict:
    """analyze(path), cached on disk per file version (a SITL log can still grow while that SITL runs) and per
    version of the checks."""
    p = path_for(log_id)
    st = p.stat()
    key = CACHE_DIR / f"{log_id}-{st.st_size}-{int(st.st_mtime)}-v{ANALYSIS_VERSION}.json"
    if key.is_file():
        return json.loads(key.read_text())
    result = analyze(p)
    result["id"] = log_id
    CACHE_DIR.mkdir(parents=True, exist_ok=True)
    for old in CACHE_DIR.glob(f"{log_id}-*.json"):
        old.unlink()
    key.write_text(json.dumps(result))
    return result


def latest_sitl_log_id() -> str | None:
    """The log SITL wrote most recently (it records the number in LASTLOG.TXT)."""
    try:
        n = int((SITL_LOG_DIR / "LASTLOG.TXT").read_text().strip())
    except (OSError, ValueError):
        return None
    return f"sitl-{n:08d}" if (SITL_LOG_DIR / f"{n:08d}.BIN").is_file() else None
