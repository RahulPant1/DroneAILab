"""Rings race: fly through hoops in order, as fast as you can. A game for the live Fly view.

Each ring stands upright in the air. You must fly through it in the direction of its arrow (`heading_deg`,
0 = east, 90 = north, like everything else in the sim), and the rings count only in order.

A ring counts when the drone's CENTRE crosses the ring's plane, going the right way, inside the ring's radius.
That's a physical fact, so it is judged on the TRUE position, not the drone's estimate.

The clock starts when the drone takes off (motors armed) and stops at the last ring. Best times per track are
kept in data/race_best.json.
"""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

BEST_TIMES_FILE = Path(__file__).resolve().parents[2] / "data" / "race_best.json"
RING_RADIUS_M = 0.9  # big, friendly hoops: the drone (0.3 m) fits through with room to spare


@dataclass(frozen=True)
class Ring:
    """An upright hoop: centre (x, y, z) in metres, flown through toward heading_deg."""

    x_m: float
    y_m: float
    z_m: float
    heading_deg: float = 0.0
    radius_m: float = RING_RADIUS_M

    @property
    def centre(self) -> np.ndarray:
        return np.array([self.x_m, self.y_m, self.z_m])

    @property
    def normal(self) -> np.ndarray:
        """Unit vector pointing the way you must fly through (horizontal)."""
        h = math.radians(self.heading_deg)
        return np.array([math.cos(h), math.sin(h), 0.0])

    def passed(self, p0: np.ndarray, p1: np.ndarray) -> bool:
        """Did the move from p0 to p1 go through this ring, the right way?"""
        n, c = self.normal, self.centre
        d0, d1 = float((p0 - c) @ n), float((p1 - c) @ n)
        if not (d0 < 0.0 <= d1) or d1 - d0 < 1e-9:
            return False  # didn't cross the ring's plane, or crossed it backwards (or slid along it)
        hit = p0 + (p1 - p0) * (-d0 / (d1 - d0)) if d1 != d0 else p1  # where the path met the plane
        return float(np.linalg.norm(hit - c)) <= self.radius_m

    def to_dict(self) -> dict:
        return asdict(self)


# Built-in tracks: easy first. Every one starts on a pad at its start point, facing east.
TRACKS: dict[str, dict] = {
    "first_flight": {
        "title": "First flight (3 rings in a row)",
        "start": (0.0, 0.0),
        "rings": [Ring(3, 0, 2), Ring(6, 0, 2), Ring(9, 0, 2)],
        "tip": "Take off, then hold W. All three rings are straight ahead at take-off height.",
    },
    "up_and_down": {
        "title": "Up and down (use R and F)",
        "start": (0.0, 0.0),
        "rings": [Ring(3, 0, 1.5), Ring(6, 0, 3.0), Ring(9, 0, 1.5), Ring(12, 0, 3.0)],
        "tip": "Rings go low, high, low, high. Hold W and use R (up) and F (down) between them.",
    },
    "zig_zag": {
        "title": "Zig-zag (use A and D)",
        "start": (0.0, 0.0),
        "rings": [Ring(3, 1.5, 2), Ring(6, -1.5, 2), Ring(9, 1.5, 2), Ring(12, -1.5, 2)],
        "tip": "Rings swap from left to right. Steer with A and D while you fly forward with W.",
    },
    "square_loop": {
        "title": "Square loop (turn with Q and E)",
        "start": (0.0, 0.0),
        "rings": [Ring(4, 0, 2, 0), Ring(7, 3.5, 2.5, 90), Ring(4, 7, 2, 180), Ring(0.5, 3.5, 2.5, 270)],
        "tip": "Follow the arrows around a square. Turn with Q (left) so W still means 'forward'.",
    },
}


def load_best_times(path: Path | None = None) -> dict[str, float]:
    path = path or BEST_TIMES_FILE
    try:
        data = json.loads(path.read_text())
        return {k: float(v) for k, v in data.items() if k in TRACKS}
    except (OSError, ValueError, AttributeError):
        return {}


def save_best_time(track_id: str, time_s: float, path: Path | None = None) -> bool:
    """Store the time if it beats the best for this track. Returns True for a new best."""
    path = path or BEST_TIMES_FILE
    best = load_best_times(path)
    if track_id in best and best[track_id] <= time_s:
        return False
    best[track_id] = round(time_s, 2)
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(best, indent=2))
    except OSError:
        pass  # can't save: still a new best for this session
    return True


class Race:
    """One attempt at a track: which ring is next, and the clock."""

    def __init__(self, track_id: str, best_file: Path | None = None):
        if track_id not in TRACKS:
            raise ValueError(f"Unknown track '{track_id}'. Choose one of: {', '.join(TRACKS)}.")
        self.track_id = track_id
        self.track = TRACKS[track_id]
        self.rings: list[Ring] = self.track["rings"]
        self.best_file = best_file
        self.best_s: float | None = load_best_times(best_file).get(track_id)  # kept in memory (read once)
        self.restart()

    def restart(self) -> None:
        self.next_ring = 0  # index of the ring to fly through next
        self.start_t_s: float | None = None
        self.finish_s: float | None = None
        self.new_best = False
        self.splits_s: list[float] = []

    def update(self, p0: np.ndarray, p1: np.ndarray, t_s: float, armed: bool) -> list[str]:
        """Advance the game by one physics step (the drone moved p0 -> p1). Returns messages to show."""
        said: list[str] = []
        if self.finish_s is not None:
            return said
        if self.start_t_s is None:
            if not armed:
                return said
            self.start_t_s = t_s
            said.append("Go! The clock is running.")
        if self.rings[self.next_ring].passed(p0, p1):
            elapsed = t_s - self.start_t_s
            self.splits_s.append(elapsed)
            self.next_ring += 1
            if self.next_ring == len(self.rings):
                self.finish_s = elapsed
                self.new_best = save_best_time(self.track_id, elapsed, self.best_file)
                if self.new_best or self.best_s is None:
                    self.best_s = round(elapsed, 2)
                said.append(f"🏁 Finished in {elapsed:.1f} s!" + (" ⭐ New best time!" if self.new_best else f" (best: {self.best_s:.1f} s)"))
            else:
                said.append(f"Ring {self.next_ring} of {len(self.rings)} ✓ ({elapsed:.1f} s)")
        return said

    def elapsed_s(self, t_s: float) -> float:
        if self.finish_s is not None:
            return self.finish_s
        return 0.0 if self.start_t_s is None else t_s - self.start_t_s

    def info(self) -> dict:
        """What the 3D view needs to draw the track (sent once per change)."""
        return {
            "id": self.track_id,
            "title": self.track["title"],
            "tip": self.track["tip"],
            "start": list(self.track["start"]),
            "rings": [r.to_dict() for r in self.rings],
        }

    def status(self, t_s: float) -> dict:
        """Live race numbers for every frame."""
        return {
            "next_ring": self.next_ring,
            "total": len(self.rings),
            "time_s": round(self.elapsed_s(t_s), 2),
            "running": self.start_t_s is not None and self.finish_s is None,
            "finished": self.finish_s is not None,
            "new_best": self.new_best,
            "best_s": self.best_s,
        }
