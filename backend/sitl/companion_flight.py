"""Fly ArduCopter SITL with and without DroneLab's companion-computer process, and measure the difference.

  precland  ArduPilot is told to land at a spot 1.8 m from the pad (as a drifting GPS would). Without the companion
            it lands there; with it, the companion's camera finds the marker and streams LANDING_TARGET, and
            ArduPilot's own precision landing (PLND_TYPE = 1) steers onto the pad.
  avoid     An AUTO mission straight through a 10 m wide wall the autopilot can't see. Without the companion it
            flies right through it (in the simulator, nothing stops it: a real drone would crash). With it, the
            companion's lidar streams OBSTACLE_DISTANCE and ArduPilot's BendyRuler planner (OA_TYPE = 1) goes round.

Both measured on SITL's ground truth (SIM_STATE). The companion is a separate process
(`python -m backend.companion.service`) on SITL's SERIAL2 port, exactly as a Jetson / Pi on a serial cable would be.

    .venv/bin/python -m backend.sitl.companion_flight              # both jobs
    .venv/bin/python -m backend.sitl.companion_flight --job avoid
    .venv/bin/python -m backend.sitl.companion_flight --write      # also docs/results/companion.*
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
import time
from pathlib import Path
from typing import Callable

from backend.companion.service import WORLDS

from . import launch
from .fly import Flight, offset_latlon

ROOT = Path(__file__).resolve().parents[2]
RESULTS = ROOT / "docs" / "results"
MSG_SIM_STATE = 108
JOB_PARAMS = {
    "precland": {"PLND_ENABLED": 1, "PLND_TYPE": 1},  # precision landing from MAVLink LANDING_TARGET
    "avoid": {"PRX1_TYPE": 2, "OA_TYPE": 1},  # proximity from MAVLink OBSTACLE_DISTANCE + BendyRuler path planning
}
GPS_ERROR_M = (1.5, -1.0)  # where ArduPilot is told to land, relative to the pad (1.8 m off: typical GPS drift)
# ArduPilot ignores a landing target more than PLND_XY_DIST_MAX (default 2.5 m) away: a bigger error needs that raised.
CRUISE_ALT_M = 10.0
AVOID_ALT_M = 5.0
AVOID_GOAL = (0.0, 24.0)
AVOID_SPEED_M_S = 3.0


class CompanionProcess:
    """Start `python -m backend.companion.service` as its own process; stop it and read its statistics."""

    def __init__(self, url: str, world: str):
        self.p = subprocess.Popen([sys.executable, "-m", "backend.companion.service", "--connect", url, "--world", world],
                                  cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        line = self.p.stdout.readline()
        if '"ready"' not in line:
            self.p.kill()
            raise RuntimeError(f"The companion didn't start: {line.strip()}")

    def stop(self) -> dict:
        self.p.terminate()
        try:
            out, _ = self.p.communicate(timeout=10)
        except subprocess.TimeoutExpired:
            self.p.kill()
            out, _ = self.p.communicate()
        for line in reversed(out.splitlines()):
            if '"stopped"' in line:
                return json.loads(line)
        return {}


def _track_until(f: Flight, done: Callable[[], bool], timeout_s: float, every_s: float = 0.1) -> list[list[float]]:
    track: list[list[float]] = []
    t0 = f.t_s
    deadline = time.monotonic() + timeout_s * f.k
    while not done():
        if time.monotonic() > deadline:
            raise TimeoutError("The flight didn't finish in time")
        with f.lock:
            tr = f.truth_enu3()
        if tr is not None:
            track.append([round(f.t_s - t0, 2), round(tr[0], 2), round(tr[1], 2), round(tr[2], 2)])
        f.sleep(every_s)
    return track


def _ready(f: Flight) -> None:
    f.keep_first_home = True  # the second flight takes off where the first one landed
    f.wait_for_home(90)
    f.wait_until("EKF position", lambda: f.ekf_ok, 90)
    f.request_rate(MSG_SIM_STATE, 20)


def precland_job(f: Flight, url: str, with_companion: bool, say: Callable[[str], None]) -> dict:
    pad_x, pad_y, _ = WORLDS["precland"]["pads"][0]
    comp = CompanionProcess(url, "precland") if with_companion else None
    try:
        f.set_mode("GUIDED")
        f.arm_when_ready()
        f.takeoff(CRUISE_ALT_M)
        aim = (pad_x + GPS_ERROR_M[0], pad_y + GPS_ERROR_M[1])
        say(f"flying to the landing point it was given ({aim[0]:.1f}, {aim[1]:.1f}), 1.8 m from the pad...")
        f.fly_to_local(aim[0], aim[1], CRUISE_ALT_M, settle_s=2.0)
        f.set_mode("LAND")
        track = _track_until(f, lambda: not f.armed, 120)
    finally:
        stats = comp.stop() if comp else {}
    with f.lock:
        tr = f.truth_enu3()
    landed = math.dist(tr[:2], (pad_x, pad_y)) if tr else None
    plnd_msgs = [m for _, m in f.text_log if "PrecLand" in m or "Plnd" in m]
    return {"with_companion": with_companion, "landed_from_pad_m": round(landed, 2) if landed is not None else None,
            "aimed_from_pad_m": round(math.hypot(*GPS_ERROR_M), 2), "companion": stats, "autopilot_messages": plnd_msgs,
            "track": track}


def avoid_job(f: Flight, url: str, with_companion: bool, say: Callable[[str], None]) -> dict:
    wall = WORLDS["avoid"]["boxes"][0]
    comp = CompanionProcess(url, "avoid") if with_companion else None
    try:
        f.set_mode("GUIDED")
        f.arm_when_ready()
        f.takeoff(AVOID_ALT_M)
        assert f.home is not None
        gx, gy = AVOID_GOAL
        f.set_param("WP_SPD", AVOID_SPEED_M_S)  # slow enough to stop / turn within the lidar's 8 m
        f.upload_mission([(*offset_latlon(*f.home, gy, gx), AVOID_ALT_M)])
        with f.lock:
            f.reached.clear()
        say("AUTO mission: straight to a point 24 m north, through the wall...")
        f.set_mode("AUTO")
        track = _track_until(f, lambda: 2 in f.reached, 150)
        f.set_mode("LAND")
        f.wait_until("landed", lambda: not f.armed, 120)
    finally:
        stats = comp.stop() if comp else {}
    inside = [p for p in track if wall.x_min <= p[1] <= wall.x_max and wall.y_min <= p[2] <= wall.y_max]
    clear = min(wall.distance_2d(p[1], p[2]) for p in track)
    length = sum(math.dist(a[1:3], b[1:3]) for a, b in zip(track, track[1:]))
    return {"with_companion": with_companion, "flew_through_wall": bool(inside),
            "min_clearance_m": round(clear, 2), "mission_time_s": track[-1][0] if track else None,
            "path_length_m": round(length, 1), "companion": stats, "track": track}


JOBS = {"precland": precland_job, "avoid": avoid_job}


def run_job(job: str, speedup: float = 1.0, say: Callable[[str], None] = print) -> dict:
    """The job twice, each on a fresh SITL: stock ArduPilot with no companion, then configured for the companion
    (its parameters need a reboot, so they're set at boot). Configured for a lidar, ArduPilot refuses to arm while
    the companion isn't sending ("PreArm: PRX1: No Data"): a drone WITHOUT the companion is the fair comparison."""
    out = {}
    for with_c in (False, True):
        label = "with the companion" if with_c else "without the companion (stock ArduPilot)"
        say(f"{job}, {label}:")
        sitl = launch.start(speedup=speedup, extra_params=JOB_PARAMS[job] if with_c else None, companion_port=with_c)
        try:
            f = Flight(sitl.url, log=lambda _t: None, timeout_scale=max(1.0, 1.0 / speedup))
            try:
                _ready(f)
                out["with" if with_c else "without"] = JOBS[job](f, sitl.companion_url, with_c, lambda t: say("  " + t))
            finally:
                f.close()
        finally:
            sitl.stop()
    return out


def summary_lines(res: dict) -> list[str]:
    lines = []
    if "precland" in res:
        a, b = res["precland"]["without"], res["precland"]["with"]
        lines.append(f"Precision landing: without the companion it landed {a['landed_from_pad_m']} m from the pad; with "
                     f"it {b['landed_from_pad_m']} m ({b['companion'].get('landing_target_sent', 0)} LANDING_TARGET messages).")
    if "avoid" in res:
        a, b = res["avoid"]["without"], res["avoid"]["with"]
        lines.append(f"Avoidance: without the companion it {'flew THROUGH the wall' if a['flew_through_wall'] else 'missed the wall'}; "
                     f"with it, {'flew through it' if b['flew_through_wall'] else 'went round'} "
                     f"(closest {b['min_clearance_m']} m, path {b['path_length_m']} m vs {a['path_length_m']} m).")
    return lines


def write_results(res: dict) -> Path:
    RESULTS.mkdir(parents=True, exist_ok=True)
    (RESULTS / "companion.json").write_text(json.dumps(res, separators=(",", ":")))
    md = ["# Companion computer + ArduPilot: perception as a separate process",
          "",
          "DroneLab's perception (`backend/companion/service.py`) runs as its own process on ArduCopter 4.7 SITL's SERIAL2 "
          "port, like a Jetson or Raspberry Pi wired to a flight controller. Measured on SITL's ground truth. Generated by "
          "`python -m backend.sitl.companion_flight --write`.", ""]
    if "precland" in res:
        a, b = res["precland"]["without"], res["precland"]["with"]
        md += ["## Precision landing (camera -> LANDING_TARGET -> ArduPilot PLND_TYPE = 1)",
               f"ArduPilot was told to land {a['aimed_from_pad_m']} m from the pad, as a drifting GPS would.", "",
               "| | Landed from the pad | Camera frames | Marker seen | LANDING_TARGET sent |", "|---|---|---|---|---|",
               f"| Without the companion | {a['landed_from_pad_m']} m | – | – | – |",
               f"| With the companion | {b['landed_from_pad_m']} m | {b['companion'].get('frames', 0)} | "
               f"{b['companion'].get('detections', 0)} | {b['companion'].get('landing_target_sent', 0)} |", ""]
    if "avoid" in res:
        a, b = res["avoid"]["without"], res["avoid"]["with"]
        md += ["## Obstacle avoidance (lidar -> OBSTACLE_DISTANCE -> ArduPilot BendyRuler, OA_TYPE = 1)",
               "An AUTO mission straight to a point 24 m north, through a 10 m wide wall 11 m away that ArduPilot doesn't know about.", "",
               "| | Through the wall? | Closest to the wall | Path length | Time |", "|---|---|---|---|---|",
               f"| Without the companion | {'**yes** (a real drone would crash)' if a['flew_through_wall'] else 'no'} | "
               f"{a['min_clearance_m']} m | {a['path_length_m']} m | {a['mission_time_s']} s |",
               f"| With the companion | {'yes' if b['flew_through_wall'] else '**no**: went round'} | {b['min_clearance_m']} m | "
               f"{b['path_length_m']} m | {b['mission_time_s']} s |", ""]
    md += ["Safety check seen on the way: with `PRX1_TYPE = 2` set, ArduPilot refuses to arm until the companion's "
           "lidar is sending (`PreArm: PRX1: No Data`).", "",
           "What would change on a real drone: only the two simulated sensor drivers (`SimCamera`, `SimLidar`), "
           "which read SITL's truth. See docs/learn/23-companion-computer.md."]
    (RESULTS / "companion.md").write_text("\n".join(md) + "\n")
    return RESULTS / "companion.md"


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--job", choices=[*JOBS, "all"], default="all")
    ap.add_argument("--speedup", type=float, default=1.0)
    ap.add_argument("--write", action="store_true")
    args = ap.parse_args(argv)
    jobs = list(JOBS) if args.job == "all" else [args.job]
    res = {j: run_job(j, args.speedup) for j in jobs}
    for line in summary_lines(res):
        print(line)
    if args.write:
        print(f"Wrote {write_results(res)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
