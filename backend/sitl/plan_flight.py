"""Fly a DroneLab flight plan (a survey, a delivery) on REAL ArduPilot, and compare it with DroneLab's own flight.

The same compiled plan (autonomy/plan.py) is uploaded to ArduCopter SITL as an AUTO mission, exactly as a ground
station would. Photos are triggered like ArduPilot's CAM_TRIGG_DIST: on a survey line, every `photo spacing`
metres measured by ArduPilot's OWN position (its EKF); each photo is placed where SITL says the drone REALLY was
(SIMSTATE). Coverage is then measured with the same code as DroneLab's.

    .venv/bin/python -m backend.sitl.plan_flight                       # north field survey, both simulators
    .venv/bin/python -m backend.sitl.plan_flight --field orchard --wind 6
    .venv/bin/python -m backend.sitl.plan_flight --write               # also save docs/results/survey-compare.*

ArduPilot's own pad sits PAD_OFFSET_M from DroneLab's home (the 3D view shows them side by side); the plan's
coordinates are converted so it flies over the same fields.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Callable

from backend.autonomy.plan import CompiledPlan, compile_plan, run_plan
from backend.autonomy.site import FARM
from backend.autonomy.survey import CAMERA, SurveyCamera, coverage
from backend.sim.environment import EnvConfig

from .fly import Flight, offset_latlon

PAD_OFFSET_M = (0.0, -4.0)  # same as bridge.PAD_OFFSET_M (ArduPilot's pad, in DroneLab's frame)
SAMPLE_HZ = 5.0
RESULTS = Path(__file__).resolve().parents[2] / "docs" / "results"


def wind_params(speed_m_s: float, toward_deg: float) -> dict[str, float]:
    """DroneLab's wind blows TOWARD an angle (0 = east, 90 = north, counter-clockwise). ArduPilot's SIM_WIND_DIR
    is the compass direction it comes FROM (0 = north, clockwise)."""
    return {"SIM_WIND_SPD": speed_m_s, "SIM_WIND_DIR": (270.0 - toward_deg) % 360.0, "SIM_WIND_TURB": 0.0}


def fly_plan(f: Flight, plan: CompiledPlan, say: Callable[[str], None] = print, speedup: float = 1.0,
             wind: tuple[float, float] | None = None) -> dict:
    """Fly a compiled plan on a connected, ready ArduPilot (on the ground or flying). Returns what happened."""
    alt, speed = plan.spec["altitude_m"], plan.spec["speed_m_s"]
    t_launch = f.t_s  # flight time is counted from here (before take-off), like DroneLab's
    f.set_param("WP_SPD", speed)  # ArduCopter 4.7: waypoint speed in m/s (was WPNAV_SPEED in cm/s)
    if wind:
        for k, v in wind_params(*wind).items():
            f.set_param(k, v)
    if not (f.armed and f.pos["rel_alt_m"] > 0.5):
        f.set_mode("GUIDED")
        say("Arming (pre-arm checks)...")
        f.arm_when_ready()
        say(f"Taking off to {alt:.0f} m...")
        f.takeoff(alt)
    assert f.home is not None
    ox, oy = PAD_OFFSET_M
    route = plan.route[1:]  # route[0] = take-off point
    # Which route points are plan waypoints (A* adds corner points between them)?
    wp_of_route: list[int | None] = []
    wi = 1
    for p in route:
        if wi < len(plan.waypoints) and math.dist(p, plan.waypoints[wi]) < 1e-6:
            wp_of_route.append(wi)
            wi += 1
        else:
            wp_of_route.append(None)
    points = []
    for x, y in route:
        lat, lon = offset_latlon(*f.home, y - oy, x - ox)  # site frame -> metres from ArduPilot's home
        points.append((lat, lon, alt))
    f.upload_mission(points)
    land = plan.land
    camera = SurveyCamera(plan.photo_spacing_m, plan.camera_lines) if plan.camera_lines else None
    track: list[list[float]] = []
    with f.lock:
        f.reached.clear()
    f.set_mode("AUTO")
    t0 = f.t_s
    say(f"Flying the plan in AUTO: {len(points)} mission items...")
    last_item = len(points) + 1  # mission item numbers: 0 home, 1 take-off, 2.. = points
    next_sample = 0.0
    deadline = time.monotonic() + (plan.estimate["time_s"] * 3 + 120) / speedup * f.k
    while True:
        if time.monotonic() > deadline:
            raise TimeoutError("The plan didn't finish in time")
        with f.lock:
            seq, reached, t = f.mission_seq, list(f.reached), f.t_s - t0
            est = f.local_enu()
            true = f.truth_enu()
        if est is None or true is None:
            f.sleep(0.02)
            continue
        site_est = (est[0] + ox, est[1] + oy)
        site_true = (true[0] + ox, true[1] + oy, est[2])
        if camera:
            k = seq - 2
            line = wp_of_route[k] if 0 <= k < len(wp_of_route) and wp_of_route[k] in plan.camera_lines else None
            camera.update(t, line, site_est, site_true)
        if t >= next_sample:
            track.append([round(t, 2), round(site_true[0], 2), round(site_true[1], 2), round(est[2], 2)])
            next_sample = t + 1.0 / SAMPLE_HZ
        if last_item in reached:
            break
        f.sleep(0.05 / max(speedup, 1.0))
    t_mission = f.t_s - t0
    if land["place"] == "home":
        say("Plan done: RTL (fly home and land)...")
        f.set_mode("RTL")
    else:
        say(f"Plan done: landing on {land['place']} (LAND, on GPS)...")
        f.set_mode("LAND")
    while f.armed:
        with f.lock:
            true, est = f.truth_enu(), f.local_enu()
            t = f.t_s - t0
        if true and est and t >= next_sample:
            track.append([round(t, 2), round(true[0] + ox, 2), round(true[1] + oy, 2), round(est[2], 2)])
            next_sample = t + 1.0 / SAMPLE_HZ
        if time.monotonic() > deadline:
            raise TimeoutError("It didn't land in time")
        f.sleep(0.1 / max(speedup, 1.0))
    with f.lock:
        true = f.truth_enu()
    pad_xy = (land["x_m"], land["y_m"]) if land["place"] != "home" else PAD_OFFSET_M
    landed_at = (true[0] + ox, true[1] + oy) if true else None
    photos = camera.photos if camera else []
    per_field = []
    for sv in plan.surveys:
        shots = [p for p in photos if p["line"] is not None and plan.labels[p["line"]].startswith(sv["field"] + " ")]
        per_field.append({"field": sv["field"], "photos": len(shots), **coverage(sv["polygon"], shots, CAMERA)})
    say("Landed.")
    return {
        "mission_time_s": round(t_mission, 1),
        "flight_time_s": round(f.t_s - t_launch, 1),
        "photos": photos,
        "coverage": per_field,
        "coverage_pct": min((c["coverage_pct"] for c in per_field), default=None),
        "landing_error_m": round(math.dist(landed_at, pad_xy), 2) if landed_at else None,
        "track": track,
    }


def compare(field: str = "north field", alt_m: float = 20.0, side_overlap_pct: float = 70.0, wind_m_s: float = 0.0,
            speedup: float = 4.0, say: Callable[[str], None] = print) -> dict:
    """Fly the same survey on DroneLab (headless) and on a fresh ArduCopter SITL. Returns both results."""
    from . import launch

    plan = compile_plan({"tasks": [{"type": "survey", "field": field}, {"type": "land", "place": "home", "precision": False}],
                         "altitude_m": alt_m, "side_overlap_pct": side_overlap_pct})
    wind = (wind_m_s, 90.0) if wind_m_s > 0 else None
    env = EnvConfig(wind_speed_m_s=wind_m_s, wind_direction_deg=90.0) if wind else None
    say(f"DroneLab: flying the {field} survey...")
    ours = run_plan(plan, env=env)
    dl = {"coverage_pct": ours.metrics["coverage_pct"], "photos": ours.metrics["photos"],
          "flight_time_s": round(ours.metrics["flight_time_s"], 1), "landing_error_m": round(ours.metrics["landing_error_m"], 2),
          "track": [[t, x, y, z] for t, x, y, z in zip(ours.telemetry["t_s"][::2], ours.telemetry["x_m"][::2],
                                                         ours.telemetry["y_m"][::2], ours.telemetry["z_m"][::2])],
          "photos_xy": [[p["x_m"], p["y_m"]] for p in ours.scenario["photos"]]}
    say(f"DroneLab: {dl['coverage_pct']} % covered, {dl['photos']} photos, {dl['flight_time_s']} s.")
    sitl = launch.start(speedup=speedup)
    try:
        f = Flight(sitl.url, log=lambda _t: None, timeout_scale=max(1.0, 1.0 / speedup))
        try:
            say("ArduPilot: waiting for GPS + EKF...")
            f.wait_for_home(90)
            f.wait_until("EKF position", lambda: f.ekf_ok, 90)
            ap = fly_plan(f, plan, say=lambda t: say("ArduPilot: " + t), speedup=speedup, wind=wind)
        finally:
            f.close()
    finally:
        sitl.stop()
    ap["photos_xy"] = [[p["x_m"], p["y_m"]] for p in ap.pop("photos")]
    say(f"ArduPilot: {ap['coverage_pct']} % covered, {len(ap['photos_xy'])} photos, {ap['flight_time_s']} s.")
    return {"field": field, "alt_m": alt_m, "side_overlap_pct": side_overlap_pct, "wind_m_s": wind_m_s,
            "plan": {"route": plan.to_dict()["route"], "polygon": plan.surveys[0]["polygon"], "lines": plan.surveys[0]["lines"],
                     "photos_estimate": plan.estimate["photos"], "time_estimate_s": plan.estimate["time_s"]},
            "dronelab": dl, "ardupilot": ap}


def write_results(res: dict) -> Path:
    RESULTS.mkdir(parents=True, exist_ok=True)
    (RESULTS / "survey-compare.json").write_text(json.dumps(res, separators=(",", ":")))
    d, a = res["dronelab"], res["ardupilot"]
    md = [
        f"# Same survey, two autopilots: {res['field']} at {res['alt_m']:.0f} m",
        "",
        f"Plan: {len(res['plan']['lines'])} survey lines, {res['side_overlap_pct']:.0f} % side overlap, about "
        f"{res['plan']['photos_estimate']} photos; wind {res['wind_m_s']:.0f} m/s. Generated by "
        "`python -m backend.sitl.plan_flight --write`.",
        "",
        "| | DroneLab (our controller) | ArduPilot (ArduCopter 4.7 SITL, AUTO) |",
        "|---|---|---|",
        f"| Field covered | {d['coverage_pct']} % | {a['coverage_pct']} % |",
        f"| Photos | {d['photos']} | {len(a['photos_xy'])} |",
        f"| Flight time (take-off to landed) | {d['flight_time_s']:.0f} s | {a['flight_time_s']:.0f} s |",
        f"| Landed from its pad | {d['landing_error_m']:.2f} m | {a['landing_error_m']:.2f} m |",
        "",
        "Photos are triggered every few metres of travel measured by each autopilot's own position estimate, and placed "
        "where the simulator says the drone really was. Coverage uses the same code for both.",
    ]
    path = RESULTS / "survey-compare.md"
    path.write_text("\n".join(md) + "\n")
    return path


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--field", default="north field", choices=[f.name for f in FARM.fields])
    ap.add_argument("--alt", type=float, default=20.0)
    ap.add_argument("--overlap", type=float, default=70.0)
    ap.add_argument("--wind", type=float, default=0.0, help="wind toward north, m/s")
    ap.add_argument("--speedup", type=float, default=4.0)
    ap.add_argument("--write", action="store_true", help="save docs/results/survey-compare.json/.md")
    args = ap.parse_args(argv)
    res = compare(args.field, args.alt, args.overlap, args.wind, args.speedup)
    if args.write:
        print(f"Wrote {write_results(res)}")
    return 0 if (res["ardupilot"]["coverage_pct"] or 0) > 90 else 1


if __name__ == "__main__":
    sys.exit(main())
