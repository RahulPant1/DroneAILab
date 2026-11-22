"""Experiment 8: Survey a field. A lawnmower pattern, a camera footprint, overlap, and real coverage."""

from __future__ import annotations

from backend.autonomy.plan import PlanError, compile_plan, run_plan
from backend.autonomy.site import FARM
from backend.sim.battery import BatteryConfig
from backend.sim.environment import EnvConfig
from backend.sim.runner import RunResult
from backend.sim.sensors import SensorConfig

from .base import Experiment, ExperimentError, Metric, Param, Plot, Series, fmt, plots_to_dicts, try_next

WIND_DIRECTION_DEG = 90.0  # toward north: across the north field's east-west lines
WINDY_M_S = 6.0


def run(params: dict, seed: int) -> RunResult:
    spec = {"tasks": [{"type": "survey", "field": params["field"]}, {"type": "land", "place": "home", "precision": False}],
            "altitude_m": params["altitude_m"], "speed_m_s": 3.0, "side_overlap_pct": params["side_overlap_pct"]}
    try:
        plan = compile_plan(spec, FARM)
    except PlanError as exc:
        raise ExperimentError(str(exc)) from None
    fails = [c["detail"] for c in plan.checks if c["status"] == "fail"]
    if fails:
        raise ExperimentError("This survey isn't safe to fly: " + " ".join(fails))
    wind = WINDY_M_S if params["conditions"] == "windy" else 0.0
    env = EnvConfig(wind_speed_m_s=wind, wind_direction_deg=WIND_DIRECTION_DEG, gust_std_m_s=0.2 * wind) if wind > 0 else None
    sensors = SensorConfig(enabled=True, gps_drift_m=1.5) if params["conditions"] != "calm" else None
    r = run_plan(plan, FARM, env=env, sensors=sensors, seed=seed, battery=BatteryConfig())
    sv = plan.surveys[0]
    cov = r.scenario["coverage"][0]
    r.metrics.update({"gsd_cm": sv["gsd_cm"], "line_spacing_m": sv["line_spacing_m"], "photo_spacing_m": sv["photo_spacing_m"],
                      "survey_lines": len(sv["lines"]), "seen_twice_pct": cov["seen_twice_pct"],
                      "footprint_across_m": sv["footprint_m"][0]})
    r.summary = summarise(params, r, sv)
    r.try_next = suggestions(params, r)
    r.plots = plots_to_dicts(
        Plot("map", "Top-down: the field, the plan, the photos", "north y (m)",
             "Green outline = the field. Dashed yellow = the planned survey lines and route; solid = where the drone really "
             "flew. Each faint rectangle = one photo's footprint on the ground. Red dots = spots no photo covered (gaps).",
             (Series("x_m", "flown path"), Series("y_m", "flown path")), kind="map",
             extra={"obstacles": [b.box.to_dict() for b in FARM.buildings], "no_fly": [n.box.to_dict() for n in FARM.no_fly],
                    "fields": [sv["polygon"]], "planned_route": r.scenario["plan"]["route"], "waypoints": [[0, 0]],
                    "photos": [[p["x_m"], p["y_m"], p["alt_m"], p["angle_rad"]] for p in r.scenario["photos"]],
                    "footprint_per_m": [sv["footprint_m"][0] / sv["alt_m"], sv["footprint_m"][1] / sv["alt_m"]],
                    "gaps": cov["gaps"], "safety_zone_m": 0.7}),
        Plot("alt", "Altitude", "altitude (m)",
             "Climb to survey height, fly the lines, come home and land. Photo footprint size grows with height.",
             (Series("z_m", "altitude"), Series("alt_setpoint_m", "target", style="dashed"))),
        Plot("speed", "Ground speed", "speed (m/s)",
             "It slows down at each turn between lines: the photos are only taken on the straight lines.",
             (Series("ground_speed_m_s", "ground speed"),)),
    )  # fmt: skip
    return r


def summarise(p: dict, r: RunResult, sv: dict) -> list[str]:
    m = r.metrics
    across, along = sv["footprint_m"]
    lines = [
        f"From {sv['alt_m']:.0f} m each photo covers {across:.1f} × {along:.1f} m of ground at {sv['gsd_cm']:.2f} cm per pixel "
        f"(a 4000-pixel-wide camera, 70° across).",
        f"{p['side_overlap_pct']:.0f} % side overlap → lines {sv['line_spacing_m']:.1f} m apart ({len(sv['lines'])} lines); "
        f"75 % front overlap → a photo every {sv['photo_spacing_m']:.1f} m.",
    ]
    if m["completed"]:
        lines.append(f"It took {m['photos']} photos in {m['flight_time_s']:.0f} s and covered {fmt(m['coverage_pct'], 1)} % of "
                     f"the {p['field']}; {fmt(m['seen_twice_pct'], 1)} % was in at least two photos (what stitching software needs).")
    else:
        lines.append("The survey didn't finish: see the events.")
    if m["coverage_pct"] is not None and m["coverage_pct"] < 99.5:
        lines.append("Gaps! With little overlap there's no margin: a small push from wind or a GPS error leaves strips no photo saw.")
    if p["conditions"] == "windy":
        lines.append(f"The {WINDY_M_S:.0f} m/s gusty wind blew toward north, across the lines: the drone had to lean into it "
                     "and its path wobbled sideways.")
    if p["conditions"] != "calm":
        lines.append("With real GPS (noise and a slow 1.5 m drift) the photos land where the drone REALLY was, not where it "
                     "thought: overlap is the safety margin that hides that error.")
    if "battery_used_mah" in m:
        lines.append(f"Battery: {m['battery_used_mah']:.0f} mAh of 3000 ({100 - m['battery_left_pct']:.0f} %).")
    return lines


def suggestions(p: dict, r: RunResult) -> list[dict]:
    out = []
    if p["side_overlap_pct"] > 20:
        out.append(try_next("No side overlap, gusty wind and real GPS: do gaps appear?", side_overlap_pct=0.0,
                            conditions="windy", altitude_m=20.0))
    if p["altitude_m"] < 35:
        out.append(try_next("Fly higher (40 m): fewer lines and photos, but each pixel sees more ground.", altitude_m=40.0))
    if p["altitude_m"] > 12:
        out.append(try_next("Fly low (10 m) for sharper photos: how many more lines and photos does it take?", altitude_m=10.0))
    if p["conditions"] == "calm":
        out.append(try_next("Use real GPS (noise + drift).", conditions="real"))
    if p["field"] != "east field":
        out.append(try_next("The east field: a slanted shape. The lines follow its longest side.", field="east field"))
    return out[:4]


EXPERIMENT = Experiment(
    id="survey",
    number=8,
    title="Survey a field (mapping)",
    objective="Photograph a whole field from above, and measure how much of it the photos really cover.",
    concept=(
        "A camera pointing down sees a rectangle of ground; it grows with height (and each pixel sees more ground).",
        "Fly parallel lines (a lawnmower pattern); neighbouring photos overlap so mapping software can stitch them.",
        "Wind and GPS error move the photos: overlap is the safety margin that stops gaps.",
    ),
    params=(
        Param("field", "Field", "", "north field", "Which field to survey.", options=tuple((f.name, f.name) for f in FARM.fields)),
        Param("altitude_m", "Altitude", "m", 20.0, "Survey height above the ground.", 8.0, 50.0, 1.0),
        Param("side_overlap_pct", "Side overlap", "%", 70.0, "How much neighbouring lines' photos overlap.", 0.0, 80.0, 5.0),
        Param("conditions", "Conditions", "", "calm", "Calm air and perfect GPS, or the real world.",
              options=(("calm", "Calm, perfect GPS"), ("real", "Real GPS (noise + 1.5 m drift)"),
                       ("windy", "Gusty 6 m/s crosswind + real GPS"))),
    ),
    key_metrics=(
        Metric("coverage_pct", "Field covered", "%", 1),
        Metric("seen_twice_pct", "In 2+ photos", "%", 1),
        Metric("photos", "Photos", "", 0),
        Metric("gsd_cm", "Ground per pixel", "cm", 2),
        Metric("flight_time_s", "Flight time", "s", 0),
        Metric("battery_used_mah", "Battery used", "mAh", 0),
    ),
    learn_slug="17-survey",
    runner=run,
)
