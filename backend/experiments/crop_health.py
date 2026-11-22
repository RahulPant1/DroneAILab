"""Experiment 10: Crop-health map. Survey a field with an NDVI camera, stitch the photos, find the sick patches."""

from __future__ import annotations

import numpy as np

from backend.autonomy.cropmap import STRESS_NDVI, build_map, remember, rule_report
from backend.autonomy.plan import PlanError, compile_plan, run_plan
from backend.autonomy.site import FARM
from backend.sim.environment import EnvConfig
from backend.sim.runner import RunResult
from backend.sim.sensors import SensorConfig

from .base import Experiment, ExperimentError, Metric, Param, Plot, Series, plots_to_dicts, try_next


def map_frame(m: dict, caption: str) -> dict:
    """The stitched map as an 'images' frame: NDVI colours, true patches (dashed circles), findings (boxes)."""
    x0, _, _, y1 = m["bounds"]
    cell = m["cell_m"]
    marks = [{"kind": "circle", "x": (t["x_m"] - x0) / cell, "y": (y1 - t["y_m"]) / cell,
              "r": (t["area_m2"] / np.pi) ** 0.5 / cell, "label": "truth"} for t in m["truth"]]
    marks += [{"kind": "box", "box": f["box"], "label": f"{i + 1}"} for i, f in enumerate(m["findings"])]
    return {"image": m["image"], "w": m["w"], "h": m["h"], "colormap": "ndvi", "marks": marks, "caption": caption,
            "alt_m": 0, "t_s": 0, "detection": None}


def run(params: dict, seed: int) -> RunResult:
    field = params["field"]
    spec = {"tasks": [{"type": "survey", "field": field}, {"type": "land", "place": "home", "precision": False}],
            "altitude_m": params["altitude_m"], "side_overlap_pct": 70}
    try:
        plan = compile_plan(spec, FARM)
    except PlanError as exc:
        raise ExperimentError(str(exc)) from None
    fails = [c["detail"] for c in plan.checks if c["status"] == "fail"]
    if fails:
        raise ExperimentError("This survey isn't safe to fly: " + " ".join(fails))
    cond = params["conditions"]
    env = EnvConfig(wind_speed_m_s=6.0, wind_direction_deg=90.0, gust_std_m_s=1.2) if cond == "windy" else None
    sensors = SensorConfig(enabled=True, gps_drift_m=1.5) if cond != "calm" else None
    r = run_plan(plan, FARM, env=env, sensors=sensors, seed=seed)
    sv = plan.surveys[0]
    m = build_map(field, sv["polygon"], r.scenario["photos"], seed=seed)
    m["id"] = remember(m)
    found_err = [t["position_error_m"] for t in m["truth"] if t["found"]]
    r.metrics.update({
        "patches_true": len(m["truth"]), "patches_found": m["found"], "false_alarms": m["false_alarms"],
        "position_error_m": float(np.mean(found_err)) if found_err else None, "field_ndvi": m["field_ndvi"],
        "stressed_m2": m["stressed_m2"], "geotag_error_m": m["geotag_error_m"], "mapped_pct": m["mapped_pct"],
        "gsd_cm": sv["gsd_cm"],
    })
    r.scenario["cropmap"] = {k: v for k, v in m.items() if k != "image"}
    r.summary = summarise(params, m) + rule_report(m)
    r.try_next = suggestions(params)
    r.plots = plots_to_dicts(
        Plot("ndvi", "The stitched crop-health (NDVI) map", "",
             f"Each photo pasted where its geotag says. Green = healthy, yellow-brown = stressed (NDVI below {STRESS_NDVI}), "
             "grey = bare soil. Dashed circles = where the problems REALLY are; numbered boxes = what was found. "
             "North is up.", (), kind="images", extra={"frames": [map_frame(m, f"{field}: {m['photos']} photos")]}),
        Plot("map", "The survey flight", "north y (m)",
             "Survey lines (dashed) and where the drone really flew; each rectangle is one photo's footprint.",
             (Series("x_m", "flown path"), Series("y_m", "flown path")), kind="map",
             extra={"fields": [sv["polygon"]], "planned_route": r.scenario["plan"]["route"], "waypoints": [[0, 0]],
                    "photos": [[p["x_m"], p["y_m"], p["alt_m"], p["angle_rad"]] for p in r.scenario["photos"]],
                    "footprint_per_m": [sv["footprint_m"][0] / sv["alt_m"], sv["footprint_m"][1] / sv["alt_m"]],
                    "obstacles": [], "safety_zone_m": 0.7}),
    )  # fmt: skip
    return r


def summarise(p: dict, m: dict) -> list[str]:
    lines = [f"Found {m['found']} of the {len(m['truth'])} problem patches hidden in the {p['field']}"
             + (f", plus {m['false_alarms']} false alarm(s)." if m["false_alarms"] else ", with no false alarms.")]
    for t in m["truth"]:
        if t["found"]:
            lines.append(f"Truth: {t['cause']} ({t['area_m2']:.0f} m²): found {t['position_error_m']:.2f} m from where it "
                         f"really is, measured as {t['found_area_m2']:.0f} m².")
        else:
            lines.append(f"Truth: {t['cause']} ({t['area_m2']:.0f} m²): MISSED.")
    if p["conditions"] != "calm":
        lines.append(f"The photos' geotags were {m['geotag_error_m']:.2f} m off on average (GPS drift), so the whole map, and "
                     "every patch on it, sits about that far off. Good enough to walk to; for centimetres, surveys use RTK "
                     "GPS or ground control points.")
    return lines


def suggestions(p: dict) -> list[dict]:
    out = []
    if p["conditions"] == "calm":
        out.append(try_next("Real GPS (1.5 m drift): how far off does the map end up?", conditions="real"))
    if p["altitude_m"] < 35:
        out.append(try_next("Fly at 40 m: fewer photos. Are the small patches still found?", altitude_m=40.0))
    if p["field"] != "east field":
        out.append(try_next("Survey the east field (a waterlogged dip).", field="east field"))
    if p["field"] != "orchard":
        out.append(try_next("The orchard: one small patch of diseased trees.", field="orchard"))
    return out[:4]


EXPERIMENT = Experiment(
    id="crop_health",
    number=10,
    title="Crop-health map (NDVI)",
    objective="Survey a field with a plant-health camera, stitch the photos into one map, and find the problem patches.",
    concept=(
        "Healthy plants reflect lots of near-infrared light and absorb red: NDVI = (NIR − red) / (NIR + red) is high (~0.8).",
        "Stitching pastes each photo where its geotag (the drone's own position estimate) says: GPS error shifts the map.",
        "Low-NDVI cells that touch form patches: where, how big, how bad. Someone still has to walk there to find the cause.",
    ),
    params=(
        Param("field", "Field", "", "north field", "Which field to survey.", options=tuple((f.name, f.name) for f in FARM.fields)),
        Param("altitude_m", "Altitude", "m", 20.0, "Survey height (higher = fewer photos, less detail).", 10.0, 50.0, 1.0),
        Param("conditions", "Conditions", "", "calm", "Calm air and perfect GPS, or the real world.",
              options=(("calm", "Calm, perfect GPS"), ("real", "Real GPS (noise + 1.5 m drift)"),
                       ("windy", "Gusty 6 m/s crosswind + real GPS"))),
    ),
    key_metrics=(
        Metric("patches_found", "Patches found", "", 0),
        Metric("patches_true", "Patches really there", "", 0),
        Metric("false_alarms", "False alarms", "", 0),
        Metric("position_error_m", "Patch position error", "m", 2),
        Metric("field_ndvi", "Field average NDVI", "", 2),
        Metric("photos", "Photos", "", 0),
    ),
    learn_slug="20-crop-health",
    runner=run,
)
