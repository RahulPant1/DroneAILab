"""Experiment 9: Precision landing. Land on a pad by GPS alone, or with a down camera that finds its marker."""

from __future__ import annotations

import numpy as np

from backend.autonomy.plan import compile_plan, run_plan
from backend.autonomy.site import FARM
from backend.sim.environment import EnvConfig
from backend.sim.runner import RunResult
from backend.sim.sensors import SensorConfig

from .base import Experiment, Metric, Param, Plot, Series, add_channel, ch, fmt, plots_to_dicts, try_next

PAD = "pad B"
WIND_DIRECTION_DEG = 30.0


def run(params: dict, seed: int) -> RunResult:
    plan = compile_plan({"tasks": [{"type": "land", "place": PAD, "precision": params["method"] == "vision"}],
                         "altitude_m": params["altitude_m"]}, FARM)
    wind = params["wind_speed_m_s"]
    env = EnvConfig(wind_speed_m_s=wind, wind_direction_deg=WIND_DIRECTION_DEG, gust_std_m_s=0.2 * wind) if wind > 0 else None
    sensors = SensorConfig(enabled=True, gps_drift_m=params["gps_drift_m"])
    r = run_plan(plan, FARM, env=env, sensors=sensors, seed=seed, lander_mode=params["method"])
    pad = FARM.pad(PAD)
    add_channel(r, "pad_distance_m", np.hypot(ch(r, "x_m") - pad.x_m, ch(r, "y_m") - pad.y_m))
    add_channel(r, "gps_error_m", np.hypot(ch(r, "x_est_m") - ch(r, "x_m"), ch(r, "y_est_m") - ch(r, "y_m")))
    frames = r.scenario.get("camera_frames", [])
    r.summary = summarise(params, r)
    r.try_next = suggestions(params, r)
    near = [i for i, d in enumerate(r.telemetry["pad_distance_m"]) if d < 6]
    r.plots = plots_to_dicts(
        Plot("map", "Top-down near the pad", "north y (m)",
             "The square = the pad (the black ring is the marker the camera looks for). Solid = where the drone really "
             "flew; dashed = where it THOUGHT it was (its GPS-based estimate).",
             (Series("x_m", "flown path"), Series("y_m", "flown path")), kind="map",
             extra={"pads": [[pad.x_m, pad.y_m, pad.size_m]], "estimate": [r.telemetry["x_est_m"], r.telemetry["y_est_m"]],
                    "window": [pad.x_m - 5, pad.x_m + 5, pad.y_m - 5, pad.y_m + 5], "first_index": near[0] if near else 0}),
        Plot("pad_distance", "How far from the pad centre", "distance (m)",
             "True horizontal distance from the drone to the pad centre, and how wrong its GPS-based position is. "
             "GPS-only landing ends up about as far off as the GPS error.",
             (Series("pad_distance_m", "distance to pad"), Series("gps_error_m", "GPS error", style="dashed"))),
        Plot("alt", "Altitude", "altitude (m)",
             "Fly over the pad, then descend. With the camera: it slows down or stops to search when the marker isn't "
             "centred or not in view.",
             (Series("z_m", "altitude"), Series("alt_setpoint_m", "target", style="dashed"))),
        Plot("camera", "What the down camera saw", "",
             "Pictures from the camera on the way down (128 × 96 pixels). The box marks the detected marker. "
             + ("GPS-only landing ignores them." if params["method"] == "gps" else ""),
             (), kind="images", extra={"frames": frames}),
    )  # fmt: skip
    return r


def summarise(p: dict, r: RunResult) -> list[str]:
    m = r.metrics
    lines = []
    if m["landing_error_m"] is None:
        return ["It didn't land: see the events."]
    gps_err = r.telemetry["gps_error_m"][-1]
    if p["method"] == "vision":
        lines.append(f"Camera landing: touched down {m['landing_error_m']:.2f} m from the pad centre (the marker is 1 m wide).")
        if m["first_correction_m"] is not None:
            lines.append(f"It first saw the marker from {fmt(r.scenario.get('first_fix_alt_m'), 1)} m up and found the pad "
                         f"{m['first_correction_m']:.2f} m from where GPS said it was, then steered onto it while descending.")
        lines.append("The camera measures where the pad is RELATIVE to the drone, so the GPS error cancels out.")
    else:
        lines.append(f"GPS-only landing: touched down {m['landing_error_m']:.2f} m from the pad centre. At touch-down its "
                     f"GPS was {gps_err:.2f} m off.")
        lines.append("It landed exactly where it THOUGHT the pad was. GPS error = landing error.")
    if p["gps_drift_m"] == 0:
        lines.append("GPS drift is off (only fast jitter): try a real-world 1-2 m drift.")
    if p["wind_speed_m_s"] > 0:
        lines.append(f"Wind {p['wind_speed_m_s']:.0f} m/s with gusts pushed it around on the way down.")
    return lines


def suggestions(p: dict, r: RunResult) -> list[dict]:
    out = []
    if p["method"] == "gps":
        out.append(try_next("Same landing with the camera.", method="vision"))
    else:
        out.append(try_next("Same landing with GPS only: how far off is it?", method="gps"))
    if p["wind_speed_m_s"] < 5:
        out.append(try_next("Add a gusty 6 m/s wind.", wind_speed_m_s=6.0))
    if p["gps_drift_m"] < 2.5:
        out.append(try_next("Bad GPS day: 3 m of drift. Does the camera still find the pad?", gps_drift_m=3.0))
    out.append(try_next("Another seed = another GPS error. Run it a few times.", ))
    return out[:4]


EXPERIMENT = Experiment(
    id="precision_landing",
    number=9,
    title="Precision landing (camera)",
    objective="Land on a 1 m pad: by GPS alone, or with a down camera that finds the pad's marker.",
    concept=(
        "GPS is often 1-2 m off, and that error wanders slowly: landing on a GPS coordinate misses by that much.",
        "A down camera sees the marker and measures where the pad is relative to the drone, so the GPS error cancels.",
        "Detect the marker in the picture, turn that pixel into a direction, stretch it to the ground: that's the target.",
    ),
    params=(
        Param("method", "Landing", "", "vision", "How it finds the pad.", options=(("vision", "Camera (marker)"), ("gps", "GPS only"))),
        Param("gps_drift_m", "GPS drift", "m", 1.5, "Slowly wandering GPS error (1 standard deviation).", 0.0, 3.0, 0.25),
        Param("wind_speed_m_s", "Wind", "m/s", 0.0, "Steady wind with gusts.", 0.0, 8.0, 0.5),
        Param("altitude_m", "Approach height", "m", 10.0, "Height it flies over to the pad at, before descending.", 5.0, 25.0, 1.0),
    ),
    key_metrics=(
        Metric("landing_error_m", "Landed from pad centre", "m", 2),
        Metric("first_correction_m", "Pad vs GPS position", "m", 2),
        Metric("marker_sightings", "Marker sightings", "", 0),
        Metric("flight_time_s", "Flight time", "s", 1),
    ),
    learn_slug="18-precision-landing",
    runner=run,
)
