"""Drone Builder: pick real-world-style parts, get the numbers that decide whether it flies (and for how long),
then fly it in the simulator.

How the numbers are worked out:
  - mass: frame + 4 motors + 4 props + battery + electronics + payload.
  - max thrust: each motor's rated thrust on its prop at its rated cell count. On fewer cells the motor spins
    slower (speed ~ Kv x volts) and thrust goes with speed squared, so thrust x (cells / rated cells)^2.
  - hover power (momentum theory): pushing air down through the prop disc of area A takes at least
    P = T^1.5 / sqrt(2 * rho * A) per motor; real props reach ~60 % of that ideal ("figure of merit").
    Bigger props move more air more slowly: much less power for the same thrust.
  - electrical power = shaft power / motor+ESC efficiency + avionics; current = power / voltage.
  - flight time = usable charge / hover current. "Usable" = down to the low-battery point (~25 % left).
The simulator's motor model is then set so that in hover it burns exactly this power, so the estimate and a
real simulated flight agree.

Rules of thumb it checks (what builders actually use):
  - thrust-to-weight (T/W) at least 2: below that there's little margin for wind and manoeuvres.
  - hover throttle under ~60 %: above it the drone feels sluggish and the battery runs down fast.
  - full-throttle current within the battery's C rating, prop fits the frame, motor not over its rated voltage.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .battery import CELL_NOMINAL_V, BatteryConfig
from .config import GRAVITY_M_S2, DroneConfig, require_valid

AIR_DENSITY = 1.225  # kg/m^3 at sea level
MOTOR_EFFICIENCY = 0.75  # electrical -> shaft (motor + ESC)
LOADED_SPEED_FRACTION = 0.75  # a loaded motor reaches ~75 % of Kv x volts
USABLE_FRACTION = 0.75  # fly down to ~25 % left (the low-battery point)
LIPO_WH_PER_KG = 160.0
INCH = 0.0254


@dataclass(frozen=True)
class Frame:
    title: str
    size_mm: int  # motor-to-motor diagonal
    mass_kg: float
    max_prop_in: float
    electronics_kg: float  # flight controller, ESCs, receiver, GPS, wiring


@dataclass(frozen=True)
class Motor:
    title: str
    mass_kg: float
    kv: int
    rated_cells: int  # the cell count its thrust figure is for
    max_cells: int  # above this it overheats / over-revs
    thrust_g: dict  # prop size (inches) -> max thrust per motor in grams at rated_cells


@dataclass(frozen=True)
class Prop:
    title: str
    diameter_in: float
    mass_kg: float
    time_constant_s: float  # bigger props spin up more slowly
    figure_of_merit: float  # how close to ideal momentum theory it gets: small props are less efficient


FRAMES = {
    "micro_180": Frame("180 mm micro (3\" props)", 180, 0.050, 3.0, 0.035),
    "racer_250": Frame("250 mm racer / freestyle (5\" props)", 250, 0.120, 5.0, 0.045),
    "classic_450": Frame("450 mm classic (10\" props)", 450, 0.280, 10.0, 0.120),
    "big_650": Frame("650 mm heavy-lift (15\" props)", 650, 0.550, 15.0, 0.150),
}
MOTORS = {
    "1404_3800kv": Motor("1404 3800 Kv (micro)", 0.009, 3800, 4, 4, {3.0: 380}),
    "2306_2450kv": Motor("2306 2450 Kv (5\" freestyle)", 0.031, 2450, 4, 4, {5.0: 1250}),
    "2207_1950kv": Motor("2207 1950 Kv (5\" on 6S)", 0.033, 1950, 6, 6, {5.0: 1450}),
    "2212_920kv": Motor("2212 920 Kv (classic 450)", 0.058, 920, 3, 4, {10.0: 850}),
    "3510_360kv": Motor("3510 360 Kv (heavy-lift)", 0.105, 360, 6, 6, {15.0: 2300}),
}
PROPS = {
    "3in": Prop("3 inch", 3.0, 0.002, 0.02, 0.45),
    "5in": Prop("5 inch", 5.0, 0.004, 0.025, 0.50),
    "10in": Prop("10 inch", 10.0, 0.012, 0.045, 0.58),
    "15in": Prop("15 inch", 15.0, 0.030, 0.07, 0.62),
}
CELL_OPTIONS = (2, 3, 4, 6)
CAPACITY_OPTIONS = (450, 850, 1300, 1500, 2200, 3000, 4000, 5000, 8000, 10000)
MAX_PAYLOAD_KG = 1.0

DEFAULT_PARTS = {"frame": "classic_450", "motor": "2212_920kv", "prop": "10in", "cells": 3, "capacity_mah": 3000,
                 "payload_kg": 0.0}


def battery_for(cells: int, capacity_mah: float) -> BatteryConfig:
    """A typical LiPo pack: mass from energy density, internal resistance ~20 mΩ per cell per Ah."""
    ah = capacity_mah / 1000.0
    return BatteryConfig(
        cells=cells,
        capacity_mah=float(capacity_mah),
        internal_resistance_ohm=round(cells * 0.020 / ah, 4),
        max_c=75.0 if capacity_mah < 2000 else 50.0 if capacity_mah < 5000 else 25.0,  # racing packs are punchier
        mass_kg=round(cells * 3.7 * ah / LIPO_WH_PER_KG + 0.01, 3),
        efficiency=MOTOR_EFFICIENCY,
    )


def catalog() -> dict:
    return {
        "frames": {k: {"title": f.title, "size_mm": f.size_mm, "mass_g": round(f.mass_kg * 1000), "max_prop_in": f.max_prop_in}
                   for k, f in FRAMES.items()},
        "motors": {k: {"title": m.title, "mass_g": round(m.mass_kg * 1000), "kv": m.kv, "rated_cells": m.rated_cells,
                       "max_cells": m.max_cells,
                       "props_in": list(m.thrust_g), "thrust_g": {str(p): g for p, g in m.thrust_g.items()}}
                   for k, m in MOTORS.items()},
        "props": {k: {"title": p.title, "diameter_in": p.diameter_in} for k, p in PROPS.items()},
        "cells": list(CELL_OPTIONS),
        "capacities_mah": list(CAPACITY_OPTIONS),
        "max_payload_kg": MAX_PAYLOAD_KG,
        "default": DEFAULT_PARTS,
    }


class DesignError(ValueError):
    """Parts that can't be combined at all. The message is plain English."""


def hover_shaft_power_w(thrust_n: float, prop_diameter_m: float, figure_of_merit: float) -> float:
    """Momentum theory: ideal power to hold `thrust_n` with a prop of this size, divided by the figure of merit."""
    area = math.pi * (prop_diameter_m / 2.0) ** 2
    return thrust_n**1.5 / math.sqrt(2.0 * AIR_DENSITY * area) / figure_of_merit


def design(parts: dict) -> dict:
    """Work out a design. Returns {config, battery, report}; raises DesignError for impossible combinations."""
    p = {**DEFAULT_PARTS, **(parts or {})}
    try:
        frame, motor, prop = FRAMES[p["frame"]], MOTORS[p["motor"]], PROPS[p["prop"]]
        cells, capacity, payload = int(p["cells"]), float(p["capacity_mah"]), float(p["payload_kg"])
    except KeyError as exc:
        raise DesignError(f"Unknown part: {exc}. Pick from the lists.") from None
    except (TypeError, ValueError):
        raise DesignError("cells, capacity_mah and payload_kg must be numbers.") from None
    if cells not in CELL_OPTIONS or capacity not in CAPACITY_OPTIONS:
        raise DesignError(f"Battery must be {CELL_OPTIONS} cells and one of {CAPACITY_OPTIONS} mAh.")
    if not 0.0 <= payload <= MAX_PAYLOAD_KG:
        raise DesignError(f"Payload must be 0–{MAX_PAYLOAD_KG:g} kg.")
    if prop.diameter_in > frame.max_prop_in:
        raise DesignError(f"{prop.title} props don't fit a {frame.size_mm} mm frame (max {frame.max_prop_in:g}\"): they'd hit each other.")
    if prop.diameter_in not in motor.thrust_g:
        sizes = ", ".join(f'{s:g}"' for s in motor.thrust_g)
        raise DesignError(f"The {motor.title} motor is made for {sizes} props, not {prop.title}.")
    if cells > motor.max_cells:
        raise DesignError(f"The {motor.title} motor is rated for {motor.max_cells}S at most: {cells}S would spin it too fast and burn it out.")

    bat = battery_for(cells, capacity)
    mass_parts = {
        "frame": frame.mass_kg, "motors": 4 * motor.mass_kg, "props": 4 * prop.mass_kg,
        "battery": bat.mass_kg, "electronics": frame.electronics_kg, "payload": payload,
    }  # fmt: skip
    mass = sum(mass_parts.values())
    volt_scale = cells / motor.rated_cells
    thrust_n = motor.thrust_g[prop.diameter_in] / 1000.0 * GRAVITY_M_S2 * volt_scale**2  # per motor
    max_speed = motor.kv * cells * CELL_NOMINAL_V * LOADED_SPEED_FRACTION * 2 * math.pi / 60.0
    weight_n = mass * GRAVITY_M_S2
    t_hover = weight_n / 4.0
    tw = 4 * thrust_n / weight_n
    d = prop.diameter_in * INCH

    # The simulator's spin-drag torque per newton of thrust, chosen so hover burns the momentum-theory power.
    if tw > 1.0:
        omega_hover = max_speed * math.sqrt(t_hover / thrust_n)
        torque_per_thrust = hover_shaft_power_w(t_hover, d, prop.figure_of_merit) / (t_hover * omega_hover)
    else:
        torque_per_thrust = 0.016
    torque_per_thrust = min(max(torque_per_thrust, 0.005), 0.05)
    scale = (frame.size_mm / 450.0) ** 2
    cfg = DroneConfig(
        name=f"My build: {frame.size_mm} mm, {motor.title.split(' (')[0]}, {prop.title}, {cells}S {capacity:.0f} mAh",
        mass_kg=round(mass, 3),
        arm_length_m=frame.size_mm / 2000.0,
        motor_max_thrust_n=round(thrust_n, 3),
        motor_max_speed_rad_s=round(max_speed, 1),
        motor_time_constant_s=prop.time_constant_s,
        motor_torque_per_thrust_m=round(torque_per_thrust, 5),
        drag_linear_n_s_m=round(0.10 * scale, 4),
        drag_quadratic_n_s2_m2=round(0.02 * scale, 4),
        angular_drag_nm_s_rad=round(0.002 * scale, 5),
    )

    # Power and endurance
    hover_shaft_w = 4 * hover_shaft_power_w(t_hover, d, prop.figure_of_merit)
    hover_elec_w = hover_shaft_w / MOTOR_EFFICIENCY + bat.avionics_w
    v_nominal = cells * 3.7
    hover_a = hover_elec_w / v_nominal
    flight_min = bat.capacity_mah / 1000.0 * USABLE_FRACTION / hover_a * 60.0
    full_shaft_w = 4 * torque_per_thrust * thrust_n * max_speed  # all motors flat out: P = Q * omega
    full_a = (full_shaft_w / MOTOR_EFFICIENCY + bat.avionics_w) / v_nominal
    hover_pct = 100.0 / tw if tw > 0 else math.inf

    warnings, verdict = [], "good"
    if tw < 1.0:
        warnings.append(f"T/W {tw:.2f}: it can't even lift itself off the ground. Lighter parts, bigger motors or more cells.")
        verdict = "won't fly"
    elif tw < 1.6:
        warnings.append(f"T/W {tw:.2f}: it can hover, but has almost no margin to climb, brake or fight wind. Aim for 2 or more.")
        verdict = "risky"
    elif tw < 2.0:
        warnings.append(f"T/W {tw:.2f}: flyable but sluggish. 2 or more is the usual minimum.")
        verdict = "ok"
    elif tw > 7:
        warnings.append(f"T/W {tw:.1f}: very powerful (racing territory): twitchy for a beginner.")
    if 1.0 <= tw and hover_pct > 60:
        warnings.append(f"Hover needs {hover_pct:.0f} % throttle: little left for control, and the battery drains fast.")
    if full_a > bat.max_current_a:
        warnings.append(f"Full throttle draws ~{full_a:.0f} A but this pack is rated for {bat.max_current_a:.0f} A "
                        f"({bat.max_c:.0f}C): short bursts are OK, long ones sag the voltage and heat the pack.")
        if full_a > 2 * bat.max_current_a and verdict == "good":
            verdict = "risky"
    if tw >= 1.0 and flight_min < 3:
        warnings.append(f"Only ~{flight_min:.1f} min of flight: a bigger battery (or a lighter build) would help.")
    if cells < motor.rated_cells:
        warnings.append(f"The motor's thrust figure is for {motor.rated_cells}S; on {cells}S it gives {volt_scale**2 * 100:.0f} % of it.")
    elif cells > motor.rated_cells:
        warnings.append(f"On {cells}S (its thrust figure is for {motor.rated_cells}S) it's more powerful but runs hotter.")

    report = {
        "verdict": verdict,
        "mass_g": round(mass * 1000),
        "mass_breakdown_g": {k: round(v * 1000) for k, v in mass_parts.items()},
        "max_thrust_g": round(4 * thrust_n / GRAVITY_M_S2 * 1000),
        "thrust_to_weight": round(tw, 2),
        "hover_throttle_pct": round(hover_pct, 1) if math.isfinite(hover_pct) else None,
        "hover_power_w": round(hover_elec_w, 1),
        "hover_current_a": round(hover_a, 1),
        "full_throttle_current_a": round(full_a, 1),
        "battery_max_current_a": round(bat.max_current_a),
        "battery_energy_wh": round(bat.energy_wh, 1),
        "flight_time_min": round(flight_min, 1) if tw >= 1.0 else None,
        "grams_per_watt": round(t_hover * 4 / GRAVITY_M_S2 * 1000 / hover_elec_w, 2),
        "warnings": warnings,
    }
    if tw >= 1.0:
        require_valid(cfg)  # every design that can fly must also be a valid simulator config
    return {"config": cfg, "battery": bat, "report": report, "parts": p}
