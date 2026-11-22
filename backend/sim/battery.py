"""LiPo battery: how much energy is left, and how the voltage sags under load.

A drone battery is several LiPo cells in series ("4S" = 4 cells). Each cell's resting voltage depends on how full
it is: 4.2 V full, ~3.85 V in the middle, dropping steeply below ~3.5 V. Under load the voltage the motors see is
lower: V = resting voltage - current x internal resistance ("sag").

Power: the motors' shaft power (torque x speed, from physics.py) divided by the motor+ESC efficiency, plus a few
watts for the flight controller and radio. Current I = P / V, and charge drains as I x time.

Why it matters for flying: motor speed is roughly Kv x voltage, so as the battery sags, the motors' top speed
(and thrust) drops. A full battery gives a little MORE thrust than the motors' rating (taken at 3.85 V/cell), a
nearly empty one a lot less. Near empty, a drone that hovered easily may not be able to climb at all.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass

# Resting (no-load) voltage of one LiPo cell vs state of charge (0..1). A typical discharge curve.
OCV_SOC = (0.0, 0.05, 0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.00)
OCV_V = (3.27, 3.50, 3.61, 3.70, 3.75, 3.79, 3.83, 3.87, 3.92, 3.98, 4.07, 4.20)

CELL_NOMINAL_V = 3.85  # motor thrust ratings in this simulator are taken at this voltage per cell
LOW_CELL_V = 3.6  # under load: time to go home (~25 % left in hover). At 3.5 V only ~12 % is left: too late to fly far
CRITICAL_CELL_V = 3.4  # under load: land now
EMPTY_SOC = 0.0
LOW_SOC_ESTIMATE = 0.25  # roughly where LOW_CELL_V is reached in hover (for "time left")


@dataclass(frozen=True)
class BatteryConfig:
    cells: int = 3  # in series: 3S = 11.1 V nominal
    capacity_mah: float = 3000.0
    internal_resistance_ohm: float = 0.025  # whole pack; small packs and old packs are higher
    max_c: float = 50.0  # rated max continuous discharge: max_c x capacity (in A)
    mass_kg: float = 0.25  # only used by the Drone Builder (the config's mass already includes it)
    efficiency: float = 0.75  # motor + ESC: electrical power -> shaft power
    avionics_w: float = 4.0  # flight controller, GPS, radio...
    drain_speedup: float = 1.0  # >1 drains faster (demos), physics unchanged

    @property
    def nominal_v(self) -> float:
        return self.cells * CELL_NOMINAL_V

    @property
    def energy_wh(self) -> float:
        return self.capacity_mah / 1000.0 * self.cells * 3.7

    @property
    def max_current_a(self) -> float:
        return self.max_c * self.capacity_mah / 1000.0

    def to_dict(self) -> dict:
        return asdict(self) | {"nominal_v": self.nominal_v, "energy_wh": round(self.energy_wh, 1)}


BATTERY_LIMITS = {
    "cells": (1, 12),
    "capacity_mah": (100.0, 30000.0),
    "internal_resistance_ohm": (0.001, 1.0),
    "max_c": (1.0, 200.0),
    "efficiency": (0.3, 0.95),
    "avionics_w": (0.0, 50.0),
    "drain_speedup": (1.0, 10.0),  # faster than this, the trip home uses too much of the (sped-up) battery
}


def validate_battery(cfg: BatteryConfig) -> list[str]:
    out = []
    for name, (lo, hi) in BATTERY_LIMITS.items():
        v = getattr(cfg, name)
        if not isinstance(v, (int, float)) or not math.isfinite(v) or not lo <= v <= hi:
            out.append(f"battery {name} = {v!r} is outside {lo}–{hi}.")
    return out


def cell_ocv(soc: float) -> float:
    """Resting voltage of one cell at this state of charge (linear between the table points)."""
    soc = min(max(soc, 0.0), 1.0)
    for i in range(len(OCV_SOC) - 1):
        if soc <= OCV_SOC[i + 1]:
            f = (soc - OCV_SOC[i]) / (OCV_SOC[i + 1] - OCV_SOC[i])
            return OCV_V[i] + f * (OCV_V[i + 1] - OCV_V[i])
    return OCV_V[-1]


class Battery:
    """The battery's state during a flight. Call step() every physics step with the motors' shaft power."""

    def __init__(self, cfg: BatteryConfig, soc: float = 1.0):
        self.cfg = cfg
        self.soc = soc
        self.used_mah = 0.0
        self.current_a = 0.0
        self.power_w = cfg.avionics_w
        self.voltage_v = cell_ocv(soc) * cfg.cells
        self.min_voltage_v = self.voltage_v
        self.peak_current_a = 0.0

    @property
    def cell_v(self) -> float:
        return self.voltage_v / self.cfg.cells

    @property
    def voltage_scale(self) -> float:
        """Motor top speed relative to its rating (speed ~ Kv x volts). 1.0 = at 3.85 V per cell."""
        return self.voltage_v / self.cfg.nominal_v

    @property
    def status(self) -> str:
        if self.soc <= EMPTY_SOC:
            return "empty"
        if self.cell_v < CRITICAL_CELL_V:
            return "critical"
        if self.cell_v < LOW_CELL_V:
            return "low"
        return "ok"

    def step(self, shaft_power_w: float, dt_s: float) -> None:
        """Draw the power the motors need for dt seconds.

        Terminal voltage V solves V = Voc - (P / V) * R, i.e. V^2 - Voc*V + P*R = 0 (the bigger root). If the pack
        can't deliver that much power at all, it delivers its maximum (at V = Voc / 2) and the voltage collapses.
        """
        cfg = self.cfg
        p = max(shaft_power_w, 0.0) / cfg.efficiency + cfg.avionics_w
        voc = cell_ocv(self.soc) * cfg.cells if self.soc > 0 else 0.0
        r = cfg.internal_resistance_ohm
        disc = voc * voc - 4.0 * p * r
        v = (voc + math.sqrt(disc)) / 2.0 if disc > 0 else voc / 2.0
        i = p / v if v > 1e-6 else 0.0
        self.voltage_v, self.current_a, self.power_w = v, i, p
        self.min_voltage_v = min(self.min_voltage_v, v)
        self.peak_current_a = max(self.peak_current_a, i)
        used = i * dt_s / 3.6 * cfg.drain_speedup  # A·s -> mAh
        self.used_mah += used
        self.soc = max(0.0, self.soc - used / cfg.capacity_mah)

    def time_left_s(self) -> float | None:
        """At the current draw, how long until the LOW voltage point (~25 % left). None when not drawing."""
        if self.current_a < 0.5:
            return None
        usable_mah = max(0.0, (self.soc - LOW_SOC_ESTIMATE) * self.cfg.capacity_mah)
        return usable_mah / self.current_a * 3.6 / self.cfg.drain_speedup

    def snapshot(self) -> dict:
        t = self.time_left_s()
        return {
            "voltage_v": round(self.voltage_v, 2),
            "cell_v": round(self.cell_v, 3),
            "current_a": round(self.current_a, 1),
            "power_w": round(self.power_w, 1),
            "soc_pct": round(self.soc * 100, 1),
            "used_mah": round(self.used_mah),
            "capacity_mah": self.cfg.capacity_mah,
            "cells": self.cfg.cells,
            "time_left_s": None if t is None else round(t),
            "status": self.status,
            "drain_speedup": self.cfg.drain_speedup,
        }
