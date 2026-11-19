"""Run the simulator from the terminal, no browser needed.

Examples:
  .venv/bin/python -m backend.cli hover --alt 2
  .venv/bin/python -m backend.cli hover --alt 2 --mass-kg 2.5          # too heavy: T/W < 1
  .venv/bin/python -m backend.cli hover --alt 2 --alt-kd 0             # no damping: more overshoot
  .venv/bin/python -m backend.cli freefall --height 20
  .venv/bin/python -m backend.cli presets
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

from backend.sim.config import GRAVITY_M_S2, PRESETS, ConfigError, get_preset, validate_config
from backend.sim.runner import RunResult, run_free_fall, run_hover


def ascii_plot(t_s, values, target=None, width=64, height=14, label="alt (m)") -> str:
    """A small text plot of one signal over time. '*' = data, '-' = target line."""
    t_s, values = np.asarray(t_s), np.asarray(values)
    lo = min(values.min(), target if target is not None else values.min(), 0.0)
    hi = max(values.max(), target if target is not None else values.max())
    if hi - lo < 1e-9:
        hi = lo + 1.0
    hi += 0.05 * (hi - lo)
    cols = np.linspace(0, len(values) - 1, width).astype(int)

    def row_of(v: float) -> int:
        return int(round((hi - v) / (hi - lo) * (height - 1)))

    grid = [[" "] * width for _ in range(height)]
    if target is not None:
        r = row_of(target)
        for c in range(width):
            grid[r][c] = "-"
    for c, idx in enumerate(cols):
        grid[row_of(values[idx])][c] = "*"

    lines = [f"  {label}"]
    for r, row in enumerate(grid):
        if target is not None and r == row_of(target):
            axis_label = f"{target:6.2f}"
        elif r in (0, height - 1):
            axis_label = f"{hi - r * (hi - lo) / (height - 1):6.2f}"
        else:
            axis_label = " " * 6
        lines.append(f"{axis_label} |{''.join(row)}")
    lines.append(" " * 7 + "+" + "-" * width)
    lines.append(" " * 8 + f"0 s{'':{width - 12}}{t_s[-1]:5.1f} s")
    return "\n".join(lines)


def build_config(args):
    cfg = get_preset(args.preset)
    changes = {}
    if args.mass_kg is not None:
        changes["mass_kg"] = args.mass_kg
    gains = {k: getattr(args, k) for k in ("alt_kp", "alt_ki", "alt_kd") if getattr(args, k) is not None}
    if gains:
        changes["gains"] = gains
    if changes:
        cfg = cfg.with_changes(**changes)
    if args.thrust_to_weight is not None:
        cfg = cfg.with_changes(motor_max_thrust_n=args.thrust_to_weight * cfg.weight_n / 4.0)
    problems = validate_config(cfg)
    if problems:
        raise ConfigError("\n  ".join(["Invalid config:", *problems]))
    return cfg


def fmt(value, unit="", digits=3) -> str:
    if value is None:
        return "n/a"
    if isinstance(value, bool):
        return "yes" if value else "no"
    return f"{value:.{digits}f}{unit}"


def print_config(cfg) -> None:
    print(f"Drone: {cfg.name}  |  mass {cfg.mass_kg:.2f} kg  |  arm {cfg.arm_length_m:.3f} m")
    print(
        f"  weight {cfg.weight_n:.2f} N  |  max thrust {cfg.max_total_thrust_n:.2f} N  |  "
        f"T/W {cfg.thrust_to_weight:.2f}  |  expected hover {cfg.hover_throttle_pct:.1f} %"
        + ("  (over 100 %: it can't hover!)" if cfg.thrust_to_weight < 1.0 else "")
    )
    g = cfg.gains
    print(f"  altitude PID: Kp {g.alt_kp}  Ki {g.alt_ki}  Kd {g.alt_kd}")


def save_json(result: RunResult, path: str | None) -> None:
    if path:
        Path(path).write_text(result.to_json(indent=1))
        print(f"\nSaved full result to {path}")


def cmd_hover(args) -> int:
    cfg = build_config(args)
    print_config(cfg)
    start = time.perf_counter()
    result = run_hover(cfg, alt_m=args.alt, duration_s=args.duration, seed=args.seed)
    wall_s = time.perf_counter() - start
    m = result.metrics

    print(f"\nHover at {args.alt} m for {args.duration} s (simulated in {wall_s:.2f} s of CPU time)\n")
    rows = [
        ("left the ground", fmt(m["left_ground"])),
        ("peak altitude", fmt(m["peak_alt_m"], " m")),
        ("final altitude", fmt(m["final_alt_m"], " m")),
        ("rise time (10→90 %)", fmt(m["rise_time_s"], " s", 2)),
        ("overshoot", fmt(m["overshoot_pct"], " %", 2)),
        (f"settling time (±{m['settling_band_m']} m)", fmt(m["settling_time_s"], " s", 2)),
        (f"hold error after {m['hold_start_s']:.0f} s: RMS / max", f"{fmt(m['hold_rms_error_m'], ' m', 4)} / {fmt(m['hold_max_error_m'], ' m', 4)}"),
        ("hover motor %", fmt(m["hover_motor_pct"], " %", 1)),
        ("rotor energy", fmt(m["rotor_energy_j"], " J", 0)),
    ]  # fmt: skip
    for name, value in rows:
        print(f"  {name:<34} {value}")

    if result.events:
        print("\nEvents:")
        for e in result.events:
            print(f"  t={e['t_s']:6.2f} s  {e['message']}")
    print("\nWhat happened:")
    for line in result.summary:
        print(f"  - {line}")
    print()
    print(ascii_plot(result.channel("t_s"), result.channel("z_m"), target=args.alt))
    save_json(result, args.json)
    return 0


def cmd_freefall(args) -> int:
    cfg = get_preset(args.preset).with_changes(drag_linear_n_s_m=0.0, drag_quadratic_n_s2_m2=0.0)
    result = run_free_fall(cfg, start_alt_m=args.height, duration_s=args.duration)
    t, z = result.channel("t_s"), result.channel("z_m")
    print(f"Free fall from {args.height} m, motors off, no air drag. Theory: z = z0 - ½·g·t²\n")
    print(f"  {'t (s)':>6} {'sim z (m)':>10} {'theory z (m)':>13}")
    for k in np.linspace(0, len(t) - 1, 6).astype(int):
        theory = max(0.0, args.height - 0.5 * GRAVITY_M_S2 * t[k] ** 2)
        print(f"  {t[k]:6.2f} {z[k]:10.3f} {theory:13.3f}")
    for e in result.events:
        print(f"\n  t={e['t_s']:.2f} s  {e['message']}")
    print()
    print(ascii_plot(t, z))
    save_json(result, args.json)
    return 0


def cmd_presets(_args) -> int:
    for pid, cfg in PRESETS.items():
        print(f"{pid:<12} {cfg.name:<14} mass {cfg.mass_kg:4.2f} kg   T/W {cfg.thrust_to_weight:4.2f}   hover {cfg.hover_throttle_pct:5.1f} %")
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="python -m backend.cli", description="DroneLab headless simulator")
    sub = parser.add_subparsers(dest="command", required=True)

    hover = sub.add_parser("hover", help="take off and hover at a target altitude")
    hover.add_argument("--alt", type=float, default=2.0, help="target altitude in metres (default 2)")
    hover.add_argument("--duration", type=float, default=10.0, help="seconds to simulate (default 10)")
    hover.add_argument("--preset", default="basic", choices=list(PRESETS))
    hover.add_argument("--mass-kg", type=float, help="override the mass")
    hover.add_argument("--thrust-to-weight", "--tw", type=float, help="set motor strength by T/W ratio")
    hover.add_argument("--alt-kp", type=float, help="altitude P gain")
    hover.add_argument("--alt-ki", type=float, help="altitude I gain")
    hover.add_argument("--alt-kd", type=float, help="altitude D gain")
    hover.add_argument("--seed", type=int, default=0)
    hover.add_argument("--json", help="also save the full RunResult to this JSON file")
    hover.set_defaults(func=cmd_hover)

    ff = sub.add_parser("freefall", help="drop with motors off (gravity sanity check)")
    ff.add_argument("--height", type=float, default=20.0)
    ff.add_argument("--duration", type=float, default=2.0)
    ff.add_argument("--preset", default="basic", choices=list(PRESETS))
    ff.add_argument("--json")
    ff.set_defaults(func=cmd_freefall)

    sub.add_parser("presets", help="list the drone presets").set_defaults(func=cmd_presets)

    args = parser.parse_args(argv)
    try:
        return args.func(args)
    except (ConfigError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
