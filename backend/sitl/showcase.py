"""Regenerate the README's comparison chart from a real run: ArduCopter (SITL) vs DroneLab, "fly 10 m north".

  .venv/bin/python -m backend.sitl.showcase      # ~1 min: starts SITL, records, writes docs/images/step-compare.svg
"""

from __future__ import annotations

import json
import sys
import time

from backend.sitl import launch
from backend.sitl.bridge import COMPARE_DISTANCE_M, SitlBridge

OUT_SVG = launch.ROOT / "docs" / "images" / "step-compare.svg"
OUT_JSON = launch.ROOT / "docs" / "results" / "step-compare.json"


def svg_step_compare(c: dict, width: int = 640, height: int = 300) -> str:
    """Both step responses on one chart, with the target line and a metrics table."""
    left, right, top, bottom = 56, 20, 40, 92
    w, h = width - left - right, height - top - bottom
    t_max = max(c["ardupilot"]["t_s"][-1], c["dronelab"]["t_s"][-1])
    y_max = max(max(c["ardupilot"]["north_m"]), max(c["dronelab"]["north_m"]), c["distance_m"]) * 1.1

    def px(t: float, y: float) -> str:
        return f"{left + t / t_max * w:.1f},{top + h - y / y_max * h:.1f}"

    def line(run: dict, color: str) -> str:
        pts = " ".join(px(t, y) for t, y in zip(run["t_s"], run["north_m"]))
        return f'<polyline points="{pts}" fill="none" stroke="{color}" stroke-width="2.5"/>'

    grid = []
    for y in range(0, int(y_max) + 1, 2):
        yy = top + h - y / y_max * h
        grid.append(f'<line x1="{left}" y1="{yy:.1f}" x2="{left + w}" y2="{yy:.1f}" stroke="#e5e8ee"/>'
                    f'<text x="{left - 8}" y="{yy + 4:.1f}" text-anchor="end" fill="#666">{y}</text>')
    for t in range(0, int(t_max) + 1, 3):
        xx = left + t / t_max * w
        grid.append(f'<text x="{xx:.1f}" y="{top + h + 16}" text-anchor="middle" fill="#666">{t}</text>')
    ty = top + h - c["distance_m"] / y_max * h

    def row(y: int, name: str, color: str, r: dict) -> str:
        settle = f'{r["settle_s"]:.1f} s' if r["settle_s"] is not None else "–"
        return (f'<text x="{left}" y="{y}" fill="{color}" font-weight="bold">{name}</text>'
                f'<text x="{left + 130}" y="{y}" fill="#333">rise {r["rise_s"]:.1f} s · overshoot {r["overshoot_m"]:.2f} m · '
                f'settles {settle} · top speed {r["peak_speed_m_s"]:.1f} m/s</text>')

    return f"""<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}" font-family="Segoe UI, Helvetica, Arial, sans-serif" font-size="12">
<rect width="100%" height="100%" fill="#ffffff"/>
<text x="{width / 2}" y="22" text-anchor="middle" font-size="15" font-weight="bold" fill="#1d3557">Same command, two autopilots: "fly {c['distance_m']:g} m north" from a 5 m hover</text>
{''.join(grid)}
<line x1="{left}" y1="{ty:.1f}" x2="{left + w}" y2="{ty:.1f}" stroke="#8a94a6" stroke-dasharray="6,5"/>
<text x="{left + w - 4}" y="{ty - 6:.1f}" text-anchor="end" fill="#8a94a6">target</text>
{line(c['dronelab'], '#4cc9f0')}
{line(c['ardupilot'], '#ff8c42')}
<line x1="{left}" y1="{top + h}" x2="{left + w}" y2="{top + h}" stroke="#999"/>
<text x="{left + w / 2}" y="{top + h + 32}" text-anchor="middle" fill="#555">seconds after the command</text>
<text x="16" y="{top + h / 2}" text-anchor="middle" fill="#555" transform="rotate(-90 16 {top + h / 2})">north (m)</text>
{row(height - 26, 'ArduCopter 4.7.3', '#ff8c42', c['ardupilot'])}
{row(height - 8, 'DroneLab', '#2b9cc4', c['dronelab'])}
</svg>"""


def main() -> int:
    b = SitlBridge()
    print("Starting ArduCopter SITL and recording the comparison (~1 min)...")
    try:
        b.start(speedup=1.0)
        t0 = time.monotonic()
        while (b.status() or {}).get("state") != "ready":
            if time.monotonic() - t0 > 120 or (b.status() or {}).get("state") == "error":
                print("SITL didn't get ready:", b.status())
                return 1
            time.sleep(0.5)
        b.handle({"action": "compare"})
        while (b.status() or {}).get("compare") is None:
            if time.monotonic() - t0 > 300:
                print("Comparison timed out:", (b.status() or {}).get("message"))
                return 1
            time.sleep(0.5)
        c = b.status()["compare"]
        if c["ardupilot"]["max_gap_s"] > 0.3:
            print("The recording has a gap even after retries; not writing a misleading chart.")
            return 1
    finally:
        b.stop()
    assert c["distance_m"] == COMPARE_DISTANCE_M
    OUT_JSON.parent.mkdir(parents=True, exist_ok=True)
    OUT_JSON.write_text(json.dumps(c, indent=1))
    OUT_SVG.parent.mkdir(parents=True, exist_ok=True)
    OUT_SVG.write_text(svg_step_compare(c))
    for k in ("ardupilot", "dronelab"):
        print(f"  {k:9s}", {m: c[k][m] for m in ("rise_s", "overshoot_m", "settle_s", "peak_speed_m_s")})
    print(f"Wrote {OUT_SVG} and {OUT_JSON}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
