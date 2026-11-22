// A small top-down SVG map of the farm (north up), with an optional flight plan drawn on it.
// Used by the 🧭 Mission planner to show the route BEFORE you press Fly.

import type { FlightPlan, Site } from './sim';

export default function FarmMap({ site, plan }: { site: Site; plan?: FlightPlan | null }) {
  const r = site.geofence_radius_m + 4;
  const pt = (x: number, y: number) => `${x},${-y}`;
  const fail = plan && !plan.ok;
  return (
    <svg className="farm-map" viewBox={`${-r} ${-r} ${2 * r} ${2 * r}`} role="img" aria-label="Top-down map of the farm and the planned route">
      <circle cx={0} cy={0} r={site.geofence_radius_m} className="fm-fence" />
      {site.fields.map((f) => (
        <g key={f.name}>
          <polygon points={f.polygon.map(([x, y]) => pt(x, y)).join(' ')} className="fm-field" />
          <text x={f.polygon.reduce((a, p) => a + p[0], 0) / f.polygon.length} y={-f.polygon.reduce((a, p) => a + p[1], 0) / f.polygon.length}
            className="fm-label">{f.name}</text>
        </g>
      ))}
      {site.no_fly.map((z) => (
        <g key={z.name}>
          <rect x={z.x_min} y={-z.y_max} width={z.x_max - z.x_min} height={z.y_max - z.y_min} className="fm-nofly"><title>No-fly: {z.why}</title></rect>
          <text x={(z.x_min + z.x_max) / 2} y={-(z.y_min + z.y_max) / 2} className="fm-label fm-nofly-label">⛔ {z.name}</text>
        </g>
      ))}
      {site.buildings.map((b) => (
        <g key={b.name}>
          <rect x={b.x_min} y={-b.y_max} width={b.x_max - b.x_min} height={b.y_max - b.y_min} className="fm-building"><title>{b.name}: {b.height_m} m tall</title></rect>
          <text x={(b.x_min + b.x_max) / 2} y={-b.y_max - 1.5} className="fm-label fm-small">{b.name} {b.height_m} m</text>
        </g>
      ))}
      {site.places.map((p) => (
        <g key={p.name}>
          <circle cx={p.x_m} cy={-p.y_m} r={1.2} className="fm-place" />
          <text x={p.x_m} y={-p.y_m - 2.2} className="fm-label fm-small">{p.name}</text>
        </g>
      ))}
      {site.pads.map((p) => (
        <g key={p.name}>
          <rect x={p.x_m - 1.8} y={-p.y_m - 1.8} width={3.6} height={3.6} className="fm-pad" />
          <text x={p.x_m} y={-p.y_m + 5.5} className="fm-label fm-small">🅿 {p.name}</text>
        </g>
      ))}
      {plan?.surveys.map((s) => s.lines.map(([a, b], i) => (
        <line key={`${s.field}${i}`} x1={a[0]} y1={-a[1]} x2={b[0]} y2={-b[1]} className="fm-line" />
      )))}
      {plan && <polyline points={plan.route.map(([x, y]) => pt(x, y)).join(' ')} className={`fm-route ${fail ? 'fm-route-bad' : ''}`} />}
      {plan && <circle cx={plan.land.x_m} cy={-plan.land.y_m} r={2.6} className="fm-land" />}
    </svg>
  );
}
