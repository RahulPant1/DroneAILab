// Top-down mission map (SVG, metres, north up): obstacles, keep-out zones, planned route, flown path(s).

import { useMemo, type ReactElement } from 'react';
import type { ChartRun } from './Chart';
import type { MapExtra, PlotDef } from './lab';

const RUN_COLORS = ['#4cc9f0', '#ff6bd6'];

export default function MapPlot({ plot, runs }: { plot: PlotDef; runs: ChartRun[] }) {
  const raw = plot.extra as MapExtra;
  const extra: MapExtra = { ...raw, obstacles: raw.obstacles ?? [], waypoints: raw.waypoints ?? [], planned_route: raw.planned_route ?? [] };
  const from = extra.first_index ?? 0;
  const xCh = plot.series[0]?.channel ?? 'x_m';
  const yCh = plot.series[1]?.channel ?? 'y_m';

  const paths = useMemo(
    () =>
      runs.map((r) => {
        const xs = r.result.telemetry[xCh] ?? [];
        const ys = r.result.telemetry[yCh] ?? [];
        const pts: [number, number][] = [];
        for (let i = from; i < xs.length; i += 2) {
          const x = xs[i], y = ys[i];
          if (x !== null && y !== null) pts.push([x, y]); // skip gaps (no value at that moment)
        }
        return pts;
      }),
    [runs, xCh, yCh, from],
  );
  const estPath = useMemo(() => {
    if (!extra.estimate) return null;
    const [ex, ey] = extra.estimate;
    const pts: string[] = [];
    for (let i = from; i < ex.length; i += 2) pts.push(`${ex[i]},${-ey[i]}`);
    return pts.join(' ');
  }, [extra.estimate, from]);

  // Bounds that fit everything, plus a 1 m border.
  const b = useMemo(() => {
    const xs: number[] = [], ys: number[] = [];
    paths.flat().forEach(([x, y]) => { xs.push(x); ys.push(y); });
    extra.planned_route.forEach(([x, y]) => { xs.push(x); ys.push(y); });
    (extra.gates ?? []).forEach(([x, y, , r]) => { xs.push(x - r, x + r); ys.push(y - r, y + r); });
    extra.waypoints.forEach(([x, y]) => { xs.push(x); ys.push(y); });
    extra.obstacles.forEach((o) => { xs.push(o.x_min, o.x_max); ys.push(o.y_min, o.y_max); });
    (extra.fields ?? []).flat().forEach(([x, y]) => { xs.push(x); ys.push(y); });
    if (extra.window) {
      const [x0, x1, y0, y1] = extra.window;
      return { x0, x1, y0, y1, w: x1 - x0, h: y1 - y0 };
    }
    const x0 = Math.min(...xs) - 1, x1 = Math.max(...xs) + 1, y0 = Math.min(...ys) - 1, y1 = Math.max(...ys) + 1;
    return { x0, x1, y0, y1, w: x1 - x0, h: y1 - y0 };
  }, [paths, extra]);

  const span = Math.max(b.w, b.h);
  const gridStep = span > 80 ? 10 : span > 30 ? 5 : span > 14 ? 2 : 1;
  const per = extra.footprint_per_m ?? [1.4, 1.05];
  const font = span * 0.035;
  const pt = (x: number, y: number) => `${x},${-y}`; // SVG y grows downward, so flip north up
  const gridLines: ReactElement[] = [];
  for (let gx = Math.ceil(b.x0 / gridStep) * gridStep; gx <= b.x1; gx += gridStep)
    gridLines.push(<line key={`x${gx}`} x1={gx} x2={gx} y1={-b.y1} y2={-b.y0} className={gx === 0 ? 'map-axis' : 'map-grid'} />);
  for (let gy = Math.ceil(b.y0 / gridStep) * gridStep; gy <= b.y1; gy += gridStep)
    gridLines.push(<line key={`y${gy}`} y1={-gy} y2={-gy} x1={b.x0} x2={b.x1} className={gy === 0 ? 'map-axis' : 'map-grid'} />);
  const z = extra.safety_zone_m;

  return (
    <figure className="chart map-chart">
      <figcaption>
        <b>{plot.title}</b>
        <span className="how">{plot.how_to_read}</span>
      </figcaption>
      <svg className="map" viewBox={`${b.x0} ${-b.y1} ${b.w} ${b.h}`} preserveAspectRatio="xMidYMid meet" role="img"
        aria-label="Top-down map of the mission">
        {gridLines}
        {(extra.fields ?? []).map((poly, i) => (
          <polygon key={`f${i}`} points={poly.map(([x, y]) => pt(x, y)).join(' ')} className="map-field" />
        ))}
        {(extra.photos ?? []).map(([x, y, alt, a], i) => {
          const along = per[1] * alt, across = per[0] * alt;
          return (
            <rect key={`p${i}`} x={x - along / 2} y={-y - across / 2} width={along} height={across} className="map-photo"
              transform={`rotate(${(-a * 180) / Math.PI} ${x} ${-y})`} />
          );
        })}
        {(extra.gaps ?? []).map(([x, y], i) => <circle key={`g${i}`} cx={x} cy={-y} r={Math.max(0.25, span * 0.004)} className="map-gap" />)}
        {(extra.no_fly ?? []).map((o, i) => (
          <rect key={`n${i}`} x={o.x_min} y={-o.y_max} width={o.x_max - o.x_min} height={o.y_max - o.y_min} className="map-nofly" />
        ))}
        {(extra.pads ?? []).map(([x, y, size], i) => (
          <g key={`pad${i}`}>
            <rect x={x - size * 0.75} y={-y - size * 0.75} width={size * 1.5} height={size * 1.5} className="map-pad" />
            <rect x={x - size * 0.375} y={-y - size * 0.375} width={size * 0.75} height={size * 0.75} className="map-marker"
              strokeWidth={size * 0.25} />
          </g>
        ))}
        {estPath && <polyline points={estPath} className="map-flown map-estimate" />}
        {extra.obstacles.map((o, i) => (
          <g key={i}>
            <rect x={o.x_min - z} y={-(o.y_max + z)} width={o.x_max - o.x_min + 2 * z} height={o.y_max - o.y_min + 2 * z}
              rx={z} className="map-zone" />
            <rect x={o.x_min} y={-o.y_max} width={o.x_max - o.x_min} height={o.y_max - o.y_min} className="map-obstacle">
              <title>Obstacle, {o.height_m} m tall</title>
            </rect>
          </g>
        ))}
        {(extra.plans ?? []).slice(0, -1).map((p, i) => (
          <polyline key={`plan${i}`} points={p.map(([x, y]) => pt(x, y)).join(' ')} className="map-plan map-old-plan" />
        ))}
        <polyline points={extra.planned_route.map(([x, y]) => pt(x, y)).join(' ')} className="map-plan" />
        {(extra.discovered ?? []).map(([x, y], i) => (
          <rect key={`d${i}`} x={x - 0.09} y={-y - 0.09} width={0.18} height={0.18} className="map-lidar-hit" />
        ))}
        {paths.map((p, i) => (
          <polyline key={i} points={p.map(([x, y]) => pt(x, y)).join(' ')} className="map-flown"
            style={{ stroke: RUN_COLORS[i % 2], strokeDasharray: i === 1 ? '5 4' : undefined }} />
        ))}
        {(extra.gates ?? []).map(([x, y, h, r, ok], i) => {
          const a = (h * Math.PI) / 180, px = -Math.sin(a) * r, py = Math.cos(a) * r; // across the ring
          const ax = Math.cos(a) * r * 0.8, ay = Math.sin(a) * r * 0.8; // the way to fly through
          return (
            <g key={`gate${i}`} className={ok ? 'map-gate ok' : 'map-gate missed'}>
              <line x1={x - px} y1={-(y - py)} x2={x + px} y2={-(y + py)} />
              <line x1={x} y1={-y} x2={x + ax} y2={-(y + ay)} className="map-gate-arrow" />
              <text x={x - px * 1.6} y={-(y - py * 1.6) + font * 0.35} fontSize={font}>{i + 1}</text>
              <title>Ring {i + 1}: {ok ? 'flown through' : 'MISSED'}</title>
            </g>
          );
        })}
        {extra.waypoints.map(([x, y], i) =>
          i === 0 ? (
            <polygon key={i} points={`${x},${-y - font * 0.6} ${x - font * 0.5},${-y + font * 0.35} ${x + font * 0.5},${-y + font * 0.35}`} className="map-start" />
          ) : (
            <g key={i}>
              <circle cx={x} cy={-y} r={font * 0.55} className="map-wp" />
              <text x={x} y={-y + font * 0.35} fontSize={font} className="map-wp-label">{i}</text>
            </g>
          ),
        )}
        {runs.map((r, ri) =>
          r.result.events
            .filter((e) => (e.type === 'collision' || e.type === 'stuck') && e.x_m !== undefined)
            .map((e, i) => (
              <text key={`${ri}-${i}`} x={e.x_m} y={-(e.y_m ?? 0) + font * 0.4} fontSize={font * 1.4}
                className={e.type === 'collision' ? 'map-crash' : 'map-stuck'}>
                {e.type === 'collision' ? '✕' : '!'}
                <title>{(runs.length > 1 ? `${r.name}: ` : '') + e.message}</title>
              </text>
            )),
        )}
      </svg>
      <div className="map-legend small">
        <span><i className="sw plan" /> planned route</span>
        {runs.map((r, i) => (
          <span key={i}><i className="sw" style={{ background: RUN_COLORS[i % 2] }} /> {runs.length > 1 ? `${r.name}: flown` : 'flown path'}</span>
        ))}
        {extra.estimate && <span><i className="sw est" /> where it thought it was</span>}
        {extra.photos && <span><i className="sw photo" /> photo footprints</span>}
        {extra.gates && <span><span className="map-gate-key ok">┃</span> ring flown through <span className="map-gate-key missed">┃</span> missed</span>}
        {extra.gaps && extra.gaps.length > 0 && <span className="map-gap-key">● gap</span>}
        {extra.obstacles.length > 0 && <span><i className="sw obstacle" /> obstacle</span>}
        {extra.obstacles.length > 0 && <span><i className="sw zone" /> keep-out zone</span>}
        <span className="map-crash">✕</span> crash <span className="map-stuck">!</span> stuck
        <span className="muted">grid: {gridStep} m · north ↑ · east →</span>
      </div>
    </figure>
  );
}
