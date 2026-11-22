// One uPlot chart for a plot spec, drawing one run, or two runs overlaid (run B dashed).

import { useEffect, useRef } from 'react';
import uPlot, { type AlignedData, type Options } from 'uplot';
import 'uplot/dist/uPlot.min.css';
import type { PlotDef, RunResult } from './lab';
import ImagesPlot from './ImagesPlot';
import MapPlot from './MapPlot';

const COLORS = ['#4cc9f0', '#ffd166', '#ef476f', '#06d6a0', '#b388ff', '#ff9f1c'];

export interface ChartRun {
  name: string; // "A", "B" or ""
  result: RunResult;
}

function seriesData(run: RunResult, plot: PlotDef): (number | null)[][] {
  const t = run.telemetry.t_s ?? [];
  return plot.series.map((s) => {
    const values = run.telemetry[s.channel];
    return values ? values.map((v) => v * s.scale) : t.map(() => null);
  });
}

export default function Chart(props: { plot: PlotDef; runs: ChartRun[]; height?: number }) {
  if (props.plot.kind === 'images') return <ImagesPlot plot={props.plot} />;
  return props.plot.kind === 'map' ? <MapPlot plot={props.plot} runs={props.runs} /> : <TimeChart {...props} />;
}

function TimeChart({ plot, runs, height = 220 }: { plot: PlotDef; runs: ChartRun[]; height?: number }) {
  const wrap = useRef<HTMLDivElement>(null);

  useEffect(() => {
    const el = wrap.current;
    if (!el || runs.length === 0) return;

    const tables = runs.map((r) => [r.result.telemetry.t_s ?? [], ...seriesData(r.result, plot)] as AlignedData);
    const data = runs.length === 1 ? tables[0] : uPlot.join(tables);

    const series: Options['series'] = [{ label: 't (s)' }];
    runs.forEach((run, ri) => {
      plot.series.forEach((s, si) => {
        const color = COLORS[si % COLORS.length];
        const label = runs.length > 1 ? `${run.name}: ${s.label}` : s.label;
        const dash = ri === 1 ? [3, 4] : s.style === 'dashed' ? [8, 5] : undefined;
        series.push(
          s.style === 'points'
            ? { label, stroke: color, paths: () => null, points: { show: true, size: 3, fill: color }, alpha: ri === 1 ? 0.5 : 0.8 }
            : { label, stroke: color, width: s.style === 'dashed' ? 1.25 : 1.75, dash, spanGaps: true, points: { show: false } },
        );
      });
    });

    const axisCommon = { stroke: '#8a94a8', grid: { stroke: '#283147', width: 1 }, ticks: { stroke: '#283147' } };
    const opts: Options = {
      width: el.clientWidth,
      height,
      series,
      scales: { x: { time: false } },
      axes: [
        { ...axisCommon, label: 'time (s)' },
        { ...axisCommon, label: plot.y_label, size: 60 },
      ],
      cursor: { drag: { x: true, y: false } }, // drag to zoom, double-click to reset
      legend: { live: true },
    };
    const chart = new uPlot(opts, data, el);
    const resize = new ResizeObserver(() => chart.setSize({ width: el.clientWidth, height }));
    resize.observe(el);
    return () => {
      resize.disconnect();
      chart.destroy();
    };
  }, [plot, runs, height]);

  return (
    <figure className="chart">
      <figcaption>
        <b>{plot.title}</b>
        <span className="how">{plot.how_to_read}</span>
      </figcaption>
      <div ref={wrap} />
    </figure>
  );
}
