// A strip of grey-scale pictures from a run (e.g. the down camera on the way down), with the detected marker boxed.

import { useEffect, useRef } from 'react';
import type { ImagesExtra, PlotDef } from './lab';

type Frame = ImagesExtra['frames'][number];

function Picture({ f }: { f: Frame }) {
  const ref = useRef<HTMLCanvasElement>(null);
  useEffect(() => {
    const ctx = ref.current?.getContext('2d');
    if (!ctx) return;
    const bytes = atob(f.image);
    const img = ctx.createImageData(f.w, f.h);
    for (let i = 0; i < bytes.length; i++) {
      const v = bytes.charCodeAt(i);
      img.data.set([v * 0.85, v, v * 0.8, 255], 4 * i);
    }
    ctx.putImageData(img, 0, 0);
    if (f.detection) {
      const b = f.detection.box;
      ctx.strokeStyle = '#ef476f';
      ctx.strokeRect(b[0] - 1, b[1] - 1, b[2] - b[0] + 3, b[3] - b[1] + 3);
    }
  }, [f]);
  return (
    <figure className="cam-frame">
      <canvas ref={ref} width={f.w} height={f.h} style={{ width: f.w * 1.6, height: f.h * 1.6 }} />
      <figcaption className="small">{f.alt_m.toFixed(1)} m up · {f.detection ? `marker: ${f.detection.pixels} px` : 'no marker'}</figcaption>
    </figure>
  );
}

export default function ImagesPlot({ plot }: { plot: PlotDef }) {
  const frames = (plot.extra as unknown as ImagesExtra | undefined)?.frames ?? [];
  return (
    <figure className="chart">
      <figcaption>
        <b>{plot.title}</b>
        <span className="how">{plot.how_to_read}</span>
      </figcaption>
      <div className="cam-strip">{frames.length ? frames.map((f, i) => <Picture key={i} f={f} />) : <span className="muted small">No pictures.</span>}</div>
    </figure>
  );
}
