// 📷 The down camera's picture, drawn over the 3D view: what a camera under the drone sees (grey-scale),
// with the detected landing marker boxed while it precision-lands.

import { useEffect, useRef, useState } from 'react';
import { sim, type CameraStatus } from './sim';

const SCALE = 2;

export default function CameraView() {
  const canvas = useRef<HTMLCanvasElement>(null);
  const [cam, setCam] = useState<CameraStatus | null>(null);
  const last = useRef<{ image: string; w: number; h: number } | null>(null);

  useEffect(() => {
    let shown: CameraStatus | null = null;
    return sim.subscribeState(() => {
      const c = sim.latest?.camera ?? null;
      if (!c) {
        if (shown) setCam((shown = null));
        last.current = null;
        return;
      }
      if (c.image) last.current = { image: c.image, w: c.w, h: c.h };
      // Re-render the caption only when something visible changed.
      if (!shown || shown.seen !== c.seen || shown.state !== c.state || shown.landing !== c.landing || c.image) setCam((shown = c));
    });
  }, []);

  useEffect(() => {
    const el = canvas.current;
    const img = last.current;
    if (!el || !img || !cam) return;
    const ctx = el.getContext('2d');
    if (!ctx) return;
    const bytes = atob(img.image);
    const data = ctx.createImageData(img.w, img.h);
    for (let i = 0; i < bytes.length; i++) {
      const v = bytes.charCodeAt(i);
      data.data[4 * i] = v * 0.85;
      data.data[4 * i + 1] = v;
      data.data[4 * i + 2] = v * 0.8;
      data.data[4 * i + 3] = 255;
    }
    ctx.putImageData(data, 0, 0);
    ctx.strokeStyle = 'rgba(255,255,255,0.35)';
    ctx.lineWidth = 0.5;
    ctx.beginPath();
    ctx.moveTo(img.w / 2, img.h / 2 - 5);
    ctx.lineTo(img.w / 2, img.h / 2 + 5);
    ctx.moveTo(img.w / 2 - 5, img.h / 2);
    ctx.lineTo(img.w / 2 + 5, img.h / 2);
    ctx.stroke();
    const d = cam.detection;
    if (d) {
      ctx.strokeStyle = cam.mode === 'gps' ? '#ffd166' : '#ef476f';
      ctx.lineWidth = 1;
      ctx.strokeRect(d.box[0] - 1, d.box[1] - 1, d.box[2] - d.box[0] + 3, d.box[3] - d.box[1] + 3);
    }
  }, [cam]);

  if (!cam) return null;
  const w = last.current?.w ?? 128, h = last.current?.h ?? 96;
  let caption = 'down camera';
  if (cam.landing) {
    caption = cam.mode === 'gps' ? 'GPS landing (camera not used)'
      : cam.state === 'gps' ? 'marker not found: landing on GPS'
      : cam.seen ? `marker found · ${cam.detection?.pixels ?? 0} px` : (cam.sightings ?? 0) > 0 ? 'final descent (marker too close to see)' : 'searching for the marker…';
  }
  return (
    <div className="camera-view" title="What a camera under the drone sees (128 × 96 pixels, 60° wide). Top of the picture = the drone's nose.">
      <canvas ref={canvas} width={w} height={h} style={{ width: w * SCALE, height: h * SCALE }} />
      <div className="small">📷 {caption}</div>
    </div>
  );
}
