// Drawing an NDVI map (bytes = NDVI × 255) in farm colours, with marks: dashed circles (truth) and numbered boxes (found).

export interface Mark { kind: 'circle' | 'box'; x?: number; y?: number; r?: number; box?: [number, number, number, number]; label?: string }

/** Soil grey-brown below 0.2, red → yellow for stressed crop, yellow-green → deep green for healthy crop. */
export function ndviColor(v: number): [number, number, number] {
  if (v < 0.2) return [120, 104, 88];
  const stops: [number, [number, number, number]][] = [[0.2, [178, 34, 34]], [0.45, [230, 160, 40]], [0.6, [235, 220, 90]], [0.72, [140, 200, 80]], [0.9, [20, 110, 40]]];
  for (let i = 1; i < stops.length; i++) {
    const [v1, c1] = stops[i];
    if (v <= v1) {
      const [v0, c0] = stops[i - 1];
      const f = (v - v0) / (v1 - v0);
      return [c0[0] + f * (c1[0] - c0[0]), c0[1] + f * (c1[1] - c0[1]), c0[2] + f * (c1[2] - c0[2])];
    }
  }
  return stops[stops.length - 1][1];
}

export function drawNdvi(ctx: CanvasRenderingContext2D, image: string, w: number, h: number, marks: Mark[] = []): void {
  const bytes = atob(image);
  const img = ctx.createImageData(w, h);
  for (let i = 0; i < bytes.length; i++) {
    const b = bytes.charCodeAt(i);
    const [r, g, bl] = b === 0 ? [15, 20, 31] : ndviColor(b / 255); // 0 = no photo there
    img.data.set([r, g, bl, 255], 4 * i);
  }
  ctx.putImageData(img, 0, 0);
  ctx.lineWidth = Math.max(1, w / 160);
  ctx.font = `bold ${Math.max(8, w / 18)}px system-ui, sans-serif`;
  for (const m of marks) {
    if (m.kind === 'circle' && m.x !== undefined && m.y !== undefined) {
      ctx.strokeStyle = 'rgba(255,255,255,0.9)';
      ctx.setLineDash([3, 3]);
      ctx.beginPath();
      ctx.arc(m.x, m.y, m.r ?? 4, 0, Math.PI * 2);
      ctx.stroke();
      ctx.setLineDash([]);
    } else if (m.kind === 'box' && m.box) {
      const [x0, y0, x1, y1] = m.box;
      ctx.strokeStyle = '#4cc9f0';
      ctx.strokeRect(x0 - 2, y0 - 2, x1 - x0 + 5, y1 - y0 + 5);
      ctx.fillStyle = '#4cc9f0';
      ctx.fillText(m.label ?? '', x1 + 4, y0 + 2);
    }
  }
}
