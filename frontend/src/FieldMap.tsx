// 🗺 Field map: the crop-health maps from the last survey flown in the Fly view (stitched on the server).

import { useEffect, useRef, useState } from 'react';
import CropReport from './CropReport';
import { drawNdvi, type Mark } from './ndvi';
import type { CropMap } from './sim';

export function marksFor(m: CropMap, showTruth: boolean): Mark[] {
  const [x0, , , y1] = m.bounds;
  const out: Mark[] = m.findings.map((f, i) => ({ kind: 'box', box: f.box, label: String(i + 1) }));
  if (showTruth)
    out.push(...m.truth.map((t) => ({ kind: 'circle' as const, x: (t.x_m - x0) / m.cell_m, y: (y1 - t.y_m) / m.cell_m, r: Math.sqrt(t.area_m2 / Math.PI) / m.cell_m })));
  return out;
}

function NdviCanvas({ m, showTruth }: { m: CropMap; showTruth: boolean }) {
  const ref = useRef<HTMLCanvasElement>(null);
  useEffect(() => {
    const ctx = ref.current?.getContext('2d');
    if (ctx) drawNdvi(ctx, m.image, m.w, m.h, marksFor(m, showTruth));
  }, [m, showTruth]);
  const scale = Math.min(2.4, 420 / Math.max(m.w, m.h));
  return <canvas ref={ref} width={m.w} height={m.h} className="ndvi-canvas" style={{ width: m.w * scale, height: m.h * scale }} />;
}

export default function FieldMap({ maps, onClose, onLearn }: { maps: CropMap[]; onClose: () => void; onLearn: (slug: string) => void }) {
  const [i, setI] = useState(0);
  const [truth, setTruth] = useState(false);
  useEffect(() => {
    const esc = (e: KeyboardEvent) => e.key === 'Escape' && onClose();
    window.addEventListener('keydown', esc);
    return () => window.removeEventListener('keydown', esc);
  }, [onClose]);
  const m = maps[Math.min(i, maps.length - 1)];
  if (!m) return null;
  return (
    <div className="modal-backdrop" onClick={onClose}>
      <div className="modal fieldmap" onClick={(e) => e.stopPropagation()} role="dialog" aria-label="Crop-health map">
        <button className="close" onClick={onClose}>✕</button>
        <h2>🗺 Crop-health map</h2>
        {maps.length > 1 && <div className="row small">{maps.map((mm, k) => <button key={k} className={k === i ? 'active' : ''} onClick={() => setI(k)}>{mm.field}</button>)}</div>}
        <p className="muted small">
          {m.photos} photos stitched at their geotags (where the drone thought it was). NDVI: green = healthy (~0.8), yellow/red = stressed
          (below 0.6), grey = soil. North is up. <button className="link" onClick={() => onLearn('20-crop-health')}>📖 How it works</button>
        </p>
        <div className="fieldmap-grid">
          <div>
            <NdviCanvas m={m} showTruth={truth} />
            <label className="inline small"><input type="checkbox" checked={truth} onChange={(e) => setTruth(e.target.checked)} /> show where the problems REALLY are (simulator only)</label>
          </div>
          <div className="small">
            <p><b>{m.field}</b>: mapped {m.mapped_pct} %, average NDVI {m.field_ndvi}. Photos were {m.geotag_error_m?.toFixed(2)} m off on average (GPS).</p>
            <ol className="findings">
              {m.findings.map((f, k) => (
                <li key={k}>{f.where}: ~{f.area_m2.toFixed(0)} m², NDVI {f.mean_ndvi} (lowest {f.min_ndvi}) at ({f.x_m.toFixed(0)}, {f.y_m.toFixed(0)})</li>
              ))}
            </ol>
            {m.findings.length === 0 && <p>No stressed patches found.</p>}
            {truth && (
              <p className="muted">Truth: {m.truth.map((t) => `${t.cause} ${t.found ? `found ${t.position_error_m?.toFixed(1)} m off` : 'MISSED'}`).join('; ')}. False alarms: {m.false_alarms}.</p>
            )}
            <CropReport id={m.id} />
          </div>
        </div>
      </div>
    </div>
  );
}
