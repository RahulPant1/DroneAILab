// 🛠 Drone Builder: pick parts, see whether it flies and for how long, then fly it (on its own battery).

import { useEffect, useState } from 'react';
import { sim } from './sim';

interface Catalog {
  frames: Record<string, { title: string; size_mm: number; mass_g: number; max_prop_in: number }>;
  motors: Record<string, { title: string; mass_g: number; kv: number; rated_cells: number; max_cells: number; props_in: number[]; thrust_g: Record<string, number> }>;
  props: Record<string, { title: string; diameter_in: number }>;
  cells: number[];
  capacities_mah: number[];
  max_payload_kg: number;
  default: Parts;
}
interface Parts {
  frame: string;
  motor: string;
  prop: string;
  cells: number;
  capacity_mah: number;
  payload_kg: number;
}
interface Report {
  verdict: 'good' | 'ok' | 'risky' | "won't fly";
  mass_g: number;
  mass_breakdown_g: Record<string, number>;
  max_thrust_g: number;
  thrust_to_weight: number;
  hover_throttle_pct: number | null;
  hover_power_w: number;
  hover_current_a: number;
  full_throttle_current_a: number;
  battery_max_current_a: number;
  battery_energy_wh: number;
  flight_time_min: number | null;
  grams_per_watt: number;
  warnings: string[];
}

const VERDICT = { good: '✅ Good build', ok: '🟡 Flyable', risky: '⚠️ Risky', "won't fly": "❌ Won't fly" } as const;

export default function Builder({ onClose, onLearn }: { onClose: () => void; onLearn: (slug: string) => void }) {
  const [cat, setCat] = useState<Catalog | null>(null);
  const [parts, setParts] = useState<Parts | null>(null);
  const [report, setReport] = useState<Report | null>(null);
  const [error, setError] = useState('');
  const [drain, setDrain] = useState(1);

  useEffect(() => {
    fetch('/api/builder/parts').then((r) => r.json()).then((c: Catalog) => {
      setCat(c);
      setParts(c.default);
    }).catch((e) => setError(String(e)));
  }, []);
  useEffect(() => {
    const esc = (e: KeyboardEvent) => e.key === 'Escape' && onClose();
    window.addEventListener('keydown', esc);
    return () => window.removeEventListener('keydown', esc);
  }, [onClose]);
  useEffect(() => {
    if (!parts) return;
    const id = window.setTimeout(async () => {
      const resp = await fetch('/api/builder/design', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ parts }) });
      const body = await resp.json();
      if (resp.ok) {
        setReport(body.report);
        setError('');
      } else {
        setReport(null);
        setError(body.detail ?? `HTTP ${resp.status}`);
      }
    }, 150);
    return () => window.clearTimeout(id);
  }, [parts]);

  if (!cat || !parts) return null;
  const set = (patch: Partial<Parts>) => setParts({ ...parts, ...patch });
  // Picking a frame or motor also picks a prop that fits it, so the common path never hits an error.
  const pickFrame = (frame: string) => {
    const f = cat.frames[frame];
    const motor = Object.entries(cat.motors).find(([, m]) => m.props_in.some((p) => p <= f.max_prop_in && p >= f.max_prop_in * 0.6))?.[0] ?? parts.motor;
    const m = cat.motors[motor];
    const prop = Object.entries(cat.props).find(([, p]) => m.props_in.includes(p.diameter_in))?.[0] ?? parts.prop;
    set({ frame, motor, prop, cells: Math.min(parts.cells, m.max_cells) });
  };
  const pickMotor = (motor: string) => {
    const m = cat.motors[motor];
    const prop = Object.entries(cat.props).find(([, p]) => m.props_in.includes(p.diameter_in))?.[0] ?? parts.prop;
    set({ motor, prop, cells: m.rated_cells });
  };
  const fly = () => {
    sim.send('design', { parts, drain_speedup: drain });
    onClose();
  };

  const r = report;
  return (
    <div className="modal-backdrop" onClick={onClose}>
      <article className="modal builder" onClick={(e) => e.stopPropagation()}>
        <button className="close" onClick={onClose} aria-label="Close">✕</button>
        <h2>🛠 Build your own drone</h2>
        <p className="muted small">Pick the parts a real builder would. The numbers below decide whether it flies, and for how long. Then fly it, on its own battery.</p>
        <div className="builder-grid">
          <label>Frame
            <select value={parts.frame} onChange={(e) => pickFrame(e.target.value)}>
              {Object.entries(cat.frames).map(([k, f]) => <option key={k} value={k}>{f.title}, {f.mass_g} g</option>)}
            </select>
          </label>
          <label>Motors (×4)
            <select value={parts.motor} onChange={(e) => pickMotor(e.target.value)}>
              {Object.entries(cat.motors).map(([k, m]) => <option key={k} value={k}>{m.title}, {m.mass_g} g</option>)}
            </select>
          </label>
          <label>Props (×4)
            <select value={parts.prop} onChange={(e) => set({ prop: e.target.value })}>
              {Object.entries(cat.props).map(([k, p]) => <option key={k} value={k}>{p.title}</option>)}
            </select>
          </label>
          <label>Battery
            <span className="inline">
              <select value={parts.cells} onChange={(e) => set({ cells: Number(e.target.value) })}>
                {cat.cells.map((c) => <option key={c} value={c}>{c}S ({(c * 3.7).toFixed(1)} V)</option>)}
              </select>
              <select value={parts.capacity_mah} onChange={(e) => set({ capacity_mah: Number(e.target.value) })}>
                {cat.capacities_mah.map((c) => <option key={c} value={c}>{c} mAh</option>)}
              </select>
            </span>
          </label>
          <label title="A camera, gimbal or package it has to carry">Payload: <b>{parts.payload_kg.toFixed(2)} kg</b>
            <input type="range" min={0} max={cat.max_payload_kg} step={0.05} value={parts.payload_kg} onChange={(e) => set({ payload_kg: Number(e.target.value) })} />
          </label>
        </div>

        {error && <p className="error">❌ {error}</p>}
        {r && (
          <>
            <div className={`builder-verdict v-${r.verdict.replace(/[^a-z]/g, '')}`}>
              <b>{VERDICT[r.verdict]}</b>
              <span>{r.flight_time_min != null ? <>about <b>{r.flight_time_min.toFixed(1)} min</b> of hover</> : 'no flight time: it can\'t lift off'}</span>
            </div>
            <table className="nums builder-nums">
              <tbody>
                <tr title="Everything together"><th>All-up weight</th><td>{r.mass_g} g</td></tr>
                <tr title="All four motors at full power"><th>Max thrust</th><td>{r.max_thrust_g} g</td></tr>
                <tr title="Thrust ÷ weight. Must be over 1 to fly; 2+ is the usual minimum; racers are 8–12."><th>Thrust-to-weight</th><td><b>{r.thrust_to_weight.toFixed(2)}</b></td></tr>
                <tr title="Throttle needed just to hover = 100 ÷ T/W"><th>Hover throttle</th><td>{r.hover_throttle_pct != null ? `${r.hover_throttle_pct.toFixed(0)} %` : '–'}</td></tr>
                <tr title="Electrical power from the battery in hover (momentum theory + motor efficiency)"><th>Hover power</th><td>{r.hover_power_w.toFixed(0)} W ({r.hover_current_a.toFixed(1)} A)</td></tr>
                <tr title="Grams lifted per watt in hover: bigger props are more efficient"><th>Efficiency</th><td>{r.grams_per_watt.toFixed(1)} g/W</td></tr>
                <tr title="All four motors at full power vs what the pack is rated to give"><th>Full-throttle current</th><td>{r.full_throttle_current_a.toFixed(0)} A (pack: {r.battery_max_current_a} A)</td></tr>
                <tr><th>Battery energy</th><td>{r.battery_energy_wh.toFixed(0)} Wh</td></tr>
              </tbody>
            </table>
            <p className="muted small">Weight: {Object.entries(r.mass_breakdown_g).filter(([, g]) => g > 0).map(([k, g]) => `${k} ${g} g`).join(' · ')}</p>
            <p className="muted small">Flight time is gentle hovering down to the low-battery point (~25 % left). Aggressive flying uses 2–3× more power.</p>
            {r.warnings.length > 0 && <ul className="problems">{r.warnings.map((w) => <li key={w}>{w}</li>)}</ul>}
          </>
        )}
        <div className="row">
          <button className="primary" disabled={!r} onClick={fly}>✈ Fly this design</button>
          <label className="inline" title="Drain the battery faster than real life, to see the low-battery failsafe without waiting the whole flight">
            battery drains
            <select value={drain} onChange={(e) => setDrain(Number(e.target.value))}>
              <option value={1}>in real time</option>
              <option value={5}>5× faster</option>
              <option value={10}>10× faster (demo)</option>
            </select>
          </label>
          <span className="spacer" />
          <button className="link" onClick={() => onLearn('16-battery-and-builder')}>📖 Learn: build your own</button>
        </div>
      </article>
    </div>
  );
}
