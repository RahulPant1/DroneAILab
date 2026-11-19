// Drone Studio: pick a preset, tweak a few key numbers, see T/W, apply.

import { useEffect, useMemo, useState } from 'react';
import { api, GRAVITY_M_S2, sim, useSimUi, type DroneConfig, type PresetsResponse } from './sim';

interface Param {
  key: keyof DroneConfig;
  label: string;
  unit: string;
  step: number;
  help: string;
}

const KEY_PARAMS: Param[] = [
  { key: 'mass_kg', label: 'Mass', unit: 'kg', step: 0.05, help: 'Total weight, battery included. Heavier means more thrust is needed just to hover.' },
  { key: 'motor_max_thrust_n', label: 'Max thrust per motor', unit: 'N', step: 0.1, help: 'How hard ONE motor pushes at full speed. All four together, compared with the weight, gives T/W.' },
  { key: 'arm_length_m', label: 'Arm length', unit: 'm', step: 0.005, help: 'Centre-to-motor distance. Longer arms give more leverage for tilting, but the drone is slower to rotate.' },
  { key: 'motor_time_constant_s', label: 'Motor response time', unit: 's', step: 0.005, help: 'How long a motor takes to reach a new speed. Smaller is snappier; larger feels "laggy".' },
  { key: 'drag_linear_n_s_m', label: 'Air drag', unit: 'N·s/m', step: 0.01, help: 'Air resistance. More drag means a lower top speed and gentler stops.' },
];

const ADVANCED_PARAMS: Param[] = [
  { key: 'motor_max_speed_rad_s', label: 'Motor max speed', unit: 'rad/s', step: 10, help: 'Top spin speed. Thrust = kT·speed², and kT is chosen so top speed gives max thrust.' },
  { key: 'motor_torque_per_thrust_m', label: 'Spin-drag ratio kQ/kT', unit: 'm', step: 0.001, help: 'Propeller drag torque per newton of thrust. This is what makes yaw possible.' },
  { key: 'drag_quadratic_n_s2_m2', label: 'Quadratic drag', unit: 'N·s²/m²', step: 0.005, help: 'Drag that grows with speed squared; it matters at high speed.' },
  { key: 'angular_drag_nm_s_rad', label: 'Spin damping', unit: 'N·m·s/rad', step: 0.0005, help: 'Air resistance to rotating.' },
];

const GAIN_HELP: Record<string, string> = {
  alt_kp: 'Altitude P: push harder the further you are from the target height.',
  alt_ki: 'Altitude I: slowly fixes small lasting height errors.',
  alt_kd: 'Altitude D: brakes vertical speed (damping). 0 = bouncy.',
  att_kp: 'Angle P: how fast to rotate back toward the wanted roll/pitch.',
  yaw_kp: 'Heading P: how fast to turn toward the wanted yaw.',
  rate_kp: 'Spin-rate P: how hard motors fight unwanted rotation.',
  rate_ki: 'Spin-rate I.',
  rate_kd: 'Spin-rate D.',
  yaw_rate_kp: 'Yaw spin-rate P.',
  yaw_rate_ki: 'Yaw spin-rate I.',
  pos_kp: 'Position P: wanted speed per metre of position error.',
  vel_kp: 'Speed P: how much to tilt per m/s of speed error.',
  vel_ki: 'Speed I: removes steady drift (e.g. from wind).',
};

function NumberSlider(props: {
  label: string;
  unit: string;
  help: string;
  value: number;
  min: number;
  max: number;
  step: number;
  onChange: (v: number) => void;
}) {
  const { label, unit, help, value, min, max, step, onChange } = props;
  return (
    <label className="param" title={help}>
      <span className="param-head">
        <span>{label}</span>
        <span className="param-value">
          <input type="number" value={value} min={min} max={max} step={step} onChange={(e) => onChange(Number(e.target.value))} />
          <span className="unit">{unit}</span>
        </span>
      </span>
      <input type="range" value={value} min={min} max={max} step={step} onChange={(e) => onChange(Number(e.target.value))} />
      <span className="help">{help}</span>
    </label>
  );
}

export default function Studio({ onLearn }: { onLearn: (slug: string) => void }) {
  const { config } = useSimUi();
  const [catalog, setCatalog] = useState<PresetsResponse | null>(null);
  const [draft, setDraft] = useState<DroneConfig | null>(null);
  const [advanced, setAdvanced] = useState(false);
  const [problems, setProblems] = useState<string[]>([]);

  useEffect(() => {
    api.presets().then(setCatalog).catch((e) => setProblems([String(e)]));
  }, []);
  useEffect(() => {
    if (config) setDraft(structuredClone(config)); // server config changed (connect / reset)
  }, [config]);

  const tw = draft ? (4 * draft.motor_max_thrust_n) / (draft.mass_kg * GRAVITY_M_S2) : 0;
  const changed = useMemo(() => JSON.stringify(strip(draft)) !== JSON.stringify(strip(config)), [draft, config]);

  if (!draft || !catalog) return <section className="panel studio"><h2>Drone Studio</h2><p className="muted">Connecting…</p></section>;

  const set = (key: keyof DroneConfig, v: number) => setDraft({ ...draft, [key]: v });
  const setGain = (key: string, v: number) => setDraft({ ...draft, gains: { ...draft.gains, [key]: v } });

  const loadPreset = (id: string) => {
    const preset = catalog.presets[id];
    if (preset) setDraft(structuredClone(preset));
    setProblems([]);
  };

  const apply = async () => {
    const res = await api.validate(strip(draft) as DroneConfig);
    setProblems(res.problems);
    if (res.valid) sim.send('reset', { config: strip(draft) });
  };

  const renderParam = (p: Param) => {
    const [min, max] = catalog.limits[p.key as string] ?? [0, 1];
    return (
      <NumberSlider key={p.key} label={p.label} unit={p.unit} help={p.help} value={draft[p.key] as number}
        min={min} max={max} step={p.step} onChange={(v) => set(p.key, v)} />
    );
  };

  return (
    <section className="panel studio">
      <h2>Drone Studio</h2>
      <label className="preset">
        <span>Preset</span>
        <select value="" onChange={(e) => loadPreset(e.target.value)}>
          <option value="" disabled>Load a preset…</option>
          {Object.entries(catalog.presets).map(([id, p]) => (
            <option key={id} value={id}>{p.name} (T/W {p.derived?.thrust_to_weight.toFixed(1)})</option>
          ))}
        </select>
      </label>
      <p className="muted small">Current: <b>{draft.name}</b></p>

      <div className={`tw ${tw < 1 ? 'bad' : tw < 1.5 ? 'warn' : 'good'}`} title="Thrust-to-weight: max total thrust divided by weight. It must be above 1 to fly.">
        <div className="tw-value">T/W {tw.toFixed(2)}</div>
        <div className="tw-note">
          {tw < 1 ? "Can't lift off: thrust is less than weight." : `Hover needs about ${(100 / tw).toFixed(0)} % throttle.`}
        </div>
      </div>

      {KEY_PARAMS.map(renderParam)}

      <button className="link" onClick={() => setAdvanced(!advanced)}>{advanced ? '▾ Hide advanced' : '▸ Advanced (more physics + PID gains)'}</button>
      {advanced && (
        <div className="advanced">
          {ADVANCED_PARAMS.map(renderParam)}
          <h3>Controller gains</h3>
          {Object.keys(draft.gains).map((k) => {
            const [min, max] = catalog.gain_limits[k] ?? [0, 10];
            return <NumberSlider key={k} label={k} unit="" help={GAIN_HELP[k] ?? ''} value={draft.gains[k]}
              min={min} max={max} step={(max - min) / 200} onChange={(v) => setGain(k, v)} />;
          })}
        </div>
      )}

      {problems.length > 0 && <ul className="problems">{problems.map((p) => <li key={p}>{p}</li>)}</ul>}
      <div className="row">
        <button className="primary" onClick={apply} disabled={!changed}>Apply &amp; reset sim</button>
        <button onClick={() => config && setDraft(structuredClone(config))} disabled={!changed}>Undo changes</button>
      </div>
      <button className="link" onClick={() => onLearn('01-thrust-and-hover')}>📖 Learn: thrust and hover</button>
    </section>
  );
}

/** Drop computed fields before comparing or sending. */
function strip(cfg: DroneConfig | null): Omit<DroneConfig, 'derived'> | null {
  if (!cfg) return null;
  const { derived: _derived, ...rest } = cfg;
  return rest;
}
