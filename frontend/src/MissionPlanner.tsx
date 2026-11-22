// 🧭 Mission planner: say what the drone should do (or build the task list by hand), see the route and the
// safety checks, then fly it in DroneLab and/or on real ArduPilot. The AI only writes the task list: the
// server computes the route and runs every check, and re-checks again before anything flies.

import { useEffect, useMemo, useState } from 'react';
import FarmMap from './FarmMap';
import { api, sim, useThrottledState, type AiPlanReply, type FlightPlan, type PlanSpec, type PlanTask, type Site } from './sim';

const EXAMPLES = [
  'Survey the north field, then land on pad B',
  'Map the east field from 12 m and hover over the gate for 10 seconds on the way back',
  'Check the orchard at 25 m, then fly over the horse paddock',
  'Fly to the school and back',
  'Survey all three fields at 8 m',
];

const DEFAULT_SPEC: PlanSpec = { tasks: [{ type: 'survey', field: 'north field' }], altitude_m: 20, speed_m_s: 3, side_overlap_pct: 70 };
const ICON = { pass: '✅', warn: '⚠️', fail: '❌' } as const;

function TaskRow({ t, i, site, set, remove, last }: { t: PlanTask; i: number; site: Site; set: (t: PlanTask) => void; remove: () => void; last: boolean }) {
  const places = [...site.pads.map((p) => p.name), ...site.places.map((p) => p.name), ...site.fields.map((f) => f.name)];
  const change = (type: PlanTask['type']) => {
    if (type === 'survey') set({ type, field: site.fields[0].name });
    else if (type === 'land') set({ type, place: 'home', precision: true });
    else set({ type, place: t.place && places.includes(t.place) ? t.place : places[2], ...(type === 'hover' ? { seconds: 10 } : {}) });
  };
  return (
    <div className="plan-task">
      <span className="muted">{i + 1}.</span>
      <select value={t.type} onChange={(e) => change(e.target.value as PlanTask['type'])}>
        <option value="survey">📷 Survey</option>
        <option value="goto">➡ Fly to</option>
        <option value="hover">⏸ Hover at</option>
        <option value="land" disabled={!last}>🛬 Land on</option>
      </select>
      {t.type === 'survey' ? (
        <select value={t.field} onChange={(e) => set({ ...t, field: e.target.value })}>
          {site.fields.map((f) => <option key={f.name}>{f.name}</option>)}
        </select>
      ) : (
        <select value={t.place} onChange={(e) => set({ ...t, place: e.target.value })}>
          {(t.type === 'land' ? site.pads.map((p) => p.name) : places).map((n) => <option key={n}>{n}</option>)}
        </select>
      )}
      {t.type === 'hover' && (
        <label className="inline"><input type="number" min={1} max={60} value={t.seconds ?? 10} onChange={(e) => set({ ...t, seconds: Number(e.target.value) })} /> s</label>
      )}
      {t.type === 'land' && (
        <label className="inline" title="Use the down camera to find the pad's marker (precision landing)">
          <input type="checkbox" checked={t.precision ?? true} onChange={(e) => set({ ...t, precision: e.target.checked })} /> 🎯 camera
        </label>
      )}
      <button className="link" onClick={remove} title="Remove this task">✕</button>
    </div>
  );
}

export default function MissionPlanner({ onClose, onLearn }: { onClose: () => void; onLearn: (slug: string) => void }) {
  const s = useThrottledState(2);
  const [site, setSite] = useState<Site | null>(null);
  const [request, setRequest] = useState('');
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState('');
  const [ai, setAi] = useState<AiPlanReply | null>(null);
  const [spec, setSpec] = useState<PlanSpec>(DEFAULT_SPEC);
  const [plan, setPlan] = useState<FlightPlan | null>(null);
  const [checkError, setCheckError] = useState('');
  const battery = s?.battery ? { cells: s.battery.cells, capacity_mah: s.battery.capacity_mah } : undefined;
  const batteryKey = battery ? `${battery.cells}:${battery.capacity_mah}` : '';

  useEffect(() => {
    api.site().then(setSite).catch((e) => setError(String(e)));
    const esc = (e: KeyboardEvent) => e.key === 'Escape' && onClose();
    window.addEventListener('keydown', esc);
    return () => window.removeEventListener('keydown', esc);
  }, [onClose]);

  // Re-check the plan whenever the task list changes (also after the AI fills it in).
  useEffect(() => {
    const id = window.setTimeout(() => {
      api.compilePlan(spec, battery).then((p) => { setPlan(p); setCheckError(''); }).catch((e: Error) => { setPlan(null); setCheckError(e.message); });
    }, 200);
    return () => window.clearTimeout(id);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [spec, batteryKey]);

  const askAi = async (text: string) => {
    if (!text.trim()) return;
    setBusy(true);
    setError('');
    setAi(null);
    try {
      const r = await api.aiPlan(text, battery);
      setAi(r);
      if (r.ok && r.spec) setSpec(r.spec);
      else setError(r.error ?? 'The AI could not make a plan.');
    } catch (e) {
      setError(`${(e as Error).message}. You can still build the plan by hand below.`);
    } finally {
      setBusy(false);
    }
  };

  const setTask = (i: number, t: PlanTask) => setSpec({ ...spec, tasks: spec.tasks.map((x, k) => (k === i ? t : x)) });
  const removeTask = (i: number) => setSpec({ ...spec, tasks: spec.tasks.filter((_, k) => k !== i) });
  const addTask = () => {
    const tasks = [...spec.tasks];
    const landing = tasks.length && tasks[tasks.length - 1].type === 'land' ? tasks.pop() : null;
    tasks.push({ type: 'goto', place: 'pond' });
    setSpec({ ...spec, tasks: landing ? [...tasks, landing] : tasks });
  };
  const sitlReady = s?.sitl?.state === 'ready';
  const notOnMap = ai?.ai?.not_on_map ?? [];
  const extraFails = useMemo(() => (ai?.plan?.checks ?? []).filter((c) => c.id === 'names' || c.id === 'left_out'), [ai]);
  // The AI's own checks (places not on the map, things it left out) only apply while its task list is unchanged.
  const aiSpecUnchanged = ai?.spec && JSON.stringify(ai.spec) === JSON.stringify(spec);
  const checks = [...(aiSpecUnchanged ? extraFails : []), ...(plan?.checks ?? [])];
  const flyable = !!plan?.ok && !(aiSpecUnchanged && notOnMap.length);

  if (!site) return null;
  return (
    <div className="modal-backdrop" onClick={onClose}>
      <div className="modal planner" onClick={(e) => e.stopPropagation()} role="dialog" aria-label="Mission planner">
        <button className="close" onClick={onClose}>✕</button>
        <h2>🧭 Mission planner · {site.title}</h2>
        <p className="muted small">
          Say what the drone should do. The local AI turns it into a task list using only names on the map; the code then plans
          the route and runs the safety checks. You decide whether it flies.{' '}
          <button className="link" onClick={() => onLearn('19-ai-mission-planner')}>📖 How it works</button>
        </p>
        <div className="row planner-ask">
          <textarea rows={2} value={request} placeholder="e.g. Survey the north field at 25 m, then land on pad B"
            onChange={(e) => setRequest(e.target.value)}
            onKeyDown={(e) => { if (e.key === 'Enter' && !e.shiftKey) { e.preventDefault(); askAi(request); } }} />
          <button className="primary" disabled={busy || !request.trim()} onClick={() => askAi(request)}>{busy ? '🧠 Planning…' : '🧠 Plan with AI'}</button>
        </div>
        <div className="row small planner-examples">
          {EXAMPLES.map((ex) => <button key={ex} className="chip" disabled={busy} onClick={() => { setRequest(ex); askAi(ex); }}>{ex}</button>)}
        </div>
        {error && <p className="error small">{error}</p>}
        {ai?.ok && ai.ai && (
          <div className="ai-understood small">
            <b>AI ({ai.model}, {ai.duration_s} s):</b> {ai.ai.understood}
            {ai.ai.assumptions.length > 0 && <ul>{ai.ai.assumptions.map((a, i) => <li key={i} className="muted">Assumed: {a}</li>)}</ul>}
            {(ai.unverified_numbers?.length ?? 0) > 0 && (
              <p className="warn">⚠ Numbers in the AI's words that aren't in your request, the map or the plan: {ai.unverified_numbers!.join(', ')}. Trust the checks below, not its words.</p>
            )}
          </div>
        )}

        <div className="planner-grid">
          <div>
            <h3>Tasks</h3>
            {spec.tasks.map((t, i) => (
              <TaskRow key={i} t={t} i={i} site={site} last={i === spec.tasks.length - 1} set={(nt) => setTask(i, nt)} remove={() => removeTask(i)} />
            ))}
            <div className="row small">
              <button onClick={addTask} disabled={spec.tasks.length >= 8}>+ task</button>
              {(!spec.tasks.length || spec.tasks[spec.tasks.length - 1].type !== 'land') && <span className="muted">then: fly home and land</span>}
            </div>
            <div className="row small plan-values">
              <label className="inline">altitude <input type="number" value={spec.altitude_m} step={1} onChange={(e) => setSpec({ ...spec, altitude_m: Number(e.target.value) })} /> m</label>
              <label className="inline">speed <input type="number" value={spec.speed_m_s} step={0.5} onChange={(e) => setSpec({ ...spec, speed_m_s: Number(e.target.value) })} /> m/s</label>
              <label className="inline" title="How much neighbouring survey lines' photos overlap">overlap <input type="number" value={spec.side_overlap_pct} step={5} onChange={(e) => setSpec({ ...spec, side_overlap_pct: Number(e.target.value) })} /> %</label>
            </div>
            <h3>Safety checks</h3>
            {checkError && <p className="error small">{checkError}</p>}
            <ul className="plan-checks">
              {checks.map((c, i) => <li key={i} className={`chk-${c.status}`}>{ICON[c.status]} <b>{c.label}:</b> {c.detail}</li>)}
            </ul>
            {plan && (
              <p className="small">
                {plan.estimate.distance_m.toFixed(0)} m · about {(plan.estimate.time_s / 60).toFixed(1)} min · ~{plan.estimate.battery_used_pct.toFixed(0)} % battery
                {plan.estimate.photos > 0 && <> · ~{plan.estimate.photos} photos ({plan.surveys.map((sv) => `${sv.gsd_cm.toFixed(1)} cm/pixel`).join(', ')})</>}
              </p>
            )}
          </div>
          <FarmMap site={site} plan={plan} />
        </div>

        <div className="row">
          <button className="primary big" disabled={!flyable} onClick={() => { sim.send('plan', { spec }); onClose(); }}
            title={flyable ? 'Fly it in DroneLab (the server checks it again first)' : 'Fix the failed checks first'}>
            ✈ Fly in DroneLab
          </button>
          <button disabled={!flyable || !sitlReady} onClick={() => sim.send('sitl', { action: 'plan', spec })}
            title={sitlReady ? 'Upload it to real ArduPilot (SITL) as an AUTO mission and fly it there too' : 'Start ArduPilot in the 🛰 ArduPilot panel first'}>
            🛰 Fly on ArduPilot too
          </button>
          <span className="muted small">{!flyable && plan ? 'A failed check blocks flying.' : 'Tip: ⏩ speeds up time while it flies.'}</span>
        </div>
      </div>
    </div>
  );
}
