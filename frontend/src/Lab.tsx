// Engineering Lab: pick an experiment, set 2–4 sliders, run it headless, read the results.

import { useEffect, useRef, useState } from 'react';
import { assistantStore, useAssistant } from './assistant';
import { labApi, type ExperimentDef, type Params, type RunResult } from './lab';
import RunView from './RunView';

function defaults(exp: ExperimentDef): Params {
  return Object.fromEntries(exp.params.map((p) => [p.id, p.default]));
}

/** Short label like "PID tuning (kp 25, kd 3.5)" listing only settings that differ from the defaults. */
function runLabel(exp: ExperimentDef, params: Params): string {
  const changed = exp.params.filter((p) => params[p.id] !== p.default).map((p) => `${p.id} ${params[p.id]}`);
  return `${exp.title}${changed.length ? ` (${changed.join(', ')})` : ' (defaults)'}`;
}

export default function Lab({ onLearn }: { onLearn: (slug: string) => void }) {
  const [experiments, setExperiments] = useState<ExperimentDef[]>([]);
  const [selected, setSelected] = useState<string>('thrust_takeoff');
  const [params, setParams] = useState<Params>({});
  const [seed, setSeed] = useState(0);
  const [result, setResult] = useState<RunResult | null>(null);
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState('');
  const [label, setLabel] = useState('');
  const [saveMsg, setSaveMsg] = useState('');

  useEffect(() => {
    labApi.experiments().then(setExperiments).catch((e) => setError(String(e.message ?? e)));
  }, []);

  const exp = experiments.find((e) => e.id === selected);
  const { pendingApply } = useAssistant();
  const applyRef = useRef<{ experiment_id: string; params: Params } | null>(null);

  // An AI suggestion the user clicked "Apply" on: switch to that experiment and run it.
  useEffect(() => {
    if (!pendingApply) return;
    const p = assistantStore.takePending();
    if (!p) return;
    if (exp && p.experiment_id === exp.id) {
      setParams(p.params);
      run(p.params);
    } else {
      applyRef.current = p;
      setSelected(p.experiment_id);
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [pendingApply]);

  useEffect(() => {
    if (!exp) return;
    setResult(null);
    setError('');
    setSaveMsg('');
    const pending = applyRef.current;
    if (pending && pending.experiment_id === exp.id) {
      applyRef.current = null;
      setParams(pending.params);
      run(pending.params);
    } else {
      setParams(defaults(exp));
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [exp]);

  const run = async (p: Params = params) => {
    if (!exp) return;
    setBusy(true);
    setError('');
    setSaveMsg('');
    try {
      const res = await labApi.run(exp.id, p, seed);
      setResult(res);
      const used = res.scenario.params ?? p;
      // Let the assistant "see" this run (it re-creates it server-side from these settings).
      assistantStore.setContext('the Lab', [{ experiment_id: exp.id, params: used, seed: res.seed, label: runLabel(exp, used) }]);
    } catch (e) {
      setError((e as Error).message);
    } finally {
      setBusy(false);
    }
  };

  const tryNext = (changes: Params) => {
    const p = { ...params, ...changes };
    setParams(p);
    run(p);
  };

  const save = async () => {
    if (!exp || !result) return;
    try {
      const saved = await labApi.save(exp.id, result.scenario.params ?? params, result.seed, label);
      setSaveMsg(`Saved as "${saved.label}". Find it in History.`);
      setLabel('');
    } catch (e) {
      setSaveMsg(`Not saved: ${(e as Error).message}`);
    }
  };

  return (
    <div className="lab">
      <aside className="panel exp-list">
        <h2>Experiments</h2>
        {experiments.map((e) => (
          <button key={e.id} className={`exp-card ${e.id === selected ? 'active' : ''}`} onClick={() => setSelected(e.id)}>
            <span className="exp-num">{e.number}</span>
            <span>{e.title}</span>
          </button>
        ))}
      </aside>

      <section className="lab-main">
        {!exp ? (
          <p className="muted">{error || 'Loading experiments…'}</p>
        ) : (
          <>
            <header className="exp-head">
              <h2>{exp.number}. {exp.title}</h2>
              <p className="objective">🎯 {exp.objective}</p>
              <ul className="concept">{exp.concept.map((c, i) => <li key={i}>{c}</li>)}</ul>
              <button className="link" onClick={() => onLearn(exp.learn_slug)}>📖 Read the lesson for this experiment</button>
            </header>

            <section className="card sliders">
              {exp.params.map((p) => (
                <label key={p.id} className="param" title={p.help}>
                  <span className="param-head">
                    <span>{p.label}</span>
                    {p.options.length === 0 && (
                      <span className="param-value">
                        <input type="number" value={params[p.id] ?? p.default} min={p.min} max={p.max} step={p.step}
                          onChange={(e) => setParams({ ...params, [p.id]: Number(e.target.value) })} />
                        <span className="unit">{p.unit}</span>
                      </span>
                    )}
                  </span>
                  {p.options.length > 0 ? (
                    <select value={String(params[p.id] ?? p.default)} onChange={(e) => setParams({ ...params, [p.id]: e.target.value })}>
                      {p.options.map(([v, l]) => <option key={v} value={v}>{l}</option>)}
                    </select>
                  ) : (
                    <input type="range" value={params[p.id] ?? p.default} min={p.min} max={p.max} step={p.step}
                      onChange={(e) => setParams({ ...params, [p.id]: Number(e.target.value) })} />
                  )}
                  <span className="help">{p.help}</span>
                </label>
              ))}
              <div className="row">
                <button className="primary big" disabled={busy} onClick={() => run()}>{busy ? 'Running…' : '▶ Run experiment'}</button>
                <button onClick={() => setParams(defaults(exp))} disabled={busy}>Reset sliders</button>
                <span className="spacer" />
                <label className="inline small" title="Random seed for noise and gusts. Same seed = same result.">
                  seed <input type="number" min={0} step={1} value={seed} onChange={(e) => setSeed(Math.max(0, Math.floor(Number(e.target.value))))} />
                </label>
              </div>
              {error && <p className="error-text">{error}</p>}
            </section>

            {result && (
              <>
                <section className="card save-row row">
                  <input className="label-input" placeholder="Label (e.g. 'Kp 25, wobbly')" value={label} maxLength={80}
                    onChange={(e) => setLabel(e.target.value)} />
                  <button onClick={save}>💾 Save run</button>
                  <span className="muted small">{saveMsg}</span>
                </section>
                <RunView result={result} experiment={exp} name={`${exp.id}-${label || 'run'}`} onTryNext={tryNext} />
              </>
            )}
          </>
        )}
      </section>
    </div>
  );
}
