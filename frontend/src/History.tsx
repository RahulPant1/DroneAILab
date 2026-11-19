// Saved runs: list, view, delete, and compare two runs on overlaid plots.

import { useCallback, useEffect, useMemo, useState } from 'react';
import Chart, { type ChartRun } from './Chart';
import { exportCsv, exportJson, formatValue, labApi, type Comparison, type ExperimentDef, type SavedRun, type SavedRunSummary } from './lab';
import RunView from './RunView';

function CompareView({ cmp, experiments }: { cmp: Comparison; experiments: ExperimentDef[] }) {
  const runs: ChartRun[] = useMemo(() => [{ name: 'A', result: cmp.a.result }, { name: 'B', result: cmp.b.result }], [cmp]);
  const sameExperiment = cmp.a.result.run_type === cmp.b.result.run_type;
  const exp = experiments.find((e) => e.id === cmp.a.result.run_type);
  const keyOrder = exp?.key_metrics.map((m) => m.key) ?? [];
  const rows = [...cmp.metrics].sort((x, y) => {
    const ix = keyOrder.indexOf(x.metric), iy = keyOrder.indexOf(y.metric);
    return (ix < 0 ? 999 : ix) - (iy < 0 ? 999 : iy);
  });
  const differingParams = cmp.params.filter((p) => JSON.stringify(p.a) !== JSON.stringify(p.b));

  return (
    <div className="run-view">
      <p>
        <b>A</b> = {cmp.a.label} (solid) &nbsp;·&nbsp; <b>B</b> = {cmp.b.label} (dashed)
        {!sameExperiment && <span className="error-text"> These runs come from different experiments; only shared signals are drawn.</span>}
      </p>
      <div className="results-grid">
        <section className="card">
          <h3>Settings that differ</h3>
          {differingParams.length === 0 ? <p className="muted small">Same settings (different seed or nothing changed).</p> : (
            <table className="kv"><thead><tr><th></th><th>A</th><th>B</th></tr></thead>
              <tbody>{differingParams.map((p) => <tr key={p.param}><th>{p.param}</th><td>{formatValue(p.a)}</td><td>{formatValue(p.b)}</td></tr>)}</tbody>
            </table>
          )}
        </section>
        <section className="card">
          <h3>Metrics</h3>
          <table className="kv"><thead><tr><th></th><th>A</th><th>B</th><th>B − A</th></tr></thead>
            <tbody>
              {rows.map((r) => (
                <tr key={r.metric} className={keyOrder.includes(r.metric) ? 'key' : ''}>
                  <th>{r.metric}</th><td>{formatValue(r.a, 3)}</td><td>{formatValue(r.b, 3)}</td>
                  <td>{r.diff === null ? '' : (r.diff > 0 ? '+' : '') + r.diff.toFixed(3)}</td>
                </tr>
              ))}
            </tbody>
          </table>
        </section>
      </div>
      <div className="charts">{cmp.a.result.plots.map((p) => <Chart key={p.id} plot={p} runs={runs} />)}</div>
    </div>
  );
}

export default function History() {
  const [runs, setRuns] = useState<SavedRunSummary[] | null>(null);
  const [experiments, setExperiments] = useState<ExperimentDef[]>([]);
  const [picked, setPicked] = useState<string[]>([]);
  const [view, setView] = useState<SavedRun | null>(null);
  const [cmp, setCmp] = useState<Comparison | null>(null);
  const [msg, setMsg] = useState('');

  const refresh = useCallback(() => {
    labApi.list().then(setRuns).catch((e) => setMsg(String(e.message ?? e)));
  }, []);
  useEffect(() => {
    refresh();
    labApi.experiments().then(setExperiments).catch(() => undefined);
  }, [refresh]);

  const expOf = (id: string) => experiments.find((e) => e.id === id);
  const toggle = (id: string) =>
    setPicked((p) => (p.includes(id) ? p.filter((x) => x !== id) : [...p, id].slice(-2)));

  const open = async (id: string) => {
    setCmp(null);
    setView(await labApi.get(id));
  };
  const compare = async () => {
    setView(null);
    setCmp(await labApi.compare(picked[0], picked[1]));
  };
  const remove = async (id: string) => {
    if (!window.confirm('Delete this saved run?')) return;
    await labApi.remove(id);
    setPicked((p) => p.filter((x) => x !== id));
    if (view?.id === id) setView(null);
    setCmp(null);
    refresh();
  };
  const deleteOld = async () => {
    const keep = Number(window.prompt('Keep how many of the newest runs?', '20'));
    if (!Number.isFinite(keep) || keep < 0) return;
    const { deleted } = await labApi.deleteOld(Math.floor(keep));
    setMsg(`Deleted ${deleted} old run(s).`);
    setPicked([]);
    setView(null);
    setCmp(null);
    refresh();
  };

  return (
    <div className="history">
      <section className="card">
        <div className="row">
          <h2>Saved runs</h2>
          <span className="muted small">{runs ? `${runs.length} / 100` : ''}</span>
          <span className="spacer" />
          <button className="primary" disabled={picked.length !== 2} onClick={compare}>Compare 2 selected</button>
          <button onClick={deleteOld} disabled={!runs?.length}>Delete old runs…</button>
        </div>
        {msg && <p className="muted small">{msg}</p>}
        {runs && runs.length === 0 && <p className="muted">No saved runs yet. Run an experiment in the Lab and click "Save run".</p>}
        {runs && runs.length > 0 && (
          <table className="runs">
            <thead><tr><th></th><th>Label</th><th>Experiment</th><th>Saved</th><th>Key results</th><th></th></tr></thead>
            <tbody>
              {runs.map((r) => {
                const e = expOf(r.experiment_id);
                return (
                  <tr key={r.id} className={picked.includes(r.id) ? 'picked' : ''}>
                    <td><input type="checkbox" checked={picked.includes(r.id)} onChange={() => toggle(r.id)} title="Select for comparison" /></td>
                    <td><button className="link" onClick={() => open(r.id)}>{r.label}</button></td>
                    <td>{e ? `${e.number}. ${e.title}` : r.experiment_id}</td>
                    <td className="muted small">{new Date(r.saved_at).toLocaleString()}</td>
                    <td className="small">
                      {(e?.key_metrics ?? []).slice(0, 3).map((m) => `${m.label}: ${formatValue(r.metrics[m.key], m.digits)}${m.unit ? ' ' + m.unit : ''}`).join(' · ')}
                    </td>
                    <td><button className="small-btn" onClick={() => remove(r.id)} title="Delete">🗑</button></td>
                  </tr>
                );
              })}
            </tbody>
          </table>
        )}
      </section>

      {view && (
        <section>
          <div className="row"><h2>{view.label}</h2><span className="spacer" />
            <button onClick={() => exportJson(view.label, view)}>⬇ JSON</button>
            <button onClick={() => exportCsv(view.label, view.result)}>⬇ CSV</button>
            <button onClick={() => setView(null)}>Close</button>
          </div>
          <RunView result={view.result} experiment={expOf(view.result.run_type)} name={view.label} />
        </section>
      )}
      {cmp && (
        <section>
          <div className="row"><h2>Comparison</h2><span className="spacer" /><button onClick={() => setCmp(null)}>Close</button></div>
          <CompareView cmp={cmp} experiments={experiments} />
        </section>
      )}
    </div>
  );
}
