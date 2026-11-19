// Shows one run's results: key metrics, "what happened", events, "try next", and plots.

import { useMemo } from 'react';
import Chart, { type ChartRun } from './Chart';
import { exportCsv, exportJson, formatValue, type ExperimentDef, type Params, type RunResult } from './lab';

export default function RunView(props: {
  result: RunResult;
  experiment?: ExperimentDef;
  name: string;
  onTryNext?: (params: Params) => void;
}) {
  const { result, experiment, name, onTryNext } = props;
  const runs: ChartRun[] = useMemo(() => [{ name: '', result }], [result]);

  return (
    <div className="run-view">
      <div className="results-grid">
        <section className="card">
          <h3>Key results</h3>
          <table className="kv">
            <tbody>
              {(experiment?.key_metrics ?? []).map((m) => (
                <tr key={m.key}>
                  <th>{m.label}</th>
                  <td>
                    {formatValue(result.metrics[m.key], m.digits)} <span className="unit">{m.unit}</span>
                  </td>
                </tr>
              ))}
            </tbody>
          </table>
          <div className="row small">
            <button onClick={() => exportCsv(name, result)} title="Telemetry as a spreadsheet (one column per signal)">⬇ CSV</button>
            <button onClick={() => exportJson(name, result)} title="Everything about this run">⬇ JSON</button>
          </div>
        </section>
        <section className="card">
          <h3>What happened</h3>
          <ul className="summary">{result.summary.map((line, i) => <li key={i}>{line}</li>)}</ul>
          {result.events.length > 0 && (
            <>
              <h3>Events</h3>
              <ul className="events">
                {result.events.map((e, i) => <li key={i}><span className="muted">t = {e.t_s.toFixed(2)} s</span> {e.message}</li>)}
              </ul>
            </>
          )}
          {onTryNext && result.try_next.length > 0 && (
            <>
              <h3>Try next</h3>
              <ul className="try-next">
                {result.try_next.map((tn, i) => (
                  <li key={i}>
                    {tn.text}{' '}
                    {Object.keys(tn.params).length > 0 && <button className="small-btn" onClick={() => onTryNext(tn.params)}>Try it ▶</button>}
                  </li>
                ))}
              </ul>
            </>
          )}
        </section>
      </div>
      <div className="charts">{result.plots.map((p) => <Chart key={p.id} plot={p} runs={runs} />)}</div>
    </div>
  );
}
