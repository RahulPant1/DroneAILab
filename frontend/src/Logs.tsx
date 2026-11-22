// 🩺 Logs: the flight-log doctor. Pick an ArduPilot log (from the simulator, or upload a real drone's .BIN),
// see the engineer's checks, the timeline and charts, and ask the local AI for a debrief.

import { useEffect, useRef, useState } from 'react';
import uPlot from 'uplot';
import 'uplot/dist/uPlot.min.css';

interface LogItem {
  id: string;
  name: string;
  source: 'simulator' | 'uploaded';
  size_mb: number;
  modified: string;
}
interface Check {
  id: string;
  title: string;
  status: 'ok' | 'warn' | 'bad' | 'info';
  value: string;
  limit: string;
  meaning: string;
}
interface Analysis {
  id: string;
  overview: {
    file: string;
    log_duration_s: number;
    armed_time_s: number;
    max_altitude_m: number | null;
    max_ground_speed_m_s: number | null;
    modes_used: string[];
    frame: string | null;
    simulated: boolean;
  };
  verdict: 'ok' | 'warn' | 'bad';
  checks: Check[];
  timeline: { t: number; kind: string; text: string }[];
  timeline_total: number;
  charts: Record<string, { labels: string[]; data: (number | null)[][] }>;
}
interface Debrief {
  answer: string;
  model: string;
  unverified_numbers: string[];
  duration_s: number;
}

const STATUS_ICON = { ok: '✅', warn: '⚠️', bad: '❌', info: 'ℹ️' } as const;
const CHART_TITLES: Record<string, string> = {
  altitude: 'Altitude: desired vs estimated (m)',
  vibration: 'Vibration (m/s², keep under 30)',
  motors: 'Motor outputs (PWM µs, 1000 = off, 2000 = full)',
  ekf: 'EKF test ratios (over 1 = reading rejected)',
  truth: 'Belief vs truth: EKF position error (m, simulator only)',
};
const COLORS = ['#4cc9f0', '#ff8c42', '#06d6a0', '#c77dff', '#ffd166', '#ef476f', '#8ecae6', '#f4a261'];

async function jsonOrError<T>(resp: Response): Promise<T> {
  if (!resp.ok) {
    let detail = `HTTP ${resp.status}`;
    try {
      detail = (await resp.json()).detail ?? detail;
    } catch {
      /* not JSON */
    }
    throw new Error(detail);
  }
  return resp.json() as Promise<T>;
}

export default function Logs({ initialId }: { initialId: string | null }) {
  const [logs, setLogs] = useState<LogItem[]>([]);
  const [selected, setSelected] = useState<string | null>(initialId);
  const [analysis, setAnalysis] = useState<Analysis | null>(null);
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState('');
  const [debrief, setDebrief] = useState<Debrief | null>(null);
  const [aiBusy, setAiBusy] = useState(false);
  const [aiError, setAiError] = useState('');
  const fileInput = useRef<HTMLInputElement>(null);

  const refresh = () =>
    fetch('/api/logs').then((r) => jsonOrError<{ logs: LogItem[] }>(r)).then((r) => {
      setLogs(r.logs);
      setSelected((cur) => cur ?? r.logs[0]?.id ?? null);
    }).catch((e) => setError(String(e.message ?? e)));

  useEffect(() => {
    refresh();
  }, []);
  useEffect(() => {
    if (initialId) setSelected(initialId);
  }, [initialId]);
  useEffect(() => {
    if (!selected) return;
    setBusy(true);
    setError('');
    setAnalysis(null);
    setDebrief(null);
    setAiError('');
    fetch(`/api/logs/${selected}/analysis`).then((r) => jsonOrError<Analysis>(r)).then(setAnalysis)
      .catch((e) => setError(String(e.message ?? e))).finally(() => setBusy(false));
  }, [selected]);

  const upload = async (file: File) => {
    setError('');
    setBusy(true);
    try {
      const r = await jsonOrError<{ id: string }>(await fetch('/api/logs/upload', {
        method: 'POST', headers: { 'X-Filename': file.name, 'Content-Type': 'application/octet-stream' }, body: file,
      }));
      await refresh();
      setSelected(r.id);
    } catch (e) {
      setError(String((e as Error).message ?? e));
      setBusy(false);
    }
  };

  const explain = async () => {
    if (!selected) return;
    setAiBusy(true);
    setAiError('');
    try {
      setDebrief(await jsonOrError<Debrief>(await fetch(`/api/logs/${selected}/explain`, {
        method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({}),
      })));
    } catch (e) {
      setAiError(String((e as Error).message ?? e));
    } finally {
      setAiBusy(false);
    }
  };

  return (
    <div className="logs">
      <aside className="logs-list">
        <h3>🩺 Flight logs</h3>
        <p className="hint">ArduPilot writes a detailed log (.BIN) every flight. The simulator's appear here; you can also upload one from a real drone.</p>
        <button onClick={() => fileInput.current?.click()} title="An ArduPilot DataFlash log (.BIN), up to 100 MB. It stays on this computer.">⬆ Upload a .BIN log</button>
        <input ref={fileInput} type="file" accept=".bin,.BIN" hidden onChange={(e) => e.target.files?.[0] && upload(e.target.files[0])} />
        <ul>
          {logs.map((l) => (
            <li key={l.id}>
              <button className={`link ${l.id === selected ? 'active' : ''}`} onClick={() => setSelected(l.id)}>
                {l.source === 'uploaded' ? '⬆ ' : '🛰 '}{l.name}
              </button>
              <span className="keys">{l.modified.slice(5, 16)} · {l.size_mb} MB</span>
            </li>
          ))}
          {logs.length === 0 && <li className="hint">No logs yet: fly ArduPilot from the Fly tab (🛰 panel) or upload one.</li>}
        </ul>
      </aside>
      <section className="logs-main">
        {error && <p className="error">{error}</p>}
        {busy && <p>Reading the log…</p>}
        {analysis && (
          <>
            <div className="row">
              <h2>{STATUS_ICON[analysis.verdict]} {analysis.overview.file}</h2>
              <span className="keys">
                {analysis.overview.simulated ? 'simulator' : 'real drone'} · {analysis.overview.frame ?? 'unknown frame'} ·
                flew {analysis.overview.armed_time_s} s · up to {analysis.overview.max_altitude_m ?? '–'} m ·
                top speed {analysis.overview.max_ground_speed_m_s ?? '–'} m/s · modes {analysis.overview.modes_used.join(' → ') || '–'}
              </span>
            </div>
            <table className="checks">
              <thead><tr><th></th><th>Check</th><th>Measured</th><th>Healthy if</th></tr></thead>
              <tbody>
                {analysis.checks.map((c) => (
                  <tr key={c.id} className={c.status} title={c.meaning}>
                    <td>{STATUS_ICON[c.status]}</td>
                    <td>{c.title}</td>
                    <td>{c.value}</td>
                    <td className="keys">{c.limit}</td>
                  </tr>
                ))}
              </tbody>
            </table>
            <p className="hint">Hover a row for what the check means.</p>

            <div className="debrief">
              <button className="primary" disabled={aiBusy} onClick={explain}
                title="The local model gets only the summary above (checks + timeline), never the raw log. Numbers it writes are checked.">
                {aiBusy ? '🤖 thinking… (up to a minute)' : '🤖 Explain this flight (local AI)'}
              </button>
              {aiError && <p className="error">{aiError}</p>}
              {debrief && (
                <div className="answer">
                  <pre>{debrief.answer}</pre>
                  <p className="keys">
                    {debrief.model} · {debrief.duration_s} s ·{' '}
                    {debrief.unverified_numbers.length
                      ? <b className="warn">numbers not found in the log data: {debrief.unverified_numbers.join(', ')}</b>
                      : 'every number checked against the log data ✓'}
                  </p>
                </div>
              )}
            </div>

            <h3>Timeline</h3>
            <ul className="timeline">
              {analysis.timeline.map((e, i) => (
                <li key={i} className={e.kind}><b>{e.t.toFixed(1)} s</b> {e.text}</li>
              ))}
              {analysis.timeline_total > analysis.timeline.length && <li className="hint">… {analysis.timeline_total - analysis.timeline.length} more</li>}
            </ul>

            <div className="log-charts">
              {Object.entries(analysis.charts).map(([key, ch]) => <LogChart key={`${analysis.id}-${key}`} title={CHART_TITLES[key] ?? key} chart={ch} />)}
            </div>
          </>
        )}
      </section>
    </div>
  );
}

function LogChart({ title, chart }: { title: string; chart: { labels: string[]; data: (number | null)[][] } }) {
  const wrap = useRef<HTMLDivElement>(null);
  useEffect(() => {
    const el = wrap.current;
    if (!el || chart.data[0].length === 0) return;
    const plot = new uPlot(
      {
        width: Math.min(el.clientWidth || 460, 560),
        height: 170,
        scales: { x: { time: false } },
        axes: [
          { label: 'time in log (s)', stroke: '#9aa5b8', grid: { stroke: '#2c3648' } },
          { stroke: '#9aa5b8', grid: { stroke: '#2c3648' } },
        ],
        series: [{ label: 't' }, ...chart.labels.map((l, i) => ({ label: l, stroke: COLORS[i % COLORS.length], width: 1.5, spanGaps: true }))],
      },
      chart.data as uPlot.AlignedData,
      el,
    );
    return () => plot.destroy();
  }, [chart]);
  return (
    <div className="log-chart">
      <h4>{title}</h4>
      <div ref={wrap} />
    </div>
  );
}
