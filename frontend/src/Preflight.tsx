// ✈ Pre-flight check: what a parameter file still needs before the first arm (backend/logdoctor/preflight.py).
// Used in the 🩺 Logs tab (upload a .params/.param file; a log's own parameters) and in the 🛠 Builder.

export interface PreflightItem {
  id: string;
  status: 'ok' | 'warn' | 'bad' | 'info';
  title: string;
  found: string;
  todo: string;
}
export interface PreflightReport {
  verdict: 'ok' | 'warn' | 'bad';
  items: PreflightItem[];
  fix_first: PreflightItem[];
  count: number;
  partial: boolean;
}

const ICON = { ok: '✅', warn: '⚠️', bad: '❌', info: '📝' } as const;

export async function checkParamFile(file: File): Promise<PreflightReport> {
  const r = await fetch('/api/preflight', { method: 'POST', headers: { 'Content-Type': 'text/plain' }, body: await file.text() });
  const body = await r.json().catch(() => ({}));
  if (!r.ok) throw new Error(body.detail ?? `HTTP ${r.status}`);
  return body as PreflightReport;
}

export default function PreflightView({ report, title }: { report: PreflightReport; title: string }) {
  const order = { bad: 0, warn: 1, info: 2, ok: 3 } as const;
  const items = [...report.items].sort((a, b) => order[a.status] - order[b.status]);
  return (
    <div className="preflight">
      <h3>{ICON[report.verdict === 'ok' ? 'ok' : report.verdict]} {title}</h3>
      <p className="hint">
        {report.count} parameters read.{' '}
        {report.partial
          ? 'A partial file (like DroneLab’s starting parameters): 📝 = still to do on the real flight controller.'
          : 'A full dump from a flight controller.'}
      </p>
      {report.fix_first.length > 0 && (
        <div className="fix-first">
          <b>Fix before the first flight:</b>
          <ol>{report.fix_first.map((i) => <li key={i.id}><b>{i.title}</b>: {i.todo}</li>)}</ol>
        </div>
      )}
      <table className="checks">
        <thead><tr><th></th><th>Item</th><th>Found</th><th>To do</th></tr></thead>
        <tbody>
          {items.map((i) => (
            <tr key={i.id} className={i.status}>
              <td>{ICON[i.status]}</td>
              <td>{i.title}</td>
              <td>{i.found}</td>
              <td className="keys">{i.todo}</td>
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  );
}
