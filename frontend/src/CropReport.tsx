// 🤖 The farmer's report on a crop-health map: written by the local AI from the findings (numbers checked).

import { useState } from 'react';
import { api } from './sim';

export default function CropReport({ id }: { id: string }) {
  const [busy, setBusy] = useState(false);
  const [out, setOut] = useState<{ answer: string; model: string; unverified_numbers: string[]; duration_s: number } | null>(null);
  const [error, setError] = useState('');
  const ask = async () => {
    setBusy(true);
    setError('');
    try {
      setOut(await api.cropReport(id));
    } catch (e) {
      setError((e as Error).message);
    } finally {
      setBusy(false);
    }
  };
  return (
    <div className="crop-report">
      <button disabled={busy} onClick={ask} title="The local AI writes a short report for the farmer from the findings (it never sees the picture)">
        {busy ? '🤖 Writing…' : "🤖 Farmer's report (local AI)"}
      </button>
      {error && <p className="error small">{error}</p>}
      {out && (
        <div className="ai-understood small">
          <pre className="answer">{out.answer}</pre>
          <span className="muted">{out.model}, {out.duration_s} s.</span>
          {out.unverified_numbers.length > 0 && <p className="warn">⚠ Numbers not in the findings: {out.unverified_numbers.join(', ')}. Don't trust those.</p>}
        </div>
      )}
    </div>
  );
}
