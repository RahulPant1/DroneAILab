// Dashboard shell: header with status, Studio (left), 3D + controls (centre), Telemetry (right).

import { useEffect, useState } from 'react';
import Controls from './Controls';
import Learn from './Learn';
import Scene from './Scene';
import { api, useSimUi, useThrottledState, type Health } from './sim';
import Studio from './Studio';
import Telemetry from './Telemetry';

const TABS: { id: string; label: string; ready: boolean; note?: string }[] = [
  { id: 'fly', label: 'Fly', ready: true },
  { id: 'lab', label: 'Lab', ready: false, note: 'Phase 2' },
  { id: 'history', label: 'History', ready: false, note: 'Phase 2' },
  { id: 'assistant', label: 'Assistant', ready: false, note: 'Phase 3' },
];

function useHealth(): Health | null {
  const [health, setHealth] = useState<Health | null>(null);
  useEffect(() => {
    const check = () => api.health().then(setHealth).catch(() => setHealth(null));
    check();
    const id = window.setInterval(check, 15000);
    return () => window.clearInterval(id);
  }, []);
  return health;
}

function MessageBar() {
  const { log } = useSimUi();
  const s = useThrottledState(4);
  const lastError = [...log].reverse().find((l) => l.kind === 'error' && Date.now() - l.at < 8000);
  const text = lastError?.text ?? s?.message ?? '';
  return <div className={`message ${lastError ? 'error' : ''}`}>{text}</div>;
}

export default function App() {
  const { status, config } = useSimUi();
  const health = useHealth();
  const [learnSlug, setLearnSlug] = useState<string | null>(null);
  const [follow, setFollow] = useState(true);

  return (
    <div className="app">
      <header>
        <h1>🚁 DroneLab</h1>
        <nav>
          {TABS.map((t) => (
            <button key={t.id} className={`tab ${t.id === 'fly' ? 'active' : ''}`} disabled={!t.ready} title={t.note ? `Coming in ${t.note}` : ''}>
              {t.label}{t.note && <span className="soon"> · {t.note}</span>}
            </button>
          ))}
        </nav>
        <div className="status">
          <span className={`pill ${status === 'open' ? 'ok' : 'bad'}`} title="Live connection to the simulator">Sim {status === 'open' ? 'live' : status}</span>
          <span className={`pill ${health?.llm === 'ok' ? 'ok' : 'off'}`} title={health?.model ?? ''}>
            AI {health?.llm === 'ok' ? `ready (${health.model})` : 'offline'}
          </span>
        </div>
      </header>
      <main>
        <Studio onLearn={setLearnSlug} />
        <section className="viewport">
          <div className="canvas-wrap">
            <Scene armLength={config?.arm_length_m ?? 0.225} follow={follow} />
            <MessageBar />
          </div>
          <Controls follow={follow} setFollow={setFollow} />
        </section>
        <Telemetry onLearn={setLearnSlug} />
      </main>
      {learnSlug && <Learn slug={learnSlug} onClose={() => setLearnSlug(null)} />}
    </div>
  );
}
