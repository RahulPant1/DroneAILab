// Dashboard shell: header with tabs and status. Fly = Studio + 3D + Telemetry; Lab; History.

import { useEffect, useState } from 'react';
import Assistant from './Assistant';
import CameraView from './CameraView';
import { assistantStore, useAssistant } from './assistant';
import Controls from './Controls';
import { RaceHud } from './RaceView';
import Logs from './Logs';
import History from './History';
import Lab from './Lab';
import Learn from './Learn';
import Play from './Play';
import Scene from './Scene';
import { api, useSimUi, useThrottledState, type Health } from './sim';
import Studio from './Studio';
import Telemetry from './Telemetry';

type TabId = 'fly' | 'lab' | 'history' | 'logs';

const TABS: { id: TabId; label: string; ready: boolean; note?: string }[] = [
  { id: 'fly', label: 'Fly', ready: true },
  { id: 'lab', label: 'Lab', ready: true },
  { id: 'history', label: 'History', ready: true },
  { id: 'logs', label: '🩺 Logs', ready: true },
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

function initialTab(): TabId {
  try {
    const saved = localStorage.getItem('dronelab.tab');
    if (saved === 'fly' || saved === 'lab' || saved === 'history' || saved === 'logs') return saved;
  } catch {
    /* storage unavailable */
  }
  return 'fly';
}

export default function App() {
  const { status, config } = useSimUi();
  const health = useHealth();
  const [openLogId, setOpenLogId] = useState<string | null>(null); // a log to show when switching to 🩺 Logs
  const [tab, setTabState] = useState<TabId>(initialTab);
  const [learnSlug, setLearnSlug] = useState<string | null>(null);
  const [follow, setFollow] = useState(true);
  const { open: assistantOpen } = useAssistant();
  const [mode, setModeState] = useState<'play' | 'engineer'>(() => {
    try { return localStorage.getItem('dronelab.mode') === 'engineer' ? 'engineer' : 'play'; } catch { return 'play'; }
  });
  const setMode = (m: 'play' | 'engineer') => {
    setModeState(m);
    try { localStorage.setItem('dronelab.mode', m); } catch { /* storage unavailable */ }
    if (m === 'play') setTab('fly');
  };
  const play = mode === 'play'; // Play always shows the 3D world (the tabs are hidden)

  const setTab = (t: TabId) => {
    setTabState(t);
    try {
      localStorage.setItem('dronelab.tab', t);
    } catch {
      /* storage unavailable */
    }
  };

  return (
    <div className="app">
      <header>
        <h1>🚁 DroneLab</h1>
        <div className="mode-switch" title="Play = big 3D view and a few big buttons. Engineer = every panel, slider and number.">
          <button className={mode === 'play' ? 'active' : ''} onClick={() => setMode('play')}>🎮 Play</button>
          <button className={mode === 'engineer' ? 'active' : ''} onClick={() => setMode('engineer')}>🔧 Engineer</button>
        </div>
        {mode === 'engineer' && <nav>
          {TABS.map((t) => (
            <button key={t.id} className={`tab ${t.id === tab ? 'active' : ''}`} disabled={!t.ready}
              title={t.note ? `Coming in ${t.note}` : ''} onClick={() => setTab(t.id)}>
              {t.label}{t.note && <span className="soon"> · {t.note}</span>}
            </button>
          ))}
        </nav>}
        {mode === 'engineer' && <button className={`assistant-toggle ${assistantOpen ? 'active' : ''}`} onClick={() => assistantStore.setOpen(!assistantOpen)}>
          💬 Assistant
        </button>}
        <div className="status">
          <span className={`pill ${status === 'open' ? 'ok' : 'bad'}`} title="Live connection to the simulator">Sim {status === 'open' ? 'live' : status}</span>
          <span className={`pill ${health?.llm === 'ok' ? 'ok' : 'off'}`} title={health?.model ?? ''}>
            AI {health?.llm === 'ok' ? `ready (${health.model})` : 'offline'}
          </span>
        </div>
      </header>

      {play && <Play armLength={config?.arm_length_m ?? 0.225} />}
      {tab === 'fly' && !play && (
        <main className="fly">
          <Studio onLearn={setLearnSlug} />
          <section className="viewport">
            <div className="canvas-wrap">
              <Scene armLength={config?.arm_length_m ?? 0.225} follow={follow} />
              <MessageBar />
              <RaceHud />
              <CameraView />
            </div>
            <Controls follow={follow} setFollow={setFollow} onLearn={setLearnSlug} onOpenLog={(id) => { setOpenLogId(id); setTab('logs'); }} />
          </section>
          <Telemetry onLearn={setLearnSlug} />
        </main>
      )}
      {!play && tab === 'lab' && <main className="scroll"><Lab onLearn={setLearnSlug} /></main>}
      {!play && tab === 'history' && <main className="scroll"><History /></main>}
      {!play && tab === 'logs' && <main className="scroll"><Logs initialId={openLogId} /></main>}

      {assistantOpen && <Assistant health={health} onApplied={() => setTab('lab')} />}
      {learnSlug && <Learn slug={learnSlug} onClose={() => setLearnSlug(null)} />}
    </div>
  );
}
