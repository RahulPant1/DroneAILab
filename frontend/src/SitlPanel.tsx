// 🛰 ArduPilot panel: start the real ArduCopter flight code (SITL), fly it next to your drone,
// and compare the same "fly 10 m north" on both autopilots.

import { useEffect, useRef, useState } from 'react';
import uPlot from 'uplot';
import 'uplot/dist/uPlot.min.css';
import { api, sim, type FaultInfo, type FaultResult, type SitlStatus, type StepRun } from './sim';

const STATE_TEXT: Record<SitlStatus['state'], string> = {
  off: 'off',
  starting: 'starting…',
  waiting_gps: 'waiting for GPS + EKF…',
  ready: 'ready',
  error: 'error',
};

const send = (action: string, extra: Record<string, unknown> = {}) => sim.send('sitl', { action, ...extra });

export default function SitlBar({ a, offline, onLearn, onOpenLog }: { a: SitlStatus | null | undefined; offline: boolean; onLearn: (slug: string) => void; onOpenLog: (id: string) => void }) {
  const [open, setOpen] = useState(false);
  const [target, setTarget] = useState({ x: 0, y: 10, alt: 5 });
  const running = !!a && a.state !== 'off' && a.state !== 'error';
  const ready = a?.state === 'ready';
  const flying = ready && !!a?.armed;
  const busy = !!a?.action;
  const faultRunning = a?.action === 'fault injection';
  const [faults, setFaults] = useState<FaultInfo[]>([]);
  const [faultId, setFaultId] = useState('gps_loss');
  useEffect(() => {
    if (open || running) api.sitlFaults().then((r) => setFaults(r.faults)).catch(() => setFaults([]));
  }, [open, running]);
  const chosen = faults.find((f) => f.id === faultId);

  if (!open && !running) {
    return (
      <div className="row small sitl-bar">
        <button className="link" disabled={offline} onClick={() => setOpen(true)}
          title="Run the REAL ArduCopter autopilot software (the code inside many real drones) in simulation, next to your drone">
          🛰 ArduPilot: fly the real autopilot software…
        </button>
      </div>
    );
  }

  return (
    <div className="sitl-panel">
      <div className="row small">
        <b>🛰 ArduPilot</b>
        <span className={`chip sitl-${a?.state ?? 'off'}`}>{STATE_TEXT[a?.state ?? 'off']}</span>
        {running ? (
          <button disabled={offline} onClick={() => send('stop')}>⏹ Stop ArduPilot</button>
        ) : (
          <button className="primary" disabled={offline} onClick={() => send('start')}
            title="Starts ArduCopter 4.7.3 in SITL on this laptop (127.0.0.1 only). Its GPS + EKF need ~30–45 s before it may arm.">
            ▶ Start ArduPilot
          </button>
        )}
        {a?.mode && (
          <span className="keys" title="ArduCopter's flight mode, GPS and estimator, as a ground station shows them">
            mode <b>{a.mode}</b> · {a.armed ? <b className="armed">ARMED</b> : 'disarmed'} · GPS {a.gps_fix === 3 ? '3D fix' : a.gps_fix === 6 ? 'RTK' : `fix ${a.gps_fix}`}, {a.sats} sats · EKF {a.ekf_ok ? '✓' : '…'}
            {a.local_m && <> · x {a.local_m[0].toFixed(1)}, y {a.local_m[1].toFixed(1)}, alt {a.local_m[2].toFixed(1)} m · heading {a.heading_deg?.toFixed(0)}°</>}
          </span>
        )}
        <span className="spacer" />
        <button className="link" onClick={() => onLearn('12-ardupilot-sitl')}>📖 Learn: ArduPilot</button>
        {!running && <button className="link" onClick={() => setOpen(false)}>hide</button>}
      </div>
      <div className="row small fault-row">
        <label className="inline" title="Break the real autopilot on purpose (SITL's SIM_* switches) and check it does the safe thing. Each run starts a fresh ArduPilot, takes off, holds 20 m north of its pad, then injects the fault.">
          🧪 Fault injection
          <select value={faultId} disabled={offline || faultRunning} onChange={(e) => setFaultId(e.target.value)}>
            <optgroup label="Safety checks (must do the safe thing)">
              {faults.filter((f) => f.kind === 'safety').map((f) => <option key={f.id} value={f.id}>{f.title}</option>)}
            </optgroup>
            <optgroup label="Known limitations (worth knowing)">
              {faults.filter((f) => f.kind === 'limitation').map((f) => <option key={f.id} value={f.id}>{f.title}</option>)}
            </optgroup>
          </select>
        </label>
        <button disabled={offline || faultRunning || faults.length === 0} onClick={() => send('fault', { fault: faultId })}>
          {faultRunning ? '⏳ running…' : '▶ Run fault test'}
        </button>
        {chosen && <span className="keys" title={chosen.injected}>expected: {chosen.expected}</span>}
      </div>
      {running && (
        <div className="row small">
          <button disabled={offline || !ready || busy} onClick={() => send('takeoff', { alt_m: target.alt })}
            title="GUIDED mode, arm (after the pre-arm checks), take off to the altitude below">🛫 Take off</button>
          <label className="inline" title="Relative to ArduPilot's own pad (the orange circle): metres east, north, up">
            Fly to x <input type="number" step={1} value={target.x} onChange={(e) => setTarget({ ...target, x: Number(e.target.value) })} />
            y <input type="number" step={1} value={target.y} onChange={(e) => setTarget({ ...target, y: Number(e.target.value) })} />
            alt <input type="number" step={1} min={1} max={30} value={target.alt} onChange={(e) => setTarget({ ...target, alt: Number(e.target.value) })} />
            <button disabled={offline || !ready} onClick={() => send('goto', { x_m: target.x, y_m: target.y, alt_m: target.alt })}>Go</button>
          </label>
          <button disabled={offline || !flying} onClick={() => send('hover')} title="Hold position (GUIDED)">⏸ Hover</button>
          <button disabled={offline || !ready} onClick={() => send('square', { side_m: 10, alt_m: target.alt })}
            title="Upload a 4-waypoint 10 m square and fly it in AUTO mode, like a mission planned in a ground station">⬜ Square mission</button>
          <button disabled={offline || !flying} onClick={() => send('rtl')} title="Return To Launch: fly home and land by itself">🏠 Return home</button>
          <button disabled={offline || !flying} onClick={() => send('land')} title="Land right here">🛬 Land</button>
          <button disabled={offline || !ready} onClick={() => send('compare')}
            title="Fly 10 m north on ArduPilot (live) and on DroneLab's controller, and plot both">📈 Compare with DroneLab</button>
        </div>
      )}
      {a && (a.message || a.log.length > 0) && (
        <div className="row small sitl-msg">
          {a.message && <span>{a.message}</span>}
          {a.log.length > 0 && <span className="keys" title="The autopilot's own messages (STATUSTEXT), newest last">autopilot: {a.log.slice(-3).join(' · ')}</span>}
        </div>
      )}
      {a?.fault_result && <FaultCard r={a.fault_result} onOpenLog={onOpenLog} />}
      {a?.compare && <Compare c={a.compare} />}
    </div>
  );
}

const VERDICT_CLASS: Record<string, string> = { PASS: 'pass', CONFIRMED: 'confirmed', FAIL: 'fail', ERROR: 'fail' };

function FaultCard({ r, onOpenLog }: { r: FaultResult; onOpenLog: (id: string) => void }) {
  return (
    <div className={`fault-card ${VERDICT_CLASS[r.verdict] ?? ''}`}>
      <div className="row small">
        <b>🧪 {r.title}</b>
        <span className="chip verdict">{r.verdict}</span>
        <span className="keys">{r.kind === 'limitation' ? 'known limitation' : 'safety check'} · {r.frame}</span>
        {r.log && <button className="link" onClick={() => onOpenLog(r.log!)} title="Open this flight's ArduPilot log in the 🩺 Logs tab">🩺 Analyse this flight's log</button>}
      </div>
      <p><b>Injected:</b> {r.injected}</p>
      <p><b>Expected:</b> {r.expected}</p>
      <p><b>Observed:</b> {r.observed}</p>
      {r.events.length > 0 && (
        <p className="keys"><b>Timeline:</b> {r.events.slice(0, 8).map(([t, text]) => `${t >= 0 ? '+' : ''}${t.toFixed(1)} s ${text}`).join(' · ')}</p>
      )}
    </div>
  );
}

function Compare({ c }: { c: NonNullable<SitlStatus['compare']> }) {
  const row = (name: string, r: StepRun) => (
    <tr>
      <td>{name}</td>
      <td>{r.rise_s?.toFixed(1) ?? '–'} s</td>
      <td>{r.overshoot_m.toFixed(2)} m</td>
      <td>{r.settle_s?.toFixed(1) ?? 'not yet'} s</td>
      <td>{r.peak_speed_m_s?.toFixed(1) ?? '–'} m/s</td>
    </tr>
  );
  return (
    <div className="sitl-compare">
      <StepChart c={c} />
      <table className="metrics">
        <thead>
          <tr>
            <th>"Fly {c.distance_m} m north"</th>
            <th title="10% to 90% of the way">rise</th>
            <th title="How far past the target it went">overshoot</th>
            <th title="From the command until it stays within 0.2 m">settles</th>
            <th>top speed</th>
          </tr>
        </thead>
        <tbody>
          {row('🛰 ArduPilot', c.ardupilot)}
          {row('DroneLab', c.dronelab)}
        </tbody>
      </table>
      {(c.ardupilot.max_gap_s ?? 0) > 0.3 && (
        <p className="error">⚠ The ArduPilot recording has a {c.ardupilot.max_gap_s!.toFixed(1)} s gap (the telemetry stream paused), so its numbers aren't reliable. Run the comparison again.</p>
      )}
      <p className="hint">Same job, two autopilots. Both are cascaded position → velocity → attitude controllers; the differences come from their gains and speed/acceleration limits (DroneLab: 3 m/s; ArduPilot GUIDED: WPNAV speed and jerk limits).</p>
    </div>
  );
}

function StepChart({ c }: { c: NonNullable<SitlStatus['compare']> }) {
  const wrap = useRef<HTMLDivElement>(null);
  // A new state frame brings a new (equal) object several times a second: only redraw for a new comparison.
  const key = `${c.ardupilot.t_s.length}:${c.ardupilot.north_m.at(-1)}:${c.dronelab.north_m.at(-1)}`;
  useEffect(() => {
    const el = wrap.current;
    if (!el) return;
    const data = uPlot.join([
      [c.ardupilot.t_s, c.ardupilot.north_m],
      [c.dronelab.t_s, c.dronelab.north_m],
    ]);
    const target = data[0].map(() => c.distance_m);
    const plot = new uPlot(
      {
        width: Math.min(el.clientWidth || 520, 640),
        height: 200,
        scales: { x: { time: false } },
        axes: [
          { label: 'time since the command (s)', stroke: '#9aa5b8', grid: { stroke: '#2c3648' } },
          { label: 'north (m)', stroke: '#9aa5b8', grid: { stroke: '#2c3648' } },
        ],
        series: [
          { label: 't (s)' },
          { label: 'ArduPilot', stroke: '#ff8c42', width: 2, spanGaps: true },
          { label: 'DroneLab', stroke: '#4cc9f0', width: 2, spanGaps: true },
          { label: 'target', stroke: '#6b7a99', dash: [6, 6], width: 1 },
        ],
      },
      [...data, target] as uPlot.AlignedData,
      el,
    );
    return () => plot.destroy();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [key]);
  return <div ref={wrap} className="sitl-chart" />;
}
