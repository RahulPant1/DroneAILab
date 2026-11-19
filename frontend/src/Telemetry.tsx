// Live numbers: where the drone is, how it's moving, how it's tilted, how hard each motor works.

import { RAD_TO_DEG, useThrottledState } from './sim';

const MOTOR_LABELS = ['M1 front-right', 'M2 rear-left', 'M3 front-left', 'M4 rear-right'];

const f = (v: number, digits = 2) => (Math.abs(v) < 0.5 * 10 ** -digits ? 0 : v).toFixed(digits);

export default function Telemetry({ onLearn }: { onLearn: (slug: string) => void }) {
  const s = useThrottledState(10);
  if (!s) return <section className="panel telemetry"><h2>Telemetry</h2><p className="muted">Waiting for the simulator…</p></section>;

  const [roll, pitch, yaw] = s.euler_rad.map((r) => r * RAD_TO_DEG);
  const speed = Math.hypot(...s.vel_m_s);

  return (
    <section className="panel telemetry">
      <h2>Telemetry</h2>
      <div className="mode-line">
        <span className={`mode mode-${s.mode}`}>{s.paused ? 'paused' : s.mode}</span>
        <span className="muted">t = {s.t_s.toFixed(1)} s</span>
      </div>

      <table className="nums">
        <thead><tr><th></th><th>East x</th><th>North y</th><th>Up z</th></tr></thead>
        <tbody>
          <tr><th title="Position in the world (metres from the start point)">Position (m)</th>{s.pos_m.map((v, i) => <td key={i}>{f(v)}</td>)}</tr>
          <tr><th title="Where the controller is trying to go">Target (m)</th><td>{f(s.setpoint.x_m)}</td><td>{f(s.setpoint.y_m)}</td><td>{f(s.setpoint.alt_m)}</td></tr>
          <tr><th title="Velocity in the world frame">Velocity (m/s)</th>{s.vel_m_s.map((v, i) => <td key={i}>{f(v)}</td>)}</tr>
        </tbody>
      </table>
      <p className="muted small">Speed {f(speed)} m/s · total thrust {f(s.thrust_total_n, 1)} N</p>

      <table className="nums">
        <thead><tr><th></th><th>Roll</th><th>Pitch</th><th>Yaw</th></tr></thead>
        <tbody>
          <tr><th>Angle (°)</th><td>{f(roll, 1)}</td><td>{f(pitch, 1)}</td><td>{f(yaw, 1)}</td></tr>
          <tr><th>Spin rate (°/s)</th>{s.rate_rad_s.map((v, i) => <td key={i}>{f(v * RAD_TO_DEG, 0)}</td>)}</tr>
        </tbody>
      </table>
      <p className="muted small">Roll + = right side down · Pitch + = nose down · Yaw + = turned left of east</p>

      <h3>Motors</h3>
      {s.motor_pct.map((pct, i) => (
        <div className="motor" key={i} title={MOTOR_LABELS[i]}>
          <span className="motor-label">{MOTOR_LABELS[i]}</span>
          <span className="bar"><span className={`fill ${pct >= 99.5 ? 'sat' : ''}`} style={{ width: `${Math.min(100, pct)}%` }} /></span>
          <span className="motor-pct">{pct.toFixed(0)}%</span>
        </div>
      ))}
      <p className="muted small">Red bar = motor at 100 %: it can't give any more.</p>
      <button className="link" onClick={() => onLearn('02-roll-pitch-yaw')}>📖 Learn: roll, pitch and yaw</button>
    </section>
  );
}
