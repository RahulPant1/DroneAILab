// Live numbers: where the drone is, how it's moving, how it's tilted, how hard each motor works.

import { compassName, RAD_TO_DEG, useThrottledState } from './sim';

const MOTOR_LABELS = ['M1 front-right', 'M2 rear-left', 'M3 front-left', 'M4 rear-right'];

const f = (v: number, digits = 2) => (Math.abs(v) < 0.5 * 10 ** -digits ? 0 : v).toFixed(digits);

export default function Telemetry({ onLearn }: { onLearn: (slug: string) => void }) {
  const s = useThrottledState(10);
  if (!s) return <section className="panel telemetry"><h2>Telemetry</h2><p className="muted">Waiting for the simulator…</p></section>;

  const [roll, pitch, yaw] = s.euler_rad.map((r) => r * RAD_TO_DEG);
  const speed = Math.hypot(...s.vel_m_s);
  const [wx, wy] = s.wind_m_s ?? [0, 0, 0];
  const windSpeed = Math.hypot(wx, wy);
  const windDir = ((Math.atan2(wy, wx) * RAD_TO_DEG) + 360) % 360;

  return (
    <section className="panel telemetry">
      <h2>Telemetry</h2>
      <div className="mode-line">
        <span className={`mode mode-${s.failsafe ? 'failsafe' : s.mode}`}>{s.paused ? 'paused' : s.failsafe ? 'failsafe' : s.mode}</span>
        <span className="muted">t = {s.t_s.toFixed(1)} s</span>
      </div>

      <table className="nums">
        <thead><tr><th></th><th>East x</th><th>North y</th><th>Up z</th></tr></thead>
        <tbody>
          <tr><th title="Position in the world (metres from the start point)">Position (m)</th>{s.pos_m.map((v, i) => <td key={i}>{f(v)}</td>)}</tr>
          {s.failsafe ? (
            <tr className="gps-lost">
              <th title="In failsafe the drone does NOT fly to this point: it only holds level and height (W/A/S/D tilt it). The point is kept, and when GPS returns the drone flies back to it. Height is still held.">Target (frozen)</th>
              <td>{f(s.setpoint.x_m)}</td><td>{f(s.setpoint.y_m)}</td><td>{f(s.setpoint.alt_m)}</td>
            </tr>
          ) : (
            <tr><th title="Where the controller is trying to go">Target (m)</th><td>{f(s.setpoint.x_m)}</td><td>{f(s.setpoint.y_m)}</td><td>{f(s.setpoint.alt_m)}</td></tr>
          )}
          <tr><th title="Velocity in the world frame">Velocity (m/s)</th>{s.vel_m_s.map((v, i) => <td key={i}>{f(v)}</td>)}</tr>
        </tbody>
      </table>
      <p className="muted small">Speed {f(speed)} m/s · total thrust {f(s.thrust_total_n, 1)} N</p>
      {s.est_pos_m && (
        <p className="muted small" title="Distance between where the drone thinks it is (its estimate) and where it really is">
          Sensors: noisy · estimate off by <b>{f(Math.hypot(...s.est_pos_m.map((v, i) => v - s.pos_m[i])), 2)} m</b>
          {s.gps_settle_left_s > 0 && <> · GPS settling ({s.gps_settle_left_s.toFixed(1)} s left)</>}
          {s.gps_outage && <> · <b className="gps-lost">GPS LOST {s.gps_lost_s.toFixed(0)} s</b></>}
          {s.failsafe && <> · <b className="gps-lost">FAILSAFE: holding level + altitude, not position</b></>}
        </p>
      )}
      {s.formation && (
        <p className="muted small" title="Closest distance between the centres of any two drones in the air. Under 0.6 m their propellers touch.">
          Formation: {s.formation.count} wingm{s.formation.count === 1 ? 'a' : 'e'}n
          {s.formation.min_gap_m !== null && (
            <> · closest gap <b className={s.formation.min_gap_m < 1.0 ? 'gps-lost' : ''}>{f(s.formation.min_gap_m, 2)} m</b></>
          )}
          {s.formation.wingmen.some((w) => w.crashed) && <> · <b className="gps-lost">{s.formation.wingmen.filter((w) => w.crashed).length} crashed</b></>}
          {s.formation.wingmen.some((w) => w.breaking) && <> · breaking formation for an obstacle</>}
        </p>
      )}
      <p className="muted small" title="The air's velocity at the drone right now (steady wind + gusts)">
        {windSpeed < 0.05 ? 'Wind: calm' : `Wind ${f(windSpeed, 1)} m/s toward ${compassName(windDir)} (${f(windDir, 0)}°)`}
      </p>

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
