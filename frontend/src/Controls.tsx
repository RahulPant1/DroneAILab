// Flight buttons, keyboard nudges, a "fly to" box, and the mission bar (course + autopilot).

import { useEffect, useState } from 'react';
import { api, compassName, sim, useSimUi, useThrottledState, type SimState } from './sim';

const MOVE_KEYS: Record<string, [keyof Sticks, number]> = {
  w: ['forward', 1], s: ['forward', -1],
  a: ['left', 1], d: ['left', -1],
  r: ['up', 1], f: ['up', -1],
  q: ['yaw', 1], e: ['yaw', -1],
};
type Sticks = { forward: number; left: number; up: number; yaw: number };

function isTyping(target: EventTarget | null): boolean {
  const el = target as HTMLElement | null;
  return !!el && (el.tagName === 'INPUT' || el.tagName === 'SELECT' || el.tagName === 'TEXTAREA');
}

/** Hold W/A/S/D/R/F/Q/E to move the target. Sends only when the set of held keys changes. */
function useKeyboardSticks(): void {
  useEffect(() => {
    const held = new Set<string>();
    let last = '';
    const send = () => {
      const s: Sticks = { forward: 0, left: 0, up: 0, yaw: 0 };
      held.forEach((k) => {
        const [axis, sign] = MOVE_KEYS[k];
        s[axis] += sign;
      });
      const key = JSON.stringify(s);
      if (key !== last) {
        last = key;
        sim.send('sticks', s);
      }
    };
    const down = (e: KeyboardEvent) => {
      const k = e.key.toLowerCase();
      if (!(k in MOVE_KEYS) || isTyping(e.target) || e.ctrlKey || e.metaKey || e.altKey) return;
      e.preventDefault();
      held.add(k);
      send();
    };
    const up = (e: KeyboardEvent) => {
      held.delete(e.key.toLowerCase());
      send();
    };
    const release = () => {
      held.clear();
      send();
    };
    window.addEventListener('keydown', down);
    window.addEventListener('keyup', up);
    window.addEventListener('blur', release);
    return () => {
      window.removeEventListener('keydown', down);
      window.removeEventListener('keyup', up);
      window.removeEventListener('blur', release);
    };
  }, []);
}

export default function Controls({ follow, setFollow }: { follow: boolean; setFollow: (v: boolean) => void }) {
  useKeyboardSticks();
  const { status } = useSimUi();
  const s = useThrottledState(5);
  const [alt, setAlt] = useState(2);
  const [target, setTarget] = useState({ x: 3, y: 0, alt: 2 });
  const auto = s?.mode === 'mission'; // the mission autopilot is flying; Hover / Land / Go take over
  const flying = s?.mode === 'takeoff' || s?.mode === 'hover' || auto;
  const offline = status !== 'open';
  const settling = (s?.gps_settle_left_s ?? 0) > 0 && s?.mode === 'idle';

  return (
    <div className="controls">
      <div className="row">
        <button className="primary big" disabled={offline || flying || s?.mode === 'land' || settling}
          onClick={() => sim.send('takeoff', { alt_m: alt })}
          title={settling ? 'Pre-arm check: the drone is averaging GPS fixes so it knows where it is before flying' : undefined}>
          {settling ? (s!.gps_outage ? '📵 No GPS fix' : `🛰 GPS settling ${s!.gps_settle_left_s.toFixed(1)} s`) : '🛫 Take off'}
        </button>
        <label className="inline" title="Take-off altitude">
          to <input type="number" min={0.3} max={50} step={0.5} value={alt} onChange={(e) => setAlt(Number(e.target.value))} /> m
        </label>
        <button disabled={offline || !flying} onClick={() => sim.send('hover')} title="Stop and hold the current position">⏸ Hover here</button>
        <button disabled={offline || !(flying || s?.mode === 'land')} onClick={() => sim.send('land')}>🛬 Land</button>
        <span className="spacer" />
        <button disabled={offline} onClick={() => sim.send(s?.paused ? 'resume' : 'pause')}>{s?.paused ? '▶ Resume' : '⏯ Pause'}</button>
        <button disabled={offline} onClick={() => sim.send('reset')} title="Put the drone back on the ground at the start">↺ Reset</button>
      </div>
      <div className="row small">
        {s?.failsafe ? (
          <span className="keys gps-lost" title="GPS-loss failsafe: no position hold. The keys tilt the drone directly; it keeps coasting until you tilt the other way.">
            Failsafe: <kbd>W</kbd><kbd>S</kbd><kbd>A</kbd><kbd>D</kbd> <b>tilt</b> the drone (it coasts, slowed by extra failsafe drag: counter-steer to stop sooner) · <kbd>R</kbd><kbd>F</kbd> up/down · <kbd>Q</kbd><kbd>E</kbd> turn
          </span>
        ) : (
          <span className="keys">
            <kbd>W</kbd><kbd>S</kbd> forward/back · <kbd>A</kbd><kbd>D</kbd> left/right · <kbd>R</kbd><kbd>F</kbd> up/down · <kbd>Q</kbd><kbd>E</kbd> turn
          </span>
        )}
        <span className="spacer" />
        <label className="inline" title="Fly to a point (metres east, north, up)">
          Fly to x <input type="number" step={0.5} value={target.x} onChange={(e) => setTarget({ ...target, x: Number(e.target.value) })} />
          y <input type="number" step={0.5} value={target.y} onChange={(e) => setTarget({ ...target, y: Number(e.target.value) })} />
          alt <input type="number" step={0.5} min={0.3} value={target.alt} onChange={(e) => setTarget({ ...target, alt: Number(e.target.value) })} />
          <button disabled={offline || !flying} onClick={() => sim.send('target', { x_m: target.x, y_m: target.y, alt_m: target.alt })}>Go</button>
        </label>
        <label className="inline"><input type="checkbox" checked={follow} onChange={(e) => setFollow(e.target.checked)} /> camera follows</label>
      </div>
      <MissionBar s={s} offline={offline} />
      <WindBar status={status} />
      <SensorBar status={status} />
    </div>
  );
}

const PLANNER_LABELS: Record<string, string> = {
  straight: 'Straight lines (ignores obstacles)',
  potential: 'Potential field (reactive)',
  astar: 'A* (plans ahead)',
};

const PHASE_LABELS: Record<string, string> = { gps: 'waiting for GPS to settle', takeoff: 'taking off', cruise: 'flying the route', land: 'landing' };

/** Load a course into the 3D world, and let the mission autopilot fly it (or fly it yourself). */
function MissionBar({ s, offline }: { s: SimState | null; offline: boolean }) {
  const { course } = useSimUi();
  const [scenarios, setScenarios] = useState<Record<string, { title: string }>>({});
  const [planner, setPlanner] = useState('astar');
  const [speed, setSpeed] = useState(1.5);
  useEffect(() => {
    api.scenarios().then((r) => setScenarios(r.scenarios)).catch(() => setScenarios({}));
  }, []);
  const m = s?.mission;

  return (
    <div className="mission-bar">
      <div className="row small">
        <label className="inline" title="Put obstacles and waypoints into the 3D world. The drone moves to the start point.">
          🗺 Course
          <select value={course?.id ?? ''} disabled={offline} onChange={(e) => sim.send('course', { scenario: e.target.value || null })}>
            <option value="">Free flight (no obstacles)</option>
            {Object.entries(scenarios).map(([id, sc]) => <option key={id} value={id}>{sc.title}</option>)}
          </select>
        </label>
        {course && (
          <>
            <label className="inline" title="How the autopilot gets past obstacles">
              planner
              <select value={planner} onChange={(e) => setPlanner(e.target.value)}>
                {Object.entries(PLANNER_LABELS).map(([id, label]) => <option key={id} value={id}>{label}</option>)}
              </select>
            </label>
            <label className="inline" title="Cruise speed along the route (0.3–3 m/s)">
              speed <input type="number" min={0.3} max={3} step={0.1} value={speed} onChange={(e) => setSpeed(Number(e.target.value))} /> m/s
            </label>
            <button className="primary" disabled={offline || speed < 0.3 || speed > 3}
              onClick={() => sim.send('mission', { planner, cruise_speed_m_s: speed })}
              title="Back to the start, plan a route, then the autopilot flies it">
              ▶ Fly mission
            </button>
          </>
        )}
        <span className="spacer" />
        {m && (
          <span className="mission-status">
            Autopilot ({m.planner}): {PHASE_LABELS[m.phase] ?? m.phase}
            {m.phase === 'cruise' && ` · heading for waypoint ${Math.min(m.next_wp, m.waypoints_total)} of ${m.waypoints_total}`}
          </span>
        )}
      </div>
      {course && (
        <p className="muted small course-legend">
          Grey boxes = obstacles ({course.obstacles[0]?.height_m ?? 4} m tall; fly above them to pass over) · orange ring on the ground = keep-out zone ·
          rings = waypoints at {course.cruise_alt_m} m (green done, yellow next) · yellow dashes = planned route. Touching a box stops the motors.
        </p>
      )}
    </div>
  );
}

/** Live wind: changes apply at once, mid-flight, and stay through Reset and new missions. */
function WindBar({ status }: { status: string }) {
  const [wind, setWind] = useState({ speed: 0, dir: 0, gust: 0 });
  const send = (w: typeof wind) => sim.send('wind', { speed_m_s: w.speed, direction_deg: w.dir, gust_m_s: w.gust });
  const change = (patch: Partial<typeof wind>) => {
    const w = { ...wind, ...patch };
    setWind(w);
    send(w);
  };
  // A fresh connection starts in calm air: send our settings again if it reconnects.
  useEffect(() => {
    if (status === 'open' && (wind.speed > 0 || wind.gust > 0)) send(wind);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [status]);
  const off = status !== 'open';

  return (
    <div className="row small wind-bar">
      <span title="Wind pushes on the drone through drag. Watch the arrow at the drone and the drift in Telemetry.">🌬 Wind</span>
      <label className="inline" title="Steady wind speed (0–12 m/s). 5 m/s is a breeze, 10 m/s is strong for a small drone.">
        <input type="range" min={0} max={12} step={0.5} value={wind.speed} disabled={off} onChange={(e) => change({ speed: Number(e.target.value) })} />
        <b className="wind-num">{wind.speed.toFixed(1)}</b> m/s
      </label>
      <label className="inline" title="Where the wind blows TOWARD (0° = east, 90° = north)">
        toward
        <input type="range" min={0} max={345} step={15} value={wind.dir} disabled={off} onChange={(e) => change({ dir: Number(e.target.value) })} />
        <b className="wind-num">{wind.dir}°</b> {compassName(wind.dir)}
      </label>
      <label className="inline" title="Typical size of random gusts on top of the steady wind (0–4 m/s)">
        gusts ±
        <input type="range" min={0} max={4} step={0.25} value={wind.gust} disabled={off} onChange={(e) => change({ gust: Number(e.target.value) })} />
        <b className="wind-num">{wind.gust.toFixed(2)}</b> m/s
      </label>
      <button disabled={off || (wind.speed === 0 && wind.gust === 0)} onClick={() => change({ speed: 0, gust: 0 })}>Calm</button>
    </div>
  );
}

/** Noisy sensors on/off. With them on, the controller flies on its ESTIMATE (the ghost drone in 3D). */
function SensorBar({ status }: { status: string }) {
  const [sensors, setSensors] = useState({ on: false, level: 1, gps: true, outage: false, failsafe: true });
  const change = (patch: Partial<typeof sensors>) => {
    const v = { ...sensors, ...patch };
    setSensors(v);
    if (v.on !== sensors.on || v.level !== sensors.level || v.gps !== sensors.gps)
      sim.send('sensors', { enabled: v.on, noise_scale: v.level, gps_noise: v.gps });
    if (v.outage !== sensors.outage || v.failsafe !== sensors.failsafe || v.on !== sensors.on)
      sim.send('gps', { outage: v.outage, failsafe: v.failsafe });
  };
  useEffect(() => {
    if (status === 'open' && sensors.on) sim.send('sensors', { enabled: true, noise_scale: sensors.level, gps_noise: sensors.gps });
    if (status === 'open' && sensors.outage) sim.send('gps', { outage: true, failsafe: sensors.failsafe });
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [status]);
  const off = status !== 'open';

  return (
    <div className="row small sensor-bar">
      <label className="inline" title="Off = perfect sensors: the controller knows exactly where the drone is. On = real-ish gyro, accelerometer, altimeter, compass and GPS noise, and an estimator that guesses the state from them.">
        <input type="checkbox" checked={sensors.on} disabled={off} onChange={(e) => change({ on: e.target.checked })} /> 📡 Sensor noise
      </label>
      <label className="inline" title="1 = typical hobby sensors, 3 = poor ones">
        level
        <input type="range" min={0.5} max={3} step={0.25} value={sensors.level} disabled={off || !sensors.on} onChange={(e) => change({ level: Number(e.target.value) })} />
        <b className="wind-num">{sensors.level.toFixed(2)}</b>×
      </label>
      <label className="inline" title="Untick for a perfect GPS while the other sensors (gyro, accelerometer, altimeter, compass) stay noisy. Shows how much of the drift comes from the GPS.">
        <input type="checkbox" checked={sensors.gps} disabled={off || !sensors.on} onChange={(e) => change({ gps: e.target.checked })} /> 🛰 GPS noise
      </label>
      <label className="inline" title="No GPS fixes at all (jamming, indoors, under a bridge). Position then comes from the motion sensors only, and drifts away fast.">
        <input type="checkbox" checked={sensors.outage} disabled={off || !sensors.on} onChange={(e) => change({ outage: e.target.checked })} /> 📵 GPS outage
      </label>
      <label className="inline" title="On GPS loss: after 1 s, stop trusting the drifting position. Hold level and altitude instead (missions are aborted). Untick to see what happens if the drone keeps trusting it.">
        <input type="checkbox" checked={sensors.failsafe} disabled={off || !sensors.on} onChange={(e) => change({ failsafe: e.target.checked })} /> failsafe
      </label>
      {sensors.on && <span className="muted">Ghost outline = where the drone <i>thinks</i> it is. The controller steers the ghost, not the real drone. On the ground it averages GPS for 5 s before it may take off.</span>}
    </div>
  );
}
