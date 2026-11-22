// Flight buttons, keyboard nudges, a "fly to" box, and the mission bar (course + autopilot).

import { useEffect, useRef, useState } from 'react';
import FieldMap from './FieldMap';
import MissionPlanner from './MissionPlanner';
import { api, compassName, sim, useSimUi, useThrottledState, type SimState } from './sim';
import SitlBar from './SitlPanel';

// Held keys move the drone. Uses e.code (the physical key), so it works on any keyboard layout
// and with Shift held. Arrow keys = same as W/A/S/D; Space/Shift = up/down (like flying in games).
const MOVE_KEYS: Record<string, [keyof Sticks, number]> = {
  KeyW: ['forward', 1], KeyS: ['forward', -1], ArrowUp: ['forward', 1], ArrowDown: ['forward', -1],
  KeyA: ['left', 1], KeyD: ['left', -1], ArrowLeft: ['left', 1], ArrowRight: ['left', -1],
  KeyR: ['up', 1], KeyF: ['up', -1], Space: ['up', 1], ShiftLeft: ['up', -1], ShiftRight: ['up', -1],
  KeyQ: ['yaw', 1], KeyE: ['yaw', -1],
};
type Sticks = { forward: number; left: number; up: number; yaw: number };

/** One-press shortcuts: T take off, L land, H hover, P pause/resume, Enter = race again (only in a race). */
function pressShortcut(code: string, takeoffAltM: number): boolean {
  const s = sim.latest;
  if (!s) return false;
  const flying = s.mode === 'takeoff' || s.mode === 'hover' || s.mode === 'mission';
  const settling = (s.gps_settle_left_s ?? 0) > 0 && s.mode === 'idle';
  switch (code) {
    case 'KeyT':
      if (!flying && s.mode !== 'land' && !settling) sim.send('takeoff', { alt_m: takeoffAltM });
      return true;
    case 'KeyL':
      if (flying) sim.send('land');
      return true;
    case 'KeyH':
      if (flying) sim.send('hover');
      return true;
    case 'KeyP':
      sim.send(s.paused ? 'resume' : 'pause');
      return true;
    case 'Enter':
    case 'NumpadEnter':
      if (!s.race) return false;
      sim.send('reset');
      return true;
  }
  return false;
}

function isTyping(target: EventTarget | null): boolean {
  const el = target as HTMLElement | null;
  return !!el && (el.tagName === 'INPUT' || el.tagName === 'SELECT' || el.tagName === 'TEXTAREA');
}

/** Hold the move keys to move the target. Sends only when the set of held keys changes. */
function useKeyboardSticks(takeoffAltM: number): void {
  const altRef = useRef(takeoffAltM);
  altRef.current = takeoffAltM;
  useEffect(() => {
    const held = new Set<string>();
    let last = '';
    const send = () => {
      const s: Sticks = { forward: 0, left: 0, up: 0, yaw: 0 };
      held.forEach((k) => {
        const [axis, sign] = MOVE_KEYS[k];
        s[axis] = Math.max(-1, Math.min(1, s[axis] + sign)); // W + Up arrow together is still full stick
      });
      const key = JSON.stringify(s);
      if (key !== last) {
        last = key;
        sim.send('sticks', s);
      }
    };
    const down = (e: KeyboardEvent) => {
      if (isTyping(e.target) || e.ctrlKey || e.metaKey || e.altKey) return;
      if (e.code in MOVE_KEYS) {
        e.preventDefault(); // no page scrolling (arrows/Space) and no "clicking" a focused button with Space
        held.add(e.code);
        send();
      } else if (!e.repeat && pressShortcut(e.code, altRef.current)) {
        e.preventDefault();
      }
    };
    const up = (e: KeyboardEvent) => {
      if (e.code in MOVE_KEYS && !isTyping(e.target)) e.preventDefault(); // Space would click a focused button on release
      held.delete(e.code);
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

export default function Controls({ follow, setFollow, onLearn, onOpenLog }: { follow: boolean; setFollow: (v: boolean) => void; onLearn: (slug: string) => void; onOpenLog: (id: string) => void }) {
  const [alt, setAlt] = useState(2);
  useKeyboardSticks(alt);
  const { status } = useSimUi();
  const s = useThrottledState(5);
  const [target, setTarget] = useState({ x: 3, y: 0, alt: 2 });
  const auto = s?.mode === 'mission' || s?.mode === 'precland'; // an autopilot is flying; Hover / Land / Go take over
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
        <label className="inline" title="Run the simulation faster than real time (handy for long surveys)">
          ⏩
          <select value={s?.time_scale ?? 1} disabled={offline} onChange={(e) => sim.send('speed', { factor: Number(e.target.value) })}>
            {[1, 2, 4, 8].map((f) => <option key={f} value={f}>{f === 1 ? 'real time' : `${f}× faster`}</option>)}
          </select>
        </label>
        <button disabled={offline} onClick={() => sim.send(s?.paused ? 'resume' : 'pause')}>{s?.paused ? '▶ Resume' : '⏯ Pause'}</button>
        <button disabled={offline} onClick={() => sim.send('reset')} title="Put the drone back on the ground at the start">↺ Reset</button>
      </div>
      <div className="row small">
        {s?.failsafe ? (
          <span className="keys gps-lost" title="GPS-loss failsafe: no position hold. The keys tilt the drone directly; it keeps coasting until you tilt the other way.">
            Failsafe: <kbd>W</kbd><kbd>S</kbd><kbd>A</kbd><kbd>D</kbd> <b>tilt</b> the drone {s.failsafe_tilt_deg.toFixed(0)}°
            {s.failsafe_drag_n_s_m > 0
              ? ' (it coasts, slowed by the extra failsafe drag: counter-steer to stop sooner)'
              : ' (real physics: it coasts until you tilt the other way)'}
            {' '}(or arrows) · <kbd>R</kbd><kbd>F</kbd> / <kbd>Space</kbd><kbd>Shift</kbd> up/down · <kbd>Q</kbd><kbd>E</kbd> turn
          </span>
        ) : (
          <span className="keys">
            <kbd>W</kbd><kbd>S</kbd> or <kbd>↑</kbd><kbd>↓</kbd> forward/back · <kbd>A</kbd><kbd>D</kbd> or <kbd>←</kbd><kbd>→</kbd> left/right
            {' '}· <kbd>Space</kbd><kbd>Shift</kbd> or <kbd>R</kbd><kbd>F</kbd> up/down · <kbd>Q</kbd><kbd>E</kbd> turn
            <span title="One-press shortcuts: T take off, L land, H hover here, P pause/resume, Enter race again (in a race)"> · <kbd>T</kbd> take off · <kbd>L</kbd> land · <kbd>H</kbd> hover · <kbd>P</kbd> pause{s?.race ? <> · <kbd>Enter</kbd> race again</> : null}</span>
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
      <RaceBar s={s} offline={offline} onLearn={onLearn} />
      <MissionBar s={s} offline={offline} onLearn={onLearn} />
      <FormationBar status={status} onLearn={onLearn} />
      <WindBar status={status} onLearn={onLearn} />
      <SensorBar status={status} onLearn={onLearn} />
      <BatteryBar s={s} offline={offline} onLearn={onLearn} />
      <SitlBar a={s?.sitl} offline={offline} onLearn={onLearn} onOpenLog={onOpenLog} />
    </div>
  );
}

/** Rings race: pick a track, then fly through the hoops in order as fast as you can. */
function RaceBar({ s, offline, onLearn }: { s: SimState | null; offline: boolean; onLearn: (slug: string) => void }) {
  const { race } = useSimUi();
  const [tracks, setTracks] = useState<Record<string, { title: string; best_s: number | null }>>({});
  const finished = s?.race?.finished ?? false;
  useEffect(() => {
    // Refresh the list (with best times) at start and whenever a race finishes.
    api.raceTracks().then((r) => setTracks(r.tracks)).catch(() => setTracks({}));
  }, [finished]);

  return (
    <div className="row small race-bar">
      <label className="inline" title="A game: fly through the hoops in order, in the direction of the arrows. The clock starts at take-off.">
        🏁 Race
        <select value={race?.id ?? ''} disabled={offline} onChange={(e) => sim.send('race', { track: e.target.value || null })}>
          <option value="">No race</option>
          {Object.entries(tracks).map(([id, t]) => (
            <option key={id} value={id}>{t.title}{t.best_s !== null ? ` (best ${t.best_s.toFixed(1)} s)` : ''}</option>
          ))}
        </select>
      </label>
      {race && (
        <>
          <button disabled={offline} onClick={() => sim.send('reset')} title="Back to the start pad, clock to zero">↺ Race again</button>
          <span className="muted">{race.tip}</span>
        </>
      )}
      <button className="link" onClick={() => onLearn('10-rings-race')}>📖 Racing tips</button>
    </div>
  );
}

const PLANNER_LABELS: Record<string, string> = {
  straight: 'Straight lines (ignores obstacles)',
  potential: 'Potential field (reactive)',
  astar: 'A* (plans ahead)',
  lidar: 'Lidar: sense & replan (knows no obstacles)',
  plan: 'flight plan',
};

const PHASE_LABELS: Record<string, string> = { gps: 'waiting for GPS to settle', takeoff: 'taking off', cruise: 'flying the route', land: 'landing' };

/** Load a course into the 3D world, and let the mission autopilot fly it (or fly it yourself). */
function MissionBar({ s, offline, onLearn }: { s: SimState | null; offline: boolean; onLearn: (slug: string) => void }) {
  const { course, cropmaps } = useSimUi();
  const [showMap, setShowMap] = useState(false);
  const [scenarios, setScenarios] = useState<Record<string, { title: string }>>({});
  const [planner, setPlanner] = useState('astar');
  const [speed, setSpeed] = useState(1.5);
  useEffect(() => {
    api.scenarios().then((r) => setScenarios(r.scenarios)).catch(() => setScenarios({}));
  }, []);
  const m = s?.mission;
  const farm = course?.id === 'farm';
  const [planning, setPlanning] = useState(false);
  const flying = !!s && s.armed && !s.on_ground;

  return (
    <div className="mission-bar">
      {showMap && cropmaps.length > 0 && <FieldMap maps={cropmaps} onClose={() => setShowMap(false)} onLearn={(slug) => { setShowMap(false); onLearn(slug); }} />}
      {planning && <MissionPlanner onClose={() => setPlanning(false)} onLearn={(slug) => { setPlanning(false); onLearn(slug); }} />}
      <div className="row small">
        <label className="inline" title="Put obstacles and waypoints into the 3D world. The drone moves to the start point.">
          🗺 Course
          <select value={course?.id ?? ''} disabled={offline} onChange={(e) => sim.send('course', { scenario: e.target.value || null })}>
            <option value="">Free flight (no obstacles)</option>
            <option value="farm">🌾 Hillside farm (surveys, deliveries, landing pads)</option>
            {Object.entries(scenarios).map(([id, sc]) => <option key={id} value={id}>{sc.title}</option>)}
          </select>
        </label>
        {farm && (
          <>
            <button className="primary" disabled={offline} onClick={() => setPlanning(true)} title="Say what to do (AI) or build a task list; see the route and safety checks; then fly">
              🧭 Mission planner
            </button>
            <button disabled={offline || !flying} onClick={() => sim.send('precland', { mode: 'vision' })}
              title="Land on the nearest pad (within 12 m): the down camera finds its marker and steers onto it">🎯 Land on pad (camera)</button>
            <button disabled={offline || !flying} onClick={() => sim.send('precland', { mode: 'gps' })}
              title="Land on the nearest pad using only its GPS position (compare how far off it lands, with 📡 Sensor noise + GPS drift)">GPS only</button>
            <label className="inline" title="Show what a camera under the drone sees">
              <input type="checkbox" checked={!!s?.camera && !s.camera.landing} disabled={offline} onChange={(e) => sim.send('camera', { enabled: e.target.checked })} /> 📷 Camera
            </label>
            {s?.survey && <span className="keys">📷 {s.survey.photos_taken} photos{s.survey.on_line ? ' · on a survey line' : ''}</span>}
            {s?.cropmap_busy && <span className="keys">🌱 stitching the map…</span>}
            {cropmaps.length > 0 && <button onClick={() => setShowMap(true)} title="The crop-health map stitched from the last survey's photos">🗺 Field map</button>}
          </>
        )}
        {course && !farm && (
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
        <label className="inline" title="Show a 360° laser scanner (72 beams, 8 m) and the map it builds while you fly. The lidar planner always uses one.">
          <input type="checkbox" checked={!!s?.lidar && !m} disabled={offline || !!m} onChange={(e) => sim.send('lidar', { enabled: e.target.checked })} /> 📡 Lidar
        </label>
        {s?.lidar && <span className="keys" title="Cells of obstacle the lidar has hit so far">map: {s.lidar.cells} cells{s.lidar.replans != null ? ` · ${s.lidar.replans} replans` : ''}</span>}
        {farm ? (
          <>
            <button className="link" onClick={() => onLearn('17-survey')}>📖 Surveys</button>
            <button className="link" onClick={() => onLearn('18-precision-landing')}>📖 Precision landing</button>
          </>
        ) : <button className="link" onClick={() => onLearn('09-exp-missions')}>📖 Learn: missions</button>}
        <button className="link" onClick={() => onLearn('15-sense-and-avoid')}>📖 Learn: sense and avoid</button>
        <span className="spacer" />
        {m && (
          <span className="mission-status">
            Autopilot ({PLANNER_LABELS[m.planner]?.split(' (')[0] ?? m.planner}): {PHASE_LABELS[m.phase] ?? m.phase}
            {m.phase === 'cruise' && ` · heading for waypoint ${Math.min(m.next_wp, m.waypoints_total)} of ${m.waypoints_total}`}
          </span>
        )}
      </div>
      {farm && (
        <p className="muted small course-legend">
          Green = fields · grey boxes = buildings (fly over them if higher, otherwise around) · red = no-fly zones (never enter) ·
          🅿 pads with a black marker ring · orange dashes = {course.site?.geofence_radius_m ?? 90} m geofence · blue rectangles = photos taken
          (orange = ArduPilot's) · yellow dashes = the planned route.{s?.no_fly ? <b className="error"> ⚠ You are inside the no-fly zone '{s.no_fly}'!</b> : null}
        </p>
      )}
      {course && !farm && (
        <p className="muted small course-legend">
          Grey boxes = obstacles ({course.obstacles[0]?.height_m ?? 4} m tall; fly above them to pass over) · orange ring on the ground = keep-out zone ·
          rings = waypoints at {course.cruise_alt_m} m (green done, yellow next) · yellow dashes = planned route. Touching a box stops the motors.
        </p>
      )}
    </div>
  );
}

/** Multi-drone: wingmen that fly with you in a shape. Settings stay through Reset; re-sent on reconnect. */
function FormationBar({ status, onLearn }: { status: string; onLearn: (slug: string) => void }) {
  const [f, setF] = useState({ count: 0, shape: 'v', spacing: 1.5, avoid: true });
  const send = (v: typeof f) => sim.send('formation', { count: v.count, shape: v.shape, spacing_m: v.spacing, avoid: v.avoid });
  const change = (patch: Partial<typeof f>) => {
    const v = { ...f, ...patch };
    setF(v);
    send(v);
  };
  useEffect(() => {
    if (status === 'open' && f.count > 0) send(f);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [status]);
  const off = status !== 'open';
  const none = f.count === 0;

  return (
    <div className="row small formation-bar">
      <label className="inline" title="Extra drones that follow you in a shape. They take off, fly, turn and land with you. Each one has its own autopilot.">
        🛩 Wingmen
        <select value={f.count} disabled={off} onChange={(e) => change({ count: Number(e.target.value) })}>
          <option value={0}>none</option>
          {[1, 2, 3, 4].map((n) => <option key={n} value={n}>{n}</option>)}
        </select>
      </label>
      <label className="inline" title="The shape they fly in, lined up with your drone's nose (Q/E turn the whole formation)">
        shape
        <select value={f.shape} disabled={off || none} onChange={(e) => change({ shape: e.target.value })}>
          <option value="v">V (like geese)</option>
          <option value="line">line (side by side)</option>
          <option value="column">column (single file)</option>
        </select>
      </label>
      <label className="inline" title="Distance between neighbours (1.5–4 m)">
        spacing
        <input type="range" min={1.5} max={4} step={0.5} value={f.spacing} disabled={off || none} onChange={(e) => change({ spacing: Number(e.target.value) })} />
        <b className="wind-num">{f.spacing.toFixed(1)}</b> m
      </label>
      <label className="inline" title="Wingmen dodge each other (sideways and up/down) and keep out of obstacles, breaking formation to follow your path through gaps. Untick to see why swarms need this: fly backwards (S) into a column.">
        <input type="checkbox" checked={f.avoid} disabled={off || none} onChange={(e) => change({ avoid: e.target.checked })} /> 🛡 avoid crashes
      </label>
      <button className="link" onClick={() => onLearn('11-formation-flight')}>📖 Learn: formation flight</button>
    </div>
  );
}

/** Live wind: changes apply at once, mid-flight, and stay through Reset and new missions. */
function WindBar({ status, onLearn }: { status: string; onLearn: (slug: string) => void }) {
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
      <button className="link" onClick={() => onLearn('07-exp-wind')}>📖 Learn: wind</button>
    </div>
  );
}

/** Noisy sensors on/off. With them on, the controller flies on its ESTIMATE (the ghost drone in 3D).
 *  Second row: GPS outage and the GPS-loss failsafe, with the failsafe's own settings grouped together. */
function SensorBar({ status, onLearn }: { status: string; onLearn: (slug: string) => void }) {
  const [sensors, setSensors] = useState({ on: false, level: 1, gps: true, outage: false, failsafe: true, drag: 0.4, tilt: 8 });
  const change = (patch: Partial<typeof sensors>) => {
    const v = { ...sensors, ...patch };
    setSensors(v);
    if (v.on !== sensors.on || v.level !== sensors.level || v.gps !== sensors.gps)
      sim.send('sensors', { enabled: v.on, noise_scale: v.level, gps_noise: v.gps });
    if (v.outage !== sensors.outage || v.failsafe !== sensors.failsafe || v.on !== sensors.on || v.drag !== sensors.drag || v.tilt !== sensors.tilt)
      sim.send('gps', { outage: v.outage, failsafe: v.failsafe, drag_n_s_m: v.drag, tilt_deg: v.tilt });
  };
  useEffect(() => {
    if (status === 'open' && sensors.on) sim.send('sensors', { enabled: true, noise_scale: sensors.level, gps_noise: sensors.gps });
    if (status === 'open' && (sensors.outage || sensors.drag !== 0.4 || sensors.tilt !== 8))
      sim.send('gps', { outage: sensors.outage, failsafe: sensors.failsafe, drag_n_s_m: sensors.drag, tilt_deg: sensors.tilt });
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [status]);
  const off = status !== 'open';
  const gpsOff = off || !sensors.on; // GPS settings only matter when the drone flies on its estimate
  const fsOff = gpsOff || !sensors.failsafe;

  return (
    <div className="sensor-bar">
      <div className="row small">
        <label className="inline" title="Off = perfect sensors: the controller knows exactly where the drone is. On = real-ish gyro, accelerometer, altimeter, compass and GPS noise, and an estimator that guesses the state from them.">
          <input type="checkbox" checked={sensors.on} disabled={off} onChange={(e) => change({ on: e.target.checked })} /> 📡 Sensor noise
        </label>
        <label className="inline" title="1 = typical hobby sensors, 3 = poor ones">
          level
          <input type="range" min={0.5} max={3} step={0.25} value={sensors.level} disabled={gpsOff} onChange={(e) => change({ level: Number(e.target.value) })} />
          <b className="wind-num">{sensors.level.toFixed(2)}</b>×
        </label>
        <label className="inline" title="Untick for a perfect GPS while the other sensors (gyro, accelerometer, altimeter, compass) stay noisy. Shows how much of the drift comes from the GPS.">
          <input type="checkbox" checked={sensors.gps} disabled={gpsOff} onChange={(e) => change({ gps: e.target.checked })} /> 🛰 GPS noise
        </label>
        <button className="link" onClick={() => onLearn('08-exp-sensor-noise')}>📖 Learn: sensors, GPS and failsafe</button>
      </div>
      <div className="row small">
        <label className="inline" title="No GPS fixes at all (jamming, indoors, under a bridge). Position then comes from the motion sensors only, and drifts away fast.">
          <input type="checkbox" checked={sensors.outage} disabled={gpsOff} onChange={(e) => change({ outage: e.target.checked })} /> 📵 GPS outage
        </label>
        <span className={`failsafe-group ${fsOff ? 'is-off' : ''}`}>
          <label className="inline" title="On GPS loss: after 1 s, stop trusting the drifting position. Hold level and altitude instead (missions are aborted). Untick to see what happens if the drone keeps trusting it.">
            <input type="checkbox" checked={sensors.failsafe} disabled={gpsOff} onChange={(e) => change({ failsafe: e.target.checked })} /> 🛟 Failsafe
          </label>
          <label className="inline" title="Extra air drag while the failsafe is active, so the drone coasts to a stop when you let go of W/A/S/D. Not real physics (a 'training wheels' helper): 0 = a real drone, which coasts until you counter-steer. The drone's own drag is 0.1. Needs the failsafe ticked.">
            drag
            <input type="range" min={0} max={1} step={0.1} value={sensors.drag} disabled={fsOff} onChange={(e) => change({ drag: Number(e.target.value) })} />
            <b className="wind-num">{sensors.drag.toFixed(1)}</b> N·s/m{sensors.drag === 0 && ' (real)'}
          </label>
          <label className="inline" title="How far a full W/A/S/D press tilts the drone in failsafe. More tilt = it speeds up faster (and is harder to stop). Needs the failsafe ticked.">
            tilt
            <input type="range" min={2} max={20} step={1} value={sensors.tilt} disabled={fsOff} onChange={(e) => change({ tilt: Number(e.target.value) })} />
            <b className="wind-num">{sensors.tilt}</b>°
          </label>
        </span>
        <span className="muted">
          {!sensors.on
            ? 'Tick 📡 Sensor noise first: with perfect sensors the drone always knows where it is, so GPS settings change nothing.'
            : !sensors.failsafe
              ? 'Failsafe off: on GPS loss the drone keeps trusting its drifting position (drag and tilt unused).'
              : 'Failsafe: 1 s after GPS is lost it holds level + height, and W/A/S/D tilt the drone.'}
        </span>
      </div>
      {sensors.on && <p className="muted small">Ghost outline = where the drone <i>thinks</i> it is. The controller steers the ghost, not the real drone. On the ground it averages GPS for 5 s before it may take off.</p>}
    </div>
  );
}

/** 🔋 Battery on/off for the current drone (a Drone Builder design brings its own). */
function BatteryBar({ s, offline, onLearn }: { s: SimState | null; offline: boolean; onLearn: (slug: string) => void }) {
  const [cfg, setCfg] = useState({ cells: 3, capacity_mah: 3000, drain_speedup: 1 });
  const on = !!s?.battery;
  const send = (enabled: boolean, c = cfg) => sim.send('battery', { enabled, ...c });
  const change = (patch: Partial<typeof cfg>) => {
    const c = { ...cfg, ...patch };
    setCfg(c);
    if (on) send(true, c);
  };
  return (
    <div className="row small battery-row">
      <label className="inline" title="Fly on a LiPo battery: it drains, its voltage sags under load (less thrust), and the failsafe brings the drone home when it's low">
        <input type="checkbox" checked={on} disabled={offline} onChange={(e) => send(e.target.checked)} /> 🔋 Battery
      </label>
      {on && s?.battery ? (
        <span className="keys">{s.battery.cells}S {s.battery.capacity_mah} mAh · {s.battery.soc_pct.toFixed(0)} % · {s.battery.voltage_v.toFixed(1)} V{s.battery.drain_speedup > 1 ? ` · drains ${s.battery.drain_speedup}× fast` : ''} (Reset = fresh battery)</span>
      ) : (
        <>
          <select value={cfg.cells} disabled={offline} onChange={(e) => change({ cells: Number(e.target.value) })}>{[2, 3, 4, 6].map((c) => <option key={c} value={c}>{c}S</option>)}</select>
          <select value={cfg.capacity_mah} disabled={offline} onChange={(e) => change({ capacity_mah: Number(e.target.value) })}>{[1300, 2200, 3000, 5000].map((c) => <option key={c} value={c}>{c} mAh</option>)}</select>
          <select value={cfg.drain_speedup} disabled={offline} onChange={(e) => change({ drain_speedup: Number(e.target.value) })} title="Drain faster than real life to see the low-battery failsafe sooner">
            <option value={1}>real time</option><option value={5}>5× faster</option><option value={10}>10× faster</option>
          </select>
        </>
      )}
      <button className="link" onClick={() => onLearn('16-battery-and-builder')}>📖 Learn: batteries</button>
    </div>
  );
}
