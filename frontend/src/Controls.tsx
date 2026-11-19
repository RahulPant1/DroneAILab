// Flight buttons, keyboard nudges, a "fly to" box, and the mission bar (course + autopilot).

import { useEffect, useState } from 'react';
import { api, sim, useSimUi, useThrottledState, type SimState } from './sim';

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

  return (
    <div className="controls">
      <div className="row">
        <button className="primary big" disabled={offline || flying || s?.mode === 'land'} onClick={() => sim.send('takeoff', { alt_m: alt })}>
          🛫 Take off
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
        <span className="keys">
          <kbd>W</kbd><kbd>S</kbd> forward/back · <kbd>A</kbd><kbd>D</kbd> left/right · <kbd>R</kbd><kbd>F</kbd> up/down · <kbd>Q</kbd><kbd>E</kbd> turn
        </span>
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
    </div>
  );
}

const PLANNER_LABELS: Record<string, string> = {
  straight: 'Straight lines (ignores obstacles)',
  potential: 'Potential field (reactive)',
  astar: 'A* (plans ahead)',
};

const PHASE_LABELS: Record<string, string> = { takeoff: 'taking off', cruise: 'flying the route', land: 'landing' };

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
