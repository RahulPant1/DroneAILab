// Flight buttons, keyboard nudges, and a "fly to" box.

import { useEffect, useState } from 'react';
import { sim, useSimUi, useThrottledState } from './sim';

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
  const flying = s?.mode === 'takeoff' || s?.mode === 'hover';
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
    </div>
  );
}
