// 🎮 Play mode: the 3D world fills the screen, with a few big buttons on top. For kids (and for just flying).
// Everything else (panels, sliders, telemetry) is in 🔧 Engineer mode, one click away.

import { useEffect, useRef, useState } from 'react';
import CameraView from './CameraView';
import { useKeyboardSticks, useTerrains } from './Controls';
import { RaceHud } from './RaceView';
import Scene from './Scene';
import { sim, useSimUi, useThrottledState } from './sim';
import { speak, useVoice } from './Voice';

const SCENES: { id: string; icon: string; label: string; run: () => void }[] = [
  { id: 'free', icon: '🎈', label: 'Free fly', run: () => { sim.send('race', { track: null }); sim.send('course', { scenario: null }); } },
  { id: 'race', icon: '🏁', label: 'Race rings', run: () => sim.send('race', { track: 'first_flight' }) },
  { id: 'eight', icon: '♾️', label: 'Figure eight', run: () => sim.send('race', { track: 'figure_eight' }) },
  { id: 'explore', icon: '🏠', label: 'Explore a house', run: () => sim.send('explore', { world: 'rooms', local: 'dwa', localisation: 'gps', movers: true }) },
  { id: 'forest', icon: '🌲', label: 'Explore a forest', run: () => sim.send('explore', { world: 'forest', local: 'dwa', localisation: 'gps', movers: false }) },
  { id: 'hills', icon: '🏔', label: 'Hills', run: () => sim.send('terrain', { id: 'hills' }) },
  { id: 'friends', icon: '🦆', label: 'Bring friends', run: () => sim.send('formation', { count: 3, shape: 'v', spacing_m: 1.5, avoid: true }) },
];

// The keys, and what to say for the same thing (one word is enough; add a number for metres: "left 3")
const CHEATS: [string, string][] = [
  ['T', 'take off'], ['W ↑', 'forward'], ['S ↓', 'back'], ['A ←', 'left'], ['D →', 'right'], ['R', 'up'], ['F', 'down'],
  ['Q', 'turn left'], ['E', 'turn right'], ['H', 'stop'], ['L', 'land'], ['', 'come home'], ['', 'turn around'],
];

function toggleFullscreen() {
  if (document.fullscreenElement) document.exitFullscreen().catch(() => undefined);
  else document.documentElement.requestFullscreen().catch(() => undefined);
}

export default function Play({ armLength }: { armLength: number }) {
  useKeyboardSticks(2);
  const s = useThrottledState(5);
  const { course, race, terrain } = useSimUi();
  const places = useTerrains().filter((p) => p.downloaded);
  const v = useVoice();
  const [cam, setCam] = useState<'behind' | 'free'>('behind');
  const flying = !!s && (s.mode === 'takeoff' || s.mode === 'hover' || s.mode === 'mission' || s.mode === 'precland' || (!!s.explore && (s.explore.phase === 'explore' || s.explore.phase === 'takeoff')) || !!s.agile);
  const where = race ? `🏁 ${race.title}` : course ? course.title : terrain ? `🏔 ${terrain.title}` : '🎈 Free fly';
  const reply = v.busy || (v.reply ? v.reply.say : 'Press 🎙 and say “take off”!');
  // say "Arrived!" / "Can't get there" out loud
  const spoken = useRef('');
  useEffect(() => {
    const m = s?.message ?? '';
    if (m.startsWith('📍') && m !== spoken.current) {
      spoken.current = m;
      if (v.talk && !m.startsWith('📍 Going')) speak(m.replace('📍', ''));
    }
  }, [s?.message, v.talk]);

  return (
    <main className="play">
      <div className="canvas-wrap play-canvas">
        <Scene armLength={armLength} follow={cam === 'free'} chase={cam === 'behind'} toy
          onGroundClick={(x, y) => { sim.send('goto_point', { x_m: x, y_m: y }); if (v.talk) speak('On my way!'); }} />
        <RaceHud />
        <CameraView />

        <div className="play-top">
          <span className="play-where">{where}</span>
          <span className="play-status">
            {s ? (flying ? `✈ flying · ${s.pos_m[2].toFixed(1)} m up` : s.on_ground ? '🛬 on the ground' : '…') : 'connecting…'}
            {s?.terrain_agl_m !== undefined && flying && Math.abs(s.pos_m[2] - s.terrain_agl_m) > 0.5 && (
              <b style={{ color: s.terrain_agl_m < 3 ? '#ef476f' : undefined }}
                title="Altitude is measured from home (the take-off spot). Over a hill the ground is closer than that!">
                {' '}· {s.terrain_agl_m.toFixed(1)} m above the ground{s.terrain_agl_m < 3 ? ' ⚠ hill!' : ''}
              </b>
            )}
            {s?.explore && !s.explore.target && ` · mapped ${Math.round(s.explore.coverage * 100)} %`}
          </span>
          {s?.est_pos_m && (
            <span className="play-status play-guess" title="With noisy sensors the drone only GUESSES where it is (the see-through ghost). This is how far its guess is from the truth.">
              🎯 its guess is <b>{Math.hypot(s.est_pos_m[0] - s.pos_m[0], s.est_pos_m[1] - s.pos_m[1], s.est_pos_m[2] - s.pos_m[2]).toFixed(2)} m</b> off
            </span>
          )}
          <span className="spacer" />
          <span className="play-small play-lights" title="Like real aircraft: a red light on the left side, green on the right">
            <b style={{ color: '#ef476f' }}>● left</b> <b style={{ color: '#06d6a0' }}>● right</b>
          </span>
          <button className="play-small" onClick={() => setCam(cam === 'behind' ? 'free' : 'behind')}
            title="Turns with the drone: starts behind it, so its left is the screen's left; drag to look from another angle, scroll to zoom. Free: the view doesn't turn when the drone does.">
            {cam === 'behind' ? '🎥 Turns with the drone' : '🔭 Free camera'}
          </button>
          <button className="play-small" onClick={toggleFullscreen} title="Full screen (Esc to leave)">⛶ Full screen</button>
        </div>

        <div className="play-scenes">
          {[...SCENES, ...places.map((p) => ({ id: p.id, icon: '🗺', label: p.title.split(',')[0], run: () => sim.send('terrain', { id: p.id }) }))].map((sc) => (
            <button key={sc.id} className="play-card" disabled={v.offline} onClick={sc.run}>
              <span className="play-card-icon">{sc.icon}</span>
              {sc.label}
            </button>
          ))}
        </div>

        <div className="play-bottom">
          <div className={`play-reply ${v.reply?.pending ? 'pending' : ''}`}>
            {v.reply && !v.busy && <span className="play-heard">“{v.reply.heard}”</span>}
            <b>{v.recording && !v.talking && !v.busy ? 'Say a command. It goes by itself when you stop talking. No need to press again.' : reply}</b>
            {s?.message && s.message !== reply && <span className="play-heard">{s.message}</span>}
          </div>
          <div className="play-buttons">
            <button className={`play-btn talk ${v.recording ? 'recording' : ''} ${v.talking ? 'hearing' : ''}`} disabled={!v.canTalk && !v.recording}
              onClick={v.toggle} title={v.mic?.ok ? 'Press and talk: it goes when you stop talking' : v.mic?.detail}>
              {v.recording ? (v.talking ? '🔴 Hearing you…' : '👂 Just talk…') : '🎙 Talk'}
            </button>
            <button className={`play-btn small ${v.handsFree ? 'on' : ''}`} disabled={!v.canTalk} onClick={() => v.setHandsFree(!v.handsFree)}
              title="Hands-free: keeps listening; every time you stop talking, the command goes. Press again to stop listening.">
              🎧 {v.handsFree ? 'Hands-free ON' : 'Hands-free'}
            </button>
            {v.reply?.pending && <button className="play-btn go" onClick={() => v.send('go')}>▶ Go!</button>}
            <button className="play-btn" disabled={v.offline || flying} onClick={() => sim.send('takeoff', { alt_m: 2 })}>🛫 Take off</button>
            <button className="play-btn" disabled={v.offline || !flying} onClick={() => sim.send('land')}>🛬 Land</button>
            <button className="play-btn" disabled={v.offline || !flying} onClick={() => v.send('come home')}>🏠 Home</button>
            <button className="play-btn stop" disabled={v.offline} onClick={() => v.send('stop')}>✋ Stop</button>
            <button className="play-btn small" onClick={() => sim.send('reset')} title="Back on the ground at the start">↺</button>
          </div>
          <div className="play-keys">👆 Click the ground: it finds its own way there · drag to look around · scroll to zoom</div>
          <div className="play-cheats" title="Each key, and what to say instead">
            {CHEATS.map(([keys, say]) => (
              <span key={say} className="play-cheat"><kbd>{keys}</kbd> “{say}”</span>
            ))}
          </div>
        </div>
      </div>
    </main>
  );
}
