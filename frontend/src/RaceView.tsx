// Rings race: the hoops in the 3D view (inside the ENU group: x east, y north, z up) and the big timer
// shown over the 3D view.

import { useFrame } from '@react-three/fiber';
import { useEffect, useMemo, useRef } from 'react';
import * as THREE from 'three';
import Label from './Label3D';
import { sim, useThrottledState, type RaceTrack, type Ring } from './sim';

const DONE = new THREE.Color('#06d6a0');
const NEXT = new THREE.Color('#ffd166');
const LATER = new THREE.Color('#d8dee9');
const DEG = Math.PI / 180;

/** One hoop, standing upright and facing its arrow. A cone in the middle shows which way to fly through. */
function Hoop({ ring, index, matRef }: { ring: Ring; index: number; matRef: (m: THREE.MeshStandardMaterial | null) => void }) {
  return (
    <group position={[ring.x_m, ring.y_m, ring.z_m]} rotation={[0, 0, ring.heading_deg * DEG]}>
      {/* A torus lies flat (in x-y); turn it about y so it stands up with its hole facing +x (the arrow). */}
      <mesh rotation={[0, Math.PI / 2, 0]}>
        <torusGeometry args={[ring.radius_m, 0.06, 12, 48]} />
        <meshStandardMaterial ref={matRef} color="#d8dee9" emissive="#222" />
      </mesh>
      {/* Arrow: a see-through cone in the hoop pointing the way to fly (cones point along +y; turn to +x). */}
      <mesh rotation={[0, 0, -Math.PI / 2]}>
        <coneGeometry args={[0.18, 0.4, 16]} />
        <meshBasicMaterial color="#ffd166" transparent opacity={0.45} />
      </mesh>
      <Label text={String(index + 1)} color="#ffd166" position={[0, 0, ring.radius_m + 0.35]} size={1.6} />
    </group>
  );
}

/** Thin poles from each hoop down to the ground: they make height and distance much easier to judge. */
function Poles({ rings }: { rings: Ring[] }) {
  const line = useMemo(() => {
    const pts = rings.flatMap((r) => [new THREE.Vector3(r.x_m, r.y_m, 0), new THREE.Vector3(r.x_m, r.y_m, r.z_m - r.radius_m)]);
    const geom = new THREE.BufferGeometry().setFromPoints(pts);
    return new THREE.LineSegments(geom, new THREE.LineBasicMaterial({ color: '#8a94a8', transparent: true, opacity: 0.5 }));
  }, [rings]);
  useEffect(() => () => line.geometry.dispose(), [line]);
  return <primitive object={line} />;
}

/** The start pad on the ground. */
function StartPad({ start }: { start: [number, number] }) {
  return (
    <group position={[start[0], start[1], 0.01]}>
      <mesh>
        <circleGeometry args={[0.6, 40]} />
        <meshBasicMaterial color="#4cc9f0" transparent opacity={0.25} />
      </mesh>
      <Label text="Start" color="#4cc9f0" position={[0, -0.9, 0.05]} size={1.4} />
    </group>
  );
}

export default function RaceView({ race }: { race: RaceTrack }) {
  const mats = useRef<(THREE.MeshStandardMaterial | null)[]>([]);
  useFrame(() => {
    const next = sim.latest?.race?.next_ring ?? 0;
    mats.current.forEach((m, i) => {
      if (!m) return;
      const c = i < next ? DONE : i === next ? NEXT : LATER;
      m.color.copy(c);
      m.emissive.copy(c).multiplyScalar(i === next ? 0.5 : 0.1); // the next hoop glows
    });
  });
  return (
    <group>
      <StartPad start={race.start} />
      <Poles rings={race.rings} />
      {race.rings.map((r, i) => (
        <Hoop key={i} ring={r} index={i} matRef={(m) => { mats.current[i] = m; }} />
      ))}
    </group>
  );
}

/** Centre and size of the track on the ground (for framing the camera). */
export function raceBounds(race: RaceTrack): { cx: number; cy: number; span: number } {
  const xs = race.rings.map((r) => r.x_m).concat(race.start[0]);
  const ys = race.rings.map((r) => r.y_m).concat(race.start[1]);
  const [x0, x1, y0, y1] = [Math.min(...xs), Math.max(...xs), Math.min(...ys), Math.max(...ys)];
  return { cx: (x0 + x1) / 2, cy: (y0 + y1) / 2, span: Math.max(x1 - x0, y1 - y0, 4) };
}

/** Big, kid-readable timer over the 3D view. */
export function RaceHud() {
  const s = useThrottledState(15);
  const r = s?.race;
  if (!r) return null;
  const best = r.best_s !== null ? `Best ${r.best_s.toFixed(1)} s` : 'No best time yet';
  return (
    <div className={`race-hud ${r.finished ? 'finished' : ''}`}>
      <div className="race-time">⏱ {r.time_s.toFixed(1)} s</div>
      <div className="race-sub">
        {r.finished
          ? (r.new_best ? '🏁 Finished! ⭐ New best time!' : `🏁 Finished! ${best}`)
          : r.running
            ? `Ring ${r.next_ring + 1} of ${r.total} · ${best}`
            : `Press Take off to start · ${best}`}
      </div>
      {r.finished && <div className="race-sub small">Press ↺ Reset to race again</div>}
    </div>
  );
}
