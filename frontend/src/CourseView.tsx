// A mission course in the 3D view (drawn inside the ENU group, so x east, y north, z up, in metres):
// obstacles, their keep-out zones on the ground, waypoint rings at cruise height, and the planned route.

import { useFrame } from '@react-three/fiber';
import { useEffect, useMemo, useRef } from 'react';
import * as THREE from 'three';
import Label from './Label3D';
import { sim, type Course, type CourseBox } from './sim';

const WP_DONE = new THREE.Color('#06d6a0');
const WP_NEXT = new THREE.Color('#ffd166');
const WP_LATER = new THREE.Color('#d8dee9');

function Obstacle({ box, ghost = false }: { box: CourseBox; ghost?: boolean }) {
  const w = box.x_max - box.x_min, d = box.y_max - box.y_min, h = box.height_m;
  const edges = useMemo(() => new THREE.EdgesGeometry(new THREE.BoxGeometry(w, d, h)), [w, d, h]);
  useEffect(() => () => edges.dispose(), [edges]);
  return (
    <group position={[(box.x_min + box.x_max) / 2, (box.y_min + box.y_max) / 2, h / 2]}>
      <mesh>
        <boxGeometry args={[w, d, h]} />
        <meshStandardMaterial color="#5c677d" transparent opacity={ghost ? 0.25 : 0.8} depthWrite={!ghost} />
      </mesh>
      <lineSegments geometry={edges}>
        <lineBasicMaterial color="#aab4c8" />
      </lineSegments>
    </group>
  );
}

/** The keep-out zone around a box, as a rounded outline on the ground (A* keeps the drone's centre outside it). */
function KeepOut({ box, zone }: { box: CourseBox; zone: number }) {
  const line = useMemo(() => {
    const pts: THREE.Vector3[] = [];
    const corners: [number, number, number][] = [
      [box.x_max, box.y_max, 0], [box.x_min, box.y_max, Math.PI / 2],
      [box.x_min, box.y_min, Math.PI], [box.x_max, box.y_min, (3 * Math.PI) / 2],
    ];
    for (const [cx, cy, a0] of corners)
      for (let i = 0; i <= 8; i++) {
        const a = a0 + (i / 8) * (Math.PI / 2);
        pts.push(new THREE.Vector3(cx + zone * Math.cos(a), cy + zone * Math.sin(a), 0.02));
      }
    const geom = new THREE.BufferGeometry().setFromPoints(pts);
    return new THREE.LineLoop(geom, new THREE.LineBasicMaterial({ color: '#ff9f1c', transparent: true, opacity: 0.6 }));
  }, [box, zone]);
  useEffect(() => () => line.geometry.dispose(), [line]);
  return <primitive object={line} />;
}

/** Planned route: a dashed yellow line at cruise height. */
function Route({ route, alt }: { route: [number, number][]; alt: number }) {
  const line = useMemo(() => {
    const geom = new THREE.BufferGeometry().setFromPoints(route.map(([x, y]) => new THREE.Vector3(x, y, alt)));
    const l = new THREE.Line(geom, new THREE.LineDashedMaterial({ color: '#ffd166', dashSize: 0.3, gapSize: 0.2 }));
    l.computeLineDistances();
    return l;
  }, [route, alt]);
  useEffect(() => () => line.geometry.dispose(), [line]);
  return <primitive object={line} />;
}

/** Waypoint rings: green = reached, yellow = next, white = later. A pole down to the ground helps judge position. */
function Waypoints({ course }: { course: Course }) {
  const mats = useRef<(THREE.MeshBasicMaterial | null)[]>([]);
  const alt = course.cruise_alt_m;
  const poles = useMemo(() => {
    const pts = course.waypoints.flatMap(([x, y]) => [new THREE.Vector3(x, y, 0), new THREE.Vector3(x, y, alt)]);
    const geom = new THREE.BufferGeometry().setFromPoints(pts);
    return new THREE.LineSegments(geom, new THREE.LineBasicMaterial({ color: '#8a94a8', transparent: true, opacity: 0.5 }));
  }, [course.waypoints, alt]);
  useEffect(() => () => poles.geometry.dispose(), [poles]);

  useFrame(() => {
    const next = sim.latest?.mission?.next_wp ?? 0; // 0 = no mission running: all rings white
    mats.current.forEach((m, i) => m?.color.copy(next === 0 ? WP_LATER : i < next ? WP_DONE : i === next ? WP_NEXT : WP_LATER));
  });

  return (
    <>
      <primitive object={poles} />
      {course.waypoints.map(([x, y], i) => (
        <group key={i}>
          {i > 0 && (
            <mesh position={[x, y, alt]}>
              <torusGeometry args={[0.35, 0.025, 8, 40]} />
              <meshBasicMaterial ref={(el) => { mats.current[i] = el; }} color="#d8dee9" />
            </mesh>
          )}
          <Label text={i === 0 ? 'Start' : i === course.waypoints.length - 1 ? `${i} (land)` : String(i)}
            color={i === 0 ? '#4cc9f0' : '#ffd166'} position={[x, y, alt + 0.55]} size={1.6} />
        </group>
      ))}
    </>
  );
}

export default function CourseView({ course }: { course: Course }) {
  return (
    <group>
      {course.obstacles.map((b, i) => (
        <group key={i}>
          <Obstacle box={b} ghost={course.explore} />
          {!course.explore && <KeepOut box={b} zone={course.safety_zone_m} />}
        </group>
      ))}
      <Waypoints course={course} />
      {course.route && <Route route={course.route} alt={course.cruise_alt_m} />}
    </group>
  );
}

/** Centre and size of the course on the ground (for framing the camera). */
export function courseBounds(course: Course): { cx: number; cy: number; span: number } {
  const xs = course.waypoints.map((p) => p[0]).concat(course.obstacles.flatMap((b) => [b.x_min, b.x_max]));
  const ys = course.waypoints.map((p) => p[1]).concat(course.obstacles.flatMap((b) => [b.y_min, b.y_max]));
  const [x0, x1, y0, y1] = [Math.min(...xs), Math.max(...xs), Math.min(...ys), Math.max(...ys)];
  return { cx: (x0 + x1) / 2, cy: (y0 + y1) / 2, span: Math.max(x1 - x0, y1 - y0, 4) };
}
