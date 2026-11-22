// 🌾 The farm in the 3D view (inside the ENU group: x east, y north, z up, metres): fields on the ground,
// no-fly zones (red), landing pads with their markers, named places, the geofence, and survey photo footprints.

import { useFrame } from '@react-three/fiber';
import { useEffect, useMemo } from 'react';
import * as THREE from 'three';
import Label from './Label3D';
import { sim, type Photo, type Site } from './sim';

const MAX_PHOTOS = 600;
const NO_FLY_HEIGHT_M = 25; // drawn this tall (it really blocks every height)

function Field({ name, polygon, crop }: { name: string; polygon: [number, number][]; crop: string }) {
  const { fill, outline, cx, cy } = useMemo(() => {
    const shape = new THREE.Shape(polygon.map(([x, y]) => new THREE.Vector2(x, y)));
    const fill = new THREE.ShapeGeometry(shape);
    const pts = [...polygon, polygon[0]].map(([x, y]) => new THREE.Vector3(x, y, 0.04));
    const outline = new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), new THREE.LineBasicMaterial({ color: '#7bd389' }));
    const cx = polygon.reduce((a, p) => a + p[0], 0) / polygon.length;
    const cy = polygon.reduce((a, p) => a + p[1], 0) / polygon.length;
    return { fill, outline, cx, cy };
  }, [polygon]);
  useEffect(() => () => { fill.dispose(); outline.geometry.dispose(); }, [fill, outline]);
  return (
    <group>
      <mesh geometry={fill} position={[0, 0, 0.02]}>
        <meshBasicMaterial color="#2d6a4f" transparent opacity={0.35} depthWrite={false} />
      </mesh>
      <primitive object={outline} />
      <Label text={name} color="#b7e4c7" position={[cx, cy, 1.2]} size={9} />
      <Label text={crop} color="#95d5b2" position={[cx, cy, 0.4]} size={5} />
    </group>
  );
}

function NoFly({ zone }: { zone: Site['no_fly'][number] }) {
  const w = zone.x_max - zone.x_min, d = zone.y_max - zone.y_min, h = NO_FLY_HEIGHT_M;
  const edges = useMemo(() => new THREE.EdgesGeometry(new THREE.BoxGeometry(w, d, h)), [w, d, h]);
  useEffect(() => () => edges.dispose(), [edges]);
  return (
    <group position={[(zone.x_min + zone.x_max) / 2, (zone.y_min + zone.y_max) / 2, h / 2]}>
      <mesh>
        <boxGeometry args={[w, d, h]} />
        <meshBasicMaterial color="#ef476f" transparent opacity={0.1} depthWrite={false} />
      </mesh>
      <lineSegments geometry={edges}>
        <lineBasicMaterial color="#ef476f" transparent opacity={0.6} />
      </lineSegments>
      <Label text={`⛔ ${zone.name}`} color="#ff8fa3" position={[0, 0, h / 2 + 1.5]} size={8} />
    </group>
  );
}

/** A landing pad: a light mat with a black square ring (the marker the down camera looks for). */
function Pad({ pad }: { pad: Site['pads'][number] }) {
  const half = pad.size_m / 2;
  return (
    <group position={[pad.x_m, pad.y_m, 0]}>
      <mesh position={[0, 0, 0.03]}>
        <planeGeometry args={[pad.size_m * 1.5, pad.size_m * 1.5]} />
        <meshBasicMaterial color="#cfd6e3" />
      </mesh>
      <mesh position={[0, 0, 0.04]} rotation={[0, 0, Math.PI / 4]}>
        <ringGeometry args={[half * 0.5 * Math.SQRT2, half * Math.SQRT2, 4, 1]} />
        <meshBasicMaterial color="#111111" />
      </mesh>
      <Label text={`🅿 ${pad.name}`} color="#e9ecef" position={[0, 0, 1.6]} size={5} />
    </group>
  );
}

function Geofence({ radius }: { radius: number }) {
  const line = useMemo(() => {
    const pts = Array.from({ length: 129 }, (_, i) => {
      const a = (i / 128) * Math.PI * 2;
      return new THREE.Vector3(radius * Math.cos(a), radius * Math.sin(a), 0.05);
    });
    const l = new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), new THREE.LineDashedMaterial({ color: '#f4a261', dashSize: 2, gapSize: 1.5 }));
    l.computeLineDistances();
    return l;
  }, [radius]);
  useEffect(() => () => line.geometry.dispose(), [line]);
  return <primitive object={line} />;
}

/** Each photo's patch of ground, as a faint rectangle: (along, across) = footprint_per_m × height. */
function Footprints({ pick, color, perM }: { pick: () => Photo[] | undefined | null; color: string; perM: [number, number] }) {
  const mesh = useMemo(() => {
    const m = new THREE.InstancedMesh(new THREE.PlaneGeometry(1, 1),
      new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.12, depthWrite: false, side: THREE.DoubleSide }), MAX_PHOTOS);
    m.count = 0;
    m.frustumCulled = false;
    return m;
  }, [color]);
  useEffect(() => () => { mesh.geometry.dispose(); (mesh.material as THREE.Material).dispose(); }, [mesh]);
  const tmp = useMemo(() => ({ m: new THREE.Matrix4(), q: new THREE.Quaternion(), p: new THREE.Vector3(), s: new THREE.Vector3(), z: new THREE.Vector3(0, 0, 1) }), []);
  useFrame(() => {
    const photos = pick();
    if (photos === undefined) return; // nothing new: keep what is drawn
    const list = (photos ?? []).slice(0, MAX_PHOTOS);
    if (list.length === mesh.count) return;
    list.forEach(([x, y, alt, angle], i) => {
      tmp.q.setFromAxisAngle(tmp.z, angle);
      tmp.p.set(x, y, 0.06 + (i % 5) * 0.004); // tiny height steps stop overlapping photos from flickering
      tmp.s.set(perM[1] * alt, perM[0] * alt, 1); // x along the line, y across it
      mesh.setMatrixAt(i, tmp.m.compose(tmp.p, tmp.q, tmp.s));
    });
    mesh.count = list.length;
    mesh.instanceMatrix.needsUpdate = true;
  });
  return <primitive object={mesh} />;
}

export default function FarmView({ site, perM }: { site: Site; perM: [number, number] }) {
  return (
    <group>
      {site.fields.map((f) => <Field key={f.name} {...f} />)}
      {site.no_fly.map((z) => <NoFly key={z.name} zone={z} />)}
      {site.pads.map((p) => <Pad key={p.name} pad={p} />)}
      {site.buildings.map((b) => (
        <Label key={b.name} text={`${b.name} · ${b.height_m} m`} color="#d8dee9" position={[(b.x_min + b.x_max) / 2, (b.y_min + b.y_max) / 2, b.height_m + 1.2]} size={5} />
      ))}
      {site.places.map((p) => (
        <group key={p.name}>
          <mesh position={[p.x_m, p.y_m, 1]}>
            <cylinderGeometry args={[0.08, 0.08, 2, 6]} />
            <meshBasicMaterial color="#4cc9f0" />
          </mesh>
          <Label text={`📍 ${p.name}`} color="#90e0ef" position={[p.x_m, p.y_m, 2.6]} size={5} />
        </group>
      ))}
      <Geofence radius={site.geofence_radius_m} />
      {/* The server sends the photo list only when it changed (a new plan starts with an empty one): keep the rest. */}
      <Footprints perM={perM} color="#4cc9f0" pick={() => sim.latest?.survey?.photos} />
      <Footprints perM={perM} color="#f4a261" pick={() => sim.latest?.sitl?.plan_result?.photos ?? null} />
    </group>
  );
}

/** Centre and size of the farm on the ground (for framing the camera). */
export function siteBounds(site: Site): { cx: number; cy: number; span: number } {
  const xs = site.fields.flatMap((f) => f.polygon.map((p) => p[0])).concat(site.no_fly.flatMap((b) => [b.x_min, b.x_max]), 0);
  const ys = site.fields.flatMap((f) => f.polygon.map((p) => p[1])).concat(site.no_fly.flatMap((b) => [b.y_min, b.y_max]), 0);
  const [x0, x1, y0, y1] = [Math.min(...xs), Math.max(...xs), Math.min(...ys), Math.max(...ys)];
  return { cx: (x0 + x1) / 2, cy: (y0 + y1) / 2, span: Math.max(x1 - x0, y1 - y0) * 0.8 };
}
