// 🗺 Autonomous exploration in the 3D view (inside the ENU group: x east, y north, z up):
// the explorer's own map on the floor (dark = unknown, light = seen free, red = walls it mapped), the people walking
// around (where they REALLY are), what the drone thinks is moving (yellow rings + arrows), its path and goal.

import { useFrame } from '@react-three/fiber';
import { useMemo, useRef } from 'react';
import * as THREE from 'three';
import { sim, type ExploreMap } from './sim';

const COLORS: [number, number, number, number][] = [
  [22, 28, 42, 200], // unknown
  [120, 150, 190, 150], // free
  [239, 71, 111, 255], // wall
];
const MAX_TRACKED = 8;

function decode(m: ExploreMap): Uint8Array {
  const raw = atob(m.cells);
  const rgba = new Uint8Array(m.nx * m.ny * 4);
  for (let k = 0; k < raw.length; k++) rgba.set(COLORS[raw.charCodeAt(k)] ?? COLORS[0], k * 4);
  return rgba;
}

export default function ExploreView() {
  const state = useRef<{ key: string; tex: THREE.DataTexture | null }>({ key: '', tex: null });
  const floor = useRef<THREE.Mesh>(null);
  const objs = useMemo(() => {
    const path = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: '#ffd166' }));
    path.frustumCulled = false;
    const goal = new THREE.Mesh(new THREE.TorusGeometry(0.35, 0.05, 8, 24), new THREE.MeshBasicMaterial({ color: '#06d6a0' }));
    const people = new THREE.InstancedMesh(new THREE.BoxGeometry(1, 1, 1), new THREE.MeshStandardMaterial({ color: '#f4a261' }), 4);
    people.frustumCulled = false;
    const rings = new THREE.InstancedMesh(new THREE.TorusGeometry(0.5, 0.04, 6, 20), new THREE.MeshBasicMaterial({ color: '#ffd166' }), MAX_TRACKED);
    rings.frustumCulled = false;
    const arrows = new THREE.LineSegments(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: '#ffd166' }));
    arrows.geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(MAX_TRACKED * 6), 3));
    arrows.frustumCulled = false;
    const pin = new THREE.Group();
    const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.03, 2.2, 8), new THREE.MeshStandardMaterial({ color: '#ffd166' }));
    pole.rotation.x = Math.PI / 2;
    pole.position.z = 1.1;
    const flag = new THREE.Mesh(new THREE.ConeGeometry(0.25, 0.5, 4), new THREE.MeshStandardMaterial({ color: '#ef476f', emissive: '#ef476f', emissiveIntensity: 0.5 }));
    flag.rotation.x = -Math.PI / 2;
    flag.position.z = 2.45;
    const spot = new THREE.Mesh(new THREE.RingGeometry(0.3, 0.42, 24), new THREE.MeshBasicMaterial({ color: '#ffd166' }));
    spot.position.z = 0.04;
    pin.add(pole, flag, spot);
    return { path, goal, people, rings, arrows, pin };
  }, []);
  const m4 = useMemo(() => new THREE.Matrix4(), []);
  const group = useRef<THREE.Group>(null);

  useFrame(() => {
    const e = sim.latest?.explore;
    if (group.current) group.current.visible = !!e;
    if (!e) return;
    // the map texture (it only comes every 0.5 s)
    if (e.map && floor.current) {
      const m = e.map;
      const key = `${m.nx}x${m.ny}:${m.cells.length}:${m.cells.slice(-64)}:${m.cells.slice(0, 32)}:${e.coverage}`;
      if (key !== state.current.key) {
        state.current.key = key;
        const data = decode(m);
        let tex = state.current.tex;
        if (!tex || tex.image.width !== m.nx || tex.image.height !== m.ny) {
          tex?.dispose();
          tex = new THREE.DataTexture(data, m.nx, m.ny, THREE.RGBAFormat);
          tex.magFilter = THREE.NearestFilter;
          tex.minFilter = THREE.NearestFilter;
          state.current.tex = tex;
          const mat = floor.current.material as THREE.MeshBasicMaterial;
          mat.map = tex;
          mat.needsUpdate = true;
        } else {
          (tex.image.data as Uint8Array).set(data);
        }
        tex.needsUpdate = true;
        floor.current.scale.set(m.nx * m.res, m.ny * m.res, 1);
        floor.current.position.set(m.x0 + (m.nx * m.res) / 2, m.y0 + (m.ny * m.res) / 2, 0.03);
      }
    }
    // path + goal at flying height
    const z = sim.latest ? Math.max(0.2, sim.latest.pos_m[2]) : 1.5;
    objs.path.geometry.setFromPoints(e.path.map(([x, y]) => new THREE.Vector3(x, y, z)));
    objs.pin.visible = !!e.target && e.phase !== 'arrived';
    if (e.target) objs.pin.position.set(e.target[0], e.target[1], 0);
    objs.goal.visible = !!e.goal && !e.target;
    if (e.goal) objs.goal.position.set(e.goal[0], e.goal[1], 0.06);
    // the people, where they really are
    objs.people.count = e.people.length;
    e.people.forEach(([x, y, size], i) => objs.people.setMatrixAt(i, m4.makeScale(size, size, 1.8).setPosition(x, y, 0.9)));
    objs.people.instanceMatrix.needsUpdate = true;
    // what the drone believes is moving, and how fast (arrow = 1 s ahead)
    const moving = e.tracked.filter(([, , vx, vy]) => Math.hypot(vx, vy) > 0.2).slice(0, MAX_TRACKED);
    objs.rings.count = moving.length;
    const pos = objs.arrows.geometry.attributes.position as THREE.BufferAttribute;
    moving.forEach(([x, y, vx, vy], i) => {
      objs.rings.setMatrixAt(i, m4.makeTranslation(x, y, 0.08));
      pos.setXYZ(2 * i, x, y, 0.1);
      pos.setXYZ(2 * i + 1, x + vx, y + vy, 0.1);
    });
    for (let i = moving.length; i < MAX_TRACKED; i++) {
      pos.setXYZ(2 * i, 0, 0, -10);
      pos.setXYZ(2 * i + 1, 0, 0, -10);
    }
    pos.needsUpdate = true;
    objs.rings.instanceMatrix.needsUpdate = true;
  });

  return (
    <group ref={group}>
      <mesh ref={floor}>
        <planeGeometry args={[1, 1]} />
        <meshBasicMaterial transparent depthWrite={false} />
      </mesh>
      <primitive object={objs.path} />
      <primitive object={objs.goal} />
      <primitive object={objs.pin} />
      <primitive object={objs.people} />
      <primitive object={objs.rings} />
      <primitive object={objs.arrows} />
    </group>
  );
}
