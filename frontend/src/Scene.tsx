// 3D view. The sim uses ENU (x east, y north, z UP); three.js uses y up.
// We put everything inside one group rotated -90° about x, so inside it we can use ENU
// coordinates and the sim's quaternion directly. The ground grid lives outside that group.
//
// Rendering is on demand: a frame is drawn only when a new sim state arrives (~30 Hz)
// or the camera is dragged, and nothing is drawn while the browser tab is hidden.

import { Grid, OrbitControls } from '@react-three/drei';
import { Canvas, useFrame, useThree } from '@react-three/fiber';
import { useEffect, useMemo, useRef } from 'react';
import * as THREE from 'three';
import CourseView, { courseBounds } from './CourseView';
import Label from './Label3D';
import { sim, useSimUi, type Vec3 } from './sim';

const ENU_TO_THREE: [number, number, number] = [-Math.PI / 2, 0, 0];

function enuToThree(p: Vec3, out = new THREE.Vector3()): THREE.Vector3 {
  return out.set(p[0], p[2], -p[1]);
}

/** Ask for a redraw whenever new state arrives, unless the tab is hidden. */
function RedrawOnState() {
  const invalidate = useThree((s) => s.invalidate);
  useEffect(() => sim.subscribeState(() => !document.hidden && invalidate()), [invalidate]);
  return null;
}

// Motor order FR, RL, FL, RR (matches backend physics.py). Positions in units of L/sqrt(2).
const MOTORS: { x: number; y: number; front: boolean }[] = [
  { x: 1, y: -1, front: true },
  { x: -1, y: 1, front: false },
  { x: 1, y: 1, front: true },
  { x: -1, y: -1, front: false },
];

function Drone({ armLength }: { armLength: number }) {
  const group = useRef<THREE.Group>(null!);
  const rotorMats = useRef<(THREE.MeshStandardMaterial | null)[]>([]);
  const a = armLength / Math.SQRT2;
  const rotorRadius = 0.45 * armLength;

  useFrame(() => {
    const s = sim.latest;
    if (!s) return;
    group.current.position.set(s.pos_m[0], s.pos_m[1], s.pos_m[2]);
    const [w, x, y, z] = s.quat;
    group.current.quaternion.set(x, y, z, w); // three.js order is (x, y, z, w)
    s.motor_pct.forEach((pct, i) => {
      const m = rotorMats.current[i];
      if (m) m.emissiveIntensity = 0.05 + (pct / 100) * 1.5; // brighter rotor = harder working motor
    });
  });

  return (
    <group ref={group}>
      <mesh>
        <boxGeometry args={[0.6 * armLength, 0.45 * armLength, 0.2 * armLength]} />
        <meshStandardMaterial color="#d8dee9" />
      </mesh>
      {/* Nose cone points forward (+x body) so you can always see the heading. */}
      <mesh position={[0.42 * armLength, 0, 0]} rotation={[0, 0, -Math.PI / 2]}>
        <coneGeometry args={[0.1 * armLength, 0.3 * armLength, 12]} />
        <meshStandardMaterial color="#ef476f" />
      </mesh>
      {MOTORS.map((m, i) => (
        <group key={i}>
          <mesh position={[(m.x * a) / 2, (m.y * a) / 2, 0]} rotation={[0, 0, Math.atan2(m.y, m.x)]}>
            <boxGeometry args={[armLength, 0.07 * armLength, 0.06 * armLength]} />
            <meshStandardMaterial color={m.front ? '#ef476f' : '#4a5568'} />
          </mesh>
          <mesh position={[m.x * a, m.y * a, 0.1 * armLength]} rotation={[Math.PI / 2, 0, 0]}>
            <cylinderGeometry args={[rotorRadius, rotorRadius, 0.01, 28]} />
            <meshStandardMaterial
              ref={(el) => {
                rotorMats.current[i] = el;
              }}
              color="#90e0ef"
              emissive="#4cc9f0"
              transparent
              opacity={0.55}
            />
          </mesh>
        </group>
      ))}
    </group>
  );
}

/** The path flown so far: a line that grows as the drone moves (last ~1500 points). */
function Trail() {
  const N = 1500;
  const { line, positions } = useMemo(() => {
    const positions = new Float32Array(N * 3);
    const geom = new THREE.BufferGeometry();
    geom.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geom.setDrawRange(0, 0);
    const line = new THREE.Line(geom, new THREE.LineBasicMaterial({ color: '#4cc9f0' }));
    line.frustumCulled = false;
    return { line, positions };
  }, []);
  const count = useRef(0);
  const lastT = useRef(-1);

  useFrame(() => {
    const s = sim.latest;
    if (!s || s.t_s === lastT.current) return;
    if (s.t_s < lastT.current) count.current = 0; // sim was reset: clear the trail
    lastT.current = s.t_s;
    const n = count.current;
    if (n > 0) {
      const i = (n - 1) * 3;
      const d = Math.hypot(positions[i] - s.pos_m[0], positions[i + 1] - s.pos_m[1], positions[i + 2] - s.pos_m[2]);
      if (d < 0.03) return;
    }
    if (n === N) {
      positions.copyWithin(0, 3);
      count.current = N - 1;
    }
    positions.set(s.pos_m, count.current * 3);
    count.current++;
    const geom = line.geometry;
    geom.attributes.position.needsUpdate = true;
    geom.setDrawRange(0, count.current);
  });

  return <primitive object={line} />;
}

/** Where the drone is trying to go (shown while armed) and its shadow on the ground. */
function Markers({ armLength }: { armLength: number }) {
  const target = useRef<THREE.Group>(null!);
  const shadow = useRef<THREE.Mesh>(null!);
  useFrame(() => {
    const s = sim.latest;
    if (!s) return;
    target.current.visible = s.armed;
    target.current.position.set(s.setpoint.x_m, s.setpoint.y_m, Math.max(0, s.setpoint.alt_m));
    shadow.current.position.set(s.pos_m[0], s.pos_m[1], 0.002);
    const mat = shadow.current.material as THREE.MeshBasicMaterial;
    mat.opacity = Math.max(0.08, 0.45 - s.pos_m[2] * 0.03);
  });
  return (
    <>
      <group ref={target}>
        <mesh>
          <sphereGeometry args={[0.05, 16, 12]} />
          <meshBasicMaterial color="#ffd166" transparent opacity={0.8} />
        </mesh>
        <mesh>
          <torusGeometry args={[0.15, 0.008, 8, 32]} />
          <meshBasicMaterial color="#ffd166" />
        </mesh>
      </group>
      <mesh ref={shadow}>
        <circleGeometry args={[armLength * 1.1, 32]} />
        <meshBasicMaterial color="#000000" transparent opacity={0.4} depthWrite={false} />
      </mesh>
    </>
  );
}

function Axes() {
  return (
    <>
      <axesHelper args={[1]} />
      <Label text="E (x)" color="#ff6b6b" position={[1.25, 0, 0]} />
      <Label text="N (y)" color="#51cf66" position={[0, 1.25, 0]} />
      <Label text="Up (z)" color="#4dabf7" position={[0, 0, 1.2]} />
    </>
  );
}

/** Keep the camera's orbit centred on the drone (moves camera and target together). */
function FollowCamera({ enabled }: { enabled: boolean }) {
  const controls = useThree((s) => s.controls) as unknown as { target: THREE.Vector3 } | null;
  const camera = useThree((s) => s.camera);
  const tmp = useMemo(() => new THREE.Vector3(), []);
  useFrame(() => {
    const s = sim.latest;
    if (!enabled || !controls || !s) return;
    enuToThree(s.pos_m, tmp);
    camera.position.add(tmp.clone().sub(controls.target));
    controls.target.copy(tmp);
  });
  return null;
}

/** The loaded mission course (if any), and a camera move to show all of it whenever a different course loads. */
function CourseLayer() {
  const { course } = useSimUi();
  return course ? <CourseView course={course} /> : null;
}

function FrameCourse() {
  const { course } = useSimUi();
  const controls = useThree((s) => s.controls) as unknown as { target: THREE.Vector3; update: () => void } | null;
  const camera = useThree((s) => s.camera);
  const invalidate = useThree((s) => s.invalidate);
  const id = course?.id ?? null;
  const last = useRef<string | null>(null);
  useEffect(() => {
    if (!controls || id === last.current) return;
    last.current = id;
    if (course) {
      const { cx, cy, span } = courseBounds(course);
      controls.target.set(cx, 1, -cy); // three.js coords: (east, up, -north)
      camera.position.set(cx, 1 + span * 0.75, -cy + span * 0.95); // from the south, looking north
    } else {
      controls.target.set(0, 0.5, 0);
      camera.position.set(2.5, 2.2, 3.5);
    }
    controls.update();
    invalidate();
  }, [id, course, controls, camera, invalidate]);
  return null;
}

export default function Scene({ armLength, follow }: { armLength: number; follow: boolean }) {
  return (
    <Canvas frameloop="demand" dpr={[1, 1.5]} camera={{ position: [2.5, 2.2, 3.5], fov: 50, near: 0.05, far: 500 }}>
      <color attach="background" args={['#0f1420']} />
      <ambientLight intensity={0.7} />
      <directionalLight position={[5, 10, 4]} intensity={1.2} />
      <Grid
        args={[60, 60]}
        cellSize={0.5}
        cellThickness={0.6}
        cellColor="#2c3648"
        sectionSize={5}
        sectionThickness={1.2}
        sectionColor="#4a5a78"
        fadeDistance={60}
        infiniteGrid
      />
      <group rotation={ENU_TO_THREE}>
        <Axes />
        <CourseLayer />
        <Trail />
        <Markers armLength={armLength} />
        <Drone armLength={armLength} />
      </group>
      <OrbitControls makeDefault target={[0, 0.5, 0]} maxPolarAngle={Math.PI / 2 - 0.02} />
      <FollowCamera enabled={follow} />
      <FrameCourse />
      <RedrawOnState />
    </Canvas>
  );
}
