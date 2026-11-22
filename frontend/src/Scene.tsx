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
import RaceView, { raceBounds } from './RaceView';
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

/** Where a drone is and how it's turned, for drawing it. */
interface Pose {
  pos_m: Vec3;
  quat: [number, number, number, number];
  motor_pct: number[];
  crashed?: boolean;
}

const WINGMAN_COLORS = ['#ffd166', '#06d6a0', '#c77dff', '#f4a261'];

/** One quadcopter. `pick` says where it is right now (null = don't draw it). Your drone is light grey;
 *  wingmen get their own body colour and a number above them. */
function Drone({ armLength, pick, body = '#d8dee9', label }: { armLength: number; pick: () => Pose | null | undefined; body?: string; label?: string }) {
  const outer = useRef<THREE.Group>(null!); // follows the position only (so the label stays upright)
  const group = useRef<THREE.Group>(null!); // also turns with the drone
  const bodyMat = useRef<THREE.MeshStandardMaterial>(null!);
  const rotorMats = useRef<(THREE.MeshStandardMaterial | null)[]>([]);
  const a = armLength / Math.SQRT2;
  const rotorRadius = 0.45 * armLength;

  useFrame(() => {
    const s = pick();
    outer.current.visible = !!s;
    if (!s) return;
    outer.current.position.set(s.pos_m[0], s.pos_m[1], s.pos_m[2]);
    const [w, x, y, z] = s.quat;
    group.current.quaternion.set(x, y, z, w); // three.js order is (x, y, z, w)
    bodyMat.current.color.set(s.crashed ? '#4a5568' : body);
    s.motor_pct.forEach((pct, i) => {
      const m = rotorMats.current[i];
      if (m) m.emissiveIntensity = 0.05 + (pct / 100) * 1.5; // brighter rotor = harder working motor
    });
  });

  return (
    <group ref={outer}>
    {label && <Label text={label} color={body} position={[0, 0, 0.35]} size={1.2} />}
    <group ref={group}>
      <mesh>
        <boxGeometry args={[0.6 * armLength, 0.45 * armLength, 0.2 * armLength]} />
        <meshStandardMaterial ref={bodyMat} color={body} />
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
    </group>
  );
}

/** Wingmen (multi-drone formation): up to 4 extra drones, drawn from the formation part of each state. */
function Wingmen({ armLength }: { armLength: number }) {
  return (
    <>
      {WINGMAN_COLORS.map((c, i) => (
        <Drone key={i} armLength={armLength} body={c} label={String(i + 1)} pick={() => sim.latest?.formation?.wingmen[i]} />
      ))}
    </>
  );
}

/** The real ArduPilot (SITL) drone, orange, on its own pad 4 m south of yours. */
const ARDUPILOT_COLOR = '#ff8c42';

function ArduPilotDrone({ armLength }: { armLength: number }) {
  return (
    <Drone armLength={armLength} body={ARDUPILOT_COLOR} label="ArduPilot" pick={() => {
      const a = sim.latest?.sitl;
      return a?.pos_m && a.quat ? { pos_m: a.pos_m, quat: a.quat, motor_pct: a.motor_pct ?? [0, 0, 0, 0] } : null;
    }} />
  );
}

/** Where the ArduPilot drone REALLY is (from the simulator), shown only when it differs from what its EKF
 *  believes (the orange drone): e.g. GPS spoofing. */
function ArduPilotTruth() {
  const m = useRef<THREE.Mesh>(null!);
  useFrame(() => {
    const a = sim.latest?.sitl;
    const t = a?.truth_m, p = a?.pos_m;
    const show = !!t && !!p && a?.state !== 'off' && Math.hypot(t[0] - p[0], t[1] - p[1]) > 0.5;
    m.current.visible = show;
    if (show) m.current.position.set(t![0], t![1], t![2]);
  });
  return (
    <mesh ref={m}>
      <sphereGeometry args={[0.35, 16, 12]} />
      <meshBasicMaterial color="#ffffff" wireframe />
    </mesh>
  );
}

function ArduPilotPad() {
  const pad = useRef<THREE.Mesh>(null!);
  useFrame(() => {
    const a = sim.latest?.sitl;
    pad.current.visible = !!a && a.state !== 'off';
    if (a) pad.current.position.set(a.pad_m[0], a.pad_m[1], 0.005);
  });
  return (
    <mesh ref={pad}>
      <circleGeometry args={[0.6, 32]} />
      <meshBasicMaterial color={ARDUPILOT_COLOR} transparent opacity={0.35} />
    </mesh>
  );
}

const pickOwnTrail = () => (sim.latest ? { t_s: sim.latest.t_s, pos_m: sim.latest.pos_m } : null);
const pickArduPilotTrail = () => {
  const a = sim.latest?.sitl;
  return a?.pos_m && a.t_s !== undefined && a.state !== 'off' ? { t_s: a.t_s, pos_m: a.pos_m } : null;
};

/** The path flown so far: a line that grows as the drone moves (last ~1500 points). */
function Trail({ pick = pickOwnTrail, color = '#4cc9f0' }: { pick?: () => { t_s: number; pos_m: Vec3 } | null; color?: string }) {
  const N = 1500;
  const { line, positions } = useMemo(() => {
    const positions = new Float32Array(N * 3);
    const geom = new THREE.BufferGeometry();
    geom.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geom.setDrawRange(0, 0);
    const line = new THREE.Line(geom, new THREE.LineBasicMaterial({ color }));
    line.frustumCulled = false;
    return { line, positions };
  }, [color]);
  const count = useRef(0);
  const lastT = useRef(-1);

  useFrame(() => {
    const s = pick();
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

/** A see-through outline at the ESTIMATED position and tilt (only while sensor noise is on).
 *  The gap between it and the real drone is the estimation error. */
function GhostDrone({ armLength }: { armLength: number }) {
  const group = useRef<THREE.Group>(null!);
  const a = armLength / Math.SQRT2;
  useFrame(() => {
    const s = sim.latest;
    group.current.visible = !!s?.est_pos_m;
    if (!s?.est_pos_m || !s.est_quat) return;
    group.current.position.set(...s.est_pos_m);
    const [w, x, y, z] = s.est_quat;
    group.current.quaternion.set(x, y, z, w);
  });
  return (
    <group ref={group} visible={false}>
      {MOTORS.map((m, i) => (
        <mesh key={i} position={[m.x * a, m.y * a, 0.1 * armLength]}>{/* a torus already lies flat (in x-y) */}
          <torusGeometry args={[0.45 * armLength, 0.006, 6, 28]} />
          <meshBasicMaterial color="#ff6bd6" transparent opacity={0.8} />
        </mesh>
      ))}
      <mesh rotation={[0, 0, Math.PI / 4]}>
        <boxGeometry args={[2 * armLength, 0.012, 0.012]} />
        <meshBasicMaterial color="#ff6bd6" transparent opacity={0.8} />
      </mesh>
      <mesh rotation={[0, 0, -Math.PI / 4]}>
        <boxGeometry args={[2 * armLength, 0.012, 0.012]} />
        <meshBasicMaterial color="#ff6bd6" transparent opacity={0.8} />
      </mesh>
    </group>
  );
}

/** An arrow above the drone showing the wind right now: points where the air is going, longer = stronger. */
function WindArrow() {
  const arrow = useMemo(() => new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 1, '#e0f7ff', 0.18, 0.1), []);
  const dir = useMemo(() => new THREE.Vector3(), []);
  useEffect(() => () => arrow.dispose(), [arrow]);
  useFrame(() => {
    const s = sim.latest;
    const w = s?.wind_m_s;
    const speed = w ? Math.hypot(w[0], w[1], w[2]) : 0;
    arrow.visible = !!s && speed > 0.05;
    if (!s || !w || !arrow.visible) return;
    arrow.position.set(s.pos_m[0], s.pos_m[1], s.pos_m[2] + 0.5);
    arrow.setDirection(dir.set(w[0], w[1], w[2]).normalize());
    arrow.setLength(0.25 + 0.12 * speed, 0.18, 0.1); // 6 m/s is about 1 m long
  });
  return <primitive object={arrow} />;
}

/** The loaded mission course (if any), and a camera move to show all of it whenever a different course loads. */
function CourseLayer() {
  const { course, race } = useSimUi();
  return <>{course && <CourseView course={course} />}{race && <RaceView race={race} />}</>;
}

function FrameCourse() {
  const { course, race } = useSimUi();
  const controls = useThree((s) => s.controls) as unknown as { target: THREE.Vector3; update: () => void } | null;
  const camera = useThree((s) => s.camera);
  const invalidate = useThree((s) => s.invalidate);
  const id = course ? `course:${course.id}` : race ? `race:${race.id}` : null;
  const last = useRef<string | null>(null);
  useEffect(() => {
    if (!controls || id === last.current) return;
    last.current = id;
    if (course || race) {
      const { cx, cy, span } = course ? courseBounds(course) : raceBounds(race!);
      controls.target.set(cx, 1, -cy); // three.js coords: (east, up, -north)
      camera.position.set(cx, 1 + span * 0.75, -cy + span * 0.95); // from the south, looking north
    } else {
      controls.target.set(0, 0.5, 0);
      camera.position.set(2.5, 2.2, 3.5);
    }
    controls.update();
    invalidate();
  }, [id, course, race, controls, camera, invalidate]);
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
        <WindArrow />
        <GhostDrone armLength={armLength} />
        <Trail />
        <Markers armLength={armLength} />
        <Drone armLength={armLength} pick={() => sim.latest} />
        <Wingmen armLength={armLength} />
        <ArduPilotPad />
        <Trail pick={pickArduPilotTrail} color={ARDUPILOT_COLOR} />
        <ArduPilotDrone armLength={armLength} />
        <ArduPilotTruth />
      </group>
      <OrbitControls makeDefault target={[0, 0.5, 0]} maxPolarAngle={Math.PI / 2 - 0.02} />
      <FollowCamera enabled={follow} />
      <FrameCourse />
      <RedrawOnState />
    </Canvas>
  );
}
