// 🌳 The world around the drone: grass, sky, fog, trees on the horizon, the sun's shadow, and 🏔 terrain.
// Everything is drawn from code (canvas textures, simple shapes): no image or model downloads, so it works offline.
// Trees and far hills are SCENERY ONLY (outside every course, > 95 m away): the drone can't hit them.
// The terrain is real for the physics too (the backend's ground height), and is drawn here from the same grid.

import { useFrame, useThree } from '@react-three/fiber';
import { useEffect, useMemo, useRef } from 'react';
import * as THREE from 'three';
import { sim, useSimUi, type TerrainInfo } from './sim';

const SUN_DIR = new THREE.Vector3(0.45, 0.8, 0.35).normalize(); // three.js frame (y up): high, from the south-east

/** A small seeded random generator, so the trees stand in the same places every time. */
function rng(seed: number) {
  let s = seed >>> 0;
  return () => {
    s = (s * 1664525 + 1013904223) >>> 0;
    return s / 4294967296;
  };
}

/** Grass: blotches of greens and a few dry patches, drawn once on a canvas, tiled across the ground. */
function makeGrass(): THREE.CanvasTexture {
  const n = 256;
  const c = document.createElement('canvas');
  c.width = c.height = n;
  const g = c.getContext('2d')!;
  g.fillStyle = '#5b7f3a';
  g.fillRect(0, 0, n, n);
  const r = rng(7);
  const greens = ['#4f7432', '#668b42', '#58803a', '#728f45', '#4a6b2e', '#7d8f4a'];
  for (let i = 0; i < 2600; i++) {
    g.fillStyle = greens[Math.floor(r() * greens.length)];
    g.globalAlpha = 0.25 + r() * 0.35;
    const x = r() * n, y = r() * n, s = 1 + r() * 4, e = 0.4 + r(), a = r() * Math.PI;
    // draw wrapped copies so the tile repeats without seams
    for (const dx of [-n, 0, n]) for (const dy of [-n, 0, n]) {
      g.beginPath();
      g.ellipse(x + dx, y + dy, s, s * e, a, 0, Math.PI * 2);
      g.fill();
    }
  }
  g.globalAlpha = 1;
  const t = new THREE.CanvasTexture(c);
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.colorSpace = THREE.SRGBColorSpace;
  t.anisotropy = 8;
  return t;
}

/** Decoded terrain heights (rows south -> north), with a lookup that matches the backend's bilinear one. */
function useHeights(t: TerrainInfo | null) {
  return useMemo(() => {
    if (!t) return null;
    const bin = atob(t.heights_b64);
    const bytes = new Uint8Array(bin.length);
    for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
    const h = new Float32Array(bytes.buffer);
    const at = (x: number, y: number) => {
      const fx = Math.min(Math.max((x - t.x_min_m) / t.cell_m, 0), t.cols - 1.000001);
      const fy = Math.min(Math.max((y - t.y_min_m) / t.cell_m, 0), t.rows - 1.000001);
      const c = Math.floor(fx), r = Math.floor(fy), tx = fx - c, ty = fy - r;
      const v = (rr: number, cc: number) => h[rr * t.cols + cc];
      return (v(r, c) * (1 - tx) + v(r, c + 1) * tx) * (1 - ty) + (v(r + 1, c) * (1 - tx) + v(r + 1, c + 1) * tx) * ty;
    };
    return { h, at };
  }, [t]);
}

function Ground({ grass, below }: { grass: THREE.Texture; below: number }) {
  const tex = useMemo(() => {
    const t = grass.clone();
    t.repeat.set(1600 / 6, 1600 / 6); // one grass tile = 6 m
    t.needsUpdate = true;
    return t;
  }, [grass]);
  return (
    <mesh position={[0, 0, below]} receiveShadow userData={{ noShadow: true }}>
      <planeGeometry args={[1600, 1600]} />
      <meshStandardMaterial map={tex} roughness={1} />
    </mesh>
  );
}

/** The terrain surface. With a satellite picture: stitched from its tiles; without: grass, darker in the dips. */
function TerrainMesh({ t, grass }: { t: TerrainInfo; grass: THREE.Texture }) {
  const heights = useHeights(t)!;
  const invalidate = useThree((s) => s.invalidate);
  const geom = useMemo(() => {
    const g = new THREE.PlaneGeometry(t.width_m, t.depth_m, t.cols - 1, t.rows - 1);
    const pos = g.attributes.position as THREE.BufferAttribute;
    const colors = new Float32Array(pos.count * 3);
    const span = Math.max(t.max_m - t.min_m, 1);
    for (let i = 0; i < pos.count; i++) {
      const col = i % t.cols, rowTop = Math.floor(i / t.cols); // PlaneGeometry starts at the top (north) row
      const z = heights.h[(t.rows - 1 - rowTop) * t.cols + col];
      pos.setZ(i, z);
      const k = (z - t.min_m) / span; // low = darker, lusher; high = lighter, drier
      colors[i * 3] = 0.85 + 0.25 * k;
      colors[i * 3 + 1] = 0.9 + 0.1 * k;
      colors[i * 3 + 2] = 0.8 + 0.15 * k;
    }
    g.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    g.computeVertexNormals();
    return g;
  }, [t, heights]);
  useEffect(() => () => geom.dispose(), [geom]);

  // The skirt: the terrain's border pushed 900 m straight outwards at the same heights. Beyond the square the
  // physics uses the nearest border height too, so what you see is what the drone would hit.
  const skirt = useMemo(() => {
    const ring: [number, number, number][] = []; // border points, anticlockwise, in ENU relative to the centre
    const hw = t.width_m / 2, hd = t.depth_m / 2;
    const H = (r: number, c: number) => heights.h[r * t.cols + c];
    for (let c = 0; c < t.cols - 1; c++) ring.push([-hw + c * t.cell_m, -hd, H(0, c)]);
    for (let r = 0; r < t.rows - 1; r++) ring.push([hw, -hd + r * t.cell_m, H(r, t.cols - 1)]);
    for (let c = t.cols - 1; c > 0; c--) ring.push([-hw + c * t.cell_m, hd, H(t.rows - 1, c)]);
    for (let r = t.rows - 1; r > 0; r--) ring.push([-hw, -hd + r * t.cell_m, H(r, 0)]);
    const pos: number[] = [], uv: number[] = [], idx: number[] = [];
    const OUT = 900;
    ring.forEach(([x, y, z]) => {
      const ox = Math.sign(x) * (Math.abs(x) >= hw - 1e-6 ? OUT : 0), oy = Math.sign(y) * (Math.abs(y) >= hd - 1e-6 ? OUT : 0);
      pos.push(x, y, z, x + ox, y + oy, z);
      uv.push(x / 6, y / 6, (x + ox) / 6, (y + oy) / 6);
    });
    const n = ring.length;
    for (let i = 0; i < n; i++) {
      const a = 2 * i, b = 2 * ((i + 1) % n);
      idx.push(a, a + 1, b, b, a + 1, b + 1);
    }
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
    g.setAttribute('uv', new THREE.Float32BufferAttribute(uv, 2));
    g.setIndex(idx);
    g.computeVertexNormals();
    return g;
  }, [t, heights]);
  useEffect(() => () => skirt.dispose(), [skirt]);
  const skirtTex = useMemo(() => {
    const g = grass.clone();
    g.needsUpdate = true;
    return g;
  }, [grass]);

  const grassTex = useMemo(() => {
    const g = grass.clone();
    g.repeat.set(t.width_m / 6, t.depth_m / 6);
    g.needsUpdate = true;
    return g;
  }, [grass, t]);

  // the satellite picture: draw every tile onto one canvas covering exactly the terrain square
  const photo = useMemo(() => {
    if (!t.image) return null;
    const px = 1024;
    const c = document.createElement('canvas');
    c.width = c.height = px;
    const tex = new THREE.CanvasTexture(c);
    tex.colorSpace = THREE.SRGBColorSpace;
    tex.anisotropy = 8;
    const g = c.getContext('2d')!;
    g.fillStyle = '#5b7f3a';
    g.fillRect(0, 0, px, px);
    const sx = px / t.width_m, sy = px / t.depth_m;
    const yTop = t.y_min_m + t.depth_m;
    for (const tile of t.image.tiles) {
      const img = new Image();
      img.onload = () => {
        g.drawImage(img, (tile.x_min_m - t.x_min_m) * sx, (yTop - tile.y_max_m) * sy,
          (tile.x_max_m - tile.x_min_m) * sx, (tile.y_max_m - tile.y_min_m) * sy);
        tex.needsUpdate = true;
        invalidate(); // the view only redraws on demand
      };
      img.src = `/api/terrain/${encodeURIComponent(t.id)}/image/${tile.file}`;
    }
    return tex;
  }, [t, invalidate]);
  useEffect(() => () => photo?.dispose(), [photo]);

  const centre: [number, number, number] = [t.x_min_m + t.width_m / 2, t.y_min_m + t.depth_m / 2, 0];
  return (
    <group userData={{ noShadow: true }}>
      <mesh geometry={geom} position={centre} receiveShadow userData={{ noShadow: true }}>
        {photo
          ? <meshStandardMaterial map={photo} roughness={1} />
          : <meshStandardMaterial map={grassTex} vertexColors roughness={1} />}
      </mesh>
      <mesh geometry={skirt} position={centre} receiveShadow userData={{ noShadow: true }}>
        <meshStandardMaterial map={skirtTex} roughness={1} side={THREE.DoubleSide} />
      </mesh>
    </group>
  );
}

/** Trees and bushes far away (scenery only), standing on the terrain when there is one. */
function Trees({ ground, avoid }: { ground: (x: number, y: number) => number; avoid: TerrainInfo | null }) {
  const trunks = useRef<THREE.InstancedMesh>(null!);
  const crowns = useRef<THREE.InstancedMesh>(null!);
  const COUNT = 420;
  useEffect(() => {
    const r = rng(42);
    const m = new THREE.Matrix4(), q = new THREE.Quaternion(), up = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI / 2);
    const col = new THREE.Color();
    for (let i = 0; i < COUNT; i++) {
      // clumps: pick a clump centre, then scatter around it
      const a = r() * Math.PI * 2, d = 100 + Math.pow(r(), 0.7) * 300;
      const x = Math.cos(a) * d + (r() - 0.5) * 18, y = Math.sin(a) * d + (r() - 0.5) * 18;
      const s = 0.7 + r() * 0.9, z = ground(x, y);
      // on a real place, no made-up trees on the real ground (hide them far below)
      const inside = !!avoid && x > avoid.x_min_m - 5 && x < avoid.x_min_m + avoid.width_m + 5 && y > avoid.y_min_m - 5 && y < avoid.y_min_m + avoid.depth_m + 5;
      if (inside) {
        m.makeScale(0, 0, 0);
        trunks.current.setMatrixAt(i, m);
        crowns.current.setMatrixAt(i, m);
        continue;
      }
      q.copy(up); // cones point along +y in three.js; turn them to +z (ENU up)
      m.compose(new THREE.Vector3(x, y, z + 1.5 * s), q, new THREE.Vector3(s, s, s));
      trunks.current.setMatrixAt(i, m);
      const tall = 1 + r() * 0.6;
      m.compose(new THREE.Vector3(x, y, z + (3 + 3.5 * tall) * s), q, new THREE.Vector3(s * (0.9 + r() * 0.4), s * tall, s * (0.9 + r() * 0.4)));
      crowns.current.setMatrixAt(i, m);
      col.setHSL(0.27 + r() * 0.08, 0.35 + r() * 0.2, 0.18 + r() * 0.12);
      crowns.current.setColorAt(i, col);
    }
    trunks.current.instanceMatrix.needsUpdate = true;
    crowns.current.instanceMatrix.needsUpdate = true;
    if (crowns.current.instanceColor) crowns.current.instanceColor.needsUpdate = true;
  }, [ground, avoid]);
  return (
    <group userData={{ noShadow: true }}>
      <instancedMesh ref={trunks} args={[undefined, undefined, COUNT]} userData={{ noShadow: true }}>
        <cylinderGeometry args={[0.18, 0.28, 3, 6]} />
        <meshStandardMaterial color="#5a4330" roughness={1} />
      </instancedMesh>
      <instancedMesh ref={crowns} args={[undefined, undefined, COUNT]} userData={{ noShadow: true }}>
        <coneGeometry args={[2.2, 7, 7]} />
        <meshStandardMaterial roughness={0.95} />
      </instancedMesh>
    </group>
  );
}

/** Low rolling hills on the horizon, faded by the fog. */
function FarHills() {
  const hills = useMemo(() => {
    const r = rng(3);
    return Array.from({ length: 18 }, (_, i) => {
      const a = (i / 18) * Math.PI * 2 + r() * 0.3, d = 620 + r() * 180;
      return { x: Math.cos(a) * d, y: Math.sin(a) * d, w: 120 + r() * 160, h: 25 + r() * 45 };
    });
  }, []);
  return (
    <group userData={{ noShadow: true }}>
      {hills.map((h, i) => (
        <mesh key={i} position={[h.x, h.y, -2]} scale={[h.w, h.w * 0.7, h.h]} userData={{ noShadow: true }}>
          <sphereGeometry args={[1, 16, 8, 0, Math.PI * 2, 0, Math.PI / 2]} />
          <meshStandardMaterial color="#5e7a52" roughness={1} />
        </mesh>
      ))}
    </group>
  );
}

/** A gradient sky dome (drawn from code: no textures). */
function SkyDome() {
  const mat = useMemo(() => new THREE.ShaderMaterial({
    side: THREE.BackSide, depthWrite: false, fog: false,
    uniforms: { top: { value: new THREE.Color('#3f7fc6') }, horizon: { value: new THREE.Color('#cfe3f2') }, sun: { value: SUN_DIR } },
    vertexShader: 'varying vec3 vDir; void main(){ vDir = normalize(position); gl_Position = projectionMatrix * modelViewMatrix * vec4(position,1.0); }',
    fragmentShader: `uniform vec3 top; uniform vec3 horizon; uniform vec3 sun; varying vec3 vDir;
      void main(){ float h = max(vDir.y, 0.0); vec3 c = mix(horizon, top, pow(h, 0.55));
        float s = max(dot(normalize(vDir), sun), 0.0); c += vec3(1.0,0.95,0.8) * (pow(s, 400.0) * 1.5 + pow(s, 12.0) * 0.15);
        if (vDir.y < 0.0) c = horizon; gl_FragColor = vec4(c, 1.0); }`,
  }), []);
  const ref = useRef<THREE.Mesh>(null!);
  useFrame(({ camera }) => ref.current.position.copy(camera.position)); // always around the camera
  return (
    <mesh ref={ref} material={mat} renderOrder={-1} frustumCulled={false}>
      <sphereGeometry args={[1500, 32, 16]} />
    </mesh>
  );
}

/** The sun: a light whose shadow box follows the drone, so its shadow is sharp wherever it flies. */
function Sun() {
  const light = useRef<THREE.DirectionalLight>(null!);
  const { scene } = useThree();
  useEffect(() => {
    scene.add(light.current.target);
    const cam = light.current.shadow.camera;
    cam.left = cam.bottom = -30;
    cam.right = cam.top = 30;
    cam.near = 1;
    cam.far = 200;
    light.current.shadow.mapSize.set(2048, 2048);
    light.current.shadow.bias = -0.0005;
    light.current.shadow.normalBias = 0.02;
    return () => void scene.remove(light.current?.target);
  }, [scene]);
  const tick = useRef(0);
  useFrame(() => {
    const p = sim.latest?.pos_m ?? [0, 0, 0];
    const t = new THREE.Vector3(p[0], p[2], -p[1]); // ENU -> three.js
    light.current.target.position.copy(t);
    light.current.position.copy(t).addScaledVector(SUN_DIR, 80);
    // every half second: make everything cast / receive shadows (new obstacles, wingmen, ...), except marked scenery
    if (tick.current++ % 30 === 0)
      scene.traverse((o) => {
        if ((o as THREE.Mesh).isMesh && !o.userData.noShadow && !o.parent?.userData.noShadow) {
          const m = (o as THREE.Mesh).material as THREE.Material;
          const lit = !!m && 'roughness' in m && !m.transparent; // only solid lit materials take part
          o.castShadow = lit;
          o.receiveShadow = lit;
        }
      });
  });
  return <directionalLight ref={light} castShadow intensity={2.4} color="#fff4e0" />;
}

/** The realistic world (sky, fog, ground, trees, sun) for scenery 'real'; terrain is drawn in both modes. */
export default function World() {
  const { scenery, terrain } = useSimUi();
  const grass = useMemo(makeGrass, []);
  useEffect(() => () => grass.dispose(), [grass]);
  const heights = useHeights(terrain);
  const ground = useMemo(() => heights?.at ?? (() => 0), [heights]);
  const real = scenery === 'real';
  return (
    <>
      {real && (
        <>
          <fog attach="fog" args={['#cfe3f2', 120, 900]} />
          <hemisphereLight args={['#cfe3f2', '#4a5a30', 0.9]} />
          <Sun />
          <SkyDome />
        </>
      )}
      <group rotation={[-Math.PI / 2, 0, 0]}>
        {terrain && <TerrainMesh t={terrain} grass={grass} />}
        {real && !terrain && <Ground grass={grass} below={-0.01} />}
        {real && <Trees ground={ground} avoid={terrain?.image ? terrain : null} />}
        {real && <FarHills />}
      </group>
    </>
  );
}
