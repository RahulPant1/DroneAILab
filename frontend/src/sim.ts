// Connection to the backend simulator (WebSocket) plus small REST helpers.
// Everything arrives in SI units and radians; components convert to degrees for display.

import { useEffect, useState, useSyncExternalStore } from 'react';

export type Vec3 = [number, number, number];
export type Quat = [number, number, number, number]; // [w, x, y, z], body -> world (ENU)

/** The EKF's view of itself (backend/sim/ekf.py health()). */
export interface EkfHealth {
  sigma: { pos_m: Vec3; vel_m_s: Vec3; att_rad: Vec3; gyro_bias_rad_s: Vec3 }; // 1 standard deviation
  gyro_bias_rad_s: Vec3; // the bias it has learnt
  test_ratio: Record<'gps_pos' | 'gps_vel' | 'baro' | 'compass', number>; // > 1 = reading rejected
  rejected: Record<string, number>;
  accepted: Record<string, number>;
  gps_resets: number;
  vel_resets: number;
}

export interface SimState {
  type: 'state';
  t_s: number;
  mode: 'idle' | 'takeoff' | 'hover' | 'land' | 'mission' | 'precland';
  time_scale: number; // ⏩ 1 = real time; 2, 4, 8 = faster
  paused: boolean;
  armed: boolean;
  on_ground: boolean;
  pos_m: Vec3;
  vel_m_s: Vec3;
  quat: Quat;
  euler_rad: Vec3; // roll, pitch, yaw
  rate_rad_s: Vec3;
  motor_pct: [number, number, number, number]; // FR, RL, FL, RR
  thrust_total_n: number;
  wind_m_s: Vec3; // the air's velocity right now (steady wind + gusts), world frame
  est_pos_m: Vec3 | null; // where the drone BELIEVES it is (sensor noise on), else null
  est_quat: Quat | null;
  gps_settle_left_s: number; // > 0: GPS still settling on the ground, take-off not allowed yet
  gps_outage: boolean; // no GPS fixes (only with sensor noise on)
  gps_lost_s: number; // how long the outage has lasted
  gps_glitch: boolean; // GPS fixes jump 5 m east (a bad fix)
  ekf: EkfHealth | null; // flying on the Kalman filter: its own uncertainty and how it judged each sensor
  failsafe: boolean; // GPS-loss failsafe active: holding level + altitude, NOT position
  failsafe_drag_n_s_m: number; // extra air drag while in failsafe (0 = real physics: it keeps coasting)
  failsafe_tilt_deg: number; // how far a full W/A/S/D press tilts the drone in failsafe
  setpoint: { x_m: number; y_m: number; alt_m: number; yaw_rad: number };
  message: string;
  mission: MissionStatus | null; // set while the mission autopilot is flying
  race: RaceStatus | null; // set while a rings-race track is loaded
  agile: AgileStatus | null; // ✈ race autopilot flying (null = off)
  explore: ExploreStatus | null; // 🗺 autonomous exploration (kept after it ends, to show the map)
  formation: FormationStatus | null; // set while wingmen are flying with you
  sitl: SitlStatus | null; // the 🛰 ArduPilot panel (real ArduCopter SITL), null until it's used
  battery: BatteryStatus | null; // null = unlimited power (battery off)
  lidar: LidarStatus | null; // 📡 the lidar's latest scan (+ its map when it changed), null when off
  survey: SurveyStatus | null; // 🌾 a flight plan with a survey camera is flying
  camera: CameraStatus | null; // 📷 down camera (on demand, or while precision landing)
  no_fly: string | null; // name of the no-fly zone the drone is in right now
  cropmap_busy: boolean; // 🌱 stitching the last survey's photos
  cropmaps?: CropMap[]; // only in the frame where new maps became ready
}

/** 🌱 A stitched crop-health (NDVI) map of one field, and what was found on it. */
export interface CropMap {
  id: string;
  field: string;
  cell_m: number;
  bounds: [number, number, number, number]; // x0, x1, y0, y1 (m)
  w: number;
  h: number;
  image: string; // NDVI × 255 per cell, north up (0 = no photo)
  mapped_pct: number;
  field_ndvi: number | null;
  stressed_m2: number;
  photos: number;
  geotag_error_m: number | null;
  false_alarms: number;
  findings: { x_m: number; y_m: number; area_m2: number; mean_ndvi: number; min_ndvi: number; where: string; box: [number, number, number, number] }[];
  truth: { x_m: number; y_m: number; cause: string; area_m2: number; found: boolean; position_error_m: number | null }[];
}

/** A photo: where it was taken (x, y), from what height, and the survey line's direction (rad). */
export type Photo = [number, number, number, number];

export interface SurveyStatus {
  photos_taken: number;
  on_line: boolean; // the camera is triggering (flying a survey line)
  photos?: Photo[]; // only sent when a new photo was taken
}

/** The down camera's picture: grey-scale, w × h, base64 bytes (one per pixel); null = keep the last one. */
export interface CameraStatus {
  landing: boolean;
  image: string | null;
  w: number;
  h: number;
  detection: { u: number; v: number; pixels: number; box: [number, number, number, number] } | null;
  mode?: 'vision' | 'gps';
  state?: string;
  sightings?: number;
  seen?: boolean;
}

export interface SiteBox { name: string; x_min: number; x_max: number; y_min: number; y_max: number; height_m: number; why?: string }
/** 🌾 The farm map shared by surveys, the mission planner and precision landing. */
export interface Site {
  title: string;
  fields: { name: string; polygon: [number, number][]; crop: string }[];
  buildings: SiteBox[];
  no_fly: SiteBox[];
  pads: { name: string; x_m: number; y_m: number; size_m: number }[];
  places: { name: string; x_m: number; y_m: number }[];
  geofence_radius_m: number;
  ceiling_m: number;
}

export interface PlanTask { type: 'survey' | 'goto' | 'hover' | 'land'; field?: string; place?: string; seconds?: number; precision?: boolean }
export interface PlanSpec { tasks: PlanTask[]; altitude_m: number; speed_m_s: number; side_overlap_pct: number; front_overlap_pct?: number }
export interface PlanCheck { id: string; label: string; status: 'pass' | 'warn' | 'fail'; detail: string }
export interface SurveyInfo {
  field: string;
  polygon: [number, number][];
  lines: [[number, number], [number, number]][];
  line_spacing_m: number;
  photo_spacing_m: number;
  footprint_m: [number, number];
  gsd_cm: number;
  photos_estimate: number;
}
/** A compiled flight plan: route + safety checks (from the server; it re-checks before flying). */
export interface FlightPlan {
  ok: boolean;
  spec: PlanSpec;
  waypoints: [number, number][];
  labels: string[];
  route: [number, number][];
  land: { place: string; x_m: number; y_m: number; precision: boolean };
  surveys: SurveyInfo[];
  checks: PlanCheck[];
  estimate: { distance_m: number; time_s: number; energy_wh: number; battery_used_pct: number; battery_left_pct: number; photos: number };
}
export interface AiPlanReply {
  ok: boolean;
  error?: string;
  raw?: string;
  spec?: PlanSpec;
  plan?: FlightPlan;
  ai?: { understood: string; assumptions: string[]; not_on_map: string[]; left_out: string[] };
  unverified_numbers?: string[];
  model?: string;
  duration_s?: number;
}

/** A 360° 2D lidar scan at the drone's height, and the occupancy map it has built. */
export interface LidarStatus {
  origin: Vec3; // where the lidar really was when it scanned
  ranges: number[]; // distance per beam; beam i points at angle i * 360/len (0 = east, anticlockwise)
  range_m: number; // = no hit
  map_version: number;
  cells: number;
  res_m: number;
  map?: [number, number][]; // only sent when the map changed
  replans?: number | null;
}

/** LiPo battery while flying (🔋). Voltage sags under load; low -> fly home, critical -> land, empty -> motors stop. */
export interface BatteryStatus {
  voltage_v: number;
  cell_v: number;
  current_a: number;
  power_w: number;
  soc_pct: number;
  used_mah: number;
  capacity_mah: number;
  cells: number;
  time_left_s: number | null; // to the low-battery point, at the current draw
  status: 'ok' | 'low' | 'critical' | 'empty';
  drain_speedup: number;
  failsafe: null | 'rtl' | 'land' | 'empty';
}

/** Fault injection on real ArduPilot: what we broke, what should happen, what did. */
export interface FaultInfo {
  id: string;
  title: string;
  injected: string;
  expected: string;
  kind: 'safety' | 'limitation';
  frame: string;
}
export interface FaultResult extends Omit<FaultInfo, 'id'> {
  id: string;
  observed: string;
  verdict: 'PASS' | 'FAIL' | 'CONFIRMED' | 'NOT REPRODUCED' | 'ERROR' | 'not run';
  metrics: Record<string, number | boolean | null>;
  events: [number, string][]; // (seconds after the fault, what happened)
  wall_s: number;
  log: string | null; // the flight's ArduPilot log id, for the 🩺 Logs tab
}

/** One 'fly 10 m north' step, for the ArduPilot vs DroneLab comparison. */
export interface StepRun {
  t_s: number[];
  north_m: number[];
  rise_s: number | null; // 10% -> 90% of the distance
  overshoot_m: number;
  settle_s: number | null; // stays within 0.2 m from here on (null = not within the window)
  peak_speed_m_s: number | null;
  max_gap_s?: number; // largest hole in the recording (ArduPilot only); > 0.3 s = numbers unreliable
}

/** Real ArduCopter flight code running in SITL, shown next to your drone. Positions are ENU, already
 *  shifted onto ArduPilot's own pad (pad_m); local_m is relative to that pad. */
export interface SitlStatus {
  state: 'off' | 'starting' | 'waiting_gps' | 'ready' | 'error';
  message: string;
  action: string | null; // what it's doing right now ("take-off", "mission", ...)
  log: string[]; // the autopilot's own recent messages
  pad_m: [number, number];
  speedup: number;
  compare: { distance_m: number; ardupilot: StepRun; dronelab: StepRun } | null;
  fault_result: FaultResult | null; // the last fault-injection run
  plan_result: { running: boolean; coverage_pct: number | null; photos_taken?: number; photos: Photo[]; flight_time_s?: number; landing_error_m?: number; land_place?: string } | null;
  truth_m?: Vec3; // where the simulator says it REALLY is (pos_m is what ArduPilot's EKF believes)
  mode?: string; // ArduCopter flight mode: STABILIZE, GUIDED, AUTO, RTL, LAND...
  armed?: boolean;
  gps_fix?: number; // 3 = 3D fix
  sats?: number;
  ekf_ok?: boolean;
  t_s?: number;
  pos_m?: Vec3;
  local_m?: Vec3;
  vel_m_s?: Vec3;
  quat?: Quat;
  euler_rad?: Vec3;
  heading_deg?: number; // compass: 0 = north, clockwise
  motor_pct?: number[];
}

/** Multi-drone formation: wingmen that follow your drone. */
export interface FormationStatus {
  count: number;
  shape: 'v' | 'line' | 'column';
  spacing_m: number;
  avoid: boolean;
  min_gap_m: number | null; // closest centre-to-centre distance between any two drones in the air (< 0.6 = touching)
  wingmen: { pos_m: Vec3; quat: Quat; motor_pct: number[]; crashed: boolean; breaking: boolean }[];
}

/** One upright hoop of the rings race (fly through it toward heading_deg: 0 = east, 90 = north). */
export interface Ring {
  x_m: number;
  y_m: number;
  z_m: number;
  heading_deg: number;
  radius_m: number;
}

/** A rings-race track loaded into the live sim: drawn in the 3D view. */
export interface RaceTrack {
  id: string;
  title: string;
  tip: string;
  start: [number, number];
  rings: Ring[];
  autopilot?: { method: string; path: [number, number, number][] }; // ✈ the last planned trajectory, drawn in 3D
}

/** ✈ Race autopilot, in every state frame while it flies. */
/** 🗺 Autonomous exploration (backend/autonomy/explore.py). */
export interface ExploreMap {
  x0: number; // world position of cell (0, 0)'s corner
  y0: number;
  res: number; // cell size (m)
  nx: number;
  ny: number;
  cells: string; // base64 bytes, ny rows of nx: 0 unknown, 1 free, 2 wall
}
export interface ExploreStatus {
  world: string;
  local: string;
  localisation: string;
  movers: boolean;
  phase: 'takeoff' | 'explore' | 'done' | 'crashed' | 'stopped';
  coverage: number; // 0..1 of the real floor
  est_err_m: number; // estimate vs truth (only the simulator knows)
  scan_matches: number;
  brakes: number;
  goals: number;
  path: [number, number][];
  goal: [number, number] | null;
  people: [number, number, number][]; // where people really are: x, y, size
  tracked: [number, number, number, number][]; // what the drone thinks is moving: x, y, vx, vy
  map?: ExploreMap; // only every 0.5 s
}

export interface AgileStatus {
  method: string; // waypoints | pid_ff | geometric
  phase: 'climb' | 'fly';
  t_s: number | null; // time into the lap
  duration_s: number | null; // the trajectory's length (null for waypoints)
  error_m: number | null; // distance from where the plan says to be now
  rings_passed: number;
}

/** Live race numbers, in every state frame. */
export interface RaceStatus {
  next_ring: number; // index of the ring to fly through next (= total when finished)
  total: number;
  time_s: number;
  running: boolean;
  finished: boolean;
  new_best: boolean;
  best_s: number | null;
}

export interface MissionStatus {
  planner: string;
  phase: 'gps' | 'takeoff' | 'cruise' | 'land' | 'landed' | 'crashed' | 'stuck';
  next_wp: number;
  waypoints_total: number;
}

export interface CourseBox {
  x_min: number;
  x_max: number;
  y_min: number;
  y_max: number;
  height_m: number;
}

/** A mission course loaded into the live sim: drawn in the 3D view. */
export interface Course {
  id: string;
  title: string;
  waypoints: [number, number][];
  obstacles: CourseBox[];
  cruise_alt_m: number;
  safety_zone_m: number; // drone radius + safety margin: A* keeps the drone's centre outside this
  route: [number, number][] | null; // planned route of the current mission (null = none yet)
  planner: string | null;
  site?: Site; // 🌾 the farm (fields, no-fly zones, pads...)
  plan?: FlightPlan | null; // the flight plan being flown on the farm
  footprint_per_m?: [number, number]; // photo footprint (across, along) per metre of height
  explore?: boolean; // 🗺 an exploration world: walls drawn see-through, no keep-out lines
}

export interface Derived {
  weight_n: number;
  thrust_to_weight: number;
  hover_throttle_pct: number;
  inertia_kg_m2: Vec3;
}

export interface DroneConfig {
  name: string;
  mass_kg: number;
  arm_length_m: number;
  motor_max_thrust_n: number;
  motor_max_speed_rad_s: number;
  motor_time_constant_s: number;
  motor_torque_per_thrust_m: number;
  drag_linear_n_s_m: number;
  drag_quadratic_n_s2_m2: number;
  angular_drag_nm_s_rad: number;
  gains: Record<string, number>;
  derived?: Derived;
}

export interface LogEntry {
  kind: 'info' | 'error';
  text: string;
  at: number;
}

type Status = 'connecting' | 'open' | 'closed';
type Listener = () => void;

class SimConnection {
  latest: SimState | null = null;
  config: DroneConfig | null = null;
  course: Course | null = null;
  race: RaceTrack | null = null;
  cropmaps: CropMap[] = []; // 🌱 from the last survey flown live
  voice: VoiceReply | null = null; // 🎙 the last voice / typed command's answer (seq goes up with each)
  voiceSeq = 0;
  status: Status = 'connecting';
  log: LogEntry[] = [];
  version = 0; // bumps on config / status / log changes (not on every state frame)

  private ws: WebSocket | null = null;
  private stateListeners = new Set<Listener>();
  private uiListeners = new Set<Listener>();

  connect(): void {
    const proto = location.protocol === 'https:' ? 'wss' : 'ws';
    const ws = new WebSocket(`${proto}://${location.host}/ws/sim`);
    this.ws = ws;
    this.setStatus('connecting');
    ws.onopen = () => this.setStatus('open');
    ws.onclose = () => {
      this.setStatus('closed');
      window.setTimeout(() => this.connect(), 2000); // backend restarted? try again
    };
    ws.onmessage = (event) => {
      const msg = JSON.parse(event.data);
      if (msg.type === 'state') {
        this.latest = msg as SimState;
        if (msg.cropmaps) {
          this.cropmaps = msg.cropmaps as CropMap[];
          this.bump();
        }
        this.stateListeners.forEach((fn) => fn());
        return;
      }
      if (msg.type === 'voice') {
        this.voice = msg as VoiceReply;
        this.voiceSeq += 1;
        this.addLog('info', `🎙 “${msg.heard}” → ${msg.say}`);
        this.bump();
        return;
      }
      if (msg.type === 'config') this.config = msg.config as DroneConfig;
      if (msg.course !== undefined) this.course = msg.course as Course | null;
      if (msg.race !== undefined) this.race = msg.race as RaceTrack | null;
      if (msg.message) this.addLog(msg.type === 'error' ? 'error' : 'info', msg.message);
      this.bump();
    };
  }

  send(cmd: string, args: Record<string, unknown> = {}): void {
    if (this.ws?.readyState === WebSocket.OPEN) this.ws.send(JSON.stringify({ cmd, ...args }));
  }

  subscribeState(fn: Listener): () => void {
    this.stateListeners.add(fn);
    return () => this.stateListeners.delete(fn);
  }

  subscribeUi = (fn: Listener): (() => void) => {
    this.uiListeners.add(fn);
    return () => this.uiListeners.delete(fn);
  };

  private addLog(kind: LogEntry['kind'], text: string): void {
    this.log = [...this.log.slice(-19), { kind, text, at: Date.now() }];
  }

  private setStatus(status: Status): void {
    this.status = status;
    this.bump();
  }

  private bump(): void {
    this.version++;
    this.uiListeners.forEach((fn) => fn());
  }
}

export const sim = new SimConnection();

/** Re-render when config, connection status or the message log change. */
export function useSimUi(): SimConnection {
  useSyncExternalStore(sim.subscribeUi, () => sim.version);
  return sim;
}

/** The latest sim state, refreshed at most `hz` times per second (cheap for text panels). */
export function useThrottledState(hz = 10): SimState | null {
  const [state, setState] = useState<SimState | null>(sim.latest);
  useEffect(() => {
    const id = window.setInterval(() => setState(sim.latest), 1000 / hz);
    return () => window.clearInterval(id);
  }, [hz]);
  return state;
}

// ---- REST ----

export interface PresetsResponse {
  presets: Record<string, DroneConfig>;
  limits: Record<string, [number, number]>;
  gain_limits: Record<string, [number, number]>;
}

export interface Health {
  sim: string;
  llm: 'ok' | 'offline';
  model: string;
}

async function getJson<T>(url: string, init?: RequestInit): Promise<T> {
  const resp = await fetch(url, init);
  if (!resp.ok) throw new Error(`${url}: HTTP ${resp.status}`);
  return resp.json() as Promise<T>;
}

/** POST JSON; on an error status, throw the server's plain-English `detail`. */
async function postJson<T>(url: string, body: unknown): Promise<T> {
  const resp = await fetch(url, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
  if (!resp.ok) {
    let detail = `HTTP ${resp.status}`;
    try {
      detail = (await resp.json()).detail ?? detail;
    } catch {
      /* not JSON */
    }
    throw new Error(detail);
  }
  return resp.json() as Promise<T>;
}

/** 🎙 What the drone answered to a voice (or typed) command. */
export interface VoiceReply {
  heard: string;
  say: string; // spoken back
  source: 'quick' | 'ai' | 'error';
  pending: boolean; // waiting for "go"
  actions: string[];
  short?: string | null; // a one-or-two-word answer to speak for instant commands ("Left!")
}

export const api = {
  voiceWarmup: () => fetch('/api/voice/warmup', { method: 'POST' }).catch(() => undefined),
  voiceStatus: () => getJson<{ available: boolean; detail: string; loaded: boolean }>('/api/voice/status'),
  voiceTranscribe: async (audio: Blob): Promise<{ text: string; seconds: number }> => {
    const r = await fetch('/api/voice/transcribe', { method: 'POST', body: audio, headers: { 'Content-Type': audio.type || 'audio/webm' } });
    if (!r.ok) throw new Error((await r.json().catch(() => null))?.detail ?? `HTTP ${r.status}`);
    return r.json();
  },
  health: () => getJson<Health>('/api/health'),
  presets: () => getJson<PresetsResponse>('/api/presets'),
  scenarios: () => getJson<{ planners: string[]; scenarios: Record<string, { title: string }> }>('/api/missions/scenarios'),
  sitlFaults: () => getJson<{ faults: FaultInfo[] }>('/api/sitl/faults'),
  raceTracks: () => getJson<{ tracks: Record<string, { title: string; tip: string; rings: number; best_s: number | null }> }>('/api/race/tracks'),
  learn: (slug: string) => getJson<{ markdown: string }>(`/api/learn/${slug}`),
  site: () => getJson<Site>('/api/site'),
  compilePlan: (spec: PlanSpec, battery?: { cells: number; capacity_mah: number }) => postJson<FlightPlan>('/api/plan/compile', { spec, battery }),
  cropReport: (id: string) => postJson<{ answer: string; model: string; unverified_numbers: string[]; duration_s: number }>('/api/crop/report', { id }),
  aiPlan: (request: string, battery?: { cells: number; capacity_mah: number }) => postJson<AiPlanReply>('/api/ai/plan', { request, battery }),
  validate: (config: DroneConfig) =>
    getJson<{ valid: boolean; problems: string[]; derived: Derived | null }>('/api/config/validate', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ config }),
    }),
};

export const RAD_TO_DEG = 180 / Math.PI;

const COMPASS = ['east', 'north-east', 'north', 'north-west', 'west', 'south-west', 'south', 'south-east'];
/** 'east', 'north', … for a direction in degrees (0 = east, 90 = north: the sim's ENU convention). */
export function compassName(deg: number): string {
  return COMPASS[Math.round((((deg % 360) + 360) % 360) / 45) % 8];
}
export const GRAVITY_M_S2 = 9.81;
