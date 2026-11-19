// Connection to the backend simulator (WebSocket) plus small REST helpers.
// Everything arrives in SI units and radians; components convert to degrees for display.

import { useEffect, useState, useSyncExternalStore } from 'react';

export type Vec3 = [number, number, number];
export type Quat = [number, number, number, number]; // [w, x, y, z], body -> world (ENU)

export interface SimState {
  type: 'state';
  t_s: number;
  mode: 'idle' | 'takeoff' | 'hover' | 'land' | 'mission';
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
  failsafe: boolean; // GPS-loss failsafe active: holding level + altitude, NOT position
  setpoint: { x_m: number; y_m: number; alt_m: number; yaw_rad: number };
  message: string;
  mission: MissionStatus | null; // set while the mission autopilot is flying
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
        this.stateListeners.forEach((fn) => fn());
        return;
      }
      if (msg.type === 'config') this.config = msg.config as DroneConfig;
      if (msg.course !== undefined) this.course = msg.course as Course | null;
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

export const api = {
  health: () => getJson<Health>('/api/health'),
  presets: () => getJson<PresetsResponse>('/api/presets'),
  scenarios: () => getJson<{ planners: string[]; scenarios: Record<string, { title: string }> }>('/api/missions/scenarios'),
  learn: (slug: string) => getJson<{ markdown: string }>(`/api/learn/${slug}`),
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
