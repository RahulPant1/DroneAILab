// Types and API calls for the Engineering Lab and saved runs.

export interface ParamDef {
  id: string;
  label: string;
  unit: string;
  default: number | string;
  help: string;
  min: number;
  max: number;
  step: number;
  options: [string, string][];
}

export interface MetricDef {
  key: string;
  label: string;
  unit: string;
  digits: number;
}

export interface ExperimentDef {
  id: string;
  number: number;
  title: string;
  objective: string;
  concept: [string, string, string];
  params: ParamDef[];
  key_metrics: MetricDef[];
  learn_slug: string;
}

export interface SeriesDef {
  channel: string;
  label: string;
  scale: number;
  style: 'line' | 'dashed' | 'points';
}

export interface PlotDef {
  id: string;
  title: string;
  y_label: string;
  how_to_read: string;
  series: SeriesDef[];
  kind?: 'time' | 'map' | 'images';
  extra?: MapExtra;
}

export interface ObstacleDef {
  x_min: number;
  x_max: number;
  y_min: number;
  y_max: number;
  height_m: number;
}

export interface MapExtra {
  obstacles: ObstacleDef[];
  waypoints: [number, number][];
  planned_route: [number, number][];
  safety_zone_m: number;
  discovered?: [number, number][]; // lidar planner: where its lidar hit something (its map)
  plans?: [number, number][][]; // lidar planner: every route it chose (the first is the straight line)
  fields?: [number, number][][]; // survey: the field outline(s)
  no_fly?: ObstacleDef[]; // no-fly zones (never entered)
  photos?: [number, number, number, number][]; // survey: x, y, height, line direction (rad) of each photo
  footprint_per_m?: [number, number]; // photo footprint (across, along) per metre of height
  gaps?: [number, number][]; // survey: spots no photo covered
  pads?: [number, number, number][]; // landing pads: x, y, marker size
  estimate?: [number[], number[]]; // where the drone THOUGHT it was (x list, y list)
  window?: [number, number, number, number]; // x0, x1, y0, y1: zoom the map to this area
  first_index?: number; // draw the paths from this sample on (e.g. only near the pad)
}

/** A strip of pictures (e.g. the down camera on the way down). */
export interface ImagesExtra {
  frames: { alt_m: number; t_s: number; image: string; w: number; h: number; detection: { box: [number, number, number, number]; pixels: number } | null }[];
}

export type Params = Record<string, number | string>;

export interface RunResult {
  run_type: string;
  config: Record<string, unknown>;
  env: Record<string, unknown>;
  seed: number;
  scenario: { experiment_id?: string; params?: Params; duration_s?: number };
  telemetry: Record<string, number[]>;
  metrics: Record<string, number | boolean | string | null>;
  events: { t_s: number; type: string; message: string; x_m?: number; y_m?: number }[];
  summary: string[];
  try_next: { text: string; params: Params }[];
  plots: PlotDef[];
}

export interface SavedRunSummary {
  id: string;
  label: string;
  saved_at: string;
  experiment_id: string;
  params: Params;
  seed: number;
  metrics: Record<string, number | boolean | string | null>;
}

export interface SavedRun {
  id: string;
  label: string;
  saved_at: string;
  result: RunResult;
}

export interface Comparison {
  a: SavedRun;
  b: SavedRun;
  metrics: { metric: string; a: unknown; b: unknown; diff: number | null }[];
  params: { param: string; a: unknown; b: unknown }[];
}

async function call<T>(url: string, init?: RequestInit): Promise<T> {
  const resp = await fetch(url, init);
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

const post = (body: unknown): RequestInit => ({
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify(body),
});

export const labApi = {
  experiments: () => call<ExperimentDef[]>('/api/experiments'),
  run: (id: string, params: Params, seed: number) => call<RunResult>(`/api/experiments/${id}/run`, post({ params, seed })),
  save: (experiment_id: string, params: Params, seed: number, label: string) =>
    call<SavedRunSummary>('/api/runs', post({ experiment_id, params, seed, label })),
  list: () => call<SavedRunSummary[]>('/api/runs'),
  get: (id: string) => call<SavedRun>(`/api/runs/${id}`),
  remove: (id: string) => call<{ deleted: string }>(`/api/runs/${id}`, { method: 'DELETE' }),
  deleteOld: (keep_newest: number) => call<{ deleted: number }>('/api/runs/delete-old', post({ keep_newest })),
  compare: (a: string, b: string) => call<Comparison>(`/api/runs/compare?a=${encodeURIComponent(a)}&b=${encodeURIComponent(b)}`),
};

// ---- formatting and export ----

export function formatValue(v: unknown, digits = 2): string {
  if (v === null || v === undefined) return 'n/a';
  if (typeof v === 'boolean') return v ? 'yes' : 'no';
  if (typeof v === 'number') return Number.isFinite(v) ? v.toFixed(digits) : 'n/a';
  return String(v);
}

function download(filename: string, text: string, type: string): void {
  const url = URL.createObjectURL(new Blob([text], { type }));
  const a = document.createElement('a');
  a.href = url;
  a.download = filename;
  a.click();
  URL.revokeObjectURL(url);
}

const safeName = (s: string) => s.replace(/[^a-z0-9_-]+/gi, '_').slice(0, 60) || 'run';

export function exportJson(name: string, data: unknown): void {
  download(`${safeName(name)}.json`, JSON.stringify(data, null, 1), 'application/json');
}

/** Telemetry as CSV: one column per channel, one row per sample (50 Hz). */
export function exportCsv(name: string, result: RunResult): void {
  const cols = Object.keys(result.telemetry);
  const n = result.telemetry.t_s?.length ?? 0;
  const lines = [cols.join(',')];
  for (let i = 0; i < n; i++) lines.push(cols.map((c) => result.telemetry[c][i]).join(','));
  download(`${safeName(name)}.csv`, lines.join('\n'), 'text/csv');
}
