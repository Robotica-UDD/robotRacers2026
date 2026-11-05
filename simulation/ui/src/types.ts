export type MissionStatus = 'idle' | 'running' | 'paused' | 'stopped' | 'completed' | 'disconnected';
export type Point = { x: number; y: number };
export type Goal = Point & { id: string };
export type CameraTracking = {
  seq: number; sim_time: number; width: number; height: number;
  capture_stamp?: number; age_ms?: number;
  received_ms?: number;
  tracking: 'visible' | 'predicted' | 'lost';
  robot_center: [number,number] | null;
  robot_box: {x:number;y:number;width:number;height:number} | null;
  marker_corners?: [number,number][] | null;
  target?: {id:string;pixel:[number,number]} | null;
  goals: {id:string;pixel:[number,number]}[];
};
export type Settings = {
  speed: number;
  scale: number;
  mass_kg: number;
  friction: number;
  skid_friction: number;
  fan_force_n: number;
};
export type SimulationState = {
  status: MissionStatus;
  message?: string;
  sim_time: number;
  pose: Point & { yaw: number; source: string; age_ms: number };
  robot: { speed: number; left_speed: number; right_speed: number; battery_voltage: number };
  mission: { goals: Goal[]; active_goal: string | number | null; waypoints: Point[]; progress: number };
  settings: Settings;
  vision: { visible: boolean; latency_ms: number | null };
  metrics: Record<string, number | string | null>;
  connected: boolean;
  scene_pending?: boolean;
  webots_url?: string;
};
export type AppConfig = {
  robot: Record<string, unknown>;
  arena: {
    width_m?: number; height_m?: number; corridor_width_m?: number;
    speed_min?: number; speed_max?: number; scale_min?: number; scale_max?: number;
    nodes?: Record<string, number[]>; edges?: string[][];
    [key: string]: unknown;
  };
  defaults?: Partial<Settings>;
  webots: { url: string; module: string };
};
export type ViewMode = 'orbital' | 'top' | 'camera';

export const initialSettings: Settings = {
  speed: 0.30, scale: 1, mass_kg: 0.18, friction: 0.8, skid_friction: 0.12, fan_force_n: 0,
};

/** No pose or telemetry is invented before the service sends it. */
export const disconnectedState: SimulationState = {
  status: 'disconnected', sim_time: 0,
  pose: { x: Number.NaN, y: Number.NaN, yaw: Number.NaN, source: '', age_ms: Number.NaN },
  robot: { speed: Number.NaN, left_speed: Number.NaN, right_speed: Number.NaN, battery_voltage: Number.NaN },
  mission: { goals: [], active_goal: null, waypoints: [], progress: 0 },
  settings: initialSettings,
  vision: { visible: false, latency_ms: null }, metrics: {}, connected: false,
};

export function isSimulationState(value: unknown): value is SimulationState {
  if (!value || typeof value !== 'object') return false;
  const v = value as Partial<SimulationState>;
  return ['idle', 'running', 'paused', 'stopped', 'completed', 'disconnected'].includes(v.status ?? '')
    && Boolean(v.settings && v.mission && v.robot && v.pose && v.vision);
}

export const statusLabels: Record<MissionStatus, string> = {
  idle: 'En espera', running: 'En recorrido', paused: 'En pausa', stopped: 'Detenido',
  completed: 'Recorrido completado', disconnected: 'Sin conexión',
};

export function formatNumber(value: number | null | undefined, digits = 2): string {
  return typeof value === 'number' && Number.isFinite(value)
    ? value.toLocaleString('es-CL', { minimumFractionDigits: digits, maximumFractionDigits: digits }) : '—';
}

export function formatTime(seconds: number): string {
  if (!Number.isFinite(seconds)) return '—';
  const s = Math.max(0, Math.floor(seconds));
  return `${String(Math.floor(s / 60)).padStart(2, '0')}:${String(s % 60).padStart(2, '0')}`;
}

export function metricValue(metrics: SimulationState['metrics'], ...keys: string[]): number | null {
  for (const key of keys) if (typeof metrics[key] === 'number' && Number.isFinite(metrics[key])) return metrics[key] as number;
  return null;
}
