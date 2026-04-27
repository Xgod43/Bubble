export type SystemState = "ready" | "tracking" | "fault" | "maintenance";
export type Tone = "normal" | "positive" | "warning";
export type SubsystemState = "online" | "idle" | "warning" | "offline";
export type ActionVariant = "primary" | "secondary" | "danger";

export interface MetricTile {
  label: string;
  value: string;
  detail: string;
  tone: Tone;
}

export interface TelemetryPair {
  label: string;
  value: string;
}

export interface SubsystemCard {
  id: string;
  name: string;
  state: SubsystemState;
  summary: string;
  detail: string;
  telemetry: TelemetryPair[];
}

export interface TimelineEvent {
  time: string;
  title: string;
  detail: string;
  tone: Tone;
}

export interface ActionCommand {
  id: string;
  label: string;
  command: string;
  variant: ActionVariant;
  detail: string;
}

export interface PreviewPoint {
  x: number;
  y: number;
  magnitude: number;
  tracked: boolean;
}

export interface DetectionSnapshot {
  pipeline: string;
  mode: string;
  fps: number;
  frameMs: number;
  dotCount: number;
  meanDisp: number;
  maxDisp: number;
  missingRatio: number;
  referenceLocked: boolean;
  previewPoints: PreviewPoint[];
  notes: string[];
}

export interface DashboardSnapshot {
  generatedAt: string;
  missionName: string;
  systemState: SystemState;
  mode: string;
  platform: string;
  uptime: string;
  headline: string;
  summary: string;
  metrics: MetricTile[];
  subsystems: SubsystemCard[];
  detections: DetectionSnapshot;
  actions: ActionCommand[];
  timeline: TimelineEvent[];
  operatorNotes: string[];
}

export interface CommandRequest {
  type: string;
  payload?: Record<string, unknown>;
}

export interface CommandResult {
  accepted: boolean;
  message: string;
  echoedType: string;
  generatedAt: string;
}
