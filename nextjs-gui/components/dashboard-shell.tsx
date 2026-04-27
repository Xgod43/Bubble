"use client";

import { useCallback, useEffect, useMemo, useState } from "react";
import type { LucideIcon } from "lucide-react";
import {
  Activity,
  AlertTriangle,
  Aperture,
  Camera,
  CheckCircle2,
  ChevronRight,
  Clock3,
  Cpu,
  Gauge,
  LocateFixed,
  PauseCircle,
  Play,
  Radar,
  Scale,
  ShieldCheck,
  Wrench
} from "lucide-react";
import { StatusPill } from "@/components/status-pill";
import { issueCommand, readSnapshot } from "@/lib/app-bridge";
import type { ActionCommand, CommandResult, DashboardSnapshot, MetricTile, SubsystemCard, TimelineEvent } from "@/lib/contracts";

type DerivedAlert = {
  id: string;
  title: string;
  detail: string;
  tone: "positive" | "warning";
};

type ChecklistItem = {
  id: string;
  label: string;
  detail: string;
  complete: boolean;
};

function getSubsystem(snapshot: DashboardSnapshot, id: string) {
  return snapshot.subsystems.find((item) => item.id === id);
}

function getTelemetryValue(subsystem: SubsystemCard | undefined, label: string) {
  return subsystem?.telemetry.find((item) => item.label === label)?.value ?? "-";
}

function deriveAlerts(snapshot: DashboardSnapshot): DerivedAlert[] {
  const alerts: DerivedAlert[] = [];
  const loadCell = getSubsystem(snapshot, "loadcell");
  const motion = getSubsystem(snapshot, "motion");

  if (snapshot.detections.missingRatio > 0.04) {
    alerts.push({
      id: "tracking-loss",
      title: "Tracking stability needs attention",
      detail: `${(snapshot.detections.missingRatio * 100).toFixed(1)}% of reference nodes are currently unmatched.`,
      tone: "warning"
    });
  }

  if (snapshot.detections.pipeline !== "native-c") {
    alerts.push({
      id: "native-offline",
      title: "Native detector is not active",
      detail: "The app is running on the fallback detection path, so Pi 5 headroom is lower than expected.",
      tone: "warning"
    });
  }

  if (loadCell?.state === "warning") {
    alerts.push({
      id: "loadcell-cal",
      title: "Load-cell calibration is pending",
      detail: "Force validation runs should be held until offset and reference weight are confirmed.",
      tone: "warning"
    });
  }

  if (motion?.state === "offline") {
    alerts.push({
      id: "motion-offline",
      title: "Motion axis is offline",
      detail: "Stepper control is not responding through the current bridge state.",
      tone: "warning"
    });
  }

  if (alerts.length === 0) {
    alerts.push({
      id: "nominal",
      title: "No critical alerts",
      detail: "The current snapshot looks nominal for an observation run.",
      tone: "positive"
    });
  }

  return alerts;
}

function deriveChecklist(snapshot: DashboardSnapshot): ChecklistItem[] {
  const vision = getSubsystem(snapshot, "vision");
  const pressure = getSubsystem(snapshot, "pressure");
  const loadCell = getSubsystem(snapshot, "loadcell");
  const motion = getSubsystem(snapshot, "motion");
  const limits = getTelemetryValue(motion, "Limits");
  const loadOffset = getTelemetryValue(loadCell, "Offset").toLowerCase();

  return [
    {
      id: "native",
      label: "Native vision path",
      detail: snapshot.detections.pipeline === "native-c" ? "C detector engaged" : "Fallback path in use",
      complete: snapshot.detections.pipeline === "native-c"
    },
    {
      id: "reference",
      label: "Reference lock",
      detail: snapshot.detections.referenceLocked ? "Neutral lattice captured" : "Reference not set",
      complete: snapshot.detections.referenceLocked
    },
    {
      id: "limits",
      label: "Motion limits",
      detail: limits,
      complete: limits.toLowerCase().includes("nominal")
    },
    {
      id: "pressure",
      label: "Pressure stream",
      detail: pressure?.state === "online" ? getTelemetryValue(pressure, "Reading") : "Sensor not ready",
      complete: pressure?.state === "online"
    },
    {
      id: "loadcell",
      label: "Load-cell calibration",
      detail: getTelemetryValue(loadCell, "Offset"),
      complete: loadOffset !== "-" && loadOffset !== "pending"
    },
    {
      id: "frame",
      label: "Viewport feed",
      detail: getTelemetryValue(vision, "Frame Size"),
      complete: snapshot.detections.dotCount > 0
    }
  ];
}

function formatTime(value: string) {
  return new Date(value).toLocaleTimeString("en-GB");
}

function actionIcon(command: string): LucideIcon {
  switch (command) {
    case "system.arm":
      return Play;
    case "vision.reset_reference":
      return LocateFixed;
    case "vision.snapshot":
      return Camera;
    case "system.stop_all":
      return PauseCircle;
    default:
      return Radar;
  }
}

function HeaderMetric({
  icon: Icon,
  label,
  value
}: {
  icon: LucideIcon;
  label: string;
  value: string;
}) {
  return (
    <div className="header-metric">
      <Icon size={16} />
      <div>
        <span>{label}</span>
        <strong>{value}</strong>
      </div>
    </div>
  );
}

function MetricRibbonItem({ metric }: { metric: MetricTile }) {
  return (
    <article className={`metric-ribbon-item metric-ribbon-item-${metric.tone}`}>
      <div className="metric-ribbon-head">
        <span>{metric.label}</span>
        <strong>{metric.value}</strong>
      </div>
      <p>{metric.detail}</p>
    </article>
  );
}

function AlertRow({ item }: { item: DerivedAlert }) {
  const Icon = item.tone === "warning" ? AlertTriangle : ShieldCheck;
  return (
    <article className={`alert-row alert-row-${item.tone}`}>
      <div className="alert-icon">
        <Icon size={16} />
      </div>
      <div>
        <h3>{item.title}</h3>
        <p>{item.detail}</p>
      </div>
    </article>
  );
}

function ChecklistRow({ item }: { item: ChecklistItem }) {
  return (
    <article className="checklist-row">
      <div className={`checklist-state ${item.complete ? "checklist-state-complete" : "checklist-state-pending"}`}>
        {item.complete ? <CheckCircle2 size={16} /> : <Clock3 size={16} />}
      </div>
      <div>
        <h3>{item.label}</h3>
        <p>{item.detail}</p>
      </div>
    </article>
  );
}

function CompactSubsystemRow({ subsystem }: { subsystem: SubsystemCard }) {
  return (
    <article className="compact-subsystem-row">
      <div className="compact-subsystem-head">
        <span>{subsystem.name}</span>
        <span className={`subsystem-state subsystem-state-${subsystem.state}`}>{subsystem.state}</span>
      </div>
      <strong>{subsystem.summary}</strong>
      <p>{subsystem.detail}</p>
    </article>
  );
}

function CommandButton({
  action,
  busy,
  onPress
}: {
  action: ActionCommand;
  busy: boolean;
  onPress: (action: ActionCommand) => void;
}) {
  const Icon = actionIcon(action.command);
  return (
    <button
      className={`command-button command-button-${action.variant}`}
      disabled={busy}
      onClick={() => onPress(action)}
      type="button"
    >
      <div className="command-button-head">
        <Icon size={18} />
        <span>{action.label}</span>
      </div>
      <small>{busy ? "Sending..." : action.detail}</small>
    </button>
  );
}

function HardwareRow({ subsystem }: { subsystem: SubsystemCard }) {
  return (
    <article className="hardware-row">
      <div className="hardware-row-main">
        <div>
          <span className="eyebrow">{subsystem.name}</span>
          <h3>{subsystem.summary}</h3>
        </div>
        <span className={`subsystem-state subsystem-state-${subsystem.state}`}>{subsystem.state}</span>
      </div>
      <p>{subsystem.detail}</p>
      <div className="hardware-telemetry">
        {subsystem.telemetry.map((item) => (
          <div key={`${subsystem.id}-${item.label}`} className="hardware-telemetry-item">
            <span>{item.label}</span>
            <strong>{item.value}</strong>
          </div>
        ))}
      </div>
    </article>
  );
}

function TimelineList({ items }: { items: TimelineEvent[] }) {
  return (
    <div className="timeline-list">
      {items.map((item) => (
        <article key={`${item.time}-${item.title}`} className={`timeline-item timeline-item-${item.tone}`}>
          <span className="timeline-time">{item.time}</span>
          <div>
            <h3>{item.title}</h3>
            <p>{item.detail}</p>
          </div>
        </article>
      ))}
    </div>
  );
}

function DetectionStage({ snapshot }: { snapshot: DashboardSnapshot }) {
  const stageStats = [
    { label: "Pipeline", value: snapshot.detections.pipeline },
    { label: "Output", value: snapshot.detections.mode },
    { label: "Tracked", value: `${snapshot.detections.dotCount} nodes` },
    { label: "Loop", value: `${snapshot.detections.frameMs.toFixed(1)} ms` },
    { label: "Mean", value: `${snapshot.detections.meanDisp.toFixed(3)} px` },
    { label: "Missing", value: `${(snapshot.detections.missingRatio * 100).toFixed(1)}%` }
  ];

  return (
    <section className="stage-shell">
      <div className="section-heading">
        <div>
          <span className="eyebrow">
            <Aperture size={14} />
            Live Detection
          </span>
          <h2>Viewport and deformation state</h2>
        </div>
        <StatusPill state={snapshot.systemState} />
      </div>

      <div className="stage-toolbar">
        <span className="toolbar-chip">{snapshot.platform}</span>
        <span className="toolbar-chip">{snapshot.mode}</span>
        <span className="toolbar-chip">
          {snapshot.detections.referenceLocked ? "Reference locked" : "Reference pending"}
        </span>
      </div>

      <div className="viewport-shell">
        <div className="viewport-grid">
          {snapshot.detections.previewPoints.map((point, index) => (
            <span
              key={`${point.x}-${point.y}-${index}`}
              className={point.tracked ? "preview-point" : "preview-point preview-point-lost"}
              style={{
                left: `${point.x}%`,
                top: `${point.y}%`,
                width: `${8 + point.magnitude * 3}px`,
                height: `${8 + point.magnitude * 3}px`
              }}
            />
          ))}
          <div className="viewport-axis viewport-axis-x" />
          <div className="viewport-axis viewport-axis-y" />
        </div>

        <div className="viewport-overlay">
          <div>
            <span>Camera to decision</span>
            <strong>{snapshot.detections.frameMs.toFixed(1)} ms</strong>
          </div>
          <div>
            <span>Frame health</span>
            <strong>{snapshot.detections.fps.toFixed(1)} FPS</strong>
          </div>
          <div>
            <span>Reference</span>
            <strong>{snapshot.detections.referenceLocked ? "Stable" : "Unset"}</strong>
          </div>
        </div>
      </div>

      <div className="stage-stats">
        {stageStats.map((item) => (
          <div key={item.label} className="stage-stat">
            <span>{item.label}</span>
            <strong>{item.value}</strong>
          </div>
        ))}
      </div>

      <div className="stage-note-list">
        {snapshot.detections.notes.map((note) => (
          <span key={note}>{note}</span>
        ))}
      </div>
    </section>
  );
}

function OperationsDeck({
  snapshot,
  pendingCommand,
  onCommand
}: {
  snapshot: DashboardSnapshot;
  pendingCommand: string | null;
  onCommand: (action: ActionCommand) => void;
}) {
  const vision = getSubsystem(snapshot, "vision");
  const pressure = getSubsystem(snapshot, "pressure");
  const loadCell = getSubsystem(snapshot, "loadcell");
  const motion = getSubsystem(snapshot, "motion");

  const contextItems = [
    { label: "Vision pipeline", value: getTelemetryValue(vision, "Pipeline") },
    { label: "Frame size", value: getTelemetryValue(vision, "Frame Size") },
    { label: "Motion limits", value: getTelemetryValue(motion, "Limits") },
    { label: "Pressure", value: getTelemetryValue(pressure, "Reading") },
    { label: "Load estimate", value: getTelemetryValue(loadCell, "Estimate") },
    { label: "Last snapshot", value: formatTime(snapshot.generatedAt) }
  ];

  return (
    <section className="ops-deck">
      <div className="section-heading">
        <div>
          <span className="eyebrow">
            <Radar size={14} />
            Command Deck
          </span>
          <h2>Run actions and live context</h2>
        </div>
      </div>

      <div className="action-grid">
        {snapshot.actions.map((action) => (
          <CommandButton
            key={action.id}
            action={action}
            busy={pendingCommand === action.command}
            onPress={onCommand}
          />
        ))}
      </div>

      <div className="detail-block">
        <div className="detail-block-head">
          <Cpu size={15} />
          <h3>Run context</h3>
        </div>
        <div className="context-grid">
          {contextItems.map((item) => (
            <div key={item.label} className="context-item">
              <span>{item.label}</span>
              <strong>{item.value}</strong>
            </div>
          ))}
        </div>
      </div>

      <div className="detail-block">
        <div className="detail-block-head">
          <Wrench size={15} />
          <h3>Operator discipline</h3>
        </div>
        <div className="note-stack">
          {snapshot.operatorNotes.map((note) => (
            <div key={note} className="note-row">
              <ChevronRight size={14} />
              <p>{note}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function ReviewLane({ snapshot }: { snapshot: DashboardSnapshot }) {
  const alerts = deriveAlerts(snapshot);
  const topAlert = alerts.find((item) => item.tone === "warning");
  const recommendations = topAlert
    ? [
        topAlert.detail,
        "Confirm lighting, focus, and reference state before re-running the sequence.",
        "If the issue persists, switch attention to the subsystem rail and recent event log."
      ]
    : [
        "Current posture is suitable for a short validation run.",
        "Capture a snapshot before changing lighting or mechanical load.",
        "Keep the event lane open during hardware debugging so failures are timestamped."
      ];

  return (
    <section className="review-band">
      <div className="review-column">
        <div className="section-heading">
          <div>
            <span className="eyebrow">
              <Activity size={14} />
              Event Lane
            </span>
            <h2>Recent state changes</h2>
          </div>
        </div>
        <TimelineList items={snapshot.timeline} />
      </div>

      <div className="review-column">
        <div className="section-heading">
          <div>
            <span className="eyebrow">
              <ShieldCheck size={14} />
              Next Actions
            </span>
            <h2>What the operator should do next</h2>
          </div>
        </div>
        <div className="note-stack">
          {recommendations.map((item) => (
            <div key={item} className="note-row">
              <ChevronRight size={14} />
              <p>{item}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

export function DashboardShell({ initialSnapshot }: { initialSnapshot: DashboardSnapshot }) {
  const [snapshot, setSnapshot] = useState(initialSnapshot);
  const [pendingCommand, setPendingCommand] = useState<string | null>(null);
  const [commandResult, setCommandResult] = useState<CommandResult | null>(null);

  const refresh = useCallback(async () => {
    const next = await readSnapshot();
    setSnapshot(next);
  }, []);

  useEffect(() => {
    const onReady = () => {
      void refresh();
    };

    window.addEventListener("pywebviewready", onReady);
    const timer = window.setInterval(() => {
      void refresh();
    }, 2500);

    return () => {
      window.removeEventListener("pywebviewready", onReady);
      window.clearInterval(timer);
    };
  }, [refresh]);

  const handleCommand = useCallback(
    async (command: ActionCommand) => {
      setPendingCommand(command.command);
      try {
        const result = await issueCommand({
          type: command.command
        });
        setCommandResult(result);
        await refresh();
      } finally {
        setPendingCommand(null);
      }
    },
    [refresh]
  );

  const alerts = useMemo(() => deriveAlerts(snapshot), [snapshot]);
  const checklist = useMemo(() => deriveChecklist(snapshot), [snapshot]);
  const motion = getSubsystem(snapshot, "motion");
  const pressure = getSubsystem(snapshot, "pressure");
  const loadCell = getSubsystem(snapshot, "loadcell");

  return (
    <main className="console-root">
      <header className="console-header">
        <div className="header-main">
          <span className="eyebrow">
            <Radar size={14} />
            Bubble Operator Console
          </span>
          <h1>{snapshot.missionName}</h1>
          <p>{snapshot.summary}</p>
        </div>

        <div className="header-meta">
          <HeaderMetric icon={Activity} label="State" value={snapshot.systemState.toUpperCase()} />
          <HeaderMetric icon={Clock3} label="Uptime" value={snapshot.uptime} />
          <HeaderMetric icon={Cpu} label="Pipeline" value={snapshot.detections.pipeline} />
          <HeaderMetric icon={Gauge} label="Pressure" value={getTelemetryValue(pressure, "Reading")} />
          <HeaderMetric icon={Scale} label="Load" value={getTelemetryValue(loadCell, "Estimate")} />
          <div className="header-pulse">
            <small className="header-pulse-label">Run posture</small>
            <StatusPill state={snapshot.systemState} />
          </div>
        </div>
      </header>

      {commandResult ? (
        <section className={`signal-band ${commandResult.accepted ? "signal-band-ok" : "signal-band-warn"}`}>
          <strong>{commandResult.message}</strong>
          <span>{formatTime(commandResult.generatedAt)}</span>
        </section>
      ) : null}

      <section className="metric-ribbon">
        {snapshot.metrics.map((metric) => (
          <MetricRibbonItem key={metric.label} metric={metric} />
        ))}
      </section>

      <section className="operations-band">
        <aside className="ops-sidebar">
          <div className="sidebar-section">
            <div className="section-heading">
              <div>
                <span className="eyebrow">
                  <ShieldCheck size={14} />
                  Run Readiness
                </span>
                <h2>Pre-flight checks</h2>
              </div>
            </div>
            <div className="checklist-list">
              {checklist.map((item) => (
                <ChecklistRow key={item.id} item={item} />
              ))}
            </div>
          </div>

          <div className="sidebar-section">
            <div className="section-heading">
              <div>
                <span className="eyebrow">
                  <AlertTriangle size={14} />
                  Active Alerts
                </span>
                <h2>Immediate attention</h2>
              </div>
            </div>
            <div className="alert-list">
              {alerts.map((item) => (
                <AlertRow key={item.id} item={item} />
              ))}
            </div>
          </div>

          <div className="sidebar-section">
            <div className="section-heading">
              <div>
                <span className="eyebrow">
                  <Cpu size={14} />
                  Subsystem Rail
                </span>
                <h2>Quick hardware posture</h2>
              </div>
            </div>
            <div className="compact-subsystem-list">
              {snapshot.subsystems.map((subsystem) => (
                <CompactSubsystemRow key={subsystem.id} subsystem={subsystem} />
              ))}
            </div>
          </div>
        </aside>

        <DetectionStage snapshot={snapshot} />

        <OperationsDeck snapshot={snapshot} pendingCommand={pendingCommand} onCommand={handleCommand} />
      </section>

      <section className="systems-band">
        <div className="section-heading">
          <div>
            <span className="eyebrow">
              <Cpu size={14} />
              Hardware Matrix
            </span>
            <h2>Subsystem status and live telemetry</h2>
          </div>
          <div className="section-summary">
            <span>{getTelemetryValue(motion, "Limits")}</span>
            <span>{snapshot.detections.dotCount} nodes tracked</span>
            <span>{formatTime(snapshot.generatedAt)}</span>
          </div>
        </div>

        <div className="hardware-list">
          {snapshot.subsystems.map((subsystem) => (
            <HardwareRow key={subsystem.id} subsystem={subsystem} />
          ))}
        </div>
      </section>

      <ReviewLane snapshot={snapshot} />
    </main>
  );
}
