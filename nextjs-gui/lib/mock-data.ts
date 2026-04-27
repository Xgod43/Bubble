import type {
  ActionCommand,
  DashboardSnapshot,
  MetricTile,
  PreviewPoint,
  SubsystemCard,
  TimelineEvent
} from "@/lib/contracts";

function formatClock(date: Date) {
  return date.toLocaleTimeString("en-GB", {
    hour: "2-digit",
    minute: "2-digit",
    second: "2-digit"
  });
}

function buildPreviewPoints(phase: number): PreviewPoint[] {
  const base = [
    [14, 18],
    [28, 24],
    [42, 21],
    [57, 19],
    [71, 24],
    [83, 18],
    [18, 47],
    [32, 44],
    [48, 49],
    [63, 45],
    [78, 48],
    [22, 72],
    [38, 69],
    [54, 73],
    [69, 70],
    [82, 74]
  ];

  return base.map(([x, y], index) => {
    const jitter = Math.sin(phase + index * 0.55) * 1.2;
    return {
      x,
      y: y + jitter,
      magnitude: Math.abs(Math.sin(phase * 0.9 + index * 0.31)) * 1.8,
      tracked: index % 7 !== 0
    };
  });
}

function buildMetrics(phase: number): MetricTile[] {
  const fps = 38 + Math.sin(phase) * 2.6;
  const latency = 1000 / fps;
  const meanDisp = 0.22 + Math.abs(Math.sin(phase * 0.8)) * 0.11;
  const maxDisp = meanDisp + 0.31;
  const miss = 0.02 + Math.abs(Math.cos(phase * 0.65)) * 0.015;

  return [
    {
      label: "Detection FPS",
      value: fps.toFixed(1),
      detail: "native-C contour pass on Pi 5 profile",
      tone: "positive"
    },
    {
      label: "Loop Latency",
      value: `${latency.toFixed(1)} ms`,
      detail: "camera to dashboard snapshot",
      tone: latency < 30 ? "positive" : "warning"
    },
    {
      label: "Mean Displacement",
      value: `${meanDisp.toFixed(3)} px`,
      detail: "rolling 2 s lattice mean",
      tone: "normal"
    },
    {
      label: "Peak Displacement",
      value: `${maxDisp.toFixed(3)} px`,
      detail: "strongest active node",
      tone: maxDisp < 1.0 ? "positive" : "warning"
    },
    {
      label: "Missing Tracks",
      value: `${(miss * 100).toFixed(1)}%`,
      detail: "reference nodes without live match",
      tone: miss < 0.05 ? "positive" : "warning"
    }
  ];
}

function buildSubsystems(phase: number): SubsystemCard[] {
  return [
    {
      id: "vision",
      name: "Vision Core",
      state: "online",
      summary: "Native blob extraction armed and stable.",
      detail: "Pre-process stays in OpenCV. Blob extraction and centroid filtering move through the C hot path.",
      telemetry: [
        { label: "Pipeline", value: "C detector + Python bridge" },
        { label: "Reference", value: "Locked" },
        { label: "Frame Size", value: "1280 x 720" }
      ]
    },
    {
      id: "motion",
      name: "Motion Axis",
      state: "idle",
      summary: "Stepper bus is staged for a short travel command.",
      detail: "Command queue is isolated from the UI thread so operator actions stay crisp under load.",
      telemetry: [
        { label: "Direction", value: "UP" },
        { label: "Pulse Rate", value: "1600 Hz" },
        { label: "Limits", value: "Nominal" }
      ]
    },
    {
      id: "pressure",
      name: "Pressure",
      state: "online",
      summary: "Sensor bridge is producing clean reads.",
      detail: "Snapshot values are shaped for the new dashboard contract and easy future logging.",
      telemetry: [
        { label: "Reading", value: `${(1008 + Math.sin(phase * 0.9) * 1.3).toFixed(2)} hPa` },
        { label: "Cadence", value: "10 Hz" },
        { label: "Bus", value: "I2C" }
      ]
    },
    {
      id: "loadcell",
      name: "Load Cell",
      state: "warning",
      summary: "Calibration required before force validation runs.",
      detail: "The new control surface keeps calibration state visible instead of burying it in a tab.",
      telemetry: [
        { label: "Estimate", value: `${(0.41 + Math.sin(phase * 0.5) * 0.03).toFixed(3)} kg` },
        { label: "Offset", value: "Pending" },
        { label: "Driver", value: "HX711" }
      ]
    }
  ];
}

function buildTimeline(now: Date): TimelineEvent[] {
  const times = [0, 18, 42, 81];
  const events: Array<[string, string, string, TimelineEvent["tone"]]> = [
    [
      "Operator console synced",
      "Next.js mission-control layout loaded with mock-safe bridge fallback.",
      "positive"
    ],
    [
      "Reference lattice locked",
      "Detection contract is returning node geometry for the viewport panel.",
      "normal"
    ],
    [
      "Load-cell calibration pending",
      "Bridge keeps this visible as a system-level warning instead of a hidden modal state.",
      "warning"
    ],
    [
      "Motion queue idle",
      "Axis commands are accepted through the command channel and can be wired to GPIO handlers on Pi 5.",
      "normal"
    ]
  ];

  return events.map(([title, detail, tone], index) => ({
    time: formatClock(new Date(now.getTime() - times[index] * 1000)),
    title,
    detail,
    tone
  }));
}

export function buildMockDashboardSnapshot(date = new Date()): DashboardSnapshot {
  const phase = date.getTime() / 3000;
  const metrics = buildMetrics(phase);
  const subsystems = buildSubsystems(phase);
  const timeline = buildTimeline(date);

  const actions: ActionCommand[] = [
    {
      id: "arm-stack",
      label: "ARM",
      command: "system.arm",
      variant: "primary",
      detail: "Prepare all subsystems for a run."
    },
    {
      id: "zero-reference",
      label: "REFERENCE ZERO",
      command: "vision.reset_reference",
      variant: "secondary",
      detail: "Capture a new neutral lattice."
    },
    {
      id: "capture-snapshot",
      label: "SNAPSHOT",
      command: "vision.snapshot",
      variant: "secondary",
      detail: "Persist the current overlay and mask."
    },
    {
      id: "stop-all",
      label: "HOLD",
      command: "system.stop_all",
      variant: "danger",
      detail: "Stop motion and release active tasks."
    }
  ];

  return {
    generatedAt: date.toISOString(),
    missionName: "Soft Gripper Bubble Console",
    systemState: "tracking",
    mode: "Operate / Pi 5",
    platform: "Raspberry Pi 5 / Camera Module 3 / Local control app",
    uptime: "03:14:52",
    headline: "Live sensing, actuation, and fault visibility for bubble-gripper runs.",
    summary:
      "Built for real operation on the rig: warnings stay in view, the live viewport stays central, and run actions stay within one reach instead of hiding behind test tabs.",
    metrics,
    subsystems,
    detections: {
      pipeline: "native-c",
      mode: "auto | overlay",
      fps: Number.parseFloat(metrics[0].value),
      frameMs: Number.parseFloat(metrics[1].value.replace(" ms", "")),
      dotCount: 16,
      meanDisp: Number.parseFloat(metrics[2].value.replace(" px", "")),
      maxDisp: Number.parseFloat(metrics[3].value.replace(" px", "")),
      missingRatio: Number.parseFloat(metrics[4].value.replace("%", "")) / 100,
      referenceLocked: true,
      previewPoints: buildPreviewPoints(phase),
      notes: [
        "Preview points are normalized so the viewport can render quickly without shipping raw frames through React.",
        "Bridge can swap this synthetic panel with live JPEG or WebRTC later without changing the operator layout.",
        "C detector is intended for the binary blob pass, while camera IO and orchestration stay swappable."
      ]
    },
    actions,
    timeline,
    operatorNotes: [
      "ARM before commanding stepper motion on a live rig.",
      "Zero the reference after major focus or lighting changes.",
      "Keep pressure and load-cell calibration visible during tests so the operator does not have to hunt for state."
    ]
  };
}
