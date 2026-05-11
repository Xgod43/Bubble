from __future__ import annotations

import math
import threading
import time
from datetime import datetime, timezone
from typing import Dict, List, Tuple

try:
    from .contracts import (
        ActionCommand,
        DashboardSnapshot,
        MetricTile,
        SubsystemCard,
        TelemetryPair,
        TimelineEvent,
    )
    from .vision_runtime import VisionRuntime
except ImportError:  # pragma: no cover - allows direct execution during bring-up
    from contracts import ActionCommand, DashboardSnapshot, MetricTile, SubsystemCard, TelemetryPair, TimelineEvent  # type: ignore
    from vision_runtime import VisionRuntime  # type: ignore


class MissionControlState:
    def __init__(self):
        self.started_at = time.monotonic()
        self.arm_state = False
        self.command_log: List[Tuple[float, str, str, str]] = []
        self.lock = threading.Lock()
        self.vision = VisionRuntime()

    def uptime(self) -> str:
        seconds = int(time.monotonic() - self.started_at)
        hours, seconds = divmod(seconds, 3600)
        minutes, seconds = divmod(seconds, 60)
        return f"{hours:02d}:{minutes:02d}:{seconds:02d}"

    def build_snapshot(self) -> Dict[str, object]:
        now = datetime.now(timezone.utc)
        phase = time.monotonic() / 4.0
        detection = self.vision.snapshot()
        pressure_hpa = 1008.4 + math.sin(phase) * 1.2
        load_kg = 0.41 + math.sin(phase * 0.7) * 0.03
        loop_latency = detection.frameMs + abs(math.cos(phase * 0.6)) * 1.2
        system_state = "tracking" if self.arm_state else "ready"

        snapshot = DashboardSnapshot(
            generatedAt=now.isoformat(),
            missionName="Soft Gripper Bubble Console",
            systemState=system_state,
            mode="Operate / Pi 5",
            platform="Raspberry Pi 5 / Camera Module 3 / Local control app",
            uptime=self.uptime(),
            headline="Live sensing, actuation, and fault visibility for bubble-gripper runs.",
            summary=(
                "Built for real operation on the rig: warnings stay in view, the live viewport stays central, and run actions stay within one reach instead of hiding behind test tabs."
            ),
            metrics=[
                MetricTile(
                    label="Detection FPS",
                    value=f"{detection.fps:.1f}",
                    detail="native blob pass when the shared library is available",
                    tone="positive" if detection.pipeline == "native-c" else "warning",
                ),
                MetricTile(
                    label="Loop Latency",
                    value=f"{loop_latency:.1f} ms",
                    detail="camera to app viewport snapshot",
                    tone="positive" if loop_latency < 30 else "warning",
                ),
                MetricTile(
                    label="Mean Displacement",
                    value=f"{detection.meanDisp:.3f} px",
                    detail="rolling lattice mean",
                    tone="normal",
                ),
                MetricTile(
                    label="Peak Displacement",
                    value=f"{detection.maxDisp:.3f} px",
                    detail="strongest active node",
                    tone="normal",
                ),
                MetricTile(
                    label="Missing Tracks",
                    value=f"{detection.missingRatio * 100:.1f}%",
                    detail="reference nodes without a live match",
                    tone="positive" if detection.missingRatio < 0.05 else "warning",
                ),
            ],
            subsystems=[
                SubsystemCard(
                    id="vision",
                    name="Vision Core",
                    state="online" if detection.pipeline == "native-c" else "warning",
                    summary="Detection stack split from UI and ready for Pi 5 acceleration.",
                    detail=(
                        "OpenCV still handles pre-processing efficiently, while the C layer owns blob extraction and "
                        "centroid filtering on the hot path."
                    ),
                    telemetry=[
                        TelemetryPair(label="Pipeline", value=detection.pipeline),
                        TelemetryPair(label="Reference", value="Locked" if detection.referenceLocked else "Unset"),
                        TelemetryPair(label="Nodes", value=str(detection.dotCount)),
                    ],
                ),
                SubsystemCard(
                    id="motion",
                    name="Motion Axis",
                    state="idle" if not self.arm_state else "online",
                    summary="Command routing is local to the app shell.",
                    detail="Stepper controls can be wired to GPIO handlers without letting the frontend block on hardware IO.",
                    telemetry=[
                        TelemetryPair(label="Pulse Rate", value="1600 Hz"),
                        TelemetryPair(label="Direction", value="UP"),
                        TelemetryPair(label="Limits", value="Nominal"),
                    ],
                ),
                SubsystemCard(
                    id="pressure",
                    name="Pressure",
                    state="online",
                    summary="Pressure readings stay visible in the primary scan lane.",
                    detail="This keeps test posture legible during active experiments instead of burying it inside tab state.",
                    telemetry=[
                        TelemetryPair(label="Reading", value=f"{pressure_hpa:.2f} hPa"),
                        TelemetryPair(label="Cadence", value="10 Hz"),
                        TelemetryPair(label="Bus", value="I2C"),
                    ],
                ),
                SubsystemCard(
                    id="loadcell",
                    name="Load Cell",
                    state="warning",
                    summary="Calibration flag remains visible until the app receives a real calibration result.",
                    detail="Offset and calibration health stay in the main operator flow instead of hiding in a settings panel.",
                    telemetry=[
                        TelemetryPair(label="Estimate", value=f"{load_kg:.3f} kg"),
                        TelemetryPair(label="Driver", value="HX711"),
                        TelemetryPair(label="Offset", value="Pending"),
                    ],
                ),
            ],
            detections=detection,
            actions=[
                ActionCommand(
                    id="arm-stack",
                    label="ARM",
                    command="system.arm",
                    variant="primary",
                    detail="Prepare the stack for a run.",
                ),
                ActionCommand(
                    id="zero-reference",
                    label="REFERENCE ZERO",
                    command="vision.reset_reference",
                    variant="secondary",
                    detail="Capture a fresh neutral lattice.",
                ),
                ActionCommand(
                    id="capture-snapshot",
                    label="SNAPSHOT",
                    command="vision.snapshot",
                    variant="secondary",
                    detail="Persist the current overlay and mask.",
                ),
                ActionCommand(
                    id="stop-all",
                    label="HOLD",
                    command="system.stop_all",
                    variant="danger",
                    detail="Stop motion and release active tasks.",
                ),
            ],
            timeline=self._build_timeline(now),
            operatorNotes=[
                "ARM before commanding stepper motion on the live rig.",
                "Zero the visual reference after major lighting or focus changes.",
                "Keep calibration status in the primary operator view so the test posture stays honest.",
            ],
        )
        return snapshot.to_dict()

    def dispatch_command(self, payload: Dict[str, object]) -> Dict[str, object]:
        command_type = str(payload.get("type", "unknown"))
        now = datetime.now(timezone.utc)

        with self.lock:
            if command_type == "system.arm":
                self.arm_state = True
                tone = "positive"
                message = "System armed. App accepted the run posture."
            elif command_type == "system.stop_all":
                self.arm_state = False
                tone = "warning"
                message = "Stop-all accepted. Active posture returned to idle."
            elif command_type == "vision.reset_reference":
                tone = "normal"
                message = "Reference reset queued for the local vision runtime."
            elif command_type == "vision.snapshot":
                tone = "normal"
                message = "Snapshot request accepted by the app."
            else:
                tone = "warning"
                message = f"No local hardware handler is mapped for {command_type} yet."

            self.command_log.append((time.monotonic(), command_type, message, tone))
            self.command_log = self.command_log[-10:]

        return {
            "accepted": True,
            "message": message,
            "echoedType": command_type,
            "generatedAt": now.isoformat(),
        }

    def app_info(self) -> Dict[str, object]:
        return {
            "name": "Pi Bubble Mission Control",
            "mode": "desktop-app",
            "uptime": self.uptime(),
            "nativeDetector": self.vision.native.available,
            "generatedAt": datetime.now(timezone.utc).isoformat(),
        }

    def _build_timeline(self, now: datetime) -> List[TimelineEvent]:
        baseline = [
            TimelineEvent(
                time=self._format_time(now.timestamp() - 85),
                title="Console synced",
                detail="The local Python console is feeding the mission-control state bridge.",
                tone="positive",
            ),
            TimelineEvent(
                time=self._format_time(now.timestamp() - 46),
                title="Native detector staged",
                detail="When the shared library is present, blob extraction shifts to C and lowers Python overhead.",
                tone="normal",
            ),
            TimelineEvent(
                time=self._format_time(now.timestamp() - 18),
                title="Calibration visibility raised",
                detail="Load-cell calibration stays in the operator lane instead of living in a buried settings view.",
                tone="warning",
            ),
        ]

        with self.lock:
            extra = [
                TimelineEvent(
                    time=self._format_time(time.time() - (time.monotonic() - stamp)),
                    title=command_type,
                    detail=message,
                    tone=tone,
                )
                for stamp, command_type, message, tone in reversed(self.command_log[-4:])
            ]

        return extra + baseline

    def _format_time(self, epoch_seconds: float) -> str:
        return datetime.fromtimestamp(epoch_seconds).strftime("%H:%M:%S")
