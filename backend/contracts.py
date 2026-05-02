from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List


@dataclass
class MetricTile:
    label: str
    value: str
    detail: str
    tone: str = "normal"


@dataclass
class TelemetryPair:
    label: str
    value: str


@dataclass
class SubsystemCard:
    id: str
    name: str
    state: str
    summary: str
    detail: str
    telemetry: List[TelemetryPair] = field(default_factory=list)


@dataclass
class TimelineEvent:
    time: str
    title: str
    detail: str
    tone: str = "normal"


@dataclass
class ActionCommand:
    id: str
    label: str
    command: str
    variant: str
    detail: str


@dataclass
class PreviewPoint:
    x: float
    y: float
    magnitude: float
    tracked: bool


@dataclass
class DetectionSnapshot:
    pipeline: str
    mode: str
    fps: float
    frameMs: float
    dotCount: int
    meanDisp: float
    maxDisp: float
    missingRatio: float
    referenceLocked: bool
    previewPoints: List[PreviewPoint] = field(default_factory=list)
    notes: List[str] = field(default_factory=list)


@dataclass
class DashboardSnapshot:
    generatedAt: str
    missionName: str
    systemState: str
    mode: str
    platform: str
    uptime: str
    headline: str
    summary: str
    metrics: List[MetricTile] = field(default_factory=list)
    subsystems: List[SubsystemCard] = field(default_factory=list)
    detections: DetectionSnapshot | None = None
    actions: List[ActionCommand] = field(default_factory=list)
    timeline: List[TimelineEvent] = field(default_factory=list)
    operatorNotes: List[str] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)
