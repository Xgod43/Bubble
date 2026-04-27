import type { SystemState, Tone } from "@/lib/contracts";

const stateLabels: Record<SystemState, string> = {
  ready: "READY",
  tracking: "TRACKING",
  fault: "FAULT",
  maintenance: "MAINT"
};

export function StatusPill({
  state,
  tone
}: {
  state: SystemState | string;
  tone?: Tone;
}) {
  const className =
    tone === "warning"
      ? "status-pill status-pill-warning"
      : state === "fault"
        ? "status-pill status-pill-warning"
        : "status-pill status-pill-positive";

  return <span className={className}>{stateLabels[state as SystemState] ?? state.toUpperCase()}</span>;
}
