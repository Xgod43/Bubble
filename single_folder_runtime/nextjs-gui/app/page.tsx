import { DashboardShell } from "@/components/dashboard-shell";
import { buildMockDashboardSnapshot } from "@/lib/mock-data";

export default function Page() {
  const snapshot = buildMockDashboardSnapshot();
  return <DashboardShell initialSnapshot={snapshot} />;
}
