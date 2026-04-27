import type { CommandRequest, CommandResult, DashboardSnapshot } from "@/lib/contracts";
import { buildMockDashboardSnapshot } from "@/lib/mock-data";

declare global {
  interface Window {
    pywebview?: {
      api?: {
        get_snapshot: () => Promise<DashboardSnapshot>;
        send_command: (request: CommandRequest) => Promise<CommandResult>;
        app_info?: () => Promise<Record<string, unknown>>;
      };
    };
  }
}

function getApi() {
  if (typeof window === "undefined") {
    return null;
  }
  return window.pywebview?.api ?? null;
}

export async function readSnapshot(): Promise<DashboardSnapshot> {
  const api = getApi();
  if (!api?.get_snapshot) {
    return buildMockDashboardSnapshot();
  }

  try {
    return await api.get_snapshot();
  } catch {
    return buildMockDashboardSnapshot();
  }
}

export async function issueCommand(request: CommandRequest): Promise<CommandResult> {
  const api = getApi();
  if (!api?.send_command) {
    return {
      accepted: true,
      message: `Mock app accepted ${request.type}.`,
      echoedType: request.type,
      generatedAt: new Date().toISOString()
    };
  }

  try {
    return await api.send_command(request);
  } catch {
    return {
      accepted: false,
      message: `App bridge unavailable for ${request.type}.`,
      echoedType: request.type,
      generatedAt: new Date().toISOString()
    };
  }
}
