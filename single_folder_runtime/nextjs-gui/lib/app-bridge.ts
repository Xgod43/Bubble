import type { CommandRequest, CommandResult, DashboardSnapshot } from "@/lib/contracts";
import { buildMockDashboardSnapshot } from "@/lib/mock-data";

const BRIDGE_BASE_URL =
  process.env.NEXT_PUBLIC_PI_BUBBLE_BRIDGE_URL?.replace(/\/$/, "") ?? "http://127.0.0.1:8787";

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

async function readSnapshotFromHttp(): Promise<DashboardSnapshot | null> {
  try {
    const response = await fetch(`${BRIDGE_BASE_URL}/api/system`, {
      cache: "no-store"
    });
    if (!response.ok) {
      return null;
    }
    return (await response.json()) as DashboardSnapshot;
  } catch {
    return null;
  }
}

async function issueCommandOverHttp(request: CommandRequest): Promise<CommandResult | null> {
  try {
    const response = await fetch(`${BRIDGE_BASE_URL}/api/commands`, {
      method: "POST",
      headers: {
        "Content-Type": "application/json"
      },
      body: JSON.stringify(request)
    });
    if (!response.ok) {
      return null;
    }
    return (await response.json()) as CommandResult;
  } catch {
    return null;
  }
}

export async function readSnapshot(): Promise<DashboardSnapshot> {
  const api = getApi();
  if (!api?.get_snapshot) {
    const httpSnapshot = await readSnapshotFromHttp();
    if (httpSnapshot) {
      return httpSnapshot;
    }
    return buildMockDashboardSnapshot();
  }

  try {
    return await api.get_snapshot();
  } catch {
    const httpSnapshot = await readSnapshotFromHttp();
    if (httpSnapshot) {
      return httpSnapshot;
    }
    return buildMockDashboardSnapshot();
  }
}

export async function issueCommand(request: CommandRequest): Promise<CommandResult> {
  const api = getApi();
  if (!api?.send_command) {
    const httpResult = await issueCommandOverHttp(request);
    if (httpResult) {
      return httpResult;
    }
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
    const httpResult = await issueCommandOverHttp(request);
    if (httpResult) {
      return httpResult;
    }
    return {
      accepted: false,
      message: `App bridge unavailable for ${request.type}.`,
      echoedType: request.type,
      generatedAt: new Date().toISOString()
    };
  }
}
