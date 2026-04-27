export {};

declare global {
  interface Window {
    pywebview?: {
      api?: {
        get_snapshot: () => Promise<import("@/lib/contracts").DashboardSnapshot>;
        send_command: (
          request: import("@/lib/contracts").CommandRequest
        ) => Promise<import("@/lib/contracts").CommandResult>;
        app_info?: () => Promise<Record<string, unknown>>;
      };
    };
  }
}
