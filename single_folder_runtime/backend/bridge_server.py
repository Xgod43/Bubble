from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Dict
from urllib.parse import urlparse

try:
    from .mission_control import MissionControlState
except ImportError:  # pragma: no cover - allows python backend/bridge_server.py during bring-up
    from mission_control import MissionControlState  # type: ignore


class MissionControlHandler(BaseHTTPRequestHandler):
    state = MissionControlState()
    server_version = "PiBubbleBridge/0.1"

    def do_OPTIONS(self) -> None:
        self._send_json({"ok": True}, status=204)

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        if path == "/api/health":
            self._send_json(
                {
                    "ok": True,
                    "generatedAt": datetime.now(timezone.utc).isoformat(),
                }
            )
            return
        if path == "/api/system":
            self._send_json(self.state.build_snapshot())
            return
        self._send_json({"ok": False, "error": f"Unknown route: {path}"}, status=404)

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        if path != "/api/commands":
            self._send_json({"ok": False, "error": f"Unknown route: {path}"}, status=404)
            return

        content_length = int(self.headers.get("Content-Length", "0") or 0)
        raw = self.rfile.read(content_length) if content_length > 0 else b"{}"
        try:
            payload = json.loads(raw.decode("utf-8"))
        except json.JSONDecodeError:
            self._send_json({"ok": False, "error": "Request body must be valid JSON."}, status=400)
            return

        self._send_json(self.state.dispatch_command(payload), status=202)

    def log_message(self, format: str, *args) -> None:  # noqa: A003
        return

    def _send_json(self, payload: Dict[str, object], status: int = 200) -> None:
        encoded = b"" if status == 204 else json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(encoded)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()
        if encoded:
            self.wfile.write(encoded)


def main() -> None:
    parser = argparse.ArgumentParser(description="Pi Bubble bridge server")
    parser.add_argument("--host", default="127.0.0.1", help="Host interface to bind")
    parser.add_argument("--port", default=8787, type=int, help="Port to bind")
    args = parser.parse_args()

    server = ThreadingHTTPServer((args.host, args.port), MissionControlHandler)
    print(f"Pi Bubble bridge listening on http://{args.host}:{args.port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
