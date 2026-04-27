from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path
from typing import Any, Dict

try:
    import webview
except ImportError as exc:  # pragma: no cover - runtime dependency
    raise SystemExit(
        "pywebview is required for the desktop app. Install it on the Pi with: python3 -m pip install pywebview"
    ) from exc


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from backend.mission_control import MissionControlState  # noqa: E402


class PiBubbleDesktopApi:
    def __init__(self):
        self.state = MissionControlState()

    def get_snapshot(self) -> Dict[str, Any]:
        return self.state.build_snapshot()

    def send_command(self, request: Dict[str, Any] | None = None) -> Dict[str, Any]:
        payload = request or {}
        if not isinstance(payload, dict):
            payload = {"type": "invalid"}
        return self.state.dispatch_command(payload)

    def app_info(self) -> Dict[str, Any]:
        return self.state.app_info()


def bundle_root() -> Path | None:
    meipass = getattr(sys, "_MEIPASS", None)
    if meipass:
        return Path(meipass)
    return None


def configure_bundled_native_lib() -> None:
    if os.environ.get("PI_BUBBLE_NATIVE_LIB"):
        return

    root = bundle_root()
    if root is None:
        return

    candidates = [
        root / "native" / "libbubble_detect.so",
        root / "native" / "bubble_detect.dll",
        root / "native" / "libbubble_detect.dylib",
    ]
    for candidate in candidates:
        if candidate.exists():
            os.environ["PI_BUBBLE_NATIVE_LIB"] = str(candidate)
            return


def resolve_frontend_index(frontend_dir: Path | None) -> Path:
    if frontend_dir is not None:
        candidate = frontend_dir / "index.html"
        if candidate.exists():
            return candidate

    bundled_root = bundle_root()
    if bundled_root is not None:
        bundled_index = bundled_root / "ui" / "index.html"
        if bundled_index.exists():
            return bundled_index

    default_index = REPO_ROOT / "nextjs-gui" / "out" / "index.html"
    if default_index.exists():
        return default_index

    raise FileNotFoundError(
        "Could not find nextjs-gui/out/index.html. Build the frontend first with: "
        "cd nextjs-gui && npm install && npm run build"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Pi Bubble desktop app")
    parser.add_argument(
        "--frontend-dir",
        type=Path,
        default=None,
        help="Optional path to the exported Next.js frontend folder containing index.html",
    )
    parser.add_argument("--debug", action="store_true", help="Enable pywebview debug mode")
    parser.add_argument("--fullscreen", action="store_true", help="Launch in fullscreen mode")
    args = parser.parse_args()

    configure_bundled_native_lib()
    index_html = resolve_frontend_index(args.frontend_dir)
    api = PiBubbleDesktopApi()

    window = webview.create_window(
        title="Pi Bubble Mission Control",
        url=index_html.as_uri(),
        js_api=api,
        width=1600,
        height=980,
        min_size=(1200, 760),
        background_color="#06090d",
        fullscreen=args.fullscreen,
    )

    webview.start(debug=args.debug)


if __name__ == "__main__":
    main()
