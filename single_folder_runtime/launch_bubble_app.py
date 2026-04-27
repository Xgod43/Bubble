from __future__ import annotations

import importlib
import sys
import traceback
from pathlib import Path


ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def desktop_frontend_ready() -> bool:
    return (ROOT / "nextjs-gui" / "out" / "index.html").exists()


def can_launch_desktop_shell() -> bool:
    try:
        importlib.import_module("webview")
        return True
    except Exception:
        return False


def launch_desktop_app() -> None:
    from desktop_app.main import main as desktop_main

    desktop_main()


def launch_modern_python_gui() -> None:
    from python_mission_control_gui import main as modern_main

    modern_main()


def launch_legacy_gui() -> None:
    from all_in_one_gui import main as legacy_main

    legacy_main(use_legacy=True)


def write_launch_diagnostic(stage: str, exc: Exception) -> None:
    log_path = ROOT / "launch_errors.log"
    try:
        with log_path.open("a", encoding="utf-8") as handle:
            handle.write(f"\n[{stage}]\n")
            handle.write("".join(traceback.format_exception(type(exc), exc, exc.__traceback__)))
    except Exception:
        pass


def main() -> None:
    try:
        launch_modern_python_gui()
        return
    except Exception as exc:
        write_launch_diagnostic("modern_python_gui", exc)

    if desktop_frontend_ready() and can_launch_desktop_shell():
        try:
            launch_desktop_app()
            return
        except Exception as exc:
            write_launch_diagnostic("desktop_app", exc)

    launch_legacy_gui()


if __name__ == "__main__":
    main()
