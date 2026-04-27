from __future__ import annotations

import importlib
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def check_import(label: str, module_name: str, required: bool = True) -> bool:
    try:
        importlib.import_module(module_name)
    except Exception as exc:
        status = "FAIL" if required else "WARN"
        print(f"[{status}] {label}: {exc}")
        return not required

    print(f"[OK]   {label}")
    return True


def check_file(label: str, path: Path, required: bool = True) -> bool:
    if path.exists():
        print(f"[OK]   {label}: {path.relative_to(ROOT)}")
        return True

    status = "FAIL" if required else "WARN"
    print(f"[{status}] {label}: missing {path.relative_to(ROOT)}")
    return not required


def main() -> int:
    ok = True
    version_ok = sys.version_info >= (3, 10)
    print(f"[{'OK' if version_ok else 'FAIL'}] Python {sys.version.split()[0]} (need >= 3.10)")
    ok = ok and version_ok

    ok = check_import("Tkinter GUI", "tkinter") and ok
    ok = check_import("NumPy", "numpy") and ok
    ok = check_import("OpenCV", "cv2") and ok
    ok = check_import("Pillow", "PIL.Image") and ok

    ok = check_import("RPi.GPIO API (use python3-rpi-lgpio on Pi 5)", "RPi.GPIO") and ok
    check_import("Picamera2 camera stack", "picamera2", required=False)
    check_import("libcamera controls", "libcamera", required=False)
    check_import("Adafruit Blinka board module", "board", required=False)
    check_import("MPRLS pressure module", "adafruit_mprls", required=False)
    check_import("pywebview desktop shell", "webview", required=False)

    hx_paths = [
        ROOT / "library" / "hx711py" / "hx711v0_5_1.py",
        ROOT / "hx711v0_5_1.py",
    ]
    if any(path.exists() for path in hx_paths):
        print("[OK]   HX711 load-cell driver")
    else:
        print("[FAIL] HX711 load-cell driver: missing hx711v0_5_1.py")
        ok = False
    check_file("Native detector build", ROOT / "native" / "build" / "libbubble_detect.so", required=False)
    runner_path = ROOT / "native" / "build" / ("stepper_runner.exe" if sys.platform.startswith("win") else "stepper_runner")
    check_file("Native stepper runner", runner_path, required=False)
    check_file("Exported Next.js UI", ROOT / "nextjs-gui" / "out" / "index.html", required=False)

    try:
        from backend.mission_control import MissionControlState

        MissionControlState().build_snapshot()
        print("[OK]   Desktop backend state")
    except Exception as exc:
        print(f"[FAIL] Desktop backend state: {exc}")
        ok = False

    print()
    if ok:
        print("Core Pi 5 Bookworm checks passed.")
        return 0

    print("Core checks failed. Install the missing required packages before hardware testing.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
