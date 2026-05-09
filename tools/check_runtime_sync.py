from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]

SYNC_PAIRS = [
    ("all_in_one_gui.py", "single_folder_runtime/all_in_one_gui.py"),
    ("surface_measurement.py", "single_folder_runtime/surface_measurement.py"),
    ("hx711v0_5_1.py", "single_folder_runtime/library/hx711py/hx711v0_5_1.py"),
    ("requirements-pi-bookworm.txt", "single_folder_runtime/requirements-pi-bookworm.txt"),
    ("python_mission_control_gui.py", "single_folder_runtime/python_mission_control_gui.py"),
    ("stepper_test_gui.py", "single_folder_runtime/stepper_test_gui.py"),
    ("launch_bubble_app.py", "single_folder_runtime/launch_bubble_app.py"),
    ("main.py", "single_folder_runtime/main.py"),
    ("JNR/dot_pipeline.py", "single_folder_runtime/JNR/dot_pipeline.py"),
    ("JNR/synthetic_generator.py", "single_folder_runtime/JNR/synthetic_generator.py"),
    ("tools/setup_pi_venv.sh", "single_folder_runtime/tools/setup_pi_venv.sh"),
    ("backend/contracts.py", "single_folder_runtime/backend/contracts.py"),
    ("backend/mission_control.py", "single_folder_runtime/backend/mission_control.py"),
    ("backend/vision_runtime.py", "single_folder_runtime/backend/vision_runtime.py"),
    ("backend/vision_native.py", "single_folder_runtime/backend/vision_native.py"),
    ("backend/bridge_server.py", "single_folder_runtime/backend/bridge_server.py"),
    ("desktop_app/main.py", "single_folder_runtime/desktop_app/main.py"),
    ("desktop_app/run_pi.sh", "single_folder_runtime/desktop_app/run_pi.sh"),
    ("desktop_app/pi_bubble_app.spec", "single_folder_runtime/desktop_app/pi_bubble_app.spec"),
    ("desktop_app/requirements-app.txt", "single_folder_runtime/desktop_app/requirements-app.txt"),
    ("desktop_app/requirements-build.txt", "single_folder_runtime/desktop_app/requirements-build.txt"),
    ("native/blob_detector.c", "single_folder_runtime/native/blob_detector.c"),
    ("native/blob_detector.h", "single_folder_runtime/native/blob_detector.h"),
    ("native/build_pi.sh", "single_folder_runtime/native/build_pi.sh"),
    ("native/limit_reader.c", "single_folder_runtime/native/limit_reader.c"),
    ("native/stepper_runner.c", "single_folder_runtime/native/stepper_runner.c"),
    ("nextjs-gui/lib/app-bridge.ts", "single_folder_runtime/nextjs-gui/lib/app-bridge.ts"),
    ("nextjs-gui/lib/contracts.ts", "single_folder_runtime/nextjs-gui/lib/contracts.ts"),
    ("nextjs-gui/lib/mock-data.ts", "single_folder_runtime/nextjs-gui/lib/mock-data.ts"),
    ("nextjs-gui/components/dashboard-shell.tsx", "single_folder_runtime/nextjs-gui/components/dashboard-shell.tsx"),
    ("nextjs-gui/components/status-pill.tsx", "single_folder_runtime/nextjs-gui/components/status-pill.tsx"),
    ("nextjs-gui/app/page.tsx", "single_folder_runtime/nextjs-gui/app/page.tsx"),
    ("nextjs-gui/app/layout.tsx", "single_folder_runtime/nextjs-gui/app/layout.tsx"),
    ("nextjs-gui/app/globals.css", "single_folder_runtime/nextjs-gui/app/globals.css"),
]


def same_text(left: Path, right: Path) -> bool:
    with left.open("r", encoding="utf-8", newline=None) as left_handle:
        left_text = left_handle.read()
    with right.open("r", encoding="utf-8", newline=None) as right_handle:
        right_text = right_handle.read()
    return left_text == right_text


def main() -> int:
    mismatches: list[tuple[Path, Path, str]] = []
    for left_rel, right_rel in SYNC_PAIRS:
        left = ROOT / left_rel
        right = ROOT / right_rel
        if not left.exists() or not right.exists():
            missing = left if not left.exists() else right
            mismatches.append((left, right, f"missing {missing.relative_to(ROOT)}"))
            continue
        if not same_text(left, right):
            mismatches.append((left, right, "content differs"))

    if not mismatches:
        print("Runtime folder is in sync with root sources.")
        return 0

    print("Runtime sync check failed:")
    for left, right, reason in mismatches:
        print(f"- {left.relative_to(ROOT)} <-> {right.relative_to(ROOT)}: {reason}")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
