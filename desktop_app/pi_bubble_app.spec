# -*- mode: python ; coding: utf-8 -*-

from pathlib import Path


ROOT = Path.cwd()
UI_DIR = ROOT / "nextjs-gui" / "out"
NATIVE_DIR = ROOT / "native" / "build"

datas = []
if UI_DIR.exists():
    datas.append((str(UI_DIR), "ui"))
if NATIVE_DIR.exists():
    datas.append((str(NATIVE_DIR), "native"))


a = Analysis(
    ["desktop_app/main.py"],
    pathex=[str(ROOT)],
    binaries=[],
    datas=datas,
    hiddenimports=["backend.mission_control", "backend.vision_runtime", "backend.vision_native"],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name="PiBubbleMissionControl",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)
