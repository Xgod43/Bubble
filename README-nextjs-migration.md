# Pi Bubble Mission Control App

This repo now has a replacement stack for the old Tkinter console, and it is designed to run as a local app instead of an operator-managed server:

- `all_in_one_gui.py`: legacy monolith
- `main.py`: single Python entrypoint that prefers the new app and falls back to the legacy GUI
- `launch_bubble_app.py`: compatibility launcher
- `nextjs-gui/`: new operator UI in Next.js, exported as static files
- `desktop_app/main.py`: local app shell using `pywebview`
- `backend/mission_control.py`: app-side state, commands, and runtime contract
- `native/blob_detector.c`: native detector for the blob hot path

## Why this shape

The old GUI mixes tabs, camera logic, hardware reads, and detection in one Python process. The new app splits concerns without turning the operator workflow into a server deployment problem:

1. `Next.js` builds the interface.
2. `pywebview` loads the exported UI as a desktop app.
3. The app shell exposes direct Python methods to the frontend through `window.pywebview.api`.
4. `blob_detector.c` takes the contour/blob extraction hot path out of Python.
5. The existing `JNR/dot_pipeline.py` still helps with pre-processing and migration safety.
6. The legacy GUI can also use the native detector path, so even fallback mode is not stuck on pure Python detection.

## Fastest practical launch path

If you just want one thing to open:

```bash
cd Pi-bubble
python3 main.py
```

Convenience launchers:

- Linux/Pi: `launch_bubble_app.sh`
- Windows: `launch_bubble_app.bat`
- Raspberry Pi desktop icon: `desktop_app/install_desktop_entry.sh`

Behavior:

- if the exported app UI exists and `pywebview` is installed, it opens the new desktop app
- otherwise it falls back to `all_in_one_gui.py`
- the legacy blob pipeline now prefers the native C detector when the shared library is available

## Pi 5 bring-up

### 1. Build the native detector

```bash
cd Pi-bubble/native
chmod +x build_pi.sh
./build_pi.sh
```

If you place the shared library somewhere else, export:

```bash
export PI_BUBBLE_NATIVE_LIB=/full/path/to/libbubble_detect.so
```

### 2. Build the exported UI

```bash
cd Pi-bubble/nextjs-gui
npm install
npm run build
```

That produces `nextjs-gui/out/`, which the desktop shell opens directly from disk.

### 3. Install the app shell dependency

```bash
cd Pi-bubble
python3 -m pip install -r desktop_app/requirements-app.txt
```

### 4. Launch the app

```bash
cd Pi-bubble
python3 main.py
```

Or on Pi:

```bash
cd Pi-bubble
chmod +x desktop_app/run_pi.sh
./desktop_app/run_pi.sh --fullscreen
```

If you want a clickable icon on the Pi desktop:

```bash
cd Pi-bubble
chmod +x desktop_app/install_desktop_entry.sh
./desktop_app/install_desktop_entry.sh
```

## Standalone bundle option

If you want a more app-like packaged folder instead of running Python manually:

```bash
cd Pi-bubble
chmod +x desktop_app/build_standalone_pi.sh
./desktop_app/build_standalone_pi.sh
```

That creates a packaged app under `dist/PiBubbleMissionControl`.

## Notes on performance

- OpenCV pre-processing is already native underneath Python, so the first C move that makes sense is blob extraction and centroid filtering.
- The frontend is exported to static files, so there is no app-side HTTP requirement for the operator.
- The legacy GUI blob pipeline now tries the native detector first, which gives you a real speed path even before full migration.
- If you want the next big speed gain after this, the next places to attack are dense optical flow and frame transport.

## UX direction

The new UI intentionally drops the old tab maze. The design is closer to a control room:

- primary state always visible
- commands in one action rack
- detection viewport front and center
- calibration and warning state surfaced in the main scan path

That gives the engineering-console feel you asked for, but as a proper local app.
