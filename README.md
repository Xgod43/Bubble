# Pi Hardware Test Workspace

This workspace contains Raspberry Pi hardware tests and a merged GUI in `all_in_one_gui.py`.

## All-in-One GUI

Run:

```bash
python3 all_in_one_gui.py
```

The GUI now includes:

1. Live Detection (dot pipeline, detection-first page)
2. Optical Flow (live deformation vectors)
3. Camera test
4. Limit switch monitor
5. Stepper move test
6. Pressure sensor read
7. Load cell calibration/read

## Python Dependencies

Install the common dependencies first:

```bash
pip install pillow opencv-python numpy
```

On Raspberry Pi, you may also need:

```bash
pip install picamera2 picamzero adafruit-circuitpython-mprls
```

GPIO-based tabs require `RPi.GPIO` on Pi OS.

## Blob Tab Notes

- Blob detection is integrated from `JNR/dot_pipeline.py`.
- The preview is embedded inside the GUI tab (no OpenCV popup windows).
- If Camera Preview is active, starting Blob Test automatically stops Camera Preview first.
- Snapshot files are saved under `logs/blob`.
- Detection tab includes a `Settings` button for advanced camera/pipeline tuning.
- Dot tab now includes advanced controls from `dot_pipeline.py`:
	- backend/index, camera size, autofocus/lens/exposure
	- polarity, area/circularity filtering, ROI
	- processing scale, match distance, mosaic scale
	- point cloud + mosaic toggles
	- periodic robustness-test logging
	- reference reset while running

## Optical Flow Tab Notes

- Optical flow is embedded in GUI as a live page with vector overlay.
- Includes runtime metrics: tracked points, mean/max displacement, and FPS.
- Includes a `Settings` button for camera and tracker tuning.
- Flow and detection are camera-exclusive: starting one stops the other.

## Pressure-to-Force Calibration

To calibrate pressure against load cell and build a pressure -> force model:

```bash
python3 pressure_loadcell_calibration.py
```

What it does:

- Tares and calibrates HX711 with a known mass.
- Captures multiple synchronized points of MPRLS pressure + load-cell force.
- Fits a linear model: `F(N) = a * P(hPa) + b`.
- Estimates effective area from slope (`A ~= a / 100`).
- Saves CSV points + JSON model under `logs/pressure_force_calibration/`.

## Log Tools

- `Clear` removes the visible log buffer.
- `Save...` exports the current log view to a text file.
