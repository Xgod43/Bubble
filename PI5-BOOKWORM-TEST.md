# Raspberry Pi 5 Bookworm Test Checklist

Status after the GUI layout fix: the Python/Tk console is ready for first Pi-side smoke testing, but the hardware path still depends on Pi packages and real wiring checks.

## Recommended OS Setup

Use Raspberry Pi OS Bookworm 64-bit with desktop. In Raspberry Pi Configuration or `raspi-config`, enable:

- Camera
- I2C

Install system packages:

```bash
sudo apt update
sudo apt install -y \
  python3-venv python3-tk python3-pil python3-pil.imagetk \
  python3-numpy python3-opencv python3-picamera2 \
  python3-rpi-lgpio python3-libgpiod libgpiod-dev \
  i2c-tools gcc pkg-config npm
```

On Raspberry Pi 5, do not rely on the old direct `RPi.GPIO` package for GPIO. This repo imports the `RPi.GPIO` API, so install `python3-rpi-lgpio`, which provides a compatible module backed by the newer GPIO stack.
If apt reports a conflict with `python3-rpi.gpio`, remove the old package first and then install `python3-rpi-lgpio`.

## Python Environment

Create a venv that can see the apt-installed camera/OpenCV/GPIO packages:

```bash
cd ~/Bubble
./tools/setup_pi_venv.sh
```

The setup script avoids Bookworm's externally-managed Python restriction (PEP 668)
by installing pip packages into `.venv`, while `--system-site-packages` keeps
apt-installed camera/OpenCV/GPIO packages visible.

## Smoke Check

Run this before touching hardware:

```bash
.venv/bin/python tools/pi5_bookworm_check.py
./launch_bubble_app.sh
```

The GUI should open with no overlapping panels. `main.py` currently launches the Python/Tk mission-control GUI first.

## Optional Builds

Native detector acceleration:
Native stepper timing is built by the same script when `libgpiod-dev` is installed.

```bash
cd native
chmod +x build_pi.sh
./build_pi.sh
cd ..
```

## Hardware Test Order

1. Open GUI only.
2. Start camera/detection with backend `picamera2`.
3. For repeatable surface/deformation measurements, choose preset `measurement`, start detection, wait for the image to settle, then press Reset Ref before applying load. This preset locks Pi Cam 3 focus/exposure/gain/AWB controls instead of letting autofocus and auto exposure drift during the run.
4. Test limit switches on GPIO17 and GPIO27.
5. Test stepper with motor power disabled first, then enabled.
6. Test pressure sensor after confirming I2C with `i2cdetect -y 1`.
7. Test load cell on GPIO5/GPIO6 after confirming HX711 wiring.
8. With pressure and load-cell readings both running, open Force Calibration, capture at least two stable pressure/load pairs at different loads, then fit the calibration. The model is `F = aP + b`, where `F` is force in newtons and `P` is pressure in hPa. The fitted model is saved to `force_calibration.json` and loaded on the next launch.

## Current Known Risks

- Stepper pulses are generated from Python timing. It is acceptable for a first functional test, but jitter should be measured before production use.
- If `native/build/stepper_runner` exists, stepper pulse generation uses the native C runner through libgpiod. Without it, the GUI falls back to Python timing.
- Pressure sensor support needs `adafruit-blinka` and `adafruit-circuitpython-mprls` from the pip requirements file.
- Re-calibrate if the gripper, tubing, sensor mounting, or load-cell setup changes.
