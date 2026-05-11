Pi Bubble single-folder runtime

How to run
1) Open terminal in this folder
2) On Raspberry Pi 5 Bookworm, install system packages:
   sudo apt update
   sudo apt install -y python3-venv python3-tk python3-pil python3-pil.imagetk python3-numpy python3-opencv python3-picamera2 python3-rpi-lgpio python3-libgpiod libgpiod-dev i2c-tools gcc pkg-config
3) Create a venv that can see apt packages:
   ./tools/setup_pi_venv.sh
4) Check dependencies:
   .venv/bin/python tools/pi5_bookworm_check.py
5) Run: ./launch_bubble_app.sh
   or on Windows: launch_bubble_app.bat

Notes
- Raspberry Pi 5 Bookworm needs python3-rpi-lgpio for the RPi.GPIO-compatible GPIO API.
- If apt reports a conflict with python3-rpi.gpio, remove python3-rpi.gpio and install python3-rpi-lgpio.
- Run native/build_pi.sh to build the native blob detector and native stepper runner.
- Load-cell support includes library/hx711py/hx711v0_5_1.py; confirm HX711 wiring before testing.
