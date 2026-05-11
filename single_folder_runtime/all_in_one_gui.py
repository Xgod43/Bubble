import csv
import base64
import json
import queue
import os
import subprocess
import sys
import threading
import time
from pathlib import Path
from urllib.parse import urlencode
from urllib.request import Request, urlopen

import tkinter as tk
import numpy as np
from tkinter import filedialog, messagebox, ttk
from tkinter.scrolledtext import ScrolledText
from surface_measurement import (
    ContactMeasurementConfig,
    SurfaceGeometry,
    measure_camera_to_surface_mm,
    measure_contact_deformation_mm,
)
from tactile_contact_pipeline import (
    TactileContactConfig,
    build_tactile_contact_frame,
    summarize_contact_map,
    surface_from_contact_map,
)

try:
    from PIL import Image, ImageTk
except Exception:
    Image = None
    ImageTk = None

try:
    import RPi.GPIO as GPIO
except Exception:
    GPIO = None


LIMIT_1_PIN = 17
LIMIT_2_PIN = 27

STEPPER_PUL_PIN = 24
STEPPER_DIR_PIN = 23
STEPPER_ENA_PIN = 26
# Driver setting: microstep 8 on a 200-step motor => 1600 pulses per revolution.
STEPPER_PULSES_PER_REV = 1600
STEPPER_FREQUENCY_HZ = STEPPER_PULSES_PER_REV
STEPPER_PULSE_DELAY = 1.0 / (2.0 * STEPPER_FREQUENCY_HZ)
STEPPER_DEPTH_ENABLED = os.environ.get("BUBBLE_STEPPER_DEPTH_ENABLED", "0").strip().lower() in {
    "1",
    "true",
    "yes",
    "on",
}
STEPPER_MM_PER_REV = float(os.environ.get("BUBBLE_STEPPER_MM_PER_REV", "8.0"))
STEPPER_MM_PER_PULSE = STEPPER_MM_PER_REV / STEPPER_PULSES_PER_REV
FORCE_CYCLE_STEPPER_FREQUENCY_HZ = float(
    os.environ.get("BUBBLE_FORCE_CYCLE_FREQUENCY_HZ", str(STEPPER_FREQUENCY_HZ * 0.5))
)

LOADCELL_DT_PIN = 5
LOADCELL_SCK_PIN = 6
LOADCELL_CALIBRATION_DELAY_S = 8.0
LOADCELL_CALIBRATION_FILENAME = "loadcell_calibration.json"
LOADCELL_READ_SAMPLES = int(os.environ.get("BUBBLE_LOADCELL_READ_SAMPLES", "9"))
LOADCELL_FILTER_WINDOW = int(os.environ.get("BUBBLE_LOADCELL_FILTER_WINDOW", "5"))
LOADCELL_STABLE_NOISE_KG = float(os.environ.get("BUBBLE_LOADCELL_STABLE_NOISE_KG", "0.12"))
FORCE_GRAVITY_MPS2 = 9.80665
FORCE_CALIBRATION_FILENAME = "force_calibration.json"
FORCE_GRAPH_HISTORY_SECONDS = 120.0
FORCE_GRAPH_MAX_POINTS = 600
FORCE_CYCLE_SAMPLE_MIN_INTERVAL_SECONDS = 0.03
FORCE_PRESSURE_DELTA_MIN_HPA = 0.25
FORCE_CALIBRATION_MIN_R2 = float(os.environ.get("BUBBLE_FORCE_CALIBRATION_MIN_R2", "0.45"))
FORCE_CALIBRATION_MAX_ZERO_FORCE_N = float(
    os.environ.get("BUBBLE_FORCE_CALIBRATION_MAX_ZERO_N", "5.0")
)
FORCE_CALIBRATION_MIN_SAMPLE_FORCE_N = float(
    os.environ.get("BUBBLE_FORCE_CALIBRATION_MIN_SAMPLE_N", "-0.50")
)
FORCE_CYCLE_LIMIT_MAX_SECONDS = float(os.environ.get("BUBBLE_FORCE_CYCLE_LIMIT_SECONDS", "45.0"))
FORCE_CYCLE_SETTLE_SECONDS = float(os.environ.get("BUBBLE_FORCE_CYCLE_SETTLE_SECONDS", "0.25"))
FORCE_DEFAULT_PRESSURE_ZERO_HPA = 1032.0
REMOTE_VISION_DEFAULT_URL = os.environ.get("BUBBLE_REMOTE_VISION_URL", "http://192.168.4.2:8765")
REMOTE_VISION_TIMEOUT_SECONDS = float(os.environ.get("BUBBLE_REMOTE_VISION_TIMEOUT_SECONDS", "3.5"))
REMOTE_VISION_TARGET_FPS = float(os.environ.get("BUBBLE_REMOTE_VISION_TARGET_FPS", "8.0"))
REMOTE_VISION_CAPTURE_WIDTH = int(os.environ.get("BUBBLE_REMOTE_VISION_CAPTURE_WIDTH", "640"))
REMOTE_VISION_CAPTURE_HEIGHT = int(os.environ.get("BUBBLE_REMOTE_VISION_CAPTURE_HEIGHT", "480"))
REMOTE_VISION_JPEG_QUALITY = int(os.environ.get("BUBBLE_REMOTE_VISION_JPEG_QUALITY", "62"))
REMOTE_VISION_TRANSPORT = os.environ.get("BUBBLE_REMOTE_VISION_TRANSPORT", "raw").strip().lower()
REMOTE_VISION_RAW_FORMAT = os.environ.get("BUBBLE_REMOTE_VISION_RAW_FORMAT", "bgr24").strip().lower()
REMOTE_VISION_PREVIEW_MAX_WIDTH = int(os.environ.get("BUBBLE_REMOTE_VISION_PREVIEW_MAX_WIDTH", "480"))
REMOTE_VISION_SURFACE_PROCESS_WIDTH = int(os.environ.get("BUBBLE_REMOTE_SURFACE_PROCESS_WIDTH", "320"))
REMOTE_VISION_POINTS_ENABLED = os.environ.get("BUBBLE_REMOTE_VISION_POINTS", "0").strip().lower() in {
    "1",
    "true",
    "yes",
    "on",
}
REMOTE_VISION_PREVIEW_ENABLED = os.environ.get("BUBBLE_REMOTE_VISION_PREVIEW", "0").strip().lower() in {
    "1",
    "true",
    "yes",
    "on",
}
CAMERA_DEFORM_CALIBRATION_FILENAME = "camera_deform_calibration.json"
CAMERA_DEFORM_FEATURE_NAMES = (
    "pressure_delta_hpa",
    "dot_mean_px",
    "dot_max_px",
    "missing_ratio",
    "contact_peak",
    "contact_top_mean",
    "contact_area_ratio",
    "residual_mean_px",
    "residual_peak_px",
    "surface_scale_px",
)
CAMERA_DEFORM_CAPTURE_SECONDS = float(os.environ.get("BUBBLE_CAMERA_DEFORM_CAPTURE_SECONDS", "1.5"))
CAMERA_DEFORM_CAPTURE_MIN_FRAMES = int(os.environ.get("BUBBLE_CAMERA_DEFORM_CAPTURE_MIN_FRAMES", "5"))
PRESSURE_CONTACT_GATE_ENABLED = os.environ.get("BUBBLE_PRESSURE_CONTACT_GATE", "1").strip().lower() not in {
    "0",
    "false",
    "no",
    "off",
}
PRESSURE_CONTACT_BASELINE_SAMPLES = int(os.environ.get("BUBBLE_PRESSURE_BASELINE_SAMPLES", "6"))
PRESSURE_CONTACT_MIN_DELTA_HPA = float(os.environ.get("BUBBLE_PRESSURE_CONTACT_DELTA_HPA", "1.0"))
PRESSURE_CONTACT_RELEASE_DELTA_HPA = float(os.environ.get("BUBBLE_PRESSURE_RELEASE_DELTA_HPA", "0.45"))
PRESSURE_CONTACT_CONFIRM_SAMPLES = int(os.environ.get("BUBBLE_PRESSURE_CONTACT_CONFIRM_SAMPLES", "2"))
PRESSURE_CONTACT_RELEASE_SAMPLES = int(os.environ.get("BUBBLE_PRESSURE_RELEASE_SAMPLES", "3"))
PRESSURE_CONTACT_NOISE_SIGMA = float(os.environ.get("BUBBLE_PRESSURE_CONTACT_NOISE_SIGMA", "5.0"))
MPRLS_I2C_BUS = 1
MPRLS_I2C_ADDR = 0x18
MPRLS_PSI_MIN = 0.0
MPRLS_PSI_MAX = 25.0
MPRLS_PSI_TO_HPA = 68.947572932

if GPIO is None:
    GPIO_HIGH = 1
    GPIO_LOW = 0
else:
    GPIO_HIGH = GPIO.HIGH
    GPIO_LOW = GPIO.LOW

DIR_UP_STATE = GPIO_HIGH
DIR_DOWN_STATE = GPIO_LOW
STEPPER_ENA_ACTIVE_STATE = GPIO_HIGH
STEPPER_ENA_INACTIVE_STATE = GPIO_LOW

LOG_MAX_LINES = 800
BLOB_PREVIEW_MAX_WIDTH = 860
BLOB_PREVIEW_MAX_HEIGHT = 500

COLOR_BG = "#f4f7f8"
COLOR_PANEL = "#ffffff"
COLOR_ACCENT = "#0b6e69"
COLOR_ACCENT_SOFT = "#d9efee"
COLOR_TEXT = "#1f2a30"
COLOR_MUTED = "#55636d"
COLOR_BORDER = "#d7e0e2"
COLOR_OK = "#1f8f63"
COLOR_WARN = "#c97a00"

BUBBLE_WIDTH_MM = 120.0
BUBBLE_HEIGHT_MM = 80.0
BUBBLE_MAX_HEIGHT_MM = 25.0
CAMERA_TO_ACRYLIC_MM = 62.4
# Positive means camera-to-surface = acrylic gap + outward bubble height.
BUBBLE_CAMERA_DISTANCE_SIGN = 1.0
SURFACE_CONTACT_DEADBAND_MM = 0.35
SURFACE_CONTACT_NOISE_CAP_MM = 0.95
SURFACE_BASELINE_ALPHA = 0.025
SURFACE_CONTACT_HOLD_MM = 0.75
CONTACT_ZERO_SAMPLE_COUNT = 8


class LinuxMPRLSSensor:
    """Minimal Linux I2C fallback for Honeywell/Adafruit MPRLS sensors."""

    _STATUS_BUSY = 0x20
    _STATUS_MEMORY_ERROR = 0x04
    _STATUS_MATH_SAT = 0x02
    _COMMAND = [0xAA, 0x00, 0x00]
    _OUTPUT_MIN = 0.10 * 16777216.0
    _OUTPUT_MAX = 0.90 * 16777216.0

    def __init__(self, bus_number=MPRLS_I2C_BUS, addr=MPRLS_I2C_ADDR):
        try:
            from smbus2 import SMBus, i2c_msg
        except Exception as exc:
            raise RuntimeError("smbus2 is required for direct MPRLS fallback.") from exc

        bus_path = Path(f"/dev/i2c-{int(bus_number)}")
        if not bus_path.exists():
            raise RuntimeError(f"{bus_path} is missing. Enable I2C in raspi-config.")

        self.bus_number = int(bus_number)
        self.addr = int(addr)
        self._SMBus = SMBus
        self._i2c_msg = i2c_msg
        self._bus = SMBus(self.bus_number)

    def close(self):
        try:
            self._bus.close()
        except Exception:
            pass

    @property
    def pressure(self):
        write = self._i2c_msg.write(self.addr, self._COMMAND)
        self._bus.i2c_rdwr(write)

        deadline = time.monotonic() + 0.12
        last_status = 0
        data = None
        while time.monotonic() < deadline:
            read = self._i2c_msg.read(self.addr, 4)
            self._bus.i2c_rdwr(read)
            data = list(read)
            last_status = int(data[0])
            if not (last_status & self._STATUS_BUSY):
                break
            time.sleep(0.005)

        if data is None or (last_status & self._STATUS_BUSY):
            raise TimeoutError("MPRLS conversion timed out.")
        if last_status & self._STATUS_MEMORY_ERROR:
            raise RuntimeError("MPRLS status reported memory error.")
        if last_status & self._STATUS_MATH_SAT:
            raise RuntimeError("MPRLS status reported math saturation.")

        raw_counts = (int(data[1]) << 16) | (int(data[2]) << 8) | int(data[3])
        pressure_psi = (
            ((raw_counts - self._OUTPUT_MIN) * (MPRLS_PSI_MAX - MPRLS_PSI_MIN))
            / (self._OUTPUT_MAX - self._OUTPUT_MIN)
        ) + MPRLS_PSI_MIN
        return pressure_psi * MPRLS_PSI_TO_HPA


class AllInOneTesterGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Soft Gripper bubble hardware console")
        self.root.geometry("1100x820")
        self.root.minsize(980, 720)

        self.root.configure(bg=COLOR_BG)
        self._configure_styles()

        self.log_queue = queue.Queue()
        self.gpio_ready = GPIO is not None

        self.camera = None
        self.picam2 = None
        self.camera_running = False

        self.limit_thread = None
        self.limit_stop_event = threading.Event()
        self.limit_state_lock = threading.Lock()
        self.limit_trigger_state = (False, False)
        self.limit_state_sampled = False

        self.stepper_thread = None
        self.stepper_stop_event = threading.Event()
        self.stepper_stop_reason = None
        self.stepper_active_direction = None
        self.stepper_limit_guard_disabled = False
        self.stepper_position_mm = 0.0
        self.stepper_contact_zero_mm = None

        self.pressure_thread = None
        self.pressure_stop_event = threading.Event()

        self.loadcell_thread = None
        self.loadcell_stop_event = threading.Event()
        self.loadcell_hx = None
        self.force_cycle_thread = None
        self.force_cycle_stop_event = threading.Event()

        self.blob_thread = None
        self.blob_stop_event = threading.Event()
        self.blob_snapshot_event = threading.Event()
        self.blob_frame_queue = queue.Queue(maxsize=2)
        self.blob_camera = None
        self.blob_module = None
        self.blob_cv2 = None
        self.blob_params = None
        self.blob_photo = None

        self.camera_start_btn = None
        self.camera_stop_btn = None
        self.limit_start_btn = None
        self.limit_stop_btn = None
        self.stepper_start_btn = None
        self.stepper_stop_btn = None
        self.pressure_start_btn = None
        self.pressure_stop_btn = None
        self.loadcell_start_btn = None
        self.loadcell_stop_btn = None
        self.loadcell_zero_btn = None
        self.loadcell_calibrate_btn = None
        self.loadcell_reset_btn = None
        self.force_cycle_start_btn = None
        self.force_cycle_stop_btn = None
        self.blob_start_btn = None
        self.blob_stop_btn = None
        self.blob_snapshot_btn = None
        self.blob_reset_ref_btn = None
        self.surface_reset_btn = None
        self.deform_calibrate_btn = None
        self.deform_fit_btn = None
        self.deform_reset_cal_btn = None
        self.deform_known_mm_entry = None
        self.deform_mode_combo = None
        self.blob_reset_reference_event = threading.Event()
        self.surface_reset_zero_event = threading.Event()

        self.flow_thread = None
        self.flow_stop_event = threading.Event()
        self.flow_frame_queue = queue.Queue(maxsize=2)
        self.flow_photo = None
        self.flow_params = None
        self.flow_camera = None
        self.flow_start_btn = None
        self.flow_stop_btn = None
        self.flow_settings_btn = None
        self.flow_state_var = tk.StringVar(value="Merged into Live Detection")
        self.flow_points_var = tk.StringVar(value="-")
        self.flow_mean_disp_var = tk.StringVar(value="-")
        self.flow_max_disp_var = tk.StringVar(value="-")
        self.flow_fps_var = tk.StringVar(value="-")
        self.flow_message_var = tk.StringVar(value="Use Live Detection output type: optical_flow_2d.")
        self.stepper_position_var = tk.StringVar(value="0.000 mm")
        self.contact_depth_stepper_var = tk.StringVar(
            value="-" if STEPPER_DEPTH_ENABLED else "unavailable (manual)"
        )
        self.contact_depth_camera_var = tk.StringVar(value="-")
        self.contact_gate_var = tk.StringVar(value="Pressure gate waiting")
        self.flow_camera_backend_var = tk.StringVar(value="auto")
        self.flow_camera_index_var = tk.StringVar(value="0")
        self.flow_cam_width_var = tk.StringVar(value="1280")
        self.flow_cam_height_var = tk.StringVar(value="960")
        self.flow_proc_scale_var = tk.StringVar(value="1.0")
        self.flow_roi_scale_var = tk.StringVar(value="0.96")
        self.flow_3d_roi_scale_var = tk.StringVar(value="1.0")
        self.flow_max_points_var = tk.StringVar(value="260")
        self.flow_quality_var = tk.StringVar(value="0.02")
        self.flow_min_distance_var = tk.StringVar(value="7")
        self.flow_reseed_var = tk.StringVar(value="90")
        self.flow_show_vectors_var = tk.BooleanVar(value=True)

        self.surface_gain_var = tk.StringVar(value="1.0")
        self.surface_smooth_var = tk.StringVar(value="1.4")
        self.surface_grid_var = tk.StringVar(value="52")
        self.surface_status_var = tk.StringVar(
            value="Contact deform zero pending. Press Reset Zero."
        )
        self.surface_scale_ema = None
        self.surface_height_ema = None
        self.surface_reference_height_map = None
        self.surface_contact_ema = None
        self.surface_zero_ready = False
        self.surface_tactile_frame = None
        self.surface_contact_baseline = None
        self.surface_contact_display_ema = None
        self.surface_measurement_mode = "flat_deform"
        self.surface_measurement_mode_var = tk.StringVar(value="Flat Deform")
        self.contact_peak_var = tk.StringVar(value="-")
        self.contact_area_var = tk.StringVar(value="-")
        self.contact_center_var = tk.StringVar(value="-")
        self.contact_residual_var = tk.StringVar(value="-")
        self.contact_force_var = tk.StringVar(value="-")
        self.camera_deform_known_mm_var = tk.StringVar(value="0.0")
        self.camera_deform_estimate_var = tk.StringVar(value="-")
        self.camera_deform_sample_count_var = tk.StringVar(value="0 samples")
        self.camera_deform_model_var = tk.StringVar(value="-")
        self.camera_deform_live_feature_var = tk.StringVar(value="-")
        self.camera_deform_status_var = tk.StringVar(value="No camera deform calibration.")
        self.camera_deform_samples = []
        self.camera_deform_model = None
        self.camera_deform_latest_features = None
        self.camera_deform_capture_active = False
        self.camera_deform_capture_target_mm = None
        self.camera_deform_capture_started_at = None
        self.camera_deform_capture_buffer = []

        self._init_force_calibration_state()
        self._init_camera_deform_calibration_state()
        self._build_ui()
        self._setup_gpio_once()
        self.root.after(100, self._drain_log_queue)
        self.root.after(60, self._drain_blob_frame_queue)
        self.root.after(500, self._update_clock)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self._set_system_status("Ready")
        self.log("GUI initialized.")

    def _configure_styles(self):
        style = ttk.Style(self.root)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass

        style.configure("App.TFrame", background=COLOR_BG)
        style.configure("HeaderCard.TFrame", background=COLOR_PANEL, relief="flat")
        style.configure("HeaderTitle.TLabel", background=COLOR_PANEL, foreground=COLOR_TEXT, font=("Segoe UI", 20, "bold"))
        style.configure("HeaderSubtitle.TLabel", background=COLOR_PANEL, foreground=COLOR_MUTED, font=("Segoe UI", 10))
        style.configure("SectionTitle.TLabel", background=COLOR_PANEL, foreground=COLOR_TEXT, font=("Segoe UI", 10, "bold"))
        style.configure("SectionHint.TLabel", background=COLOR_PANEL, foreground=COLOR_MUTED, font=("Segoe UI", 9))
        style.configure("StatusLabel.TLabel", background=COLOR_PANEL, foreground=COLOR_MUTED, font=("Segoe UI", 9, "bold"))
        style.configure("StatusReady.TLabel", background=COLOR_ACCENT_SOFT, foreground=COLOR_ACCENT, font=("Segoe UI", 10, "bold"), padding=(10, 4))
        style.configure("StatusActive.TLabel", background="#e6f5ed", foreground=COLOR_OK, font=("Segoe UI", 10, "bold"), padding=(10, 4))
        style.configure("StatusWarn.TLabel", background="#fff1dc", foreground=COLOR_WARN, font=("Segoe UI", 10, "bold"), padding=(10, 4))
        style.configure("Clock.TLabel", background=COLOR_PANEL, foreground=COLOR_MUTED, font=("Consolas", 10, "bold"))

        style.configure("TLabel", foreground=COLOR_TEXT)
        style.configure("TButton", padding=(10, 6), font=("Segoe UI", 10, "bold"))
        style.map("TButton", background=[("active", COLOR_ACCENT_SOFT)])

        style.configure("TLabelframe", background=COLOR_PANEL, bordercolor=COLOR_BORDER, relief="solid")
        style.configure("TLabelframe.Label", background=COLOR_PANEL, foreground=COLOR_TEXT, font=("Segoe UI", 10, "bold"))

        style.configure("TNotebook", background=COLOR_BG, borderwidth=0)
        style.configure("TNotebook.Tab", font=("Segoe UI", 10, "bold"), padding=(14, 8))
        style.map("TNotebook.Tab", background=[("selected", COLOR_PANEL), ("!selected", "#e7edee")])

        style.configure("TPanedwindow", background=COLOR_BG)
        style.configure("Sash", sashthickness=8)

        style.configure("Accent.TButton", background=COLOR_ACCENT, foreground="#ffffff")
        style.map("Accent.TButton", background=[("active", "#0a5f5a")])

        style.configure("Danger.TButton", background="#a34b2a", foreground="#ffffff")
        style.map("Danger.TButton", background=[("active", "#8d3f23")])
        style.configure(
            "StepperDir.TButton",
            padding=(8, 4),
            background=COLOR_PANEL,
            foreground=COLOR_TEXT,
            bordercolor=COLOR_BORDER,
        )
        style.map(
            "StepperDir.TButton",
            background=[("active", COLOR_ACCENT_SOFT), ("disabled", "#edf1f2")],
            foreground=[("disabled", "#7d8a8d")],
        )
        style.configure(
            "StepperDirActive.TButton",
            padding=(8, 4),
            background=COLOR_ACCENT,
            foreground="#ffffff",
            bordercolor=COLOR_ACCENT,
        )
        style.map(
            "StepperDirActive.TButton",
            background=[("active", "#0a5f5a"), ("disabled", "#6aa89c")],
            foreground=[("disabled", "#eef8f6")],
        )

    def _build_ui(self):
        main = ttk.Frame(self.root, padding=12, style="App.TFrame")
        main.pack(fill="both", expand=True)
        main.columnconfigure(0, weight=1)
        main.rowconfigure(2, weight=1)

        header = ttk.Frame(main, padding=14, style="HeaderCard.TFrame")
        header.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        header.columnconfigure(0, weight=1)
        header.columnconfigure(1, weight=1)

        left = ttk.Frame(header, style="HeaderCard.TFrame")
        left.grid(row=0, column=0, sticky="w")

        ttk.Label(left, text="Soft Gripper bubble hardware console", style="HeaderTitle.TLabel").grid(
            row=0, column=0, sticky="w"
        )

        ttk.Label(
            left,
            text=(
                "Unified panel for camera, motion, pressure, load, and dot pipeline detection."
            ),
            style="HeaderSubtitle.TLabel",
        ).grid(row=1, column=0, sticky="w", pady=(2, 0))

        right = ttk.Frame(header, style="HeaderCard.TFrame")
        right.grid(row=0, column=1, sticky="e")
        right.columnconfigure(1, weight=1)

        ttk.Label(right, text="SYSTEM", style="StatusLabel.TLabel").grid(
            row=0, column=0, sticky="e", padx=(0, 10)
        )
        self.system_status_var = tk.StringVar(value="-")
        self.system_status_value_label = ttk.Label(
            right,
            textvariable=self.system_status_var,
            style="StatusReady.TLabel",
        )
        self.system_status_value_label.grid(row=0, column=1, sticky="e")

        self.header_clock_var = tk.StringVar(value=time.strftime("%H:%M:%S"))
        ttk.Label(right, textvariable=self.header_clock_var, style="Clock.TLabel").grid(
            row=1, column=1, sticky="e", pady=(8, 0)
        )

        action_row = ttk.Frame(main, style="App.TFrame")
        action_row.grid(row=1, column=0, sticky="ew", pady=(0, 8))
        action_row.columnconfigure(0, weight=1)

        ttk.Label(
            action_row,
            text="Quick actions",
            style="SectionHint.TLabel",
        ).grid(row=0, column=0, sticky="w", padx=(4, 0))

        quick_buttons = ttk.Frame(action_row, style="App.TFrame")
        quick_buttons.grid(row=0, column=1, sticky="e")
        ttk.Button(
            quick_buttons,
            text="Stop All",
            style="Danger.TButton",
            command=self.stop_all_tests,
        ).grid(row=0, column=0, sticky="e")
        ttk.Button(
            quick_buttons,
            text="Clear Log",
            command=self.clear_log,
        ).grid(row=0, column=1, sticky="e", padx=(8, 0))

        ttk.Button(
            quick_buttons,
            text="Save Log",
            style="Accent.TButton",
            command=self.save_log,
        ).grid(row=0, column=2, sticky="e", padx=(8, 0))

        self.log_panel_visible = True
        self.log_toggle_btn = ttk.Button(
            quick_buttons,
            text="Hide Log",
            command=self.toggle_log_panel,
        )
        self.log_toggle_btn.grid(row=0, column=3, sticky="e", padx=(8, 0))

        content_pane = ttk.Panedwindow(main, orient="vertical")
        content_pane.grid(row=2, column=0, sticky="nsew")
        self.content_pane = content_pane

        notebook_shell = ttk.Frame(main, style="App.TFrame")
        notebook_shell.columnconfigure(0, weight=1)
        notebook_shell.rowconfigure(0, weight=1)
        content_pane.add(notebook_shell, weight=5)

        notebook = ttk.Notebook(notebook_shell)
        notebook.grid(row=0, column=0, sticky="nsew")

        self._build_blob_tab(notebook)
        self._build_force_tab(notebook)
        self._build_camera_tab(notebook)
        self._build_limit_tab(notebook)
        self._build_stepper_tab(notebook)
        self._build_pressure_tab(notebook)
        self._build_loadcell_tab(notebook)
        self._build_system_tests_tab(notebook)
        self._build_session_log_panel(content_pane)

    def _build_session_log_panel(self, parent):
        panel = ttk.Frame(parent, padding=(0, 8, 0, 0), style="App.TFrame")
        panel.columnconfigure(0, weight=1)
        panel.rowconfigure(1, weight=1)
        self.log_panel = panel
        parent.add(panel, weight=2)

        log_toolbar = ttk.Frame(panel)
        log_toolbar.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        ttk.Label(
            log_toolbar,
            text="Session Log",
            style="SectionHint.TLabel",
        ).pack(side="left")

        ttk.Label(
            log_toolbar,
            text="Drag separator above to resize",
            style="SectionHint.TLabel",
        ).pack(side="left", padx=(12, 0))

        ttk.Button(log_toolbar, text="Clear", command=self.clear_log).pack(side="right")
        ttk.Button(log_toolbar, text="Save...", command=self.save_log).pack(
            side="right", padx=(0, 8)
        )

        self.log_text = ScrolledText(panel, height=10, state="disabled", wrap="word")
        self.log_text.configure(font=("Consolas", 10), bg="#f9fbfc", fg=COLOR_TEXT, relief="flat")
        self.log_text.grid(row=1, column=0, sticky="nsew")

    def toggle_log_panel(self):
        if self.log_panel_visible:
            self.content_pane.forget(self.log_panel)
            self.log_toggle_btn.configure(text="Show Log")
            self.log_panel_visible = False
            self.log("Session Log hidden.")
            return

        self.content_pane.add(self.log_panel, weight=2)
        self.log_toggle_btn.configure(text="Hide Log")
        self.log_panel_visible = True
        self.log("Session Log shown.")

    def _build_system_tests_tab(self, notebook):
        tab = ttk.Frame(notebook, padding=12)
        try:
            notebook.insert(2, tab, text="3) System Tests")
        except tk.TclError:
            notebook.add(tab, text="3) System Tests")

        tab.columnconfigure(0, weight=1)
        tab.columnconfigure(1, weight=1)

        ttk.Label(
            tab,
            text="One-page checkout for camera, vision modes, motion, pressure, and load sensors.",
            style="SectionHint.TLabel",
        ).grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 8))

        vision = ttk.LabelFrame(tab, text="Vision", padding=10)
        vision.grid(row=1, column=0, sticky="nsew", padx=(0, 6))
        vision.columnconfigure(1, weight=1)

        hardware = ttk.LabelFrame(tab, text="Hardware", padding=10)
        hardware.grid(row=1, column=1, sticky="nsew", padx=(6, 0))
        hardware.columnconfigure(1, weight=1)

        def add_row(parent, row, name, status_var, actions, detail_var=None):
            ttk.Label(parent, text=name + ":").grid(row=row, column=0, sticky="w", pady=(5, 0))
            ttk.Label(parent, textvariable=status_var).grid(row=row, column=1, sticky="w", pady=(5, 0))
            action_frame = ttk.Frame(parent)
            action_frame.grid(row=row, column=2, sticky="e", pady=(5, 0), padx=(8, 0))
            for idx, (label, command) in enumerate(actions):
                ttk.Button(action_frame, text=label, command=command).grid(
                    row=0,
                    column=idx,
                    sticky="ew",
                    padx=(0 if idx == 0 else 6, 0),
                )
            if detail_var is not None:
                ttk.Label(parent, textvariable=detail_var, style="SectionHint.TLabel").grid(
                    row=row + 1,
                    column=1,
                    columnspan=2,
                    sticky="w",
                )

        add_row(
            vision,
            0,
            "Camera",
            self.camera_status_var,
            (("Open", self.start_camera), ("Stop", self.stop_camera)),
        )
        add_row(
            vision,
            2,
            "Dot pipeline",
            self.blob_state_var,
            (("Start", self.start_blob_test), ("Stop", self.stop_blob_test)),
            detail_var=self.blob_message_var,
        )
        add_row(
            hardware,
            0,
            "Limit switches",
            self.limit_monitor_state_var,
            (("Start", self.start_limit_monitor), ("Stop", self.stop_limit_monitor)),
            detail_var=self.limit1_detail_var,
        )
        ttk.Label(hardware, textvariable=self.limit2_detail_var, style="SectionHint.TLabel").grid(
            row=2,
            column=1,
            columnspan=2,
            sticky="w",
        )
        add_row(
            hardware,
            3,
            "Stepper",
            self.stepper_state_var,
            (("Move", self.start_stepper_move), ("Stop", self.stop_stepper_move)),
        )
        add_row(
            hardware,
            5,
            "Pressure",
            self.pressure_state_var,
            (("Start", self.start_pressure_read), ("Stop", self.stop_pressure_read)),
            detail_var=self.pressure_value_var,
        )
        add_row(
            hardware,
            7,
            "Load cell",
            self.loadcell_state_var,
            (("Start", self.start_loadcell_read), ("Stop", self.stop_loadcell_read)),
            detail_var=self.loadcell_weight_var,
        )

        footer = ttk.Frame(tab)
        footer.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(12, 0))
        footer.columnconfigure(0, weight=1)
        ttk.Label(footer, textvariable=self.system_status_var, style="SectionHint.TLabel").grid(
            row=0,
            column=0,
            sticky="w",
        )
        ttk.Button(footer, text="Stop All", style="Danger.TButton", command=self.stop_all_tests).grid(
            row=0,
            column=1,
            sticky="e",
        )

    def _build_camera_tab(self, notebook):
        tab = ttk.Frame(notebook, padding=12)
        notebook.add(tab, text="4) Camera Test")
        tab.columnconfigure(2, weight=1)

        self.camera_status_var = tk.StringVar(value="Stopped")

        ttk.Label(
            tab,
            text="Open camera preview. Use Stop to close the preview.",
        ).grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 8))

        self.camera_start_btn = ttk.Button(tab, text="Open Camera", command=self.start_camera)
        self.camera_start_btn.grid(
            row=1, column=0, sticky="w"
        )
        self.camera_stop_btn = ttk.Button(tab, text="Stop Camera", command=self.stop_camera)
        self.camera_stop_btn.grid(
            row=1, column=1, sticky="w", padx=(8, 0)
        )

        ttk.Label(tab, text="Status:").grid(row=2, column=0, sticky="w", pady=(10, 0))
        ttk.Label(tab, textvariable=self.camera_status_var).grid(
            row=2, column=1, sticky="w", pady=(10, 0)
        )
        self._set_camera_controls(False)

    def _build_limit_tab(self, notebook):
        tab = ttk.Frame(notebook, padding=12)
        notebook.add(tab, text="5) Limit Switch Test")
        tab.columnconfigure(2, weight=1)

        self.limit1_status_var = tk.StringVar(value="not triggered")
        self.limit2_status_var = tk.StringVar(value="not triggered")
        self.limit1_detail_var = tk.StringVar(value="Limit 1: not triggered")
        self.limit2_detail_var = tk.StringVar(value="Limit 2: not triggered")
        self.limit_monitor_state_var = tk.StringVar(value="Stopped")

        ttk.Label(
            tab,
            text=(
                f"Monitoring GPIO{LIMIT_1_PIN} and GPIO{LIMIT_2_PIN}. "
                "When pressed, status changes to TRIGGERED."
            ),
        ).grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 8))

        self.limit_start_btn = ttk.Button(
            tab, text="Start Monitor", command=self.start_limit_monitor
        )
        self.limit_start_btn.grid(
            row=1, column=0, sticky="w"
        )
        self.limit_stop_btn = ttk.Button(tab, text="Stop Monitor", command=self.stop_limit_monitor)
        self.limit_stop_btn.grid(
            row=1, column=1, sticky="w", padx=(8, 0)
        )

        ttk.Label(tab, text="Monitor:").grid(row=2, column=0, sticky="w", pady=(10, 0))
        ttk.Label(tab, textvariable=self.limit_monitor_state_var).grid(
            row=2, column=1, sticky="w", pady=(10, 0)
        )

        ttk.Label(tab, text="Limit 1:").grid(row=3, column=0, sticky="w", pady=(10, 0))
        ttk.Label(tab, textvariable=self.limit1_status_var).grid(
            row=3, column=1, sticky="w", pady=(10, 0)
        )

        ttk.Label(tab, text="Limit 2:").grid(row=4, column=0, sticky="w", pady=(6, 0))
        ttk.Label(tab, textvariable=self.limit2_status_var).grid(
            row=4, column=1, sticky="w", pady=(6, 0)
        )
        self._set_limit_controls(False)

    def _build_stepper_tab(self, notebook):
        tab = ttk.Frame(notebook, padding=12)
        notebook.add(tab, text="6) Stepper Test")
        tab.columnconfigure(4, weight=1)

        self.stepper_direction_var = tk.StringVar(value="up")
        self.stepper_seconds_var = tk.StringVar(value="1.0")
        self.stepper_state_var = tk.StringVar(value="Ready")

        ttk.Label(
            tab,
            text=(
                "Command-style move: select up or down, enter seconds, then click Move. "
                "(Pins: PUL=GPIO24, DIR=GPIO23, ENA=GPIO26, pulses/rev=1600)"
            ),
        ).grid(row=0, column=0, columnspan=4, sticky="w", pady=(0, 8))

        ttk.Label(tab, text="Direction:").grid(row=1, column=0, sticky="w")
        self._build_stepper_direction_buttons(tab, row=1, column=1, sticky="w")

        ttk.Label(tab, text="Seconds:").grid(row=1, column=2, sticky="w", padx=(16, 0))
        ttk.Entry(tab, textvariable=self.stepper_seconds_var, width=10).grid(
            row=1, column=3, sticky="w"
        )

        self.stepper_start_btn = ttk.Button(tab, text="Move", command=self.start_stepper_move)
        self.stepper_start_btn.grid(
            row=2, column=0, sticky="w", pady=(10, 0)
        )
        self.stepper_stop_btn = ttk.Button(tab, text="Stop", command=self.stop_stepper_move)
        self.stepper_stop_btn.grid(
            row=2, column=1, sticky="w", pady=(10, 0), padx=(8, 0)
        )

        ttk.Label(tab, text="State:").grid(row=3, column=0, sticky="w", pady=(10, 0))
        ttk.Label(tab, textvariable=self.stepper_state_var).grid(
            row=3, column=1, sticky="w", pady=(10, 0)
        )
        ttk.Label(tab, text="Position:").grid(row=4, column=0, sticky="w", pady=(6, 0))
        ttk.Label(tab, textvariable=self.stepper_position_var).grid(
            row=4, column=1, sticky="w", pady=(6, 0)
        )
        ttk.Label(tab, text="Press depth:").grid(row=5, column=0, sticky="w", pady=(6, 0))
        ttk.Label(tab, textvariable=self.contact_depth_stepper_var).grid(
            row=5, column=1, sticky="w", pady=(6, 0)
        )
        self._set_stepper_controls(False)

    def _build_pressure_tab(self, notebook):
        tab = ttk.Frame(notebook, padding=12)
        notebook.add(tab, text="7) Pressure Test")
        tab.columnconfigure(2, weight=1)

        self.pressure_value_var = tk.StringVar(value="-")
        self.pressure_state_var = tk.StringVar(value="Stopped")

        ttk.Label(
            tab,
            text="Reads pressure from MPRLS and displays value in hPa.",
        ).grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 8))

        self.pressure_start_btn = ttk.Button(
            tab, text="Start Reading", command=self.start_pressure_read
        )
        self.pressure_start_btn.grid(
            row=1, column=0, sticky="w"
        )
        self.pressure_stop_btn = ttk.Button(
            tab, text="Stop Reading", command=self.stop_pressure_read
        )
        self.pressure_stop_btn.grid(
            row=1, column=1, sticky="w", padx=(8, 0)
        )

        ttk.Label(tab, text="State:").grid(row=2, column=0, sticky="w", pady=(10, 0))
        ttk.Label(tab, textvariable=self.pressure_state_var).grid(
            row=2, column=1, sticky="w", pady=(10, 0)
        )

        ttk.Label(tab, text="Pressure:").grid(row=3, column=0, sticky="w", pady=(10, 0))
        ttk.Label(tab, textvariable=self.pressure_value_var).grid(
            row=3, column=1, sticky="w", pady=(10, 0)
        )
        ttk.Label(tab, text="Estimated force:").grid(row=4, column=0, sticky="w", pady=(10, 0))
        ttk.Label(tab, textvariable=self.force_estimate_var).grid(
            row=4, column=1, sticky="w", pady=(10, 0)
        )
        self._set_pressure_controls(False)

    def _build_loadcell_tab(self, notebook):
        tab = ttk.Frame(notebook, padding=12)
        notebook.add(tab, text="8) Load Cell Test")
        tab.columnconfigure(4, weight=1)

        self.loadcell_known_weight_var = tk.StringVar(value="500")
        self.loadcell_weight_var = tk.StringVar(value="-")
        self.loadcell_state_var = tk.StringVar(value="Stopped")

        ttk.Label(
            tab,
            text=(
                "Start uses the saved load-cell calibration and tares the sensor. "
                "Use Calibrate only when you want to save a new known-weight calibration."
            ),
        ).grid(row=0, column=0, columnspan=4, sticky="w", pady=(0, 8))

        ttk.Label(tab, text="Known weight (g):").grid(row=1, column=0, sticky="w")
        ttk.Entry(tab, textvariable=self.loadcell_known_weight_var, width=12).grid(
            row=1, column=1, sticky="w"
        )

        self.loadcell_start_btn = ttk.Button(tab, text="Start", command=self.start_loadcell_read)
        self.loadcell_start_btn.grid(
            row=2, column=0, sticky="w", pady=(10, 0)
        )
        self.loadcell_stop_btn = ttk.Button(tab, text="Stop", command=self.stop_loadcell_read)
        self.loadcell_stop_btn.grid(
            row=2, column=1, sticky="w", pady=(10, 0), padx=(8, 0)
        )
        self.loadcell_zero_btn = ttk.Button(tab, text="Zero Load", command=self.zero_loadcell_live)
        self.loadcell_zero_btn.grid(
            row=2, column=2, sticky="w", pady=(10, 0), padx=(8, 0)
        )
        self.loadcell_calibrate_btn = ttk.Button(
            tab, text="Cal Load", command=self.calibrate_loadcell
        )
        self.loadcell_calibrate_btn.grid(
            row=2, column=3, sticky="w", pady=(10, 0), padx=(8, 0)
        )
        self.loadcell_reset_btn = ttk.Button(
            tab, text="Reset Load", command=self.reset_loadcell_calibration
        )
        self.loadcell_reset_btn.grid(
            row=2, column=4, sticky="w", pady=(10, 0), padx=(8, 0)
        )

        ttk.Label(tab, text="State:").grid(row=3, column=0, sticky="w", pady=(10, 0))
        ttk.Label(tab, textvariable=self.loadcell_state_var).grid(
            row=3, column=1, sticky="w", pady=(10, 0)
        )

        ttk.Label(tab, text="Weight:").grid(row=4, column=0, sticky="w", pady=(10, 0))
        ttk.Label(tab, textvariable=self.loadcell_weight_var).grid(
            row=4, column=1, sticky="w", pady=(10, 0)
        )
        self._set_loadcell_controls(False)

    def _build_force_tab(self, notebook):
        tab = ttk.Frame(notebook, padding=12)
        notebook.add(tab, text="2) Force")
        tab.columnconfigure(0, weight=1)
        tab.rowconfigure(1, weight=1)

        ttk.Label(
            tab,
            text=(
                "Pressure-derived force, load-cell force comparison, and live error tracking."
            ),
        ).grid(row=0, column=0, sticky="w", pady=(0, 8))

        force_pane = ttk.Panedwindow(tab, orient="horizontal")
        force_pane.grid(row=1, column=0, sticky="nsew")

        graph_area = ttk.Frame(force_pane)
        graph_area.columnconfigure(0, weight=1)
        graph_area.rowconfigure(0, weight=2)
        graph_area.rowconfigure(1, weight=2)
        graph_area.rowconfigure(2, weight=1)
        graph_area.rowconfigure(3, weight=1)

        side_area = ttk.Frame(force_pane)
        side_area.columnconfigure(0, weight=1)

        force_pane.add(graph_area, weight=4)
        force_pane.add(side_area, weight=2)

        compare_frame = ttk.LabelFrame(graph_area, text="Force comparison", padding=8)
        compare_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 10), pady=(0, 8))
        compare_frame.columnconfigure(0, weight=1)
        compare_frame.rowconfigure(0, weight=1)
        self.force_compare_canvas = tk.Canvas(
            compare_frame,
            height=230,
            background="#f9fbfc",
            highlightthickness=1,
            highlightbackground=COLOR_BORDER,
        )
        self.force_compare_canvas.grid(row=0, column=0, sticky="nsew")
        self.force_compare_canvas.bind("<Configure>", lambda _event: self._draw_force_graphs())

        error_frame = ttk.LabelFrame(graph_area, text="Force error", padding=8)
        error_frame.grid(row=1, column=0, sticky="nsew", padx=(0, 10), pady=(8, 0))
        error_frame.columnconfigure(0, weight=1)
        error_frame.rowconfigure(0, weight=1)
        self.force_error_canvas = tk.Canvas(
            error_frame,
            height=230,
            background="#f9fbfc",
            highlightthickness=1,
            highlightbackground=COLOR_BORDER,
        )
        self.force_error_canvas.grid(row=0, column=0, sticky="nsew")
        self.force_error_canvas.bind("<Configure>", lambda _event: self._draw_force_graphs())

        deform_frame = ttk.LabelFrame(graph_area, text="Deformation monitor", padding=8)
        deform_frame.grid(row=2, column=0, sticky="nsew", padx=(0, 10), pady=(8, 0))
        deform_frame.columnconfigure(0, weight=1)
        deform_frame.rowconfigure(0, weight=1)
        self.force_deform_canvas = tk.Canvas(
            deform_frame,
            height=150,
            background="#f9fbfc",
            highlightthickness=1,
            highlightbackground=COLOR_BORDER,
        )
        self.force_deform_canvas.grid(row=0, column=0, sticky="nsew")
        self.force_deform_canvas.bind("<Configure>", lambda _event: self._draw_force_graphs())

        distance_frame = ttk.LabelFrame(graph_area, text="Distance estimate", padding=8)
        distance_frame.grid(row=3, column=0, sticky="nsew", padx=(0, 10), pady=(8, 0))
        distance_frame.columnconfigure(0, weight=1)
        distance_frame.rowconfigure(0, weight=1)
        self.force_distance_canvas = tk.Canvas(
            distance_frame,
            height=150,
            background="#f9fbfc",
            highlightthickness=1,
            highlightbackground=COLOR_BORDER,
        )
        self.force_distance_canvas.grid(row=0, column=0, sticky="nsew")
        self.force_distance_canvas.bind("<Configure>", lambda _event: self._draw_force_graphs())

        panel = ttk.LabelFrame(side_area, text="Force from pressure", padding=12)
        panel.grid(row=0, column=0, sticky="new")
        self._build_force_calibration_panel(panel)
        self.root.after(100, self._draw_force_graphs)

    def _build_blob_tab(self, notebook):
        tab = ttk.Frame(notebook, padding=12)
        notebook.add(tab, text="1) Live Detection")
        tab.columnconfigure(0, weight=1)
        tab.rowconfigure(1, weight=1)

        self.blob_state_var = tk.StringVar(value="Stopped")
        self.blob_dot_count_var = tk.StringVar(value="-")
        self.blob_pick_mode_var = tk.StringVar(value="-")
        self.blob_fps_var = tk.StringVar(value="-")
        self.blob_latency_var = tk.StringVar(value="-")
        self.blob_mean_disp_var = tk.StringVar(value="-")
        self.blob_max_disp_var = tk.StringVar(value="-")
        self.blob_missing_ratio_var = tk.StringVar(value="-")
        self.blob_new_tracks_var = tk.StringVar(value="-")
        self.blob_lost_tracks_var = tk.StringVar(value="-")
        self.blob_message_var = tk.StringVar(value="Tune controls and click Start Blob Test.")

        self.blob_camera_backend_var = tk.StringVar(value="auto")
        self.blob_camera_index_var = tk.StringVar(value="0")
        self.blob_cam_width_var = tk.StringVar(value="1280")
        self.blob_cam_height_var = tk.StringVar(value="960")
        self.blob_autofocus_var = tk.StringVar(value="continuous")
        self.blob_lens_position_var = tk.StringVar(value="1.0")
        self.blob_exposure_ev_var = tk.StringVar(value="0.8")
        self.blob_exposure_time_us_var = tk.StringVar(value="")
        self.blob_analogue_gain_var = tk.StringVar(value="")
        self.blob_awb_mode_var = tk.StringVar(value="auto")
        self.blob_colour_gains_var = tk.StringVar(value="")

        self.blob_mode_var = tk.StringVar(value="dark")
        self.blob_min_area_var = tk.StringVar(value="260")
        self.blob_max_area_var = tk.StringVar(value="3200")
        self.blob_min_circularity_var = tk.StringVar(value="0.35")
        self.blob_use_roi_var = tk.BooleanVar(value=True)
        self.blob_roi_percent_var = tk.StringVar(value="62")
        self.blob_proc_scale_var = tk.StringVar(value="0.65")
        self.blob_match_dist_var = tk.StringVar(value="9.0")
        self.blob_mosaic_scale_var = tk.StringVar(value="0.6")
        self.blob_use_pointcloud_var = tk.BooleanVar(value=False)
        self.blob_use_mosaic_var = tk.BooleanVar(value=False)
        self.blob_run_robust_test_var = tk.BooleanVar(value=False)
        self.blob_profile_var = tk.StringVar(value="fast")
        self.blob_view_type_var = tk.StringVar(value="overlay")
        self.blob_use_illum_norm_var = tk.BooleanVar(value=True)
        self.blob_illum_method_var = tk.StringVar(value="clahe")
        self.blob_distance_scale_var = tk.StringVar(value="1.0")
        self.blob_distance_unit_var = tk.StringVar(value="px")
        self.vision_backend_var = tk.StringVar(
            value=os.environ.get("BUBBLE_VISION_BACKEND", "local")
        )
        self.remote_vision_url_var = tk.StringVar(value=REMOTE_VISION_DEFAULT_URL)

        ttk.Label(
            tab,
            text=(
                "Detection is driven by JNR/dot_pipeline.py. This page is optimized for live monitoring; advanced tuning is in Settings."
            ),
        ).grid(row=0, column=0, sticky="w", pady=(0, 8))

        main_pane = ttk.Panedwindow(tab, orient="horizontal")
        main_pane.grid(row=1, column=0, sticky="nsew")

        left_panel = ttk.Frame(main_pane)
        left_panel.columnconfigure(0, weight=1)
        left_panel.rowconfigure(0, weight=1)

        right_panel = ttk.Frame(main_pane)
        right_panel.columnconfigure(0, weight=1)
        right_panel.rowconfigure(2, weight=1)

        main_pane.add(left_panel, weight=4)
        main_pane.add(right_panel, weight=3)

        preview_frame = ttk.LabelFrame(left_panel, text="Detection Output", padding=8)
        preview_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        preview_frame.columnconfigure(0, weight=1)
        preview_frame.rowconfigure(0, weight=1)

        self.blob_preview_label = ttk.Label(
            preview_frame,
            text="Output stopped",
            anchor="center",
        )
        self.blob_preview_label.grid(row=0, column=0, sticky="nsew")

        controls = ttk.LabelFrame(right_panel, text="Run Control", padding=8)
        controls.grid(row=0, column=0, sticky="ew")
        controls.columnconfigure(1, weight=1)
        controls.columnconfigure(3, weight=1)

        ttk.Label(controls, text="Preset:").grid(row=0, column=0, sticky="w")
        ttk.Combobox(
            controls,
            textvariable=self.blob_profile_var,
            values=("fast", "balanced", "precision", "measurement"),
            state="readonly",
            width=12,
        ).grid(row=0, column=1, sticky="ew")
        ttk.Button(
            controls,
            text="Apply",
            command=self._apply_blob_preset,
        ).grid(row=0, column=2, columnspan=2, sticky="ew", padx=(10, 0))

        ttk.Label(controls, text="Backend:").grid(row=1, column=0, sticky="w", pady=(8, 0))
        ttk.Combobox(
            controls,
            textvariable=self.blob_camera_backend_var,
            values=("auto", "picamera2", "opencv"),
            state="readonly",
            width=12,
        ).grid(row=1, column=1, sticky="ew", pady=(8, 0))

        ttk.Label(controls, text="Cam index:").grid(row=1, column=2, sticky="w", padx=(12, 0), pady=(8, 0))
        ttk.Entry(controls, textvariable=self.blob_camera_index_var, width=8).grid(
            row=1, column=3, sticky="ew", pady=(8, 0)
        )

        ttk.Label(controls, text="Width:").grid(row=2, column=0, sticky="w", pady=(6, 0))
        ttk.Entry(controls, textvariable=self.blob_cam_width_var, width=12).grid(
            row=2, column=1, sticky="ew", pady=(6, 0)
        )

        ttk.Label(controls, text="Height:").grid(row=2, column=2, sticky="w", pady=(6, 0), padx=(12, 0))
        ttk.Entry(controls, textvariable=self.blob_cam_height_var, width=12).grid(
            row=2, column=3, sticky="ew", pady=(6, 0)
        )

        ttk.Label(controls, text="Polarity:").grid(row=3, column=0, sticky="w", pady=(6, 0))
        ttk.Combobox(
            controls,
            textvariable=self.blob_mode_var,
            values=("auto", "dark", "bright"),
            state="readonly",
            width=10,
        ).grid(row=3, column=1, sticky="ew", pady=(6, 0))

        ttk.Label(controls, text="Proc scale:").grid(row=3, column=2, sticky="w", pady=(6, 0), padx=(12, 0))
        ttk.Entry(controls, textvariable=self.blob_proc_scale_var, width=12).grid(
            row=3, column=3, sticky="ew", pady=(6, 0)
        )

        ttk.Label(controls, text="Output type:").grid(row=4, column=0, sticky="w", pady=(6, 0))
        ttk.Combobox(
            controls,
            textvariable=self.blob_view_type_var,
            values=(
                "auto",
                "mosaic",
                "overlay",
                "vector",
                "heatmap",
                "pointcloud",
                "binary",
                "blob",
                "optical_flow_2d",
                "surface_3d",
            ),
            state="readonly",
        ).grid(row=4, column=1, sticky="ew", pady=(6, 0))

        ttk.Checkbutton(controls, text="Use ROI", variable=self.blob_use_roi_var).grid(
            row=4, column=2, sticky="w", pady=(6, 0), padx=(12, 0)
        )
        ttk.Label(controls, text="ROI(%):").grid(row=4, column=3, sticky="w", pady=(6, 0))
        ttk.Entry(controls, textvariable=self.blob_roi_percent_var, width=12).grid(
            row=4, column=3, sticky="e", pady=(6, 0)
        )

        ttk.Checkbutton(controls, text="PointCloud", variable=self.blob_use_pointcloud_var).grid(
            row=5, column=0, sticky="w", pady=(6, 0)
        )
        ttk.Checkbutton(controls, text="Mosaic", variable=self.blob_use_mosaic_var).grid(
            row=5, column=1, sticky="w", pady=(6, 0)
        )

        ttk.Checkbutton(
            controls,
            text="Robustness test (periodic)",
            variable=self.blob_run_robust_test_var,
        ).grid(row=5, column=2, columnspan=2, sticky="w", pady=(6, 0))

        ttk.Checkbutton(
            controls,
            text="Illumination normalization",
            variable=self.blob_use_illum_norm_var,
        ).grid(row=6, column=0, columnspan=2, sticky="w", pady=(6, 0))
        ttk.Label(controls, text="Illum method:").grid(row=6, column=2, sticky="w", pady=(6, 0), padx=(12, 0))
        ttk.Combobox(
            controls,
            textvariable=self.blob_illum_method_var,
            values=("clahe", "clahe_bg"),
            state="readonly",
        ).grid(row=6, column=3, sticky="ew", pady=(6, 0))

        ttk.Label(controls, text="Dist scale:").grid(row=7, column=0, sticky="w", pady=(6, 0))
        ttk.Entry(controls, textvariable=self.blob_distance_scale_var, width=12).grid(
            row=7, column=1, sticky="ew", pady=(6, 0)
        )
        ttk.Label(controls, text="Dist unit:").grid(row=7, column=2, sticky="w", pady=(6, 0), padx=(12, 0))
        ttk.Entry(controls, textvariable=self.blob_distance_unit_var, width=12).grid(
            row=7, column=3, sticky="ew", pady=(6, 0)
        )

        ttk.Label(controls, text="Vision backend:").grid(row=8, column=0, sticky="w", pady=(6, 0))
        ttk.Combobox(
            controls,
            textvariable=self.vision_backend_var,
            values=("local", "remote CUDA"),
            state="readonly",
        ).grid(row=8, column=1, sticky="ew", pady=(6, 0))
        ttk.Label(controls, text="Remote URL:").grid(
            row=8, column=2, sticky="w", pady=(6, 0), padx=(12, 0)
        )
        ttk.Entry(controls, textvariable=self.remote_vision_url_var, width=18).grid(
            row=8, column=3, sticky="ew", pady=(6, 0)
        )

        action_row = ttk.Frame(controls)
        action_row.grid(row=9, column=0, columnspan=4, sticky="ew", pady=(10, 0))
        action_row.columnconfigure(0, weight=1)
        action_row.columnconfigure(1, weight=1)
        action_row.columnconfigure(2, weight=1)
        action_row.columnconfigure(3, weight=1)

        self.blob_start_btn = ttk.Button(
            action_row, text="Start Dot Pipeline", command=self.start_blob_test
        )
        self.blob_start_btn.grid(row=0, column=0, sticky="ew")

        self.blob_stop_btn = ttk.Button(action_row, text="Stop", command=self.stop_blob_test)
        self.blob_stop_btn.grid(row=0, column=1, sticky="ew", padx=(8, 0))

        self.blob_snapshot_btn = ttk.Button(
            action_row,
            text="Snapshot",
            command=self.request_blob_snapshot,
        )
        self.blob_snapshot_btn.grid(row=0, column=2, sticky="ew", padx=(8, 0))

        self.blob_reset_ref_btn = ttk.Button(
            action_row,
            text="Reset Reference",
            command=self.request_blob_reference_reset,
        )
        self.blob_reset_ref_btn.grid(row=0, column=3, sticky="ew", padx=(8, 0))

        self.blob_settings_btn = ttk.Button(
            action_row,
            text="Settings",
            command=self.open_blob_settings_dialog,
        )
        self.blob_settings_btn.grid(row=1, column=0, columnspan=4, sticky="ew", pady=(8, 0))

        surface = ttk.LabelFrame(right_panel, text="3D Surface", padding=8)
        surface.grid(row=1, column=0, sticky="ew", pady=(10, 0))
        surface.columnconfigure(1, weight=1)
        surface.columnconfigure(3, weight=1)

        ttk.Label(
            surface,
            textvariable=self.surface_status_var,
            wraplength=280,
            justify="left",
        ).grid(row=0, column=0, columnspan=4, sticky="w")

        ttk.Label(surface, text="Mode:").grid(row=1, column=0, sticky="w", pady=(6, 0))
        self.deform_mode_combo = ttk.Combobox(
            surface,
            textvariable=self.surface_measurement_mode_var,
            values=("Flat Deform", "3D Object"),
            state="readonly",
        )
        self.deform_mode_combo.grid(row=1, column=1, columnspan=3, sticky="ew", pady=(6, 0))
        self.deform_mode_combo.bind("<<ComboboxSelected>>", self._apply_surface_measurement_mode)

        self.surface_reset_btn = ttk.Button(
            surface,
            text="Reset Zero",
            command=self.reset_surface_baseline,
        )
        self.surface_reset_btn.grid(row=2, column=0, columnspan=4, sticky="ew", pady=(6, 0))

        ttk.Label(surface, text="Gain:").grid(row=3, column=0, sticky="w", pady=(6, 0))
        ttk.Entry(surface, textvariable=self.surface_gain_var, width=10).grid(
            row=3, column=1, sticky="ew", pady=(6, 0)
        )
        ttk.Label(surface, text="Smooth:").grid(
            row=3, column=2, sticky="w", padx=(12, 0), pady=(6, 0)
        )
        ttk.Entry(surface, textvariable=self.surface_smooth_var, width=10).grid(
            row=3, column=3, sticky="ew", pady=(6, 0)
        )

        ttk.Label(surface, text="Grid:").grid(row=4, column=0, sticky="w", pady=(6, 0))
        ttk.Entry(surface, textvariable=self.surface_grid_var, width=10).grid(
            row=4, column=1, sticky="ew", pady=(6, 0)
        )

        deform_cal = ttk.LabelFrame(right_panel, text="Deform Calibration", padding=8)
        deform_cal.grid(row=2, column=0, sticky="ew", pady=(10, 0))
        deform_cal.columnconfigure(1, weight=1)
        deform_cal.columnconfigure(3, weight=1)

        ttk.Label(deform_cal, text="Known mm:").grid(row=0, column=0, sticky="w")
        self.deform_known_mm_entry = ttk.Entry(
            deform_cal,
            textvariable=self.camera_deform_known_mm_var,
            width=10,
        )
        self.deform_known_mm_entry.grid(row=0, column=1, sticky="ew")
        ttk.Label(deform_cal, text="Estimate:").grid(row=0, column=2, sticky="w", padx=(12, 0))
        ttk.Label(deform_cal, textvariable=self.camera_deform_estimate_var).grid(
            row=0, column=3, sticky="w"
        )

        deform_actions = ttk.Frame(deform_cal)
        deform_actions.grid(row=1, column=0, columnspan=4, sticky="ew", pady=(6, 0))
        for col in range(3):
            deform_actions.columnconfigure(col, weight=1)
        self.deform_calibrate_btn = ttk.Button(
            deform_actions,
            text="Capture Sample",
            command=self.capture_camera_deform_sample,
        )
        self.deform_calibrate_btn.grid(row=0, column=0, sticky="ew")
        self.deform_fit_btn = ttk.Button(
            deform_actions,
            text="Fit",
            command=self.fit_camera_deform_calibration,
        )
        self.deform_fit_btn.grid(row=0, column=1, sticky="ew", padx=(8, 0))
        self.deform_reset_cal_btn = ttk.Button(
            deform_actions,
            text="Reset",
            command=self.reset_camera_deform_calibration,
        )
        self.deform_reset_cal_btn.grid(row=0, column=2, sticky="ew", padx=(8, 0))

        ttk.Label(deform_cal, text="Samples:").grid(row=2, column=0, sticky="w", pady=(6, 0))
        ttk.Label(deform_cal, textvariable=self.camera_deform_sample_count_var).grid(
            row=2, column=1, sticky="w", pady=(6, 0)
        )
        ttk.Label(deform_cal, text="Model:").grid(row=2, column=2, sticky="w", padx=(12, 0), pady=(6, 0))
        ttk.Label(deform_cal, textvariable=self.camera_deform_model_var).grid(
            row=2, column=3, sticky="w", pady=(6, 0)
        )
        ttk.Label(deform_cal, textvariable=self.camera_deform_live_feature_var).grid(
            row=3, column=0, columnspan=4, sticky="w", pady=(4, 0)
        )
        ttk.Label(deform_cal, textvariable=self.camera_deform_status_var, wraplength=280).grid(
            row=4, column=0, columnspan=4, sticky="w", pady=(4, 0)
        )

        metrics = ttk.LabelFrame(right_panel, text="Live Metrics", padding=8)
        metrics.grid(row=3, column=0, sticky="nsew", pady=(10, 0))
        metrics.columnconfigure(1, weight=1)
        metrics.columnconfigure(3, weight=1)

        ttk.Label(metrics, text="State:").grid(row=0, column=0, sticky="w")
        ttk.Label(metrics, textvariable=self.blob_state_var).grid(row=0, column=1, sticky="w")
        ttk.Label(metrics, text="Picked mode:").grid(row=0, column=2, sticky="w", padx=(12, 0))
        ttk.Label(metrics, textvariable=self.blob_pick_mode_var).grid(row=0, column=3, sticky="w")

        ttk.Label(metrics, text="Dots:").grid(row=1, column=0, sticky="w", pady=(4, 0))
        ttk.Label(metrics, textvariable=self.blob_dot_count_var).grid(row=1, column=1, sticky="w", pady=(4, 0))
        ttk.Label(metrics, text="FPS:").grid(row=1, column=2, sticky="w", padx=(12, 0), pady=(4, 0))
        ttk.Label(metrics, textvariable=self.blob_fps_var).grid(row=1, column=3, sticky="w", pady=(4, 0))

        ttk.Label(metrics, text="Frame time:").grid(row=2, column=0, sticky="w", pady=(4, 0))
        ttk.Label(metrics, textvariable=self.blob_latency_var).grid(row=2, column=1, sticky="w", pady=(4, 0))
        ttk.Label(metrics, text="Mean disp(px):").grid(row=2, column=2, sticky="w", padx=(12, 0), pady=(4, 0))
        ttk.Label(metrics, textvariable=self.blob_mean_disp_var).grid(row=2, column=3, sticky="w", pady=(4, 0))

        ttk.Label(metrics, text="Max disp(px):").grid(row=3, column=0, sticky="w", pady=(4, 0))
        ttk.Label(metrics, textvariable=self.blob_max_disp_var).grid(row=3, column=1, sticky="w", pady=(4, 0))
        ttk.Label(metrics, text="Missing ratio:").grid(row=3, column=2, sticky="w", padx=(12, 0), pady=(4, 0))
        ttk.Label(metrics, textvariable=self.blob_missing_ratio_var).grid(row=3, column=3, sticky="w", pady=(4, 0))

        ttk.Label(metrics, text="New tracks:").grid(row=4, column=0, sticky="w", pady=(4, 0))
        ttk.Label(metrics, textvariable=self.blob_new_tracks_var).grid(row=4, column=1, sticky="w", pady=(4, 0))
        ttk.Label(metrics, text="Lost tracks:").grid(row=4, column=2, sticky="w", padx=(12, 0), pady=(4, 0))
        ttk.Label(metrics, textvariable=self.blob_lost_tracks_var).grid(row=4, column=3, sticky="w", pady=(4, 0))

        ttk.Label(metrics, text="Stepper depth:").grid(row=5, column=0, sticky="w", pady=(4, 0))
        ttk.Label(metrics, textvariable=self.contact_depth_stepper_var).grid(row=5, column=1, sticky="w", pady=(4, 0))
        ttk.Label(metrics, text="Camera depth:").grid(row=5, column=2, sticky="w", padx=(12, 0), pady=(4, 0))
        ttk.Label(metrics, textvariable=self.contact_depth_camera_var).grid(row=5, column=3, sticky="w", pady=(4, 0))
        ttk.Label(metrics, text="Contact gate:").grid(row=6, column=0, sticky="w", pady=(4, 0))
        ttk.Label(metrics, textvariable=self.contact_gate_var).grid(
            row=6, column=1, columnspan=3, sticky="w", pady=(4, 0)
        )
        ttk.Label(metrics, text="Contact peak:").grid(row=7, column=0, sticky="w", pady=(4, 0))
        ttk.Label(metrics, textvariable=self.contact_peak_var).grid(row=7, column=1, sticky="w", pady=(4, 0))
        ttk.Label(metrics, text="Area:").grid(row=7, column=2, sticky="w", padx=(12, 0), pady=(4, 0))
        ttk.Label(metrics, textvariable=self.contact_area_var).grid(row=7, column=3, sticky="w", pady=(4, 0))
        ttk.Label(metrics, text="Center:").grid(row=8, column=0, sticky="w", pady=(4, 0))
        ttk.Label(metrics, textvariable=self.contact_center_var).grid(row=8, column=1, sticky="w", pady=(4, 0))
        ttk.Label(metrics, text="Residual:").grid(row=8, column=2, sticky="w", padx=(12, 0), pady=(4, 0))
        ttk.Label(metrics, textvariable=self.contact_residual_var).grid(row=8, column=3, sticky="w", pady=(4, 0))
        ttk.Label(metrics, text="Force:").grid(row=9, column=0, sticky="w", pady=(4, 0))
        ttk.Label(metrics, textvariable=self.contact_force_var).grid(
            row=9, column=1, columnspan=3, sticky="w", pady=(4, 0)
        )

        ttk.Label(
            metrics,
            textvariable=self.blob_message_var,
            wraplength=300,
            justify="left",
        ).grid(row=10, column=0, columnspan=4, sticky="w", pady=(10, 0))

        self._set_blob_controls(False)

    def _apply_blob_preset(self):
        preset = self.blob_profile_var.get().strip().lower()
        if preset == "fast":
            self.blob_camera_backend_var.set("auto")
            self.blob_autofocus_var.set("continuous")
            self.blob_lens_position_var.set("1.0")
            self.blob_exposure_ev_var.set("0.8")
            self.blob_exposure_time_us_var.set("")
            self.blob_analogue_gain_var.set("")
            self.blob_awb_mode_var.set("auto")
            self.blob_colour_gains_var.set("")
            self.blob_proc_scale_var.set("0.65")
            self.blob_min_area_var.set("260")
            self.blob_max_area_var.set("3200")
            self.blob_min_circularity_var.set("0.35")
            self.blob_match_dist_var.set("9.0")
            self.blob_use_mosaic_var.set(False)
            self.blob_use_pointcloud_var.set(False)
            self.blob_use_roi_var.set(True)
            self.blob_roi_percent_var.set("62")
            self.blob_view_type_var.set("overlay")
            self.blob_use_illum_norm_var.set(True)
            self.blob_illum_method_var.set("clahe")
            self.blob_distance_scale_var.set("1.0")
            self.blob_distance_unit_var.set("px")
        elif preset == "precision":
            self.blob_camera_backend_var.set("auto")
            self.blob_autofocus_var.set("continuous")
            self.blob_lens_position_var.set("1.0")
            self.blob_exposure_ev_var.set("0.8")
            self.blob_exposure_time_us_var.set("")
            self.blob_analogue_gain_var.set("")
            self.blob_awb_mode_var.set("auto")
            self.blob_colour_gains_var.set("")
            self.blob_proc_scale_var.set("1.0")
            self.blob_min_area_var.set("120")
            self.blob_max_area_var.set("4200")
            self.blob_min_circularity_var.set("0.28")
            self.blob_match_dist_var.set("6.0")
            self.blob_use_mosaic_var.set(True)
            self.blob_use_pointcloud_var.set(True)
            self.blob_use_roi_var.set(True)
            self.blob_roi_percent_var.set("78")
            self.blob_view_type_var.set("mosaic")
            self.blob_use_illum_norm_var.set(True)
            self.blob_illum_method_var.set("clahe_bg")
            self.blob_distance_scale_var.set("1.0")
            self.blob_distance_unit_var.set("px")
        elif preset == "measurement":
            self.blob_camera_backend_var.set("picamera2")
            self.blob_autofocus_var.set("manual")
            self.blob_lens_position_var.set("12.0")
            self.blob_exposure_ev_var.set("0.0")
            self.blob_exposure_time_us_var.set("8000")
            self.blob_analogue_gain_var.set("1.0")
            self.blob_awb_mode_var.set("manual")
            self.blob_colour_gains_var.set("1.5,1.5")
            self.blob_proc_scale_var.set("1.0")
            self.blob_min_area_var.set("120")
            self.blob_max_area_var.set("4200")
            self.blob_min_circularity_var.set("0.28")
            self.blob_match_dist_var.set("6.0")
            self.blob_use_mosaic_var.set(False)
            self.blob_use_pointcloud_var.set(False)
            self.blob_use_roi_var.set(True)
            self.blob_roi_percent_var.set("74")
            self.blob_view_type_var.set("surface_3d")
            self.blob_use_illum_norm_var.set(True)
            self.blob_illum_method_var.set("clahe_bg")
            self.blob_distance_scale_var.set("1.0")
            self.blob_distance_unit_var.set("mm")
        else:
            self.blob_camera_backend_var.set("auto")
            self.blob_autofocus_var.set("continuous")
            self.blob_lens_position_var.set("1.0")
            self.blob_exposure_ev_var.set("0.8")
            self.blob_exposure_time_us_var.set("")
            self.blob_analogue_gain_var.set("")
            self.blob_awb_mode_var.set("auto")
            self.blob_colour_gains_var.set("")
            self.blob_proc_scale_var.set("1.0")
            self.blob_min_area_var.set("220")
            self.blob_max_area_var.set("3600")
            self.blob_min_circularity_var.set("0.32")
            self.blob_match_dist_var.set("8.0")
            self.blob_use_mosaic_var.set(True)
            self.blob_use_pointcloud_var.set(True)
            self.blob_use_roi_var.set(True)
            self.blob_roi_percent_var.set("68")
            self.blob_view_type_var.set("auto")
            self.blob_use_illum_norm_var.set(True)
            self.blob_illum_method_var.set("clahe_bg")
            self.blob_distance_scale_var.set("1.0")
            self.blob_distance_unit_var.set("px")

        self.blob_message_var.set(f"Preset applied: {preset}.")
        if preset == "measurement" and hasattr(self, "surface_measurement_mode_var"):
            self.surface_measurement_mode_var.set("3D Object")
        self._apply_surface_measurement_mode()
        self.log(f"Dot pipeline preset applied: {preset}.")

    @staticmethod
    def _apply_illumination_normalization(frame_bgr, cv2, method):
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

        if method == "clahe":
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            norm_gray = clahe.apply(gray)
        else:
            clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
            enhanced = clahe.apply(gray)
            background = cv2.GaussianBlur(enhanced, (0, 0), sigmaX=17, sigmaY=17)
            norm_float = (enhanced.astype(np.float32) / (background.astype(np.float32) + 1.0)) * 128.0
            norm_gray = np.clip(norm_float, 0, 255).astype(np.uint8)

        return cv2.cvtColor(norm_gray, cv2.COLOR_GRAY2BGR)

    @staticmethod
    def _build_surface_ellipse_mask(shape, cx, cy, rx, ry):
        height_px, width_px = shape[:2]
        if rx <= 1 or ry <= 1:
            return np.zeros((height_px, width_px), dtype=bool)

        yy, xx = np.ogrid[:height_px, :width_px]
        nx = (xx - cx) / rx
        ny = (yy - cy) / ry
        return (nx * nx + ny * ny) <= 1.0

    def _parse_surface_float(self, variable, default, min_value, max_value):
        try:
            value = float(variable.get())
        except (TypeError, ValueError):
            value = default
        return float(np.clip(value, min_value, max_value))

    def _parse_surface_int(self, variable, default, min_value, max_value):
        try:
            value = int(float(variable.get()))
        except (TypeError, ValueError):
            value = default
        return int(np.clip(value, min_value, max_value))

    def _reset_contact_deform_zero_state(self, status_text):
        self.surface_scale_ema = None
        self.surface_height_ema = None
        self.surface_reference_height_map = None
        self.surface_contact_ema = None
        self.surface_zero_ready = False
        self.surface_tactile_frame = None
        self.surface_contact_baseline = None
        self.surface_contact_display_ema = None
        if hasattr(self, "surface_reset_zero_event"):
            self.surface_reset_zero_event.clear()
        self._set_var(self.surface_status_var, status_text)
        if hasattr(self, "blob_mean_disp_var"):
            self._set_var(self.blob_mean_disp_var, "0.00")
        if hasattr(self, "blob_max_disp_var"):
            self._set_var(self.blob_max_disp_var, "0.00")
        self._clear_contact_signature_readouts()

    def reset_surface_baseline(self):
        self._reset_pressure_contact_gate("manual reset")
        self._reset_contact_deform_zero_state(
            "Contact deform zero will be set on the next camera frame."
        )
        if not self._is_blob_running():
            self._set_contact_depth_zero()
            return

        self._set_var(self.surface_status_var, "Contact deform zero reset requested.")
        self._set_contact_depth_zero()
        self.surface_reset_zero_event.set()
        self.log("Contact deform zero reset requested.")

    def _is_3d_object_mode(self):
        return getattr(self, "surface_measurement_mode", "flat_deform") == "3d_object"

    def _sync_surface_measurement_mode_from_var(self):
        value = "Flat Deform"
        if hasattr(self, "surface_measurement_mode_var"):
            value = str(self.surface_measurement_mode_var.get()).strip()
        normalized = value.lower()
        if normalized in {"3d object", "object 3d", "3d_object", "object"}:
            self.surface_measurement_mode = "3d_object"
            if hasattr(self, "surface_measurement_mode_var"):
                self.surface_measurement_mode_var.set("3D Object")
        else:
            self.surface_measurement_mode = "flat_deform"
            if hasattr(self, "surface_measurement_mode_var"):
                self.surface_measurement_mode_var.set("Flat Deform")
        return self.surface_measurement_mode

    def _set_widget_state(self, widget, state):
        if widget is None:
            return

        def apply_state():
            if not self.root.winfo_exists():
                return
            try:
                widget.configure(state=state)
            except tk.TclError:
                pass

        if self.root.winfo_exists():
            self.root.after(0, apply_state)

    def _refresh_deform_mode_controls(self):
        object_mode = self._is_3d_object_mode()
        running = self._is_blob_running() if hasattr(self, "_is_blob_running") else False
        self._set_button_enabled(self.deform_calibrate_btn, running and not object_mode)
        self._set_button_enabled(self.deform_fit_btn, not object_mode)
        self._set_button_enabled(self.deform_reset_cal_btn, not object_mode)
        self._set_widget_state(self.deform_known_mm_entry, "disabled" if object_mode else "normal")
        self._set_widget_state(self.deform_mode_combo, "disabled" if running else "readonly")

    def _apply_surface_measurement_mode(self, *_event):
        object_mode = self._sync_surface_measurement_mode_from_var() == "3d_object"
        if object_mode:
            if hasattr(self, "blob_view_type_var") and not self._is_blob_running():
                self.blob_view_type_var.set("surface_3d")
            self._set_var(self.camera_deform_estimate_var, "3D contact mode")
            self._set_var(self.contact_depth_camera_var, "relative map")
            self._set_var(
                self.camera_deform_status_var,
                "3D Object mode: Cal Deform disabled. Use Reset Zero before contact.",
            )
        else:
            if str(self.camera_deform_estimate_var.get()).startswith("3D"):
                self._set_var(self.camera_deform_estimate_var, "-")
            if str(self.contact_depth_camera_var.get()) == "relative map":
                self._set_var(self.contact_depth_camera_var, "-")
            if str(self.camera_deform_status_var.get()).startswith("3D Object mode"):
                self._set_var(
                    self.camera_deform_status_var,
                    "Flat Deform mode: Cal Deform estimates one plain-sheet mm value.",
                )
        self._refresh_deform_mode_controls()

    def _clear_contact_signature_readouts(self):
        for variable in (
            getattr(self, "contact_peak_var", None),
            getattr(self, "contact_area_var", None),
            getattr(self, "contact_center_var", None),
            getattr(self, "contact_residual_var", None),
            getattr(self, "contact_force_var", None),
        ):
            if variable is not None:
                self._set_var(variable, "-")

    def _update_contact_signature_readouts(self, features, stats=None, tactile_frame=None):
        stats = stats if stats and stats.get("ready") else None
        if stats is None:
            self._set_var(self.contact_peak_var, "-")
            self._set_var(self.contact_area_var, "-")
            self._set_var(self.contact_center_var, "-")
        else:
            peak_pct = float(stats.get("peak", 0.0)) * 100.0
            top_pct = float(stats.get("top_mean", 0.0)) * 100.0
            area_pct = float(stats.get("area_ratio", 0.0))
            self._set_var(self.contact_peak_var, f"{peak_pct:.0f}% peak | {top_pct:.0f}% top")
            self._set_var(self.contact_area_var, f"{area_pct:.1f}%")
            center = stats.get("center")
            if center is None:
                self._set_var(self.contact_center_var, "-")
            else:
                cx, cy = center
                self._set_var(self.contact_center_var, f"{cx:.0f}, {cy:.0f} px")

        residual_mean = float(getattr(tactile_frame, "residual_mean_px", 0.0) or 0.0)
        residual_peak = float(getattr(tactile_frame, "residual_peak_px", 0.0) or 0.0)
        if residual_peak > 0.0:
            self._set_var(self.contact_residual_var, f"{residual_mean:.3f}/{residual_peak:.3f} px")
        else:
            self._set_var(self.contact_residual_var, "-")

        with self.sensor_data_lock:
            load_kg = getattr(self, "latest_loadcell_kg", None)
            pressure_force = getattr(self, "latest_pressure_force_n", None)
        if self._finite_number(load_kg):
            self._set_var(self.contact_force_var, f"{float(load_kg):.4f} kg load")
        elif self._finite_number(pressure_force):
            pressure_kg = float(pressure_force) / FORCE_GRAVITY_MPS2
            self._set_var(self.contact_force_var, f"{pressure_kg:.4f} kg p-eq")
        else:
            self._set_var(self.contact_force_var, "-")

    def _build_tactile_contact_config(self, params):
        return TactileContactConfig(
            bubble_width_mm=BUBBLE_WIDTH_MM,
            bubble_height_mm=BUBBLE_HEIGHT_MM,
            bubble_max_height_mm=BUBBLE_MAX_HEIGHT_MM,
            grid_width=self._parse_surface_int(self.surface_grid_var, 52, 24, 140),
            roi_scale=float(params.get("roi_scale", 1.0)) if params.get("use_roi", False) else 1.0,
            use_roi=bool(params.get("use_roi", False)),
            gain=self._parse_surface_float(self.surface_gain_var, 1.0, 0.2, 3.0),
            smooth=self._parse_surface_float(self.surface_smooth_var, 1.4, 0.0, 4.0),
        )

    def _build_surface_height_map(self, centroids, reference_centroids, displacements, frame_shape, params, cv2):
        if not reference_centroids or not displacements:
            return None, None, None

        config = self._build_tactile_contact_config(params)
        frame = build_tactile_contact_frame(
            reference_centroids,
            displacements,
            frame_shape,
            config,
            cv2=cv2,
            previous_scale_px=self.surface_scale_ema,
        )
        if frame is None:
            self.surface_tactile_frame = None
            return None, None, None

        self.surface_tactile_frame = frame
        self.surface_scale_ema = frame.scale_px
        height_full = frame.height_map.astype(np.float32)
        if self.surface_height_ema is None or self.surface_height_ema.shape != height_full.shape:
            self.surface_height_ema = height_full
        else:
            self.surface_height_ema = cv2.addWeighted(
                self.surface_height_ema.astype(np.float32),
                0.72,
                height_full,
                0.28,
                0.0,
            )
        return (
            np.where(frame.ellipse_mask, self.surface_height_ema, 0.0).astype(np.float32),
            frame.ellipse_mask,
            frame.scale_px,
        )

    @staticmethod
    def _measure_camera_to_surface_mm(height_map, ellipse_mask=None):
        return measure_camera_to_surface_mm(
            height_map,
            ellipse_mask=ellipse_mask,
            geometry=SurfaceGeometry(
                bubble_max_height_mm=BUBBLE_MAX_HEIGHT_MM,
                camera_to_acrylic_mm=CAMERA_TO_ACRYLIC_MM,
                camera_distance_sign=BUBBLE_CAMERA_DISTANCE_SIGN,
            ),
        )

    def _measure_contact_deformation_mm(self, height_map, ellipse_mask=None, cv2=None):
        stats, next_reference, next_contact_ema = measure_contact_deformation_mm(
            height_map,
            reference_height_map=self.surface_reference_height_map,
            contact_ema=self.surface_contact_ema,
            ellipse_mask=ellipse_mask,
            cv2=cv2,
            geometry=SurfaceGeometry(
                bubble_max_height_mm=BUBBLE_MAX_HEIGHT_MM,
                camera_to_acrylic_mm=CAMERA_TO_ACRYLIC_MM,
                camera_distance_sign=BUBBLE_CAMERA_DISTANCE_SIGN,
            ),
            config=ContactMeasurementConfig(
                deadband_mm=SURFACE_CONTACT_DEADBAND_MM,
                noise_cap_mm=SURFACE_CONTACT_NOISE_CAP_MM,
                baseline_alpha=SURFACE_BASELINE_ALPHA,
                contact_hold_mm=SURFACE_CONTACT_HOLD_MM,
            ),
        )
        self.surface_reference_height_map = next_reference
        self.surface_contact_ema = next_contact_ema
        return stats

    @staticmethod
    def _draw_surface_history_graph(width, height, history, cv2):
        graph = np.zeros((height, width, 3), dtype=np.uint8)
        graph[:, :, :] = (11, 22, 32)
        if height <= 8 or width <= 8:
            return graph

        margin_l = 46
        margin_r = 12
        margin_t = 18
        margin_b = 18
        plot_w = max(1, width - margin_l - margin_r)
        plot_h = max(1, height - margin_t - margin_b)
        left = margin_l
        right = width - margin_r
        top = margin_t
        bottom = height - margin_b

        safe_history = list(history or [])
        peak_values = np.array(
            [np.nan if item[0] is None else float(item[0]) for item in safe_history],
            dtype=np.float32,
        )
        top_values = np.array(
            [np.nan if item[1] is None else float(item[1]) for item in safe_history],
            dtype=np.float32,
        )

        combined = np.concatenate([peak_values, top_values]) if safe_history else np.array([], dtype=np.float32)
        valid = combined[np.isfinite(combined)]
        y_max = 1.0
        if valid.size > 0:
            y_max = max(1.0, float(np.max(valid)) * 1.15)

        for idx in range(4):
            y = int(round(top + (idx * plot_h / 3.0)))
            value = y_max * (1.0 - (idx / 3.0))
            cv2.line(graph, (left, y), (right, y), (36, 54, 68), 1)
            cv2.putText(
                graph,
                f"{value:.1f}",
                (6, min(height - 4, y + 4)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.34,
                (150, 178, 196),
                1,
                cv2.LINE_AA,
            )

        cv2.rectangle(graph, (left, top), (right, bottom), (55, 82, 100), 1)

        def draw_series(values, color):
            if values.size < 2:
                return
            prev_pt = None
            denom = max(1, values.size - 1)
            for idx, value in enumerate(values):
                if not np.isfinite(value):
                    prev_pt = None
                    continue
                x = int(round(left + (idx / denom) * plot_w))
                y = int(round(bottom - (np.clip(float(value), 0.0, y_max) / y_max) * plot_h))
                point = (int(np.clip(x, left, right)), int(np.clip(y, top, bottom)))
                if prev_pt is not None:
                    cv2.line(graph, prev_pt, point, color, 2, cv2.LINE_AA)
                prev_pt = point

        draw_series(peak_values, (80, 205, 255))
        draw_series(top_values, (255, 210, 80))

        cv2.putText(
            graph,
            "contact intensity history",
            (left, 13),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.40,
            (130, 238, 255),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            graph,
            "peak",
            (max(left + 170, width - 132), 13),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.34,
            (80, 205, 255),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            graph,
            "top mean",
            (max(left + 214, width - 82), 13),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.34,
            (255, 210, 80),
            1,
            cv2.LINE_AA,
        )
        return graph

    @staticmethod
    def _build_flow_3d_plot(
        gray_frame,
        height_map,
        cv2,
        roi_mask=None,
        camera_gray_frame=None,
        surface_history=None,
    ):
        camera_source = camera_gray_frame if camera_gray_frame is not None else gray_frame
        out_h, out_w = camera_source.shape[:2]
        show_graph = surface_history is not None
        graph_h = max(72, min(128, int(out_h * 0.18))) if show_graph and out_h > 220 else 0
        camera_h = max(96, int(out_h * (0.42 if show_graph else 0.58)))
        min_plot_h = max(72, int(out_h * (0.26 if show_graph else 0.18)))
        if camera_h + graph_h + min_plot_h > out_h:
            camera_h = max(1, out_h - graph_h - min_plot_h)
        camera_h = min(camera_h, max(1, out_h - graph_h - 1))
        plot_h = max(1, out_h - camera_h - graph_h)

        if len(camera_source.shape) == 2:
            camera_canvas = cv2.cvtColor(camera_source, cv2.COLOR_GRAY2BGR)
        else:
            camera_canvas = camera_source.copy()
        camera_canvas = cv2.convertScaleAbs(camera_canvas, alpha=0.78, beta=0)
        camera_view = cv2.resize(camera_canvas, (out_w, camera_h), interpolation=cv2.INTER_AREA)

        output = np.zeros((out_h, out_w, 3), dtype=np.uint8)
        output[:camera_h, :, :] = camera_view

        plot_canvas = np.zeros((plot_h, out_w, 3), dtype=np.uint8)
        plot_canvas[:, :, :] = (9, 22, 32)
        cv2.line(output, (0, camera_h - 1), (out_w - 1, camera_h - 1), (45, 73, 96), 1)
        graph_canvas = (
            AllInOneTesterGUI._draw_surface_history_graph(out_w, graph_h, surface_history, cv2)
            if graph_h > 0
            else None
        )

        if height_map is None or height_map.size == 0:
            output[camera_h:camera_h + plot_h, :, :] = plot_canvas
            if graph_canvas is not None:
                output[camera_h + plot_h:, :, :] = graph_canvas
            return output

        height_work = height_map.astype(np.float32)
        max_val = float(np.max(height_work))
        if max_val > 1.0:
            height_work = height_work / max_val

        if roi_mask is not None:
            roi_binary = roi_mask.astype(bool)
            height_work = np.where(roi_binary, height_work, 0.0)
        else:
            roi_binary = None

        height_plot = cv2.resize(height_work, (out_w, plot_h), interpolation=cv2.INTER_AREA)
        plot_roi_binary = None
        if roi_binary is not None:
            plot_roi_binary = cv2.resize(
                roi_binary.astype(np.uint8),
                (out_w, plot_h),
                interpolation=cv2.INTER_NEAREST,
            ).astype(bool)

        # Smooth and shade the height map to render a filled surface.
        height = height_plot.astype(np.float32)
        height = cv2.GaussianBlur(height, (0, 0), sigmaX=1.6, sigmaY=1.4)
        if plot_roi_binary is not None:
            mask_float = plot_roi_binary.astype(np.float32)
            soft_mask = cv2.GaussianBlur(mask_float, (0, 0), sigmaX=2.0, sigmaY=2.0)
            soft_mask = np.clip(soft_mask, 0.0, 1.0)
            height = height * soft_mask
        height = np.clip(height, 0.0, 1.0)

        color_map = cv2.applyColorMap((height * 255.0).astype(np.uint8), cv2.COLORMAP_TURBO)
        dx = cv2.Sobel(height, cv2.CV_32F, 1, 0, ksize=3)
        dy = cv2.Sobel(height, cv2.CV_32F, 0, 1, ksize=3)
        nx = -dx * 1.5
        ny = -dy * 1.5
        nz = 1.0
        norm = np.sqrt(nx * nx + ny * ny + nz * nz)
        norm = np.where(norm < 1e-6, 1.0, norm)
        shade = (nx * 0.25 + ny * -0.35 + nz * 1.0) / norm
        shade = np.clip(shade, 0.0, 1.0)
        shade = 0.35 + 0.65 * shade

        step = max(4, min(10, out_w // 120))
        x_scale = 0.95
        y_scale = 0.62
        z_scale = max(28.0, plot_h * 0.48)
        z_x_shift = max(10.0, min(28.0, out_w * 0.028))
        horizon = int(plot_h * 0.68)
        center_x = out_w * 0.5

        def project(gx, gy, z):
            px = (gx - center_x) * x_scale + center_x + z * z_x_shift
            py = (gy - plot_h * 0.5) * y_scale + horizon - z * z_scale
            return (int(np.clip(px, 0, out_w - 1)), int(np.clip(py, 0, plot_h - 1)))

        cells = []
        for gy in range(0, plot_h - step, step):
            cy = min(plot_h - 1, gy + step // 2)
            for gx in range(0, out_w - step, step):
                cx = min(out_w - 1, gx + step // 2)
                if plot_roi_binary is not None and not plot_roi_binary[cy, cx]:
                    continue

                z00 = float(height[gy, gx])
                z10 = float(height[gy, gx + step])
                z11 = float(height[gy + step, gx + step])
                z01 = float(height[gy + step, gx])

                p0 = project(gx, gy, z00)
                p1 = project(gx + step, gy, z10)
                p2 = project(gx + step, gy + step, z11)
                p3 = project(gx, gy + step, z01)

                shade_val = float(shade[cy, cx])
                base = color_map[cy, cx].astype(np.float32)
                color = np.clip(base * shade_val, 0, 255).astype(np.uint8)
                avg_y = (p0[1] + p1[1] + p2[1] + p3[1]) * 0.25
                avg_z = (z00 + z10 + z11 + z01) * 0.25
                cells.append(
                    (
                        avg_y,
                        -avg_z,
                        np.array([p0, p1, p2, p3], dtype=np.int32),
                        (int(color[0]), int(color[1]), int(color[2])),
                    )
                )

        for _avg_y, _avg_z, polygon, color in sorted(cells, key=lambda item: (item[0], item[1])):
            cv2.fillConvexPoly(
                plot_canvas,
                polygon,
                color,
                lineType=cv2.LINE_AA,
            )

        plot_canvas = cv2.rotate(plot_canvas, cv2.ROTATE_180)

        cv2.putText(
            plot_canvas,
            "bubble surface",
            (12, 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (255, 230, 80),
            1,
        )
        output[camera_h:camera_h + plot_h, :, :] = plot_canvas
        if graph_canvas is not None:
            cv2.line(output, (0, camera_h + plot_h - 1), (out_w - 1, camera_h + plot_h - 1), (45, 73, 96), 1)
            output[camera_h + plot_h:, :, :] = graph_canvas
        return output

    def _build_optical_flow_tab(self, notebook):
        tab = ttk.Frame(notebook, padding=12)
        notebook.add(tab, text="2) Optical Flow")
        tab.columnconfigure(0, weight=3)
        tab.columnconfigure(1, weight=2)
        tab.rowconfigure(1, weight=1)

        self.flow_state_var = tk.StringVar(value="Stopped")
        self.flow_points_var = tk.StringVar(value="-")
        self.flow_mean_disp_var = tk.StringVar(value="-")
        self.flow_max_disp_var = tk.StringVar(value="-")
        self.flow_fps_var = tk.StringVar(value="-")
        self.flow_message_var = tk.StringVar(value="Start optical flow to track live dot movement.")

        self.flow_camera_backend_var = tk.StringVar(value="auto")
        self.flow_camera_index_var = tk.StringVar(value="0")
        self.flow_cam_width_var = tk.StringVar(value="1280")
        self.flow_cam_height_var = tk.StringVar(value="960")
        self.flow_proc_scale_var = tk.StringVar(value="1.0")
        self.flow_roi_scale_var = tk.StringVar(value="0.96")
        self.flow_3d_roi_scale_var = tk.StringVar(value="1.0")
        self.flow_max_points_var = tk.StringVar(value="260")
        self.flow_quality_var = tk.StringVar(value="0.02")
        self.flow_min_distance_var = tk.StringVar(value="7")
        self.flow_reseed_var = tk.StringVar(value="90")
        self.flow_show_vectors_var = tk.BooleanVar(value=True)

        ttk.Label(
            tab,
            text=(
                "Realtime optical flow tracking for dot deformation. "
                "Use Settings for advanced tracking parameters."
            ),
        ).grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 8))

        preview_frame = ttk.LabelFrame(tab, text="Optical Flow Live", padding=8)
        preview_frame.grid(row=1, column=0, sticky="nsew", padx=(0, 10))
        preview_frame.columnconfigure(0, weight=1)
        preview_frame.rowconfigure(0, weight=1)

        self.flow_preview_label = ttk.Label(
            preview_frame,
            text="Preview stopped",
            anchor="center",
        )
        self.flow_preview_label.grid(row=0, column=0, sticky="nsew")

        side = ttk.LabelFrame(tab, text="Run Control", padding=8)
        side.grid(row=1, column=1, sticky="nsew")
        side.columnconfigure(1, weight=1)

        self.flow_start_btn = ttk.Button(side, text="Start Optical Flow", command=self.start_optical_flow)
        self.flow_start_btn.grid(row=0, column=0, columnspan=2, sticky="ew")

        self.flow_stop_btn = ttk.Button(side, text="Stop", command=self.stop_optical_flow)
        self.flow_stop_btn.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(8, 0))

        self.flow_settings_btn = ttk.Button(side, text="Settings", command=self.open_flow_settings_dialog)
        self.flow_settings_btn.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(8, 0))

        ttk.Label(side, text="State:").grid(row=3, column=0, sticky="w", pady=(10, 0))
        ttk.Label(side, textvariable=self.flow_state_var).grid(row=3, column=1, sticky="w", pady=(10, 0))

        ttk.Label(side, text="Tracked points:").grid(row=4, column=0, sticky="w", pady=(4, 0))
        ttk.Label(side, textvariable=self.flow_points_var).grid(row=4, column=1, sticky="w", pady=(4, 0))

        ttk.Label(side, text="Mean disp(px):").grid(row=5, column=0, sticky="w", pady=(4, 0))
        ttk.Label(side, textvariable=self.flow_mean_disp_var).grid(row=5, column=1, sticky="w", pady=(4, 0))

        ttk.Label(side, text="Max disp(px):").grid(row=6, column=0, sticky="w", pady=(4, 0))
        ttk.Label(side, textvariable=self.flow_max_disp_var).grid(row=6, column=1, sticky="w", pady=(4, 0))

        ttk.Label(side, text="FPS:").grid(row=7, column=0, sticky="w", pady=(4, 0))
        ttk.Label(side, textvariable=self.flow_fps_var).grid(row=7, column=1, sticky="w", pady=(4, 0))

        ttk.Label(
            side,
            textvariable=self.flow_message_var,
            wraplength=280,
            justify="left",
        ).grid(row=8, column=0, columnspan=2, sticky="w", pady=(10, 0))

        self._set_flow_controls(False)

    def _setup_gpio_once(self):
        if GPIO is None:
            self.log("RPi.GPIO is not available. GPIO-based tests cannot run on this machine.")
            return

        try:
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            self.gpio_ready = True
        except Exception as exc:
            self.gpio_ready = False
            self.log(f"RPi.GPIO is unavailable on this board/OS: {exc}")
            self.log("Stepper and limit switch tests will use native libgpiod helpers when available.")

    def _gpio_available(self):
        return GPIO is not None and bool(getattr(self, "gpio_ready", GPIO is not None))

    def _require_gpio(self):
        if self._gpio_available():
            return True
        self.log("GPIO test requested, but RPi.GPIO is unavailable or incompatible.")
        messagebox.showerror(
            "GPIO Unavailable",
            "RPi.GPIO is unavailable or incompatible on this board/OS. "
            "Use native libgpiod helpers for stepper and limit switch tests.",
        )
        return False

    def _set_var(self, variable, value):
        if self.root.winfo_exists():
            self.root.after(0, variable.set, value)

    def _set_contact_gate_status(self, value):
        if hasattr(self, "contact_gate_var"):
            self._set_var(self.contact_gate_var, value)

    def _set_contact_depth_zero(self):
        if STEPPER_DEPTH_ENABLED:
            self.stepper_contact_zero_mm = float(self.stepper_position_mm)
        self._refresh_contact_depth_readouts()

    def _stepper_contact_depth_mm(self):
        if not STEPPER_DEPTH_ENABLED:
            return None
        if self.stepper_contact_zero_mm is None:
            return None
        return max(0.0, float(self.stepper_position_mm) - float(self.stepper_contact_zero_mm))

    def _apply_stepper_motion(self, direction_name, pulses=None, elapsed_seconds=None, frequency_hz=None):
        if not STEPPER_DEPTH_ENABLED:
            return
        if pulses is None:
            seconds = max(0.0, float(elapsed_seconds or 0.0))
            pulses = int(round(seconds * float(frequency_hz or STEPPER_FREQUENCY_HZ)))
        delta_mm = max(0, int(pulses)) * STEPPER_MM_PER_PULSE
        if direction_name == "up":
            delta_mm = -delta_mm
        self.stepper_position_mm += delta_mm
        self._refresh_contact_depth_readouts()

    def _estimate_camera_depth_mm(self, contact_stats):
        if not contact_stats or not contact_stats.get("ready"):
            return None
        top_mean = float(contact_stats.get("top_mean", 0.0))
        peak = float(contact_stats.get("peak", 0.0))
        area_fraction = np.clip(float(contact_stats.get("area_ratio", 0.0)) / 100.0, 0.0, 0.95)
        intensity_depth = BUBBLE_MAX_HEIGHT_MM * 0.72 * np.clip(max(top_mean, peak * 0.35), 0.0, 1.0)
        area_depth = BUBBLE_MAX_HEIGHT_MM * (1.0 - np.sqrt(max(0.0, 1.0 - area_fraction)))
        return float(max(intensity_depth, area_depth))

    def _camera_deform_calibration_path(self):
        return Path(__file__).resolve().parent / CAMERA_DEFORM_CALIBRATION_FILENAME

    def _init_camera_deform_calibration_state(self):
        if not hasattr(self, "camera_deform_samples"):
            self.camera_deform_samples = []
        if not hasattr(self, "camera_deform_model"):
            self.camera_deform_model = None
        if not hasattr(self, "camera_deform_latest_features"):
            self.camera_deform_latest_features = None
        if not hasattr(self, "camera_deform_capture_active"):
            self.camera_deform_capture_active = False
            self.camera_deform_capture_target_mm = None
            self.camera_deform_capture_started_at = None
            self.camera_deform_capture_buffer = []
        if not getattr(self, "_camera_deform_calibration_loaded", False):
            self._camera_deform_calibration_loaded = True
            self._load_camera_deform_calibration()

    @staticmethod
    def _camera_deform_feature_vector(features):
        values = []
        for name in CAMERA_DEFORM_FEATURE_NAMES:
            try:
                value = float(features.get(name, 0.0))
            except (TypeError, ValueError, AttributeError):
                value = 0.0
            if not np.isfinite(value):
                value = 0.0
            values.append(value)
        return np.asarray(values, dtype=np.float64)

    @staticmethod
    def _fit_camera_deform_model(samples):
        usable = []
        for sample in samples:
            try:
                target_mm = float(sample["target_mm"])
                features = sample["features"]
            except (KeyError, TypeError, ValueError):
                continue
            if not np.isfinite(target_mm):
                continue
            usable.append((AllInOneTesterGUI._camera_deform_feature_vector(features), max(0.0, target_mm)))

        if len(usable) < 2:
            return None

        x_matrix = np.vstack([item[0] for item in usable])
        y_vector = np.asarray([item[1] for item in usable], dtype=np.float64)
        mean = np.mean(x_matrix, axis=0)
        scale = np.std(x_matrix, axis=0)
        scale = np.where(scale < 1e-6, 1.0, scale)
        x_norm = (x_matrix - mean) / scale
        design = np.column_stack([np.ones(x_norm.shape[0], dtype=np.float64), x_norm])
        ridge = np.eye(design.shape[1], dtype=np.float64) * 1e-3
        ridge[0, 0] = 0.0
        try:
            solution = np.linalg.solve((design.T @ design) + ridge, design.T @ y_vector)
        except np.linalg.LinAlgError:
            solution, *_ = np.linalg.lstsq(design, y_vector, rcond=None)

        prediction = design @ solution
        total = float(np.sum((y_vector - float(np.mean(y_vector))) ** 2))
        residual = float(np.sum((y_vector - prediction) ** 2))
        r2 = None if total <= 1e-9 else 1.0 - (residual / total)
        return {
            "feature_names": list(CAMERA_DEFORM_FEATURE_NAMES),
            "intercept": float(solution[0]),
            "coefficients": [float(value) for value in solution[1:]],
            "mean": [float(value) for value in mean],
            "scale": [float(value) for value in scale],
            "sample_count": int(len(usable)),
            "r2": None if r2 is None else float(r2),
        }

    def _predict_camera_deform_mm(self, features, model=None):
        model = self.camera_deform_model if model is None else model
        if not model:
            return None
        try:
            if list(model.get("feature_names", [])) != list(CAMERA_DEFORM_FEATURE_NAMES):
                return None
            values = self._camera_deform_feature_vector(features)
            mean = np.asarray(model["mean"], dtype=np.float64)
            scale = np.asarray(model["scale"], dtype=np.float64)
            coeffs = np.asarray(model["coefficients"], dtype=np.float64)
            estimate = float(model["intercept"] + np.dot((values - mean) / scale, coeffs))
        except (KeyError, TypeError, ValueError):
            return None
        if not np.isfinite(estimate):
            return None
        return max(0.0, estimate)

    def _refresh_camera_deform_calibration_readouts(self):
        sample_count = len(getattr(self, "camera_deform_samples", []))
        self._set_var(
            self.camera_deform_sample_count_var,
            f"{sample_count} sample{'s' if sample_count != 1 else ''}",
        )
        model = getattr(self, "camera_deform_model", None)
        if model:
            r2 = model.get("r2")
            r2_text = "" if r2 is None else f" | R2 {float(r2):.3f}"
            self._set_var(
                self.camera_deform_model_var,
                f"linear {int(model.get('sample_count', sample_count))} samples{r2_text}",
            )
        else:
            self._set_var(self.camera_deform_model_var, "-")

    def _load_camera_deform_calibration(self):
        path = self._camera_deform_calibration_path()
        if not path.exists():
            return
        try:
            with path.open("r", encoding="utf-8") as handle:
                payload = json.load(handle)
            samples = list(payload.get("samples", []))
            model = payload.get("model")
            if model and list(model.get("feature_names", [])) != list(CAMERA_DEFORM_FEATURE_NAMES):
                model = None
            if model is None and len(samples) >= 2:
                model = self._fit_camera_deform_model(samples)
        except Exception as exc:
            self.log(f"Could not load camera deform calibration: {exc}")
            return
        with self.sensor_data_lock:
            self.camera_deform_samples = samples
            self.camera_deform_model = model
        self._refresh_camera_deform_calibration_readouts()
        if model:
            self._set_var(self.camera_deform_status_var, "Loaded camera deform calibration.")
        elif samples:
            self._set_var(self.camera_deform_status_var, "Loaded deform samples. Fit calibration when ready.")

    def _save_camera_deform_calibration(self):
        with self.sensor_data_lock:
            samples = list(self.camera_deform_samples)
            model = self.camera_deform_model
        payload = {
            "model_type": "deform_mm = linear_ridge(pressure + dot/camera features)",
            "feature_names": list(CAMERA_DEFORM_FEATURE_NAMES),
            "model": model,
            "samples": samples,
            "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        }
        try:
            with self._camera_deform_calibration_path().open("w", encoding="utf-8") as handle:
                json.dump(payload, handle, indent=2)
        except Exception as exc:
            self.log(f"Could not save camera deform calibration: {exc}")

    @staticmethod
    def _camera_deform_capture_stats(buffer):
        rows = []
        for item in buffer:
            features = item.get("features", item)
            row = []
            for name in CAMERA_DEFORM_FEATURE_NAMES:
                try:
                    value = float(features.get(name, 0.0))
                except (TypeError, ValueError, AttributeError):
                    value = 0.0
                row.append(value if np.isfinite(value) else 0.0)
            rows.append(row)
        if not rows:
            return None
        matrix = np.asarray(rows, dtype=np.float64)
        return {
            "features": {
                name: float(value)
                for name, value in zip(CAMERA_DEFORM_FEATURE_NAMES, np.median(matrix, axis=0))
            },
            "feature_std": {
                name: float(value)
                for name, value in zip(CAMERA_DEFORM_FEATURE_NAMES, np.std(matrix, axis=0))
            },
            "feature_min": {
                name: float(value)
                for name, value in zip(CAMERA_DEFORM_FEATURE_NAMES, np.min(matrix, axis=0))
            },
            "feature_max": {
                name: float(value)
                for name, value in zip(CAMERA_DEFORM_FEATURE_NAMES, np.max(matrix, axis=0))
            },
        }

    @staticmethod
    def _camera_deform_capture_quality(features, feature_std, frame_count):
        reasons = []
        quality = "stable"
        if frame_count < CAMERA_DEFORM_CAPTURE_MIN_FRAMES:
            quality = "bad"
            reasons.append("few frames")
        if float(features.get("missing_ratio", 0.0)) > 0.45:
            quality = "bad"
            reasons.append("missing dots high")
        elif float(features.get("missing_ratio", 0.0)) > 0.25:
            quality = "noisy"
            reasons.append("missing dots moderate")
        if float(feature_std.get("pressure_delta_hpa", 0.0)) > 0.35:
            quality = "noisy" if quality == "stable" else quality
            reasons.append("pressure noisy")
        if float(feature_std.get("dot_mean_px", 0.0)) > 0.35:
            quality = "noisy" if quality == "stable" else quality
            reasons.append("dot mean noisy")
        if float(feature_std.get("dot_max_px", 0.0)) > 0.85:
            quality = "noisy" if quality == "stable" else quality
            reasons.append("dot peak noisy")
        if float(feature_std.get("contact_peak", 0.0)) > 0.06:
            quality = "noisy" if quality == "stable" else quality
            reasons.append("contact intensity noisy")
        return quality, reasons

    def capture_camera_deform_sample(self):
        if self._is_3d_object_mode():
            messagebox.showwarning(
                "Camera Deform Calibration",
                "3D Object mode uses relative contact maps. Switch to Flat Deform to capture mm samples.",
            )
            self.log("Cal Deform ignored in 3D Object mode.")
            return
        try:
            target_mm = float(self.camera_deform_known_mm_var.get())
        except (TypeError, ValueError):
            messagebox.showwarning("Camera Deform Calibration", "Enter a valid deform distance in mm.")
            return
        if target_mm < 0.0:
            messagebox.showwarning("Camera Deform Calibration", "Deform distance must be 0 mm or greater.")
            return

        with self.sensor_data_lock:
            features = None if self.camera_deform_latest_features is None else dict(self.camera_deform_latest_features)
            capture_active = bool(self.camera_deform_capture_active)
        if capture_active:
            messagebox.showwarning("Camera Deform Calibration", "A deform sample capture is already running.")
            return
        if not self._is_blob_running():
            messagebox.showwarning(
                "Camera Deform Calibration",
                "Start Live Detection before capturing a deform sample.",
            )
            return
        if not features:
            messagebox.showwarning(
                "Camera Deform Calibration",
                "Start Live Detection and wait for camera/pressure features before capturing.",
            )
            return

        now = time.monotonic()
        with self.sensor_data_lock:
            self.camera_deform_capture_active = True
            self.camera_deform_capture_target_mm = float(target_mm)
            self.camera_deform_capture_started_at = now
            self.camera_deform_capture_buffer = [{"t": now, "features": dict(features)}]
        self._set_var(
            self.camera_deform_status_var,
            (
                f"Capturing {CAMERA_DEFORM_CAPTURE_SECONDS:.1f}s sample for "
                f"{target_mm:.3f} mm..."
            ),
        )
        self.log(
            f"Camera deform robust capture started: target={target_mm:.3f} mm, "
            f"duration={CAMERA_DEFORM_CAPTURE_SECONDS:.1f}s."
        )
        self.root.after(
            max(100, int(CAMERA_DEFORM_CAPTURE_SECONDS * 1000.0)),
            self._finish_camera_deform_sample_capture,
        )

    def _finish_camera_deform_sample_capture(self):
        with self.sensor_data_lock:
            if not self.camera_deform_capture_active:
                return
            target_mm = self.camera_deform_capture_target_mm
            started_at = self.camera_deform_capture_started_at
            buffer = list(self.camera_deform_capture_buffer)
            self.camera_deform_capture_active = False
            self.camera_deform_capture_target_mm = None
            self.camera_deform_capture_started_at = None
            self.camera_deform_capture_buffer = []

        stats = self._camera_deform_capture_stats(buffer)
        frame_count = len(buffer)
        if stats is None or frame_count < CAMERA_DEFORM_CAPTURE_MIN_FRAMES:
            self._set_var(
                self.camera_deform_status_var,
                (
                    f"Capture failed: only {frame_count} frame"
                    f"{'s' if frame_count != 1 else ''}. Keep Live Detection running."
                ),
            )
            self.log(f"Camera deform sample rejected: only {frame_count} capture frame(s).")
            return

        features = stats["features"]
        feature_std = stats["feature_std"]
        quality, reasons = self._camera_deform_capture_quality(features, feature_std, frame_count)
        duration = 0.0 if started_at is None else max(0.0, time.monotonic() - float(started_at))
        sample = {
            "target_mm": float(target_mm),
            "features": features,
            "feature_std": feature_std,
            "feature_min": stats["feature_min"],
            "feature_max": stats["feature_max"],
            "frame_count": int(frame_count),
            "capture_seconds": float(duration),
            "quality": quality,
            "quality_notes": reasons,
            "captured_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        }
        with self.sensor_data_lock:
            self.camera_deform_samples.append(sample)
            count = len(self.camera_deform_samples)

        self._refresh_camera_deform_calibration_readouts()
        note = f" ({', '.join(reasons)})" if reasons else ""
        self._set_var(
            self.camera_deform_status_var,
            f"Captured sample {count}: {quality}, {frame_count} frames{note}.",
        )
        self.log(
            "Camera deform sample "
            f"{count}: target={target_mm:.3f} mm, "
            f"dP={sample['features']['pressure_delta_hpa']:+.2f} hPa, "
            f"dot={sample['features']['dot_mean_px']:.3f}/{sample['features']['dot_max_px']:.3f} px, "
            f"std dot={sample['feature_std']['dot_mean_px']:.3f}, "
            f"quality={quality}."
        )
        if count >= 2:
            self.fit_camera_deform_calibration(show_warning=False)
        else:
            self._save_camera_deform_calibration()

    def fit_camera_deform_calibration(self, show_warning=True):
        if self._is_3d_object_mode():
            if show_warning:
                messagebox.showwarning(
                    "Camera Deform Calibration",
                    "3D Object mode does not fit a single deform-mm model.",
                )
            self._set_var(
                self.camera_deform_status_var,
                "3D Object mode: Cal Deform disabled. Use contact signature metrics.",
            )
            self.log("Fit Deform ignored in 3D Object mode.")
            return False
        with self.sensor_data_lock:
            samples = list(self.camera_deform_samples)
        model = self._fit_camera_deform_model(samples)
        if model is None:
            if show_warning:
                messagebox.showwarning(
                    "Camera Deform Calibration",
                    "Capture at least 2 deform samples before fitting.",
                )
            self._set_var(self.camera_deform_status_var, "Need at least 2 deform samples.")
            return False
        with self.sensor_data_lock:
            self.camera_deform_model = model
        self._refresh_camera_deform_calibration_readouts()
        self._save_camera_deform_calibration()
        r2 = model.get("r2")
        r2_text = "" if r2 is None else f", R2={float(r2):.3f}"
        self._set_var(
            self.camera_deform_status_var,
            f"Camera deform calibrated with {model['sample_count']} samples{r2_text}.",
        )
        self.log(f"Camera deform calibration fit with {model['sample_count']} samples{r2_text}.")
        return True

    def reset_camera_deform_calibration(self):
        with self.sensor_data_lock:
            self.camera_deform_samples.clear()
            self.camera_deform_model = None
            self.camera_deform_capture_active = False
            self.camera_deform_capture_target_mm = None
            self.camera_deform_capture_started_at = None
            self.camera_deform_capture_buffer = []
        self._refresh_camera_deform_calibration_readouts()
        self._set_var(self.camera_deform_estimate_var, "-")
        self._set_var(self.camera_deform_status_var, "Camera deform calibration reset.")
        try:
            self._camera_deform_calibration_path().unlink(missing_ok=True)
        except Exception as exc:
            self.log(f"Could not remove camera deform calibration: {exc}")
        self.log("Camera deform calibration reset.")

    def _update_camera_deform_live_features(
        self,
        *,
        mean_disp,
        max_disp,
        missing_ratio,
        contact_stats=None,
        tactile_frame=None,
        scale_px=None,
        gate=None,
    ):
        with self.sensor_data_lock:
            pressure = self.latest_pressure_hpa
            contact_zero = getattr(self, "contact_pressure_zero_hpa", None)
            force_zero = getattr(self, "force_pressure_zero_hpa", None)
            model = self.camera_deform_model
        if gate and gate.get("delta") is not None:
            pressure_delta = float(gate["delta"])
        elif pressure is not None and contact_zero is not None:
            pressure_delta = float(pressure) - float(contact_zero)
        elif pressure is not None and force_zero is not None:
            pressure_delta = float(pressure) - float(force_zero)
        elif pressure is not None:
            pressure_delta = float(pressure) - FORCE_DEFAULT_PRESSURE_ZERO_HPA
        else:
            pressure_delta = 0.0

        stats = contact_stats if contact_stats and contact_stats.get("ready") else {}
        features = {
            "pressure_delta_hpa": pressure_delta,
            "dot_mean_px": float(mean_disp or 0.0),
            "dot_max_px": float(max_disp or 0.0),
            "missing_ratio": float(missing_ratio or 0.0),
            "contact_peak": float(stats.get("peak", 0.0)),
            "contact_top_mean": float(stats.get("top_mean", 0.0)),
            "contact_area_ratio": float(stats.get("area_ratio", 0.0)),
            "residual_mean_px": float(getattr(tactile_frame, "residual_mean_px", 0.0) or 0.0),
            "residual_peak_px": float(getattr(tactile_frame, "residual_peak_px", 0.0) or 0.0),
            "surface_scale_px": float(scale_px or 0.0),
        }
        with self.sensor_data_lock:
            self.camera_deform_latest_features = dict(features)
            if self.camera_deform_capture_active:
                self.camera_deform_capture_buffer.append(
                    {"t": time.monotonic(), "features": dict(features)}
                )

        self._set_var(
            self.camera_deform_live_feature_var,
            (
                f"dP {features['pressure_delta_hpa']:+.2f} hPa | "
                f"dot {features['dot_mean_px']:.2f}/{features['dot_max_px']:.2f} px | "
                f"contact {features['contact_peak']:.3f}"
            ),
        )
        self._update_contact_signature_readouts(features, stats=stats, tactile_frame=tactile_frame)
        if self._is_3d_object_mode():
            center = stats.get("center") if stats else None
            if center is None:
                center_text = "-"
            else:
                center_text = f"{center[0]:.0f}, {center[1]:.0f} px"
            self._set_var(
                self.camera_deform_live_feature_var,
                (
                    f"dP {features['pressure_delta_hpa']:+.2f} hPa | "
                    f"contact {features['contact_peak'] * 100.0:.0f}% | "
                    f"area {features['contact_area_ratio']:.1f}% | "
                    f"center {center_text}"
                ),
            )
            self._set_var(self.camera_deform_estimate_var, "3D contact mode")
            self._set_var(self.contact_depth_camera_var, "relative map")
            self._append_force_cycle_sample()
            return
        estimate = self._predict_camera_deform_mm(features, model=model)
        if estimate is None:
            self._set_var(self.camera_deform_estimate_var, "-")
        else:
            self._set_var(self.camera_deform_estimate_var, f"{estimate:.2f} mm cal")
            self._set_var(self.contact_depth_camera_var, f"{estimate:.2f} mm cal")
        self._append_force_cycle_sample()

    def _refresh_contact_depth_readouts(self, camera_depth_mm=None, update_camera=False):
        self._set_var(self.stepper_position_var, f"{self.stepper_position_mm:.3f} mm")
        if not STEPPER_DEPTH_ENABLED:
            self._set_var(self.contact_depth_stepper_var, "unavailable (manual)")
        else:
            stepper_depth = self._stepper_contact_depth_mm()
            if stepper_depth is None:
                self._set_var(self.contact_depth_stepper_var, "-")
            else:
                self._set_var(self.contact_depth_stepper_var, f"{stepper_depth:.3f} mm")
        if update_camera:
            if camera_depth_mm is None:
                self._set_var(self.contact_depth_camera_var, "-")
            else:
                self._set_var(self.contact_depth_camera_var, f"{camera_depth_mm:.2f} mm approx")

    def _reset_pressure_contact_gate(self, source="manual", seed_latest=True):
        if not PRESSURE_CONTACT_GATE_ENABLED:
            self._set_contact_gate_status("disabled")
            return
        with self.sensor_data_lock:
            latest_pressure = self.latest_pressure_hpa if seed_latest else None
            self.contact_pressure_zero_hpa = None
            self.contact_pressure_noise_hpa = 0.0
            self.contact_pressure_delta_hpa = None
            self.contact_pressure_threshold_hpa = PRESSURE_CONTACT_MIN_DELTA_HPA
            self.contact_pressure_is_contact = False
            self.contact_pressure_state = "waiting_pressure"
            self.contact_pressure_baseline_samples = []
            self.contact_pressure_above_count = 0
            self.contact_pressure_below_count = 0
            self.contact_pressure_zero_request_sent = False
            if latest_pressure is not None:
                self.contact_pressure_baseline_samples.append(float(latest_pressure))
                self.contact_pressure_state = "zeroing"
        status = "zeroing pressure" if latest_pressure is not None else "waiting pressure"
        self._set_contact_gate_status(status)
        self.log(f"Pressure contact gate reset ({source}).")

    def _request_pressure_zero_from_camera_open(self):
        with self.sensor_data_lock:
            self.force_pressure_zero_hpa = None
            self.force_pressure_zero_pending = True
            self.force_pressure_zero_samples = []
            self.latest_pressure_force_n = None
            if hasattr(self, "force_graph_history"):
                self.force_graph_history.clear()

        self._refresh_force_pressure_zero()
        self._set_var(
            self.force_calibration_status_var,
            "Camera opened; collecting pressure zero baseline.",
        )
        self._reset_pressure_contact_gate("camera open", seed_latest=False)
        self._refresh_force_live_pair()
        self._update_force_estimate()
        self._request_force_graph_redraw()
        self.log("Pressure zero baseline reset for camera-open experiment.")

        if not self._is_thread_running(self.pressure_thread):
            self.start_pressure_read()

    @staticmethod
    def _robust_pressure_noise(samples):
        if not samples:
            return 0.0
        arr = np.asarray(samples, dtype=np.float32)
        median = float(np.median(arr))
        mad = float(np.median(np.abs(arr - median)))
        return max(0.03, 1.4826 * mad)

    def _pressure_contact_threshold(self, noise_hpa):
        return max(
            PRESSURE_CONTACT_MIN_DELTA_HPA,
            float(noise_hpa) * PRESSURE_CONTACT_NOISE_SIGMA,
        )

    def _update_pressure_contact_gate(self, pressure_hpa):
        if not PRESSURE_CONTACT_GATE_ENABLED:
            self._set_contact_gate_status("disabled")
            return

        pressure = float(pressure_hpa)
        request_camera_zero = False
        log_message = None

        with self.sensor_data_lock:
            samples = self.contact_pressure_baseline_samples
            if self.contact_pressure_zero_hpa is None:
                samples.append(pressure)
                max_samples = max(1, PRESSURE_CONTACT_BASELINE_SAMPLES)
                if len(samples) < max_samples:
                    self.contact_pressure_state = "zeroing"
                    self.contact_pressure_delta_hpa = 0.0
                    status = f"zeroing pressure {len(samples)}/{max_samples}"
                else:
                    recent = samples[-max_samples:]
                    zero = float(np.median(np.asarray(recent, dtype=np.float32)))
                    noise = self._robust_pressure_noise(recent)
                    self.contact_pressure_zero_hpa = zero
                    self.contact_pressure_noise_hpa = noise
                    self.contact_pressure_threshold_hpa = self._pressure_contact_threshold(noise)
                    self.contact_pressure_state = "not_touching"
                    self.contact_pressure_delta_hpa = pressure - zero
                    status = (
                        f"not touching dP {self.contact_pressure_delta_hpa:+.2f} hPa "
                        f"(thr {self.contact_pressure_threshold_hpa:.2f})"
                    )
                    log_message = f"Pressure contact zero set at {zero:.2f} hPa."
            else:
                zero = float(self.contact_pressure_zero_hpa)
                delta = pressure - zero
                noise = float(self.contact_pressure_noise_hpa)
                threshold = self._pressure_contact_threshold(noise)
                release_threshold = max(
                    PRESSURE_CONTACT_RELEASE_DELTA_HPA,
                    min(threshold * 0.65, threshold - 0.05),
                )
                self.contact_pressure_delta_hpa = delta
                self.contact_pressure_threshold_hpa = threshold

                if self.contact_pressure_is_contact:
                    if delta <= release_threshold:
                        self.contact_pressure_below_count += 1
                    else:
                        self.contact_pressure_below_count = 0

                    if self.contact_pressure_below_count >= max(1, PRESSURE_CONTACT_RELEASE_SAMPLES):
                        self.contact_pressure_is_contact = False
                        self.contact_pressure_state = "not_touching"
                        self.contact_pressure_zero_request_sent = False
                        self.contact_pressure_below_count = 0
                        self.contact_pressure_above_count = 0
                        self.contact_pressure_zero_hpa = pressure
                        self.contact_pressure_baseline_samples = [pressure]
                        self.contact_pressure_delta_hpa = 0.0
                        status = "not touching dP +0.00 hPa"
                        log_message = "Pressure contact released; camera contact held at zero."
                    else:
                        self.contact_pressure_state = "contact"
                        status = f"CONTACT dP {delta:+.2f} hPa"
                else:
                    if delta >= threshold:
                        self.contact_pressure_above_count += 1
                    else:
                        self.contact_pressure_above_count = 0

                    if self.contact_pressure_above_count >= max(1, PRESSURE_CONTACT_CONFIRM_SAMPLES):
                        self.contact_pressure_is_contact = True
                        self.contact_pressure_state = "contact"
                        self.contact_pressure_below_count = 0
                        status = f"CONTACT dP {delta:+.2f} hPa"
                        if not self.contact_pressure_zero_request_sent:
                            self.contact_pressure_zero_request_sent = True
                            request_camera_zero = True
                            log_message = (
                                "Pressure contact detected; capturing camera zero at first contact "
                                f"(dP={delta:+.2f} hPa)."
                            )
                    else:
                        self.contact_pressure_state = "not_touching"
                        if delta < threshold * 0.45:
                            alpha = 0.025
                            self.contact_pressure_zero_hpa = (zero * (1.0 - alpha)) + (pressure * alpha)
                            samples.append(pressure)
                            keep = max(8, PRESSURE_CONTACT_BASELINE_SAMPLES * 4)
                            if len(samples) > keep:
                                del samples[:-keep]
                            self.contact_pressure_noise_hpa = self._robust_pressure_noise(samples)
                        status = f"not touching dP {delta:+.2f} hPa (thr {threshold:.2f})"

        self._set_contact_gate_status(status)
        if log_message:
            self.log(log_message)
        if request_camera_zero:
            self.surface_reset_zero_event.set()

    def _pressure_contact_gate_snapshot(self):
        if not PRESSURE_CONTACT_GATE_ENABLED:
            return {
                "enabled": False,
                "ready": True,
                "contact": True,
                "pressure": None,
                "zero": None,
                "delta": None,
                "threshold": None,
                "status": "disabled",
                "state": "disabled",
            }
        with self.sensor_data_lock:
            pressure = self.latest_pressure_hpa
            zero = self.contact_pressure_zero_hpa
            delta = self.contact_pressure_delta_hpa
            threshold = self.contact_pressure_threshold_hpa
            state = self.contact_pressure_state
            contact = self.contact_pressure_is_contact
            samples = len(self.contact_pressure_baseline_samples)
        ready = pressure is not None and zero is not None
        if pressure is None:
            status = "waiting pressure"
        elif not ready:
            status = f"zeroing pressure {samples}/{max(1, PRESSURE_CONTACT_BASELINE_SAMPLES)}"
        elif contact:
            status = f"CONTACT dP {float(delta or 0.0):+.2f} hPa"
        else:
            status = (
                f"not touching dP {float(delta or 0.0):+.2f} hPa "
                f"(thr {float(threshold or PRESSURE_CONTACT_MIN_DELTA_HPA):.2f})"
            )
        return {
            "enabled": True,
            "ready": ready,
            "contact": bool(contact and ready),
            "pressure": pressure,
            "zero": zero,
            "delta": delta,
            "threshold": threshold,
            "status": status,
            "state": state,
        }

    def _set_system_status(self, value):
        if hasattr(self, "system_status_var"):
            self._set_var(self.system_status_var, value)

        if not hasattr(self, "system_status_value_label"):
            return

        if isinstance(value, str) and value.startswith("Running:"):
            style_name = "StatusActive.TLabel"
        elif value == "Ready":
            style_name = "StatusReady.TLabel"
        else:
            style_name = "StatusWarn.TLabel"

        def apply_style():
            if not self.root.winfo_exists():
                return
            try:
                self.system_status_value_label.configure(style=style_name)
            except tk.TclError:
                pass

        self.root.after(0, apply_style)

    def _update_clock(self):
        if not self.root.winfo_exists():
            return
        if hasattr(self, "header_clock_var"):
            self.header_clock_var.set(time.strftime("%H:%M:%S"))
        self.root.after(1000, self._update_clock)

    def stop_all_tests(self):
        self.stop_force_cycle()
        self.stop_optical_flow()
        self.stop_blob_test()
        self.stop_camera()
        self.stop_limit_monitor()
        self.stop_stepper_move()
        self.stop_pressure_read()
        self.stop_loadcell_read()
        self.log("Stop all requested.")

    def _set_button_enabled(self, button, enabled):
        if button is None:
            return

        state = "normal" if enabled else "disabled"

        def apply_state():
            if not self.root.winfo_exists():
                return
            try:
                button.configure(state=state)
            except tk.TclError:
                pass

        if self.root.winfo_exists():
            self.root.after(0, apply_state)

    def _set_start_stop_controls(self, start_button, stop_button, running):
        self._set_button_enabled(start_button, not running)
        self._set_button_enabled(stop_button, running)

    def _set_stepper_direction(self, direction):
        normalized = str(direction).strip().lower()
        if normalized not in {"up", "down"}:
            return
        self.stepper_direction_var.set(normalized)
        self._refresh_stepper_direction_buttons()

    def _register_stepper_direction_button(self, direction, button):
        if not hasattr(self, "_stepper_direction_buttons"):
            self._stepper_direction_buttons = []
        self._stepper_direction_buttons.append((str(direction).strip().lower(), button))
        self._refresh_stepper_direction_buttons()

    def _refresh_stepper_direction_buttons(self):
        current = str(self.stepper_direction_var.get()).strip().lower()
        if current not in {"up", "down"}:
            current = "up"
            self.stepper_direction_var.set(current)
        for direction, button in getattr(self, "_stepper_direction_buttons", []):
            try:
                button.configure(
                    style="StepperDirActive.TButton" if direction == current else "StepperDir.TButton"
                )
            except tk.TclError:
                continue

    def _set_stepper_direction_buttons_enabled(self, enabled):
        for _direction, button in getattr(self, "_stepper_direction_buttons", []):
            self._set_button_enabled(button, enabled)

    def _build_stepper_direction_buttons(
        self,
        parent,
        *,
        row=None,
        column=None,
        sticky="ew",
        padx=0,
        pady=0,
        frame_style="TFrame",
    ):
        frame = ttk.Frame(parent, style=frame_style)
        frame.columnconfigure(0, weight=1)
        frame.columnconfigure(1, weight=1)
        up_btn = ttk.Button(
            frame,
            text="Up",
            style="StepperDir.TButton",
            command=lambda: self._set_stepper_direction("up"),
        )
        up_btn.grid(row=0, column=0, sticky="ew", padx=(0, 4))
        down_btn = ttk.Button(
            frame,
            text="Down",
            style="StepperDir.TButton",
            command=lambda: self._set_stepper_direction("down"),
        )
        down_btn.grid(row=0, column=1, sticky="ew")
        self._register_stepper_direction_button("up", up_btn)
        self._register_stepper_direction_button("down", down_btn)
        if row is not None and column is not None:
            frame.grid(row=row, column=column, sticky=sticky, padx=padx, pady=pady)
        return frame

    def _set_camera_controls(self, running):
        self._set_start_stop_controls(self.camera_start_btn, self.camera_stop_btn, running)

    def _set_limit_controls(self, running):
        self._set_start_stop_controls(self.limit_start_btn, self.limit_stop_btn, running)

    def _set_stepper_controls(self, running):
        self._set_start_stop_controls(self.stepper_start_btn, self.stepper_stop_btn, running)
        self._set_stepper_direction_buttons_enabled(not running)

    def _set_pressure_controls(self, running):
        self._set_start_stop_controls(self.pressure_start_btn, self.pressure_stop_btn, running)

    def _set_loadcell_controls(self, running):
        self._set_start_stop_controls(self.loadcell_start_btn, self.loadcell_stop_btn, running)
        self._set_button_enabled(self.loadcell_zero_btn, running)
        self._set_button_enabled(self.loadcell_calibrate_btn, not running)
        self._set_button_enabled(self.loadcell_reset_btn, not running)

    def _set_force_cycle_controls(self, running):
        self._set_start_stop_controls(self.force_cycle_start_btn, self.force_cycle_stop_btn, running)

    def _set_blob_controls(self, running):
        self._set_start_stop_controls(self.blob_start_btn, self.blob_stop_btn, running)
        self._set_button_enabled(self.blob_snapshot_btn, running)
        self._set_button_enabled(self.blob_reset_ref_btn, running)
        self._set_button_enabled(self.blob_settings_btn, not running)
        self._set_button_enabled(self.surface_reset_btn, running)
        self._refresh_deform_mode_controls()

    def _set_flow_controls(self, running):
        self._set_start_stop_controls(self.flow_start_btn, self.flow_stop_btn, running)
        self._set_button_enabled(self.flow_settings_btn, not running)

    @staticmethod
    def _close_stream_device(stream):
        if not isinstance(stream, dict):
            return
        backend = str(stream.get("backend", "")).lower()
        device = stream.get("device")
        if device is None:
            return

        if backend == "picamera2":
            try:
                device.stop()
            except Exception:
                pass
            try:
                if hasattr(device, "close"):
                    device.close()
            except Exception:
                pass
        else:
            try:
                if hasattr(device, "release"):
                    device.release()
            except Exception:
                pass

    def _wait_worker_stop(self, thread_getter, timeout=2.5):
        deadline = time.perf_counter() + timeout
        while time.perf_counter() < deadline:
            worker = thread_getter()
            if worker is None or not worker.is_alive():
                return True
            time.sleep(0.05)
        return False

    @staticmethod
    def _is_thread_running(worker_thread):
        return worker_thread is not None and worker_thread.is_alive()

    def _refresh_system_status(self):
        active = []
        if self.camera_running:
            active.append("camera preview")
        if self._is_thread_running(self.limit_thread):
            active.append("limit monitor")
        if self._is_thread_running(self.stepper_thread):
            active.append("stepper")
        if self._is_thread_running(self.pressure_thread):
            active.append("pressure")
        if self._is_thread_running(self.loadcell_thread):
            active.append("load cell")
        if self._is_thread_running(self.blob_thread):
            active.append("blob detector")
        if self._is_thread_running(self.flow_thread):
            active.append("optical flow")

        if active:
            self._set_system_status("Running: " + ", ".join(active))
        else:
            self._set_system_status("Ready")

    def log(self, message):
        timestamp = time.strftime("%H:%M:%S")
        self.log_queue.put(f"[{timestamp}] {message}")

    def clear_log(self):
        if not self.root.winfo_exists():
            return

        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.configure(state="disabled")
        self.log("Log cleared.")

    def save_log(self):
        default_dir = Path(__file__).resolve().parent / "logs"
        default_dir.mkdir(parents=True, exist_ok=True)
        default_name = f"all_in_one_log_{time.strftime('%Y%m%d_%H%M%S')}.txt"

        file_path = filedialog.asksaveasfilename(
            title="Save Log",
            initialdir=str(default_dir),
            initialfile=default_name,
            defaultextension=".txt",
            filetypes=(("Text files", "*.txt"), ("All files", "*.*")),
        )
        if not file_path:
            return

        try:
            text = self.log_text.get("1.0", "end-1c")
            Path(file_path).write_text(text, encoding="utf-8")
            self.log(f"Log saved to {file_path}")
        except Exception as exc:
            self.log(f"Failed to save log: {exc}")
            messagebox.showerror("Save Log Error", f"Could not save log file:\n{exc}")

    def _drain_log_queue(self):
        if not self.root.winfo_exists():
            return

        pending = []
        while True:
            try:
                pending.append(self.log_queue.get_nowait())
            except queue.Empty:
                break

        if pending:
            self.log_text.configure(state="normal")
            self.log_text.insert("end", "\n".join(pending) + "\n")

            line_count = int(self.log_text.index("end-1c").split(".")[0])
            if line_count > LOG_MAX_LINES:
                remove_lines = line_count - LOG_MAX_LINES
                self.log_text.delete("1.0", f"{remove_lines + 1}.0")

            self.log_text.see("end")
            self.log_text.configure(state="disabled")

        self.root.after(100, self._drain_log_queue)

    def _drain_blob_frame_queue(self):
        if not self.root.winfo_exists():
            return

        latest = None
        while True:
            try:
                latest = self.blob_frame_queue.get_nowait()
            except queue.Empty:
                break

        if latest is not None:
            self._render_blob_frame(latest)

        self.root.after(60, self._drain_blob_frame_queue)

    def _drain_flow_frame_queue(self):
        if not self.root.winfo_exists():
            return

        latest = None
        while True:
            try:
                latest = self.flow_frame_queue.get_nowait()
            except queue.Empty:
                break

        if latest is not None:
            self._render_flow_frame(latest)

        self.root.after(60, self._drain_flow_frame_queue)

    @staticmethod
    def _fit_preview_size(src_width, src_height, max_width, max_height):
        src_width = max(1, int(src_width))
        src_height = max(1, int(src_height))
        max_width = max(1, int(max_width))
        max_height = max(1, int(max_height))

        scale = min(max_width / float(src_width), max_height / float(src_height))
        target_w = max(1, int(src_width * scale))
        target_h = max(1, int(src_height * scale))
        return target_w, target_h

    def _render_blob_frame(self, payload):
        if Image is None or ImageTk is None:
            return

        frame_rgb = payload.get("frame_rgb")
        if frame_rgb is None:
            return

        resample_mode = Image.LANCZOS
        if hasattr(Image, "Resampling"):
            resample_mode = Image.Resampling.LANCZOS

        pil_image = Image.fromarray(frame_rgb)
        container_width = max(320, int(self.blob_preview_label.winfo_width()))
        container_height = max(240, int(self.blob_preview_label.winfo_height()))
        target_size = self._fit_preview_size(
            pil_image.width,
            pil_image.height,
            container_width,
            container_height,
        )
        if target_size != pil_image.size:
            pil_image = pil_image.resize(target_size, resample_mode)

        self.blob_photo = ImageTk.PhotoImage(image=pil_image)
        self.blob_preview_label.configure(image=self.blob_photo, text="")

        self.blob_dot_count_var.set(str(payload.get("dot_count", "-")))
        self.blob_pick_mode_var.set(str(payload.get("picked_mode", "-")))

        fps = payload.get("fps")
        if isinstance(fps, (int, float)) and fps > 0:
            self.blob_fps_var.set(f"{fps:.1f}")
        else:
            self.blob_fps_var.set("-")

        frame_ms = payload.get("frame_ms")
        if isinstance(frame_ms, (int, float)) and frame_ms >= 0:
            self.blob_latency_var.set(f"{frame_ms:.1f} ms")
        else:
            self.blob_latency_var.set("-")

        mean_disp = payload.get("mean_disp")
        if isinstance(mean_disp, (int, float)):
            self.blob_mean_disp_var.set(f"{mean_disp:.2f}")
        else:
            self.blob_mean_disp_var.set("-")

        max_disp = payload.get("max_disp")
        if isinstance(max_disp, (int, float)):
            self.blob_max_disp_var.set(f"{max_disp:.2f}")
        else:
            self.blob_max_disp_var.set("-")

        missing_ratio = payload.get("missing_ratio")
        if isinstance(missing_ratio, (int, float)):
            self.blob_missing_ratio_var.set(f"{missing_ratio:.3f}")
        else:
            self.blob_missing_ratio_var.set("-")

        new_tracks = payload.get("new_tracks")
        if isinstance(new_tracks, int):
            self.blob_new_tracks_var.set(str(new_tracks))
        else:
            self.blob_new_tracks_var.set("-")

        lost_tracks = payload.get("lost_tracks")
        if isinstance(lost_tracks, int):
            self.blob_lost_tracks_var.set(str(lost_tracks))
        else:
            self.blob_lost_tracks_var.set("-")

        distance_text = payload.get("distance_text")
        if isinstance(distance_text, str) and distance_text:
            self.blob_message_var.set(distance_text)

    def _clear_blob_frame_queue(self):
        while True:
            try:
                self.blob_frame_queue.get_nowait()
            except queue.Empty:
                break

    def _clear_flow_frame_queue(self):
        while True:
            try:
                self.flow_frame_queue.get_nowait()
            except queue.Empty:
                break

    def _enqueue_blob_frame(self, payload):
        while self.blob_frame_queue.full():
            try:
                self.blob_frame_queue.get_nowait()
            except queue.Empty:
                break

        try:
            self.blob_frame_queue.put_nowait(payload)
        except queue.Full:
            pass

    def _enqueue_flow_frame(self, payload):
        while self.flow_frame_queue.full():
            try:
                self.flow_frame_queue.get_nowait()
            except queue.Empty:
                break

        try:
            self.flow_frame_queue.put_nowait(payload)
        except queue.Full:
            pass

    def _render_flow_frame(self, payload):
        if Image is None or ImageTk is None:
            return

        frame_rgb = payload.get("frame_rgb")
        if frame_rgb is None:
            return

        resample_mode = Image.LANCZOS
        if hasattr(Image, "Resampling"):
            resample_mode = Image.Resampling.LANCZOS

        pil_image = Image.fromarray(frame_rgb)
        container_width = max(320, int(self.flow_preview_label.winfo_width()))
        container_height = max(240, int(self.flow_preview_label.winfo_height()))
        target_size = self._fit_preview_size(
            pil_image.width,
            pil_image.height,
            container_width,
            container_height,
        )
        if target_size != pil_image.size:
            pil_image = pil_image.resize(target_size, resample_mode)

        self.flow_photo = ImageTk.PhotoImage(image=pil_image)
        self.flow_preview_label.configure(image=self.flow_photo, text="")

        point_count = payload.get("point_count")
        if isinstance(point_count, int):
            self.flow_points_var.set(str(point_count))

        mean_disp = payload.get("mean_disp")
        if isinstance(mean_disp, (int, float)):
            self.flow_mean_disp_var.set(f"{mean_disp:.2f}")

        max_disp = payload.get("max_disp")
        if isinstance(max_disp, (int, float)):
            self.flow_max_disp_var.set(f"{max_disp:.2f}")

        fps = payload.get("fps")
        if isinstance(fps, (int, float)) and fps > 0:
            self.flow_fps_var.set(f"{fps:.1f}")
        else:
            self.flow_fps_var.set("-")

    def _is_blob_running(self):
        return self.blob_thread is not None and self.blob_thread.is_alive()

    def _is_flow_running(self):
        return self.flow_thread is not None and self.flow_thread.is_alive()

    def open_blob_settings_dialog(self):
        if self._is_blob_running():
            messagebox.showinfo("Dot Settings", "Stop detection before changing advanced settings.")
            return

        dialog = tk.Toplevel(self.root)
        dialog.title("Detection Settings")
        dialog.geometry("720x560")
        dialog.transient(self.root)
        dialog.grab_set()

        frm = ttk.Frame(dialog, padding=12)
        frm.pack(fill="both", expand=True)
        frm.columnconfigure(0, weight=1)
        frm.rowconfigure(1, weight=1)

        ttk.Label(
            frm,
            text="Grouped by dot_pipeline stages: Camera, Detection, Tracking, Visualization",
            style="SectionHint.TLabel",
        ).grid(row=0, column=0, sticky="w")

        settings_notebook = ttk.Notebook(frm)
        settings_notebook.grid(row=1, column=0, sticky="nsew", pady=(8, 0))

        camera_tab = ttk.Frame(settings_notebook, padding=12)
        camera_tab.columnconfigure(1, weight=1)
        camera_tab.columnconfigure(3, weight=1)
        settings_notebook.add(camera_tab, text="Camera")

        ttk.Label(camera_tab, text="Backend:").grid(row=0, column=0, sticky="w")
        ttk.Combobox(
            camera_tab,
            textvariable=self.blob_camera_backend_var,
            values=("auto", "picamera2", "opencv"),
            state="readonly",
        ).grid(row=0, column=1, sticky="ew")
        ttk.Label(camera_tab, text="Index:").grid(row=0, column=2, sticky="w", padx=(12, 0))
        ttk.Entry(camera_tab, textvariable=self.blob_camera_index_var).grid(row=0, column=3, sticky="ew")

        ttk.Label(camera_tab, text="Width:").grid(row=1, column=0, sticky="w", pady=(6, 0))
        ttk.Entry(camera_tab, textvariable=self.blob_cam_width_var).grid(row=1, column=1, sticky="ew", pady=(6, 0))
        ttk.Label(camera_tab, text="Height:").grid(row=1, column=2, sticky="w", pady=(6, 0), padx=(12, 0))
        ttk.Entry(camera_tab, textvariable=self.blob_cam_height_var).grid(row=1, column=3, sticky="ew", pady=(6, 0))

        ttk.Label(camera_tab, text="Autofocus:").grid(row=2, column=0, sticky="w", pady=(6, 0))
        ttk.Combobox(
            camera_tab,
            textvariable=self.blob_autofocus_var,
            values=("continuous", "manual", "off"),
            state="readonly",
        ).grid(row=2, column=1, sticky="ew", pady=(6, 0))
        ttk.Label(camera_tab, text="Lens Position:").grid(row=2, column=2, sticky="w", pady=(6, 0), padx=(12, 0))
        ttk.Entry(camera_tab, textvariable=self.blob_lens_position_var).grid(row=2, column=3, sticky="ew", pady=(6, 0))

        ttk.Label(camera_tab, text="Exposure EV:").grid(row=3, column=0, sticky="w", pady=(6, 0))
        ttk.Entry(camera_tab, textvariable=self.blob_exposure_ev_var).grid(row=3, column=1, sticky="ew", pady=(6, 0))
        ttk.Label(camera_tab, text="Exposure time (us):").grid(
            row=3,
            column=2,
            sticky="w",
            pady=(6, 0),
            padx=(12, 0),
        )
        ttk.Entry(camera_tab, textvariable=self.blob_exposure_time_us_var).grid(
            row=3,
            column=3,
            sticky="ew",
            pady=(6, 0),
        )

        ttk.Label(camera_tab, text="Analogue gain:").grid(row=4, column=0, sticky="w", pady=(6, 0))
        ttk.Entry(camera_tab, textvariable=self.blob_analogue_gain_var).grid(
            row=4,
            column=1,
            sticky="ew",
            pady=(6, 0),
        )
        ttk.Label(camera_tab, text="AWB:").grid(
            row=4,
            column=2,
            sticky="w",
            pady=(6, 0),
            padx=(12, 0),
        )
        ttk.Combobox(
            camera_tab,
            textvariable=self.blob_awb_mode_var,
            values=("auto", "manual"),
            state="readonly",
        ).grid(row=4, column=3, sticky="ew", pady=(6, 0))

        ttk.Label(camera_tab, text="Colour gains R,B:").grid(row=5, column=0, sticky="w", pady=(6, 0))
        ttk.Entry(camera_tab, textvariable=self.blob_colour_gains_var).grid(
            row=5,
            column=1,
            sticky="ew",
            pady=(6, 0),
        )

        detect_tab = ttk.Frame(settings_notebook, padding=12)
        detect_tab.columnconfigure(1, weight=1)
        detect_tab.columnconfigure(3, weight=1)
        settings_notebook.add(detect_tab, text="Detection")

        ttk.Label(detect_tab, text="Polarity:").grid(row=0, column=0, sticky="w")
        ttk.Combobox(
            detect_tab,
            textvariable=self.blob_mode_var,
            values=("auto", "dark", "bright"),
            state="readonly",
        ).grid(row=0, column=1, sticky="ew")

        ttk.Label(detect_tab, text="Min area:").grid(row=1, column=0, sticky="w", pady=(6, 0))
        ttk.Entry(detect_tab, textvariable=self.blob_min_area_var).grid(row=1, column=1, sticky="ew", pady=(6, 0))
        ttk.Label(detect_tab, text="Max area:").grid(row=1, column=2, sticky="w", pady=(6, 0), padx=(12, 0))
        ttk.Entry(detect_tab, textvariable=self.blob_max_area_var).grid(row=1, column=3, sticky="ew", pady=(6, 0))

        ttk.Label(detect_tab, text="Min circularity:").grid(row=2, column=0, sticky="w", pady=(6, 0))
        ttk.Entry(detect_tab, textvariable=self.blob_min_circularity_var).grid(row=2, column=1, sticky="ew", pady=(6, 0))
        ttk.Checkbutton(detect_tab, text="Use ROI", variable=self.blob_use_roi_var).grid(row=2, column=2, sticky="w", padx=(12, 0), pady=(6, 0))
        ttk.Entry(detect_tab, textvariable=self.blob_roi_percent_var).grid(row=2, column=3, sticky="ew", pady=(6, 0))
        ttk.Label(detect_tab, text="ROI (%)").grid(row=3, column=3, sticky="w")

        ttk.Checkbutton(
            detect_tab,
            text="Illumination normalization",
            variable=self.blob_use_illum_norm_var,
        ).grid(row=4, column=0, columnspan=2, sticky="w", pady=(8, 0))
        ttk.Label(detect_tab, text="Illum method:").grid(row=4, column=2, sticky="w", pady=(8, 0), padx=(12, 0))
        ttk.Combobox(
            detect_tab,
            textvariable=self.blob_illum_method_var,
            values=("clahe", "clahe_bg"),
            state="readonly",
        ).grid(row=4, column=3, sticky="ew", pady=(8, 0))

        track_tab = ttk.Frame(settings_notebook, padding=12)
        track_tab.columnconfigure(1, weight=1)
        settings_notebook.add(track_tab, text="Tracking")

        ttk.Label(track_tab, text="Proc scale:").grid(row=0, column=0, sticky="w")
        ttk.Entry(track_tab, textvariable=self.blob_proc_scale_var).grid(row=0, column=1, sticky="ew")
        ttk.Label(track_tab, text="Match distance:").grid(row=1, column=0, sticky="w", pady=(6, 0))
        ttk.Entry(track_tab, textvariable=self.blob_match_dist_var).grid(row=1, column=1, sticky="ew", pady=(6, 0))
        ttk.Label(track_tab, text="Distance scale:").grid(row=2, column=0, sticky="w", pady=(6, 0))
        ttk.Entry(track_tab, textvariable=self.blob_distance_scale_var).grid(row=2, column=1, sticky="ew", pady=(6, 0))
        ttk.Label(track_tab, text="Distance unit:").grid(row=3, column=0, sticky="w", pady=(6, 0))
        ttk.Entry(track_tab, textvariable=self.blob_distance_unit_var).grid(row=3, column=1, sticky="ew", pady=(6, 0))
        ttk.Checkbutton(track_tab, text="Periodic robustness test", variable=self.blob_run_robust_test_var).grid(
            row=4, column=0, columnspan=2, sticky="w", pady=(8, 0)
        )

        viz_tab = ttk.Frame(settings_notebook, padding=12)
        viz_tab.columnconfigure(1, weight=1)
        settings_notebook.add(viz_tab, text="Visualization")

        ttk.Label(viz_tab, text="Mosaic scale:").grid(row=0, column=0, sticky="w")
        ttk.Entry(viz_tab, textvariable=self.blob_mosaic_scale_var).grid(row=0, column=1, sticky="ew")
        ttk.Label(viz_tab, text="Output type:").grid(row=1, column=0, sticky="w", pady=(8, 0))
        ttk.Combobox(
            viz_tab,
            textvariable=self.blob_view_type_var,
            values=(
                "auto",
                "mosaic",
                "overlay",
                "vector",
                "heatmap",
                "pointcloud",
                "binary",
                "blob",
                "optical_flow_2d",
                "surface_3d",
            ),
            state="readonly",
        ).grid(row=1, column=1, sticky="ew", pady=(8, 0))
        ttk.Checkbutton(viz_tab, text="Enable mosaic view", variable=self.blob_use_mosaic_var).grid(
            row=2, column=0, columnspan=2, sticky="w", pady=(8, 0)
        )
        ttk.Checkbutton(viz_tab, text="Enable pointcloud panel", variable=self.blob_use_pointcloud_var).grid(
            row=3, column=0, columnspan=2, sticky="w", pady=(4, 0)
        )

        action = ttk.Frame(frm)
        action.grid(row=2, column=0, sticky="e", pady=(10, 0))
        ttk.Button(action, text="Apply Preset", command=self._apply_blob_preset).pack(side="right", padx=(0, 8))
        ttk.Button(action, text="Close", command=dialog.destroy).pack(side="right")

    def open_flow_settings_dialog(self):
        if self._is_flow_running():
            messagebox.showinfo("Flow Settings", "Stop optical flow before changing settings.")
            return

        dialog = tk.Toplevel(self.root)
        dialog.title("Optical Flow Settings")
        dialog.geometry("460x460")
        dialog.transient(self.root)
        dialog.grab_set()

        frm = ttk.Frame(dialog, padding=12)
        frm.pack(fill="both", expand=True)
        frm.columnconfigure(1, weight=1)

        fields = [
            ("Camera Backend", self.flow_camera_backend_var),
            ("Camera Index", self.flow_camera_index_var),
            ("Width", self.flow_cam_width_var),
            ("Height", self.flow_cam_height_var),
            ("Proc Scale", self.flow_proc_scale_var),
            ("ROI Scale", self.flow_roi_scale_var),
            ("3D ROI Scale", self.flow_3d_roi_scale_var),
            ("Max Points", self.flow_max_points_var),
            ("Quality", self.flow_quality_var),
            ("Min Distance", self.flow_min_distance_var),
            ("Reseed Interval", self.flow_reseed_var),
        ]

        for idx, (label, variable) in enumerate(fields):
            ttk.Label(frm, text=label + ":").grid(row=idx, column=0, sticky="w", pady=4)
            ttk.Entry(frm, textvariable=variable).grid(row=idx, column=1, sticky="ew", pady=4)

        ttk.Checkbutton(frm, text="Show vectors", variable=self.flow_show_vectors_var).grid(
            row=len(fields), column=0, columnspan=2, sticky="w", pady=(8, 0)
        )

        action = ttk.Frame(frm)
        action.grid(row=len(fields) + 1, column=0, columnspan=2, sticky="e", pady=(10, 0))
        ttk.Button(action, text="Close", command=dialog.destroy).pack(side="right")

    def _collect_flow_params(self):
        backend = self.flow_camera_backend_var.get().strip().lower()
        if backend not in {"auto", "picamera2", "opencv"}:
            raise ValueError("Flow backend must be auto, picamera2, or opencv.")

        camera_index = int(self.flow_camera_index_var.get().strip())
        width = int(self.flow_cam_width_var.get().strip())
        height = int(self.flow_cam_height_var.get().strip())
        proc_scale = float(self.flow_proc_scale_var.get().strip())
        roi_scale = float(self.flow_roi_scale_var.get().strip())
        roi_3d_scale = float(self.flow_3d_roi_scale_var.get().strip())
        max_points = int(self.flow_max_points_var.get().strip())
        quality = float(self.flow_quality_var.get().strip())
        min_distance = float(self.flow_min_distance_var.get().strip())
        reseed_interval = int(self.flow_reseed_var.get().strip())

        if camera_index < 0:
            raise ValueError("Flow camera index must be >= 0.")
        if width <= 0 or height <= 0:
            raise ValueError("Flow width/height must be > 0.")
        if not (0.2 <= proc_scale <= 1.0):
            raise ValueError("Flow processing scale must be between 0.2 and 1.0.")
        if not (0.1 <= roi_scale <= 1.0):
            raise ValueError("Flow ROI scale must be between 0.1 and 1.0.")
        if not (0.1 <= roi_3d_scale <= 1.0):
            raise ValueError("3D flow ROI scale must be between 0.1 and 1.0.")
        if max_points <= 0:
            raise ValueError("Flow max points must be > 0.")
        if not (0.001 <= quality <= 0.5):
            raise ValueError("Flow quality must be between 0.001 and 0.5.")
        if min_distance <= 0:
            raise ValueError("Flow min distance must be > 0.")
        if reseed_interval < 1:
            raise ValueError("Flow reseed interval must be >= 1.")

        return {
            "backend": backend,
            "camera_index": camera_index,
            "width": width,
            "height": height,
            "proc_scale": proc_scale,
            "roi_scale": roi_scale,
            "roi_3d_scale": roi_3d_scale,
            "max_points": max_points,
            "quality": quality,
            "min_distance": min_distance,
            "reseed_interval": reseed_interval,
            "show_vectors": bool(self.flow_show_vectors_var.get()),
        }

    def start_optical_flow(self):
        if self._is_flow_running():
            self.log("Optical flow is already running.")
            return

        if self.camera_running:
            self.log("Camera preview is running. Stopping preview before optical flow starts.")
            self.stop_camera()
        if self._is_blob_running():
            self.log("Live detection is running. Stopping detection before optical flow starts.")
            self.stop_blob_test()
            if self._is_blob_running():
                self.log("Cannot start optical flow: detection is still shutting down.")
                messagebox.showwarning(
                    "Camera Busy",
                    "Detection is still releasing the camera. Please wait a moment and try again.",
                )
                return
            time.sleep(0.2)

        try:
            self._load_blob_backend()
            self.flow_params = self._collect_flow_params()
        except Exception as exc:
            self.flow_state_var.set("Error")
            self.flow_message_var.set(str(exc))
            self.log(f"Optical flow setup error: {exc}")
            messagebox.showerror("Optical Flow Error", str(exc))
            self._set_flow_controls(False)
            self._refresh_system_status()
            return

        self.flow_stop_event.clear()
        self._clear_flow_frame_queue()
        self.flow_points_var.set("-")
        self.flow_mean_disp_var.set("-")
        self.flow_max_disp_var.set("-")
        self.flow_fps_var.set("-")
        self.flow_preview_label.configure(image="", text="Starting optical flow...")
        self.flow_photo = None

        self.flow_thread = threading.Thread(target=self._flow_worker, daemon=True)
        self.flow_thread.start()

        self.flow_state_var.set("Starting")
        self.flow_message_var.set("Opening camera...")
        self._set_flow_controls(True)
        self._refresh_system_status()
        self.log("Optical flow start requested.")

    def _flow_worker(self):
        pipe = None
        cv2 = None
        stream = None
        error_message = None

        prev_gray = None
        prev_pts = None
        frame_counter = 0
        fps = 0.0
        fps_start = time.perf_counter()

        try:
            pipe = self.blob_module
            cv2 = self.blob_cv2
            params = dict(self.flow_params)

            stream = pipe.open_camera_stream(
                camera_index=params["camera_index"],
                width=params["width"],
                height=params["height"],
                backend=params["backend"],
                autofocus="continuous",
                lens_position=1.0,
                exposure_ev=0.8,
            )
            self.flow_camera = stream

            self._set_var(self.flow_state_var, "Running")
            self._set_var(self.flow_message_var, "Optical flow running.")
            self._refresh_system_status()

            while not self.flow_stop_event.is_set():
                ok, frame_bgr = pipe.read_camera_frame(stream)
                if not ok or frame_bgr is None:
                    raise RuntimeError("Camera read failed in optical flow worker.")

                if params["proc_scale"] != 1.0:
                    frame_bgr = cv2.resize(
                        frame_bgr,
                        (
                            int(frame_bgr.shape[1] * params["proc_scale"]),
                            int(frame_bgr.shape[0] * params["proc_scale"]),
                        ),
                        interpolation=cv2.INTER_AREA,
                    )

                gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
                h, w = gray.shape[:2]
                roi_mask = np.zeros((h, w), dtype=np.uint8)
                cx, cy = w // 2, h // 2
                rx = max(8, int((w * params["roi_scale"]) * 0.5))
                ry = max(8, int((h * params["roi_scale"]) * 0.5))
                cv2.ellipse(roi_mask, (cx, cy), (rx, ry), 0, 0, 360, 255, -1)

                if prev_gray is None or prev_pts is None or len(prev_pts) < 8 or (frame_counter % params["reseed_interval"] == 0):
                    prev_pts = cv2.goodFeaturesToTrack(
                        gray,
                        maxCorners=params["max_points"],
                        qualityLevel=params["quality"],
                        minDistance=params["min_distance"],
                        mask=roi_mask,
                    )
                    prev_gray = gray
                    frame_counter += 1
                    continue

                next_pts, status, _err = cv2.calcOpticalFlowPyrLK(prev_gray, gray, prev_pts, None)
                if next_pts is None or status is None:
                    prev_gray = gray
                    prev_pts = None
                    frame_counter += 1
                    continue

                good_old = prev_pts[status.flatten() == 1]
                good_new = next_pts[status.flatten() == 1]

                valid_old = []
                valid_new = []
                for old_pt, new_pt in zip(good_old, good_new):
                    ox, oy = old_pt.ravel()
                    nx, ny = new_pt.ravel()
                    if 0 <= int(nx) < roi_mask.shape[1] and 0 <= int(ny) < roi_mask.shape[0] and roi_mask[int(ny), int(nx)] > 0:
                        valid_old.append((ox, oy))
                        valid_new.append((nx, ny))

                if not valid_new:
                    prev_gray = gray
                    prev_pts = None
                    frame_counter += 1
                    continue

                disp_vals = [float(np.hypot(nx - ox, ny - oy)) for (ox, oy), (nx, ny) in zip(valid_old, valid_new)]
                mean_disp = float(np.mean(disp_vals)) if disp_vals else 0.0
                max_disp = float(np.max(disp_vals)) if disp_vals else 0.0

                display = frame_bgr.copy()
                if params["show_vectors"]:
                    for (ox, oy), (nx, ny) in zip(valid_old, valid_new):
                        cv2.arrowedLine(display, (int(ox), int(oy)), (int(nx), int(ny)), (0, 255, 255), 1, tipLength=0.25)
                        cv2.circle(display, (int(nx), int(ny)), 2, (0, 220, 0), -1)

                frame_counter += 1
                now = time.perf_counter()
                elapsed = now - fps_start
                if elapsed >= 0.5:
                    fps = frame_counter / elapsed
                    frame_counter = 0
                    fps_start = now

                frame_rgb = cv2.cvtColor(display, cv2.COLOR_BGR2RGB)
                self._enqueue_flow_frame(
                    {
                        "frame_rgb": frame_rgb,
                        "point_count": len(valid_new),
                        "mean_disp": mean_disp,
                        "max_disp": max_disp,
                        "fps": fps,
                    }
                )

                prev_gray = gray
                prev_pts = np.array(valid_new, dtype=np.float32).reshape(-1, 1, 2)

        except Exception as exc:
            error_message = str(exc)
            self.log(f"Optical flow worker error: {exc}")
            self._set_var(self.flow_state_var, "Error")
            self._set_var(self.flow_message_var, error_message)
        finally:
            if stream is not None and pipe is not None:
                try:
                    pipe.close_camera_stream(stream)
                except Exception:
                    pass
            self._close_stream_device(stream)

            self.flow_camera = None
            self.flow_thread = None
            if error_message is None:
                self._set_var(self.flow_state_var, "Stopped")
                self._set_var(self.flow_message_var, "Optical flow stopped.")

            self._set_flow_controls(False)
            self._refresh_system_status()
            self.log("Optical flow stopped.")

    def stop_optical_flow(self):
        was_running = self._is_flow_running()
        self.flow_stop_event.set()

        if not was_running:
            self.flow_state_var.set("Stopped")
            self.flow_message_var.set("Optical flow is not running.")
            self._set_flow_controls(False)
            self._refresh_system_status()
            return

        self.flow_state_var.set("Stopping")
        self.flow_message_var.set("Stopping optical flow...")
        self._set_button_enabled(self.flow_start_btn, False)
        self._set_button_enabled(self.flow_stop_btn, False)
        self.log("Optical flow stop requested.")

        if self.flow_thread is not None and self.flow_thread.is_alive():
            self.flow_thread.join(timeout=1.5)
            if self.flow_thread is not None and self.flow_thread.is_alive():
                self.log("Optical flow worker is taking longer than expected to stop.")
                self._close_stream_device(self.flow_camera)
                stopped = self._wait_worker_stop(lambda: self.flow_thread, timeout=2.0)
                if not stopped:
                    self.log("Optical flow still active after forced camera release.")

    def _configure_cv2_multicore(self, cv2):
        cpu_count = os.cpu_count() or 1
        thread_text = os.environ.get("PI_BUBBLE_VISION_THREADS", "").strip()
        if thread_text:
            try:
                thread_count = int(thread_text)
            except ValueError:
                thread_count = cpu_count
        else:
            thread_count = cpu_count
        thread_count = max(1, min(thread_count, cpu_count))

        try:
            cv2.setUseOptimized(True)
        except Exception:
            pass

        actual_threads = thread_count
        try:
            cv2.setNumThreads(thread_count)
            actual_threads = int(cv2.getNumThreads())
        except Exception:
            actual_threads = thread_count

        self.log(f"OpenCV optimized mode enabled; vision threads={actual_threads}/{cpu_count}.")
        return actual_threads

    def _load_blob_backend(self):
        if self.blob_module is not None and self.blob_cv2 is not None:
            return

        if Image is None or ImageTk is None:
            raise RuntimeError("Pillow is required for embedded preview. Install with: pip install pillow")

        try:
            import cv2
        except Exception as exc:
            raise RuntimeError(f"OpenCV import failed: {exc}") from exc
        self._configure_cv2_multicore(cv2)

        try:
            from JNR import dot_pipeline as dot_pipeline_module
        except Exception as exc:
            raise RuntimeError(f"Could not import JNR/dot_pipeline.py: {exc}") from exc

        required = [
            "preprocess",
            "threshold_image",
            "detect_centroids_from_binary",
            "detect_blobs",
            "extract_centroids",
            "build_blob_view",
            "draw_displacement_vectors",
            "build_displacement_heatmap",
            "overlay_heatmap",
            "build_pointcloud_view",
            "build_mosaic",
            "update_tracks",
            "robustness_test",
            "open_camera_stream",
            "read_camera_frame",
            "close_camera_stream",
            "compute_displacements",
        ]
        missing = [name for name in required if not hasattr(dot_pipeline_module, name)]
        if missing:
            raise RuntimeError("dot_pipeline.py is missing required functions: " + ", ".join(missing))

        self.blob_module = dot_pipeline_module
        self.blob_cv2 = cv2

    @staticmethod
    def _parse_optional_int(variable, label):
        text = variable.get().strip()
        if not text:
            return None
        try:
            return int(float(text))
        except ValueError as exc:
            raise ValueError(f"{label} must be blank or a number.") from exc

    @staticmethod
    def _parse_optional_float(variable, label):
        text = variable.get().strip()
        if not text:
            return None
        try:
            return float(text)
        except ValueError as exc:
            raise ValueError(f"{label} must be blank or a number.") from exc

    @staticmethod
    def _parse_colour_gains(variable):
        text = variable.get().strip()
        if not text:
            return None
        pieces = [piece.strip() for piece in text.split(",")]
        if len(pieces) != 2:
            raise ValueError("Colour gains must be blank or two values: R,B.")
        try:
            red_gain = float(pieces[0])
            blue_gain = float(pieces[1])
        except ValueError as exc:
            raise ValueError("Colour gains must be numeric, for example 1.5,1.5.") from exc
        if red_gain <= 0.0 or blue_gain <= 0.0:
            raise ValueError("Colour gains must be greater than 0.")
        return red_gain, blue_gain

    def _collect_blob_params(self):
        self._sync_surface_measurement_mode_from_var()
        camera_backend = self.blob_camera_backend_var.get().strip().lower()
        if camera_backend not in {"auto", "picamera2", "opencv"}:
            raise ValueError("Backend must be auto, picamera2, or opencv.")

        autofocus = self.blob_autofocus_var.get().strip().lower()
        if autofocus not in {"continuous", "manual", "off"}:
            raise ValueError("Autofocus must be continuous, manual, or off.")

        mode = self.blob_mode_var.get().strip().lower()
        if mode not in {"auto", "dark", "bright"}:
            raise ValueError("Mode must be auto, dark, or bright.")

        view_type = self.blob_view_type_var.get().strip().lower()
        if self._is_3d_object_mode():
            view_type = "surface_3d"
            self.blob_view_type_var.set("surface_3d")
        if view_type == "optical_flow":
            view_type = "optical_flow_2d"
        if view_type == "optical_flow_3d":
            view_type = "surface_3d"
        valid_view_types = {
            "auto",
            "mosaic",
            "overlay",
            "vector",
            "heatmap",
            "pointcloud",
            "binary",
            "blob",
            "optical_flow_2d",
            "surface_3d",
            "optical_flow_3d",
        }
        if view_type not in valid_view_types:
            raise ValueError(
                "Output type must be one of: auto, mosaic, overlay, vector, heatmap, pointcloud, binary, blob, optical_flow_2d, surface_3d, optical_flow_3d."
            )

        vision_backend = self.vision_backend_var.get().strip().lower().replace(" ", "_")
        if vision_backend in {"remote", "cuda", "remote_nvidia"}:
            vision_backend = "remote_cuda"
        if vision_backend not in {"local", "remote_cuda"}:
            raise ValueError("Vision backend must be local or remote CUDA.")
        remote_vision_url = self.remote_vision_url_var.get().strip().rstrip("/")
        if vision_backend == "remote_cuda" and not remote_vision_url:
            raise ValueError("Remote CUDA URL is required.")
        if remote_vision_url and not remote_vision_url.startswith(("http://", "https://")):
            raise ValueError("Remote CUDA URL must start with http:// or https://.")
        if vision_backend == "remote_cuda" and camera_backend in {"auto", "opencv"}:
            self.log("Remote CUDA low-load mode uses Picamera2 for Pi CSI capture.")
            camera_backend = "picamera2"
            self.blob_camera_backend_var.set("picamera2")

        illum_method = self.blob_illum_method_var.get().strip().lower()
        if illum_method not in {"clahe", "clahe_bg"}:
            raise ValueError("Illumination method must be clahe or clahe_bg.")

        camera_index = int(self.blob_camera_index_var.get().strip())
        cam_width = int(self.blob_cam_width_var.get().strip())
        cam_height = int(self.blob_cam_height_var.get().strip())
        lens_position = float(self.blob_lens_position_var.get().strip())
        exposure_ev = float(self.blob_exposure_ev_var.get().strip())
        exposure_time_us = self._parse_optional_int(
            self.blob_exposure_time_us_var,
            "Exposure time",
        )
        analogue_gain = self._parse_optional_float(
            self.blob_analogue_gain_var,
            "Analogue gain",
        )
        awb_mode = self.blob_awb_mode_var.get().strip().lower()
        if awb_mode not in {"auto", "manual"}:
            raise ValueError("AWB mode must be auto or manual.")
        colour_gains = self._parse_colour_gains(self.blob_colour_gains_var)

        min_area = float(self.blob_min_area_var.get().strip())
        max_area = float(self.blob_max_area_var.get().strip())
        min_circularity = float(self.blob_min_circularity_var.get().strip())
        roi_percent = float(self.blob_roi_percent_var.get().strip())
        proc_scale = float(self.blob_proc_scale_var.get().strip())
        match_dist = float(self.blob_match_dist_var.get().strip())
        mosaic_scale = float(self.blob_mosaic_scale_var.get().strip())
        distance_scale = float(self.blob_distance_scale_var.get().strip())
        distance_unit = self.blob_distance_unit_var.get().strip()

        if vision_backend == "remote_cuda":
            capped_width = min(cam_width, REMOTE_VISION_CAPTURE_WIDTH)
            capped_height = min(cam_height, REMOTE_VISION_CAPTURE_HEIGHT)
            if (capped_width, capped_height) != (cam_width, cam_height):
                self.log(
                    "Remote CUDA low-load capture capped "
                    f"{cam_width}x{cam_height} -> {capped_width}x{capped_height}."
                )
                cam_width = capped_width
                cam_height = capped_height
                self.blob_cam_width_var.set(str(cam_width))
                self.blob_cam_height_var.set(str(cam_height))
            if proc_scale != 1.0:
                self.log("Remote CUDA low-load mode disables Pi-side resize.")
                proc_scale = 1.0
                self.blob_proc_scale_var.set("1.0")

        if camera_index < 0:
            raise ValueError("Camera index must be >= 0.")
        if cam_width <= 0 or cam_height <= 0:
            raise ValueError("Camera width/height must be > 0.")
        if min_area < 10:
            raise ValueError("Min area must be >= 10.")
        if max_area <= min_area:
            raise ValueError("Max area must be greater than min area.")
        if not (0.01 <= min_circularity <= 0.95):
            raise ValueError("Min circularity must be between 0.01 and 0.95.")
        if not (10.0 <= roi_percent <= 100.0):
            raise ValueError("ROI size must be between 10 and 100 percent.")
        if not (0.2 <= proc_scale <= 1.0):
            raise ValueError("Processing scale must be between 0.2 and 1.0.")
        if match_dist <= 0:
            raise ValueError("Match distance must be > 0.")
        if exposure_time_us is not None and not (50 <= exposure_time_us <= 1000000):
            raise ValueError("Exposure time must be blank or 50-1000000 us.")
        if analogue_gain is not None and not (0.1 <= analogue_gain <= 64.0):
            raise ValueError("Analogue gain must be blank or between 0.1 and 64.0.")
        if awb_mode == "manual" and colour_gains is None:
            raise ValueError("Manual AWB needs colour gains, for example 1.5,1.5.")
        if not (0.2 <= mosaic_scale <= 1.0):
            raise ValueError("Mosaic scale must be between 0.2 and 1.0.")
        if distance_scale <= 0:
            raise ValueError("Distance scale must be > 0.")
        if not distance_unit:
            raise ValueError("Distance unit is required (e.g. px, mm).")

        params = {
            "camera_index": camera_index,
            "camera_backend": camera_backend,
            "cam_width": cam_width,
            "cam_height": cam_height,
            "autofocus": autofocus,
            "lens_position": lens_position,
            "exposure_ev": exposure_ev,
            "exposure_time_us": exposure_time_us,
            "analogue_gain": analogue_gain,
            "awb_mode": awb_mode,
            "colour_gains": colour_gains,
            "proc_scale": proc_scale,
            "match_dist": match_dist,
            "mosaic_scale": mosaic_scale,
            "use_pointcloud": bool(self.blob_use_pointcloud_var.get()),
            "use_mosaic": bool(self.blob_use_mosaic_var.get()),
            "run_robust_test": bool(self.blob_run_robust_test_var.get()),
            "distance_scale": distance_scale,
            "distance_unit": distance_unit,
            "vision_backend": vision_backend,
            "remote_vision_url": remote_vision_url,
        }
        params["mode"] = mode
        params["min_area"] = min_area
        params["max_area"] = max_area
        params["min_circularity"] = min_circularity
        params["use_roi"] = bool(self.blob_use_roi_var.get())
        params["roi_scale"] = roi_percent / 100.0
        params["view_type"] = view_type
        params["measurement_mode"] = "3d_object" if self._is_3d_object_mode() else "flat_deform"
        params["use_illum_norm"] = bool(self.blob_use_illum_norm_var.get())
        params["illum_method"] = illum_method
        return params

    @staticmethod
    def _wait_with_cancel(seconds, stop_event, slice_seconds=0.05):
        end_time = time.perf_counter() + seconds
        while time.perf_counter() < end_time:
            if stop_event.is_set():
                return False
            remaining = end_time - time.perf_counter()
            time.sleep(min(slice_seconds, max(0.0, remaining)))
        return True

    def start_camera(self):
        if self._is_flow_running():
            self.log("Camera preview blocked while optical flow is running.")
            messagebox.showwarning(
                "Optical Flow Running",
                "Stop optical flow before opening camera preview.",
            )
            return

        if self._is_blob_running():
            self.log("Camera preview blocked while Blob test is running.")
            messagebox.showwarning(
                "Blob Test Running",
                "Stop Blob Test before opening camera preview.",
            )
            return

        if self.camera_running:
            self.log("Camera preview is already running.")
            return

        try:
            from picamzero import Camera

            self.camera = Camera()
            self.camera.start_preview()
            self.camera_running = True
            self._reset_contact_deform_zero_state("Contact deform zero pending. Press Reset Zero.")
            self._request_pressure_zero_from_camera_open()
            self.camera_status_var.set("Preview running (picamzero)")
            self._set_camera_controls(True)
            self._refresh_system_status()
            self.log("Camera preview started (picamzero).")
            return
        except Exception as picamzero_exc:
            self.log(f"picamzero camera start failed: {picamzero_exc}")

        try:
            from picamera2 import Picamera2

            self.picam2 = Picamera2()
            self.picam2.configure(self.picam2.create_preview_configuration())
            self.picam2.start()
            self.camera_running = True
            self._reset_contact_deform_zero_state("Contact deform zero pending. Press Reset Zero.")
            self._request_pressure_zero_from_camera_open()
            self.camera_status_var.set("Camera running (picamera2)")
            self._set_camera_controls(True)
            self._refresh_system_status()
            self.log("Camera started (picamera2).")
        except Exception as picamera2_exc:
            self.camera_running = False
            self.camera_status_var.set("Failed to open")
            self._set_camera_controls(False)
            self._refresh_system_status()
            self.log(f"picamera2 start failed: {picamera2_exc}")
            messagebox.showerror(
                "Camera Error",
                "Could not open camera. Check picamzero/picamera2 installation and camera connection.",
            )

    def stop_camera(self):
        stopped = False

        if self.camera is not None:
            try:
                self.camera.stop_preview()
                stopped = True
            except Exception:
                pass
            finally:
                self.camera = None

        if self.picam2 is not None:
            try:
                self.picam2.stop()
                stopped = True
            except Exception:
                pass
            try:
                if hasattr(self.picam2, "close"):
                    self.picam2.close()
            except Exception:
                pass
            finally:
                self.picam2 = None

        self.camera_running = False
        self.camera_status_var.set("Stopped")
        self._set_camera_controls(False)
        self._refresh_system_status()
        if stopped:
            self.log("Camera stopped.")

    def start_limit_monitor(self):
        if self.limit_thread is not None and self.limit_thread.is_alive():
            self.log("Limit monitor is already running.")
            return

        self._ensure_limit_detail_vars()

        native_reader = None
        limit_backend = os.environ.get("PI_BUBBLE_LIMIT_BACKEND", "gpio").strip().lower()
        use_native_first = limit_backend in {"native", "libgpiod"}
        using_gpio = False

        if not use_native_first:
            using_gpio = self._setup_limit_gpio_inputs(show_error=False)
            if using_gpio:
                self.log("Limit monitor backend: RPi.GPIO.")

        if not using_gpio:
            native_reader = self._ensure_native_limit_reader()
            if native_reader is not None:
                self.log("Limit monitor backend: native libgpiod.")
            elif use_native_first:
                self.log("Native limit backend requested but unavailable; trying RPi.GPIO.")

        if native_reader is None and not using_gpio:
            if not self._require_gpio():
                return
            if not self._setup_limit_gpio_inputs(show_error=True):
                return
            self.log("Limit monitor backend: RPi.GPIO fallback.")

        with self.limit_state_lock:
            self.limit_trigger_state = (False, False)
            self.limit_state_sampled = False
        self._set_var(self.limit1_status_var, "waiting...")
        self._set_var(self.limit2_status_var, "waiting...")
        self._set_var(self.limit1_detail_var, "Limit 1: waiting...")
        self._set_var(self.limit2_detail_var, "Limit 2: waiting...")

        self.limit_stop_event.clear()
        self.limit_thread = threading.Thread(target=self._limit_worker, args=(native_reader,), daemon=True)
        self.limit_thread.start()
        backend_label = "native" if native_reader is not None else "RPi.GPIO"
        self.limit_monitor_state_var.set(f"Running ({backend_label})")
        self._set_limit_controls(True)
        self._refresh_system_status()
        self.log(
            f"Limit monitor started on GPIO{LIMIT_1_PIN} and GPIO{LIMIT_2_PIN}."
        )

    def _ensure_limit_detail_vars(self):
        if not hasattr(self, "limit1_detail_var"):
            self.limit1_detail_var = tk.StringVar(
                value=f"Limit 1: {self.limit1_status_var.get()}"
            )
        if not hasattr(self, "limit2_detail_var"):
            self.limit2_detail_var = tk.StringVar(
                value=f"Limit 2: {self.limit2_status_var.get()}"
            )

    def _resolve_native_limit_reader(self):
        env_override = os.environ.get("PI_BUBBLE_LIMIT_READER")
        if env_override:
            candidate = Path(env_override)
            if candidate.exists():
                return str(candidate)

        reader_name = "limit_reader.exe" if os.name == "nt" else "limit_reader"
        candidate = Path(__file__).resolve().parent / "native" / "build" / reader_name
        if candidate.exists():
            return str(candidate)
        return None

    def _ensure_native_limit_reader(self):
        reader = self._resolve_native_limit_reader()
        if reader is not None:
            return reader

        script = Path(__file__).resolve().parent / "native" / "build_pi.sh"
        if not script.exists():
            self.log("Native limit reader not found; build script is missing.")
            return None

        try:
            result = subprocess.run(
                ["bash", str(script)],
                cwd=str(script.parent),
                capture_output=True,
                text=True,
                timeout=30,
                check=False,
            )
        except Exception as exc:
            self.log(f"Native limit reader auto-build failed: {exc}")
            return None

        reader = self._resolve_native_limit_reader()
        if reader is not None:
            self.log("Native limit reader built automatically.")
            return reader

        output = (result.stderr or result.stdout or "").strip()
        if result.returncode == 0:
            self.log("Native limit reader was not produced; falling back to RPi.GPIO if usable.")
        else:
            self.log(f"Native limit reader build failed: {output[:240]}")
        return None

    def _setup_limit_gpio_inputs(self, show_error=True):
        if not self._gpio_available():
            return False
        try:
            GPIO.setup(LIMIT_1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(LIMIT_2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            return True
        except Exception as exc:
            self._handle_limit_gpio_error(exc, show_dialog=show_error)
            return False

    def _handle_limit_gpio_error(self, exc, show_dialog=True):
        message = str(exc)
        if "Cannot determine SOC peripheral base address" in message:
            self.gpio_ready = False
            self.log("RPi.GPIO cannot determine the SOC base address on this board/OS.")
            self.log("Build/use native limit_reader with libgpiod for limit switch testing.")
        else:
            self.log(f"Limit GPIO setup error: {message}")
        if show_dialog and self.root.winfo_exists():
            self.root.after(0, lambda: messagebox.showerror("Limit GPIO Error", message))

    def _apply_limit_state(self, current_state, last_state):
        current_state = (bool(current_state[0]), bool(current_state[1]))
        with self.limit_state_lock:
            self.limit_trigger_state = current_state
            self.limit_state_sampled = True

        self._stop_stepper_if_direction_limit_triggered(current_state)

        if current_state == last_state:
            return last_state

        l1_triggered, l2_triggered = current_state
        l1_text = "TRIGGERED" if l1_triggered else "not triggered"
        l2_text = "TRIGGERED" if l2_triggered else "not triggered"
        self._set_var(self.limit1_status_var, l1_text)
        self._set_var(self.limit2_status_var, l2_text)
        self._set_var(self.limit1_detail_var, f"Limit 1: {l1_text}")
        self._set_var(self.limit2_detail_var, f"Limit 2: {l2_text}")

        if l1_triggered:
            self.log("Limit 1 triggered.")
        if l2_triggered:
            self.log("Limit 2 triggered.")
        if not l1_triggered and not l2_triggered:
            self.log("Both limits are not triggered.")

        return current_state

    def _get_limit_trigger_state(self):
        with self.limit_state_lock:
            return tuple(self.limit_trigger_state)

    def _has_limit_state_sample(self):
        with self.limit_state_lock:
            return bool(self.limit_state_sampled)

    @staticmethod
    def _format_triggered_limits(limit_state):
        names = []
        if limit_state[0]:
            names.append("Limit 1")
        if limit_state[1]:
            names.append("Limit 2")
        return " and ".join(names)

    @staticmethod
    def _limit_block_for_direction(limit_state, direction_name):
        if direction_name == "up" and limit_state[0]:
            return "Limit 1"
        if direction_name == "down" and limit_state[1]:
            return "Limit 2"
        return None

    def _stop_stepper_if_direction_limit_triggered(self, limit_state):
        if self.stepper_limit_guard_disabled:
            return False
        if not self._is_thread_running(self.stepper_thread):
            return False
        direction_name = self.stepper_active_direction
        reason_limit = self._limit_block_for_direction(limit_state, direction_name)
        if reason_limit is None:
            return False

        reason = f"{reason_limit} while moving {direction_name}"
        if self.stepper_stop_reason != reason:
            self.stepper_stop_reason = reason
            self.log(f"Stepper stop requested by {reason}.")
        self.stepper_stop_event.set()
        self._set_var(self.stepper_state_var, f"Stopped by {reason}")
        return True

    def _ensure_limit_monitor_for_stepper(self, show_dialog=True):
        if self._is_thread_running(self.limit_thread):
            return self._wait_for_limit_state_sample(show_dialog=show_dialog)

        self.log("Starting limit monitor before stepper move.")
        self.start_limit_monitor()
        return self._wait_for_limit_state_sample(show_dialog=show_dialog)

    def _wait_for_limit_state_sample(self, timeout=1.0, show_dialog=True):
        deadline = time.perf_counter() + timeout
        while time.perf_counter() < deadline:
            if self._has_limit_state_sample():
                return True
            if not self._is_thread_running(self.limit_thread):
                break
            time.sleep(0.02)

        message = "Limit monitor could not report a fresh state. Stepper move was blocked."
        self.log(message)
        if show_dialog and self.root.winfo_exists():
            messagebox.showerror("Limit Monitor Required", message)
        return False

    def _run_native_limit_reader(self, reader_path):
        chip_name = os.environ.get("PI_BUBBLE_GPIO_CHIP", "gpiochip0")
        command = [
            reader_path,
            "--chip",
            chip_name,
            "--pin1",
            str(LIMIT_1_PIN),
            "--pin2",
            str(LIMIT_2_PIN),
            "--interval-ms",
            "50",
            "--active-low",
            "1",
        ]
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )

        last_state = (None, None)
        while not self.limit_stop_event.is_set():
            line = process.stdout.readline()
            if not line:
                break
            parts = line.strip().split()
            if len(parts) < 2:
                continue
            current_state = (parts[0] == "1", parts[1] == "1")
            last_state = self._apply_limit_state(current_state, last_state)

        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=1.0)

        _stdout, stderr = process.communicate()
        if process.returncode not in (0, -15) and not self.limit_stop_event.is_set():
            message = stderr.strip() or f"limit_reader exited with code {process.returncode}"
            raise RuntimeError(message)

    def _limit_worker(self, native_reader=None):
        last_state = (None, None)
        try:
            if native_reader is not None:
                try:
                    self.log(f"Native limit reader active: {native_reader}")
                    self._run_native_limit_reader(native_reader)
                    return
                except Exception as exc:
                    self.log(f"Native limit reader error: {exc}")
                    if not self._gpio_available():
                        return
                    self.log("Falling back to RPi.GPIO limit polling.")
                    if not self._setup_limit_gpio_inputs(show_error=True):
                        return

            while not self.limit_stop_event.is_set():
                l1_triggered = GPIO.input(LIMIT_1_PIN) == GPIO_LOW
                l2_triggered = GPIO.input(LIMIT_2_PIN) == GPIO_LOW
                current_state = (l1_triggered, l2_triggered)
                last_state = self._apply_limit_state(current_state, last_state)

                time.sleep(0.05)
        except Exception as exc:
            self.log(f"Limit monitor error: {exc}")
        finally:
            self._set_var(self.limit_monitor_state_var, "Stopped")
            self._set_limit_controls(False)
            self._refresh_system_status()

    def stop_limit_monitor(self):
        self.limit_stop_event.set()
        if self.limit_thread is not None and self.limit_thread.is_alive():
            self.limit_thread.join(timeout=1.0)
        self.limit_thread = None
        self.limit_monitor_state_var.set("Stopped")
        self._set_limit_controls(False)
        self._refresh_system_status()
        self.log("Limit monitor stopped.")

    def start_stepper_move(self):
        direction = self.stepper_direction_var.get().strip().lower()
        try:
            seconds = float(self.stepper_seconds_var.get().strip())
        except ValueError:
            messagebox.showerror("Invalid Seconds", "Seconds must be a number.")
            return

        self._start_stepper_move_command(direction, seconds, show_dialog=True)

    def _start_stepper_move_command(
        self,
        direction,
        seconds,
        show_dialog=True,
        frequency_hz=None,
        limit_guard_disabled=False,
    ):
        if self.stepper_thread is not None and self.stepper_thread.is_alive():
            self.log("Stepper is already moving.")
            return False

        direction = str(direction).strip().lower()
        if direction not in {"up", "down"}:
            if show_dialog:
                messagebox.showerror("Invalid Direction", "Direction must be up or down.")
            self.log(f"Invalid stepper direction requested: {direction}")
            return False

        if seconds <= 0:
            if show_dialog:
                messagebox.showerror("Invalid Seconds", "Seconds must be greater than 0.")
            self.log("Invalid stepper seconds requested.")
            return False

        if not self._ensure_limit_monitor_for_stepper(show_dialog=show_dialog):
            return False

        if not limit_guard_disabled:
            limit_state = self._get_limit_trigger_state()
            reason_limit = self._limit_block_for_direction(limit_state, direction)
            if reason_limit is not None:
                reason = f"{reason_limit} while moving {direction}"
                self._set_var(self.stepper_state_var, f"Blocked by {reason}")
                self.log(f"Stepper move blocked because {reason} is triggered.")
                if show_dialog:
                    messagebox.showwarning(
                        "Limit Triggered",
                        f"Stepper cannot move {direction} while {reason_limit} is triggered.",
                    )
                return False
        else:
            self.log(f"Stepper limit guard bypass enabled for {direction} release move.")

        native_runner = self._ensure_native_stepper_runner()
        if native_runner is None:
            if not self._gpio_available():
                if show_dialog:
                    self._require_gpio()
                else:
                    self.log("Stepper GPIO unavailable; movement blocked.")
                return False
            try:
                GPIO.setup(STEPPER_PUL_PIN, GPIO.OUT, initial=GPIO_LOW)
                GPIO.setup(STEPPER_DIR_PIN, GPIO.OUT, initial=DIR_DOWN_STATE)
                GPIO.setup(STEPPER_ENA_PIN, GPIO.OUT, initial=STEPPER_ENA_INACTIVE_STATE)
            except Exception as exc:
                self._handle_stepper_gpio_error(exc)
                return False

        # Keep command naming consistent with existing stepper script wiring.
        direction_state = DIR_DOWN_STATE if direction == "up" else DIR_UP_STATE

        self.stepper_stop_event.clear()
        self.stepper_stop_reason = None
        self.stepper_active_direction = direction
        self.stepper_limit_guard_disabled = bool(limit_guard_disabled)
        self.stepper_thread = threading.Thread(
            target=self._stepper_worker,
            args=(direction, direction_state, seconds, native_runner, frequency_hz),
            daemon=True,
        )
        self.stepper_thread.start()
        self._set_var(self.stepper_state_var, f"Running ({direction} for {seconds:.2f}s)")
        self._set_stepper_controls(True)
        self._refresh_system_status()
        self.log(f"Stepper command: {direction} for {seconds:.2f} seconds.")
        return True

    def _resolve_native_stepper_runner(self):
        env_override = os.environ.get("PI_BUBBLE_STEPPER_RUNNER")
        if env_override:
            candidate = Path(env_override)
            if candidate.exists():
                return str(candidate)

        runner_name = "stepper_runner.exe" if os.name == "nt" else "stepper_runner"
        candidate = Path(__file__).resolve().parent / "native" / "build" / runner_name
        if candidate.exists():
            return str(candidate)
        return None

    def _ensure_native_stepper_runner(self):
        runner = self._resolve_native_stepper_runner()
        if runner is not None:
            return runner

        script = Path(__file__).resolve().parent / "native" / "build_pi.sh"
        if not script.exists():
            self.log("Native stepper runner not found; build script is missing.")
            return None

        try:
            result = subprocess.run(
                ["bash", str(script)],
                cwd=str(script.parent),
                capture_output=True,
                text=True,
                timeout=30,
                check=False,
            )
        except Exception as exc:
            self.log(f"Native stepper runner auto-build failed: {exc}")
            return None

        runner = self._resolve_native_stepper_runner()
        if runner is not None:
            self.log("Native stepper runner built automatically.")
            return runner

        output = (result.stderr or result.stdout or "").strip()
        if result.returncode == 0:
            self.log("Native stepper runner was not produced; falling back to RPi.GPIO if usable.")
        else:
            self.log(f"Native stepper runner build failed: {output[:240]}")
        return None

    def _handle_stepper_gpio_error(self, exc):
        message = str(exc)
        if "Cannot determine SOC peripheral base address" in message:
            self.gpio_ready = False
            self.log("RPi.GPIO cannot determine the SOC base address on this board/OS.")
            self.log("Build/use native stepper_runner with libgpiod for stepper control.")
        else:
            self.log(f"Stepper GPIO setup error: {message}")
        if self.root.winfo_exists():
            self.root.after(0, lambda: messagebox.showerror("Stepper GPIO Error", message))

    def _run_native_stepper(self, runner_path, direction_state, seconds, frequency_hz=None):
        started_at = time.perf_counter()
        frequency = float(frequency_hz or STEPPER_FREQUENCY_HZ)
        chip_name = os.environ.get("PI_BUBBLE_GPIO_CHIP", "gpiochip0")
        command = [
            runner_path,
            "--chip",
            chip_name,
            "--pul",
            str(STEPPER_PUL_PIN),
            "--dir",
            str(STEPPER_DIR_PIN),
            "--ena",
            str(STEPPER_ENA_PIN),
            "--dir-state",
            str(int(direction_state)),
            "--ena-active",
            str(int(STEPPER_ENA_ACTIVE_STATE)),
            "--ena-inactive",
            str(int(STEPPER_ENA_INACTIVE_STATE)),
            "--frequency",
            str(frequency),
            "--seconds",
            f"{float(seconds):.6f}",
        ]
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )

        while process.poll() is None:
            if self.stepper_stop_event.is_set():
                process.terminate()
                try:
                    process.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=1.0)
                return True, time.perf_counter() - started_at
            time.sleep(0.02)

        _stdout, stderr = process.communicate()
        if process.returncode != 0:
            message = stderr.strip() or f"stepper_runner exited with code {process.returncode}"
            raise RuntimeError(message)
        return False, time.perf_counter() - started_at

    def _stepper_worker(self, direction_name, direction_state, seconds, native_runner=None, frequency_hz=None):
        frequency = float(frequency_hz or STEPPER_FREQUENCY_HZ)
        pulse_delay = 1.0 / (2.0 * frequency)
        try:
            if native_runner is not None:
                try:
                    self.log(f"Native stepper runner active: {native_runner}")
                    stopped, elapsed_seconds = self._run_native_stepper(
                        native_runner,
                        direction_state,
                        seconds,
                        frequency_hz=frequency,
                    )
                    self._apply_stepper_motion(
                        direction_name,
                        elapsed_seconds=elapsed_seconds,
                        frequency_hz=frequency,
                    )
                    if stopped or self.stepper_stop_event.is_set():
                        reason = self.stepper_stop_reason or "user"
                        self.log(f"Stepper movement stopped by {reason}.")
                    else:
                        self.log(f"Stepper movement complete: {direction_name}.")
                    return
                except Exception as exc:
                    self.log(f"Native stepper runner error: {exc}")
                    if not self._gpio_available():
                        raise RuntimeError(
                            "Native stepper runner failed and RPi.GPIO fallback is unavailable."
                        ) from exc
                    self.log("Falling back to Python stepper timing.")
                    native_runner = None
                    try:
                        GPIO.setup(STEPPER_PUL_PIN, GPIO.OUT, initial=GPIO_LOW)
                        GPIO.setup(STEPPER_DIR_PIN, GPIO.OUT, initial=DIR_DOWN_STATE)
                        GPIO.setup(STEPPER_ENA_PIN, GPIO.OUT, initial=STEPPER_ENA_INACTIVE_STATE)
                    except Exception as gpio_exc:
                        self._handle_stepper_gpio_error(gpio_exc)
                        raise

            GPIO.output(STEPPER_ENA_PIN, STEPPER_ENA_ACTIVE_STATE)
            GPIO.output(STEPPER_DIR_PIN, direction_state)
            time.sleep(0.002)

            end_time = time.perf_counter() + seconds
            pulse_count = 0
            while time.perf_counter() < end_time and not self.stepper_stop_event.is_set():
                GPIO.output(STEPPER_PUL_PIN, GPIO_HIGH)
                time.sleep(pulse_delay)
                GPIO.output(STEPPER_PUL_PIN, GPIO_LOW)
                time.sleep(pulse_delay)
                pulse_count += 1

            self._apply_stepper_motion(direction_name, pulses=pulse_count)

            if self.stepper_stop_event.is_set():
                reason = self.stepper_stop_reason or "user"
                self.log(f"Stepper movement stopped by {reason}.")
            else:
                self.log(f"Stepper movement complete: {direction_name}.")
        except Exception as exc:
            self.log(f"Stepper error: {exc}")
        finally:
            if native_runner is None and GPIO is not None:
                try:
                    GPIO.output(STEPPER_PUL_PIN, GPIO_LOW)
                    GPIO.output(STEPPER_ENA_PIN, STEPPER_ENA_INACTIVE_STATE)
                except Exception:
                    pass
            if self.stepper_stop_reason and self.stepper_stop_reason != "user":
                self._set_var(self.stepper_state_var, f"Stopped by {self.stepper_stop_reason}")
            else:
                self._set_var(self.stepper_state_var, "Ready")
            self.stepper_active_direction = None
            self.stepper_limit_guard_disabled = False
            self._set_stepper_controls(False)
            self._refresh_system_status()

    def stop_stepper_move(self):
        if self._is_thread_running(self.stepper_thread) and self.stepper_stop_reason is None:
            self.stepper_stop_reason = "user"
        self.stepper_stop_event.set()
        if self.stepper_thread is not None and self.stepper_thread.is_alive():
            self.stepper_thread.join(timeout=1.0)
        self.stepper_thread = None
        if self.stepper_stop_reason and self.stepper_stop_reason != "user":
            self.stepper_state_var.set(f"Stopped by {self.stepper_stop_reason}")
        else:
            self.stepper_state_var.set("Ready")
        self.stepper_active_direction = None
        self.stepper_limit_guard_disabled = False
        self._set_stepper_controls(False)

        if GPIO is not None:
            try:
                GPIO.output(STEPPER_PUL_PIN, GPIO_LOW)
                GPIO.output(STEPPER_ENA_PIN, STEPPER_ENA_INACTIVE_STATE)
            except Exception:
                pass

        self._refresh_system_status()
        self.log("Stepper stop requested.")

    def start_pressure_read(self):
        if self.pressure_thread is not None and self.pressure_thread.is_alive():
            self.log("Pressure read is already running.")
            return

        self.pressure_stop_event.clear()
        self.pressure_thread = threading.Thread(target=self._pressure_worker, daemon=True)
        self.pressure_thread.start()
        self.pressure_state_var.set("Running")
        self._set_pressure_controls(True)
        self._refresh_system_status()
        self.log("Pressure read started.")

    @staticmethod
    def _open_pressure_i2c():
        errors = []

        try:
            from adafruit_extended_bus import ExtendedI2C

            return ExtendedI2C(MPRLS_I2C_BUS), f"ExtendedI2C({MPRLS_I2C_BUS})"
        except Exception as exc:
            errors.append(f"ExtendedI2C: {exc}")

        try:
            import board

            if hasattr(board, "I2C"):
                return board.I2C(), "board.I2C"
            errors.append("board.I2C missing")

            try:
                import busio

                scl = getattr(board, "SCL", None)
                sda = getattr(board, "SDA", None)
                if scl is not None and sda is not None:
                    return busio.I2C(scl, sda), "busio.I2C(board.SCL, board.SDA)"
                errors.append("board.SCL/SDA missing")
            except Exception as exc:
                errors.append(f"busio.I2C: {exc}")
        except Exception as exc:
            errors.append(f"board import: {exc}")

        detail = "; ".join(errors) if errors else "no backend attempted"
        raise RuntimeError(
            "Could not open CircuitPython I2C bus. Enable I2C, install Blinka support, "
            f"or use direct smbus2 fallback. Details: {detail}"
        )

    def _open_pressure_sensor(self):
        errors = []
        try:
            import adafruit_mprls

            i2c, i2c_backend = self._open_pressure_i2c()
            sensor = adafruit_mprls.MPRLS(
                i2c,
                addr=MPRLS_I2C_ADDR,
                psi_min=MPRLS_PSI_MIN,
                psi_max=MPRLS_PSI_MAX,
            )
            return sensor, f"adafruit_mprls via {i2c_backend}", None
        except Exception as exc:
            errors.append(f"adafruit_mprls: {exc}")

        try:
            sensor = LinuxMPRLSSensor(MPRLS_I2C_BUS, MPRLS_I2C_ADDR)
            return sensor, f"smbus2 /dev/i2c-{MPRLS_I2C_BUS} addr 0x{MPRLS_I2C_ADDR:02X}", sensor.close
        except Exception as exc:
            errors.append(f"smbus2 direct: {exc}")

        raise RuntimeError("Pressure sensor unavailable. " + " | ".join(errors))

    def _pressure_worker(self):
        pressure_close = None
        try:
            sensor, pressure_backend, pressure_close = self._open_pressure_sensor()
            self.log(f"MPRLS pressure sensor initialized via {pressure_backend}.")

            read_failures = 0
            while not self.pressure_stop_event.is_set():
                try:
                    pressure_hpa = float(sensor.pressure)
                    read_failures = 0
                    self._record_pressure_reading(pressure_hpa)
                    self._set_var(self.pressure_value_var, f"{pressure_hpa:.2f} hPa")
                except Exception as read_exc:
                    read_failures += 1
                    if read_failures == 1 or read_failures % 5 == 0:
                        self.log(f"Pressure read retry {read_failures}: {read_exc}")
                    if read_failures >= 12:
                        raise
                    self._set_var(self.pressure_value_var, "Retrying...")
                time.sleep(0.5)
        except Exception as exc:
            self.log(f"Pressure sensor error: {exc}")
            self._set_var(self.pressure_value_var, "Error")
        finally:
            if pressure_close is not None:
                try:
                    pressure_close()
                except Exception:
                    pass
            self._set_var(self.pressure_state_var, "Stopped")
            self._set_pressure_controls(False)
            self._refresh_system_status()

    def stop_pressure_read(self):
        self.pressure_stop_event.set()
        if self.pressure_thread is not None and self.pressure_thread.is_alive():
            self.pressure_thread.join(timeout=1.0)
        self.pressure_thread = None
        self.pressure_state_var.set("Stopped")
        self._set_pressure_controls(False)
        self._refresh_system_status()
        self.log("Pressure read stopped.")

    def _init_force_calibration_state(self):
        if not hasattr(self, "sensor_data_lock"):
            self.sensor_data_lock = threading.Lock()
            self.latest_pressure_hpa = None
            self.latest_loadcell_kg = None
            self.latest_loadcell_force_n = None
            self.latest_loadcell_raw_kg = None
            self.latest_loadcell_filtered_raw_kg = None
            self.loadcell_live_zero_kg = 0.0
            self.loadcell_filter_buffer = []
            self.loadcell_noise_kg = 0.0
            self.loadcell_filter_count = 0
            self.latest_pressure_force_n = None
            self.force_pressure_zero_hpa = FORCE_DEFAULT_PRESSURE_ZERO_HPA
            self.force_pressure_zero_pending = False
            self.force_pressure_zero_samples = []
            self.force_calibration_samples = []
            self.force_calibration_coeffs = None
            self.contact_pressure_zero_hpa = None
            self.contact_pressure_noise_hpa = 0.0
            self.contact_pressure_delta_hpa = None
            self.contact_pressure_threshold_hpa = PRESSURE_CONTACT_MIN_DELTA_HPA
            self.contact_pressure_is_contact = False
            self.contact_pressure_state = "waiting_pressure"
            self.contact_pressure_baseline_samples = []
            self.contact_pressure_above_count = 0
            self.contact_pressure_below_count = 0
            self.contact_pressure_zero_request_sent = False

        if not hasattr(self, "latest_loadcell_force_n"):
            self.latest_loadcell_force_n = None
        if not hasattr(self, "latest_loadcell_raw_kg"):
            self.latest_loadcell_raw_kg = None
        if not hasattr(self, "latest_loadcell_filtered_raw_kg"):
            self.latest_loadcell_filtered_raw_kg = None
        if not hasattr(self, "loadcell_live_zero_kg"):
            self.loadcell_live_zero_kg = 0.0
        if not hasattr(self, "loadcell_filter_buffer"):
            self.loadcell_filter_buffer = []
        if not hasattr(self, "loadcell_noise_kg"):
            self.loadcell_noise_kg = 0.0
        if not hasattr(self, "loadcell_filter_count"):
            self.loadcell_filter_count = 0
        if not hasattr(self, "latest_pressure_force_n"):
            self.latest_pressure_force_n = None
        if not hasattr(self, "force_pressure_zero_hpa"):
            self.force_pressure_zero_hpa = FORCE_DEFAULT_PRESSURE_ZERO_HPA
        if not hasattr(self, "force_pressure_zero_pending"):
            self.force_pressure_zero_pending = False
            self.force_pressure_zero_samples = []
        if not hasattr(self, "contact_pressure_zero_hpa"):
            self.contact_pressure_zero_hpa = None
            self.contact_pressure_noise_hpa = 0.0
            self.contact_pressure_delta_hpa = None
            self.contact_pressure_threshold_hpa = PRESSURE_CONTACT_MIN_DELTA_HPA
            self.contact_pressure_is_contact = False
            self.contact_pressure_state = "waiting_pressure"
            self.contact_pressure_baseline_samples = []
            self.contact_pressure_above_count = 0
            self.contact_pressure_below_count = 0
            self.contact_pressure_zero_request_sent = False
        if not hasattr(self, "force_graph_history"):
            self.force_graph_history = []
            self.force_graph_redraw_pending = False
        if not hasattr(self, "force_cycle_sample_history"):
            self.force_cycle_sample_history = []
            self.force_cycle_capture_active = False
            self.force_cycle_capture_started_at = None
            self.force_cycle_capture_last_time = None
            self.force_cycle_phase = "idle"
            self.force_cycle_phase_started_at = None
            self.force_cycle_down_seconds = None
            self.force_cycle_shape_name = "sample"
            self.force_cycle_report_sample_target = None
            self.force_cycle_trial = 1
            self.force_cycle_target_depth_mm = None

        if not hasattr(self, "force_estimate_var"):
            self.force_estimate_var = tk.StringVar(value="-")
            self.force_pressure_zero_var = tk.StringVar(value=f"{FORCE_DEFAULT_PRESSURE_ZERO_HPA:.2f} hPa")
            self.force_calibration_status_var = tk.StringVar(value="Set pressure zero, then capture 2+ bubble samples.")
            self.force_calibration_sample_count_var = tk.StringVar(value="0 samples")
            self.force_calibration_model_var = tk.StringVar(value="F = a*dP + b")
            self.force_calibration_live_pair_var = tk.StringVar(value="Pressure -, dP -, load -")
            self.force_cycle_shape_var = tk.StringVar(value="sample")
            self.force_cycle_depth_target_var = tk.StringVar(value="limit")
            self.force_cycle_report_sample_count_var = tk.StringVar(value="10")
            self.force_cycle_trial_var = tk.StringVar(value="1")
        elif not hasattr(self, "force_cycle_shape_var"):
            self.force_cycle_shape_var = tk.StringVar(value="sample")
        if not hasattr(self, "force_cycle_depth_target_var"):
            self.force_cycle_depth_target_var = tk.StringVar(value="limit")
        if not hasattr(self, "force_cycle_report_sample_count_var"):
            self.force_cycle_report_sample_count_var = tk.StringVar(value="10")
        if not hasattr(self, "force_cycle_trial_var"):
            self.force_cycle_trial_var = tk.StringVar(value="1")

        if not hasattr(self, "_force_calibration_loaded"):
            self._force_calibration_loaded = True
            self._load_force_calibration()

    def _force_calibration_path(self):
        return Path(__file__).resolve().parent / FORCE_CALIBRATION_FILENAME

    @staticmethod
    def _format_force_model(slope, intercept, suffix=""):
        sign = "+" if intercept >= 0 else "-"
        model = f"F = {slope:.6f}dP {sign} {abs(intercept):.3f}"
        return f"{model} {suffix}" if suffix else model

    @staticmethod
    def _pressure_delta_from_zero(pressure_hpa, zero_hpa):
        if pressure_hpa is None or zero_hpa is None:
            return None
        return float(pressure_hpa) - float(zero_hpa)

    def _refresh_force_pressure_zero(self):
        if not hasattr(self, "force_pressure_zero_var"):
            return
        with self.sensor_data_lock:
            zero = self.force_pressure_zero_hpa
        zero_text = "-" if zero is None else f"{zero:.2f} hPa"
        self._set_var(self.force_pressure_zero_var, zero_text)

    def _set_force_pressure_zero(self, pressure_hpa, source="manual"):
        zero = float(pressure_hpa)
        with self.sensor_data_lock:
            self.force_pressure_zero_hpa = zero
            self.force_pressure_zero_pending = False
            self.force_pressure_zero_samples = []
            self.latest_pressure_force_n = None
            if hasattr(self, "force_graph_history"):
                self.force_graph_history.clear()
        self._refresh_force_pressure_zero()
        self._refresh_force_live_pair()
        if source == "manual":
            self.force_calibration_status_var.set("Pressure zero set. Press bubble into load cell.")
            self.log(f"Force pressure zero set at {zero:.2f} hPa.")
        return True

    def set_force_pressure_zero(self):
        with self.sensor_data_lock:
            pressure = self.latest_pressure_hpa
        if pressure is None:
            messagebox.showwarning(
                "Pressure Zero",
                "Start pressure reading before setting zero.",
            )
            return False
        ok = self._set_force_pressure_zero(pressure, source="manual")
        self._update_force_estimate()
        self._append_force_graph_sample()
        self._request_force_graph_redraw()
        return ok

    def _load_force_calibration(self):
        with self.sensor_data_lock:
            self.force_calibration_coeffs = None
            self.force_calibration_samples = []
            self.latest_pressure_force_n = None

        self.force_calibration_sample_count_var.set("0 samples")
        self.force_calibration_model_var.set("F = a*dP + b")
        self.force_estimate_var.set("-")
        self.force_calibration_status_var.set(
            "Force calibration required before use. Set P Zero, Zero Load, then capture 2+ samples."
        )
        if self._force_calibration_path().exists():
            self.log("Saved force calibration was not auto-loaded; recalibrate force for this session.")
        self._request_force_graph_redraw()

    def _save_force_calibration(self):
        with self.sensor_data_lock:
            coeffs = self.force_calibration_coeffs
            samples = list(self.force_calibration_samples)
            zero = self.force_pressure_zero_hpa

        if coeffs is None:
            return

        slope, intercept = coeffs
        payload = {
            "model": "force_n = slope_n_per_hpa * pressure_delta_hpa + intercept_n",
            "slope": slope,
            "intercept": intercept,
            "pressure_zero_hpa": zero,
            "samples": [
                {"pressure_delta_hpa": pressure_delta, "force_n": force}
                for pressure_delta, force in samples
            ],
        }
        try:
            with self._force_calibration_path().open("w", encoding="utf-8") as handle:
                json.dump(payload, handle, indent=2)
        except Exception as exc:
            self.log(f"Could not save force calibration: {exc}")

    def _build_force_calibration_panel(self, parent):
        self._init_force_calibration_state()
        parent.columnconfigure(1, weight=1)
        value_wrap = 280

        ttk.Label(parent, text="Live pair:").grid(row=0, column=0, sticky="w")
        ttk.Label(
            parent,
            textvariable=self.force_calibration_live_pair_var,
            wraplength=value_wrap,
            justify="left",
        ).grid(
            row=0, column=1, sticky="ew"
        )

        ttk.Label(parent, text="Estimated force:").grid(row=1, column=0, sticky="w", pady=(8, 0))
        ttk.Label(
            parent,
            textvariable=self.force_estimate_var,
            wraplength=value_wrap,
            justify="left",
        ).grid(
            row=1, column=1, sticky="ew", pady=(8, 0)
        )

        ttk.Label(parent, text="Pressure zero:").grid(row=2, column=0, sticky="w", pady=(8, 0))
        ttk.Label(
            parent,
            textvariable=self.force_pressure_zero_var,
            wraplength=value_wrap,
            justify="left",
        ).grid(
            row=2, column=1, sticky="ew", pady=(8, 0)
        )

        ttk.Label(parent, text="Samples:").grid(row=3, column=0, sticky="w", pady=(8, 0))
        ttk.Label(
            parent,
            textvariable=self.force_calibration_sample_count_var,
            wraplength=value_wrap,
            justify="left",
        ).grid(
            row=3, column=1, sticky="ew", pady=(8, 0)
        )

        ttk.Label(parent, text="Model:").grid(row=4, column=0, sticky="w", pady=(8, 0))
        ttk.Label(
            parent,
            textvariable=self.force_calibration_model_var,
            wraplength=value_wrap,
            justify="left",
        ).grid(
            row=4, column=1, sticky="ew", pady=(8, 0)
        )

        ttk.Label(parent, text="Status:").grid(row=5, column=0, sticky="w", pady=(8, 0))
        ttk.Label(
            parent,
            textvariable=self.force_calibration_status_var,
            wraplength=value_wrap,
            justify="left",
        ).grid(
            row=5, column=1, sticky="ew", pady=(8, 0)
        )

        ttk.Label(parent, text="Shape label:").grid(row=6, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(parent, textvariable=self.force_cycle_shape_var).grid(
            row=6, column=1, sticky="ew", pady=(8, 0)
        )

        report = ttk.Frame(parent)
        report.grid(row=7, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        report.columnconfigure(1, weight=1)
        report.columnconfigure(3, weight=1)
        ttk.Label(report, text="Report samples:").grid(row=0, column=0, sticky="w", padx=(0, 6))
        ttk.Combobox(
            report,
            textvariable=self.force_cycle_report_sample_count_var,
            values=("all", "10", "25", "30"),
            state="readonly",
            width=8,
        ).grid(row=0, column=1, sticky="ew", padx=(0, 10))
        ttk.Label(report, text="Trial:").grid(row=0, column=2, sticky="w", padx=(0, 6))
        ttk.Combobox(
            report,
            textvariable=self.force_cycle_trial_var,
            values=("1", "2", "3"),
            state="readonly",
            width=6,
        ).grid(row=0, column=3, sticky="ew")

        actions = ttk.Frame(parent)
        actions.grid(row=8, column=0, columnspan=2, sticky="ew", pady=(12, 0))
        for col in range(3):
            actions.columnconfigure(col, weight=1)
        ttk.Button(actions, text="Set P Zero", command=self.set_force_pressure_zero).grid(
            row=0, column=0, sticky="ew", padx=(0, 8)
        )
        ttk.Button(actions, text="Zero Load", command=self.zero_loadcell_live).grid(
            row=0, column=1, sticky="ew", padx=(0, 8)
        )
        ttk.Button(actions, text="Calibrate", command=self.add_force_calibration_sample).grid(
            row=0, column=2, sticky="ew"
        )
        ttk.Button(actions, text="Fit Calibration", command=self.fit_force_calibration).grid(
            row=1, column=0, sticky="ew", padx=(0, 8), pady=(8, 0)
        )
        ttk.Button(actions, text="Show Linear Graph", command=self.show_force_linear_equation_graph).grid(
            row=1, column=1, sticky="ew", padx=(0, 8), pady=(8, 0)
        )
        ttk.Button(actions, text="Reset Samples", command=self.reset_force_bubble_samples).grid(
            row=1, column=2, sticky="ew", pady=(8, 0)
        )
        ttk.Button(actions, text="Clear", command=self.clear_force_calibration).grid(
            row=3, column=0, columnspan=3, sticky="ew", pady=(8, 0)
        )
        self.force_cycle_start_btn = ttk.Button(
            actions, text="Run Force Cycle", command=self.start_force_cycle
        )
        self.force_cycle_start_btn.grid(
            row=2, column=0, columnspan=2, sticky="ew", padx=(0, 8), pady=(8, 0)
        )
        self.force_cycle_stop_btn = ttk.Button(
            actions, text="Stop Cycle", command=self.stop_force_cycle
        )
        self.force_cycle_stop_btn.grid(row=2, column=2, sticky="ew", pady=(8, 0))
        self._set_force_cycle_controls(False)

    def _record_pressure_reading(self, pressure_hpa):
        pressure = float(pressure_hpa)
        zero_captured = False
        zero_pending_count = None
        zero_required_count = max(1, PRESSURE_CONTACT_BASELINE_SAMPLES)
        zero_value = None
        with self.sensor_data_lock:
            self.latest_pressure_hpa = pressure
            if self.force_pressure_zero_pending:
                self.force_pressure_zero_samples.append(pressure)
                zero_pending_count = len(self.force_pressure_zero_samples)
                if zero_pending_count >= zero_required_count:
                    recent = self.force_pressure_zero_samples[-zero_required_count:]
                    zero_value = float(np.median(np.asarray(recent, dtype=np.float32)))
                    self.force_pressure_zero_hpa = zero_value
                    self.force_pressure_zero_pending = False
                    self.force_pressure_zero_samples = []
                    self.latest_pressure_force_n = None
                    zero_captured = True
            elif self.force_pressure_zero_hpa is None:
                self.force_pressure_zero_hpa = pressure
                zero_value = pressure
                zero_captured = True
        if zero_pending_count is not None and not zero_captured:
            self._set_var(
                self.force_calibration_status_var,
                f"Collecting pressure zero {zero_pending_count}/{zero_required_count}.",
            )
        if zero_captured:
            self._refresh_force_pressure_zero()
            self._set_var(
                self.force_calibration_status_var,
                "Pressure zero captured. Press bubble into load cell.",
            )
            self.log(f"Force pressure zero auto-captured at {float(zero_value):.2f} hPa.")
        self._update_pressure_contact_gate(pressure)
        self._refresh_force_live_pair()
        self._update_force_estimate()
        self._append_force_graph_sample()

    def _reset_loadcell_filter(self, reset_live_zero=False):
        with self.sensor_data_lock:
            self.latest_loadcell_kg = None
            self.latest_loadcell_force_n = None
            self.latest_loadcell_raw_kg = None
            self.latest_loadcell_filtered_raw_kg = None
            if reset_live_zero:
                self.loadcell_live_zero_kg = 0.0
            self.loadcell_filter_buffer = []
            self.loadcell_noise_kg = 0.0
            self.loadcell_filter_count = 0

    def zero_loadcell_live(self):
        with self.sensor_data_lock:
            current_kg = self.latest_loadcell_kg
            if current_kg is None:
                current_kg = self.latest_loadcell_filtered_raw_kg
            if current_kg is None:
                messagebox.showwarning(
                    "Zero Load",
                    "Start load-cell reading and wait for a stable value first.",
                )
                return False
            self.loadcell_live_zero_kg += float(current_kg)
            self.latest_loadcell_kg = 0.0
            self.latest_loadcell_force_n = 0.0
            if hasattr(self, "force_graph_history"):
                self.force_graph_history.clear()
        self._set_var(self.loadcell_weight_var, "0.0000 kg (zeroed)")
        self._refresh_force_live_pair()
        self._update_force_estimate()
        self._request_force_graph_redraw()
        self.log(f"Load-cell live zero adjusted by {float(current_kg):+.4f} kg.")
        return True

    def _record_loadcell_reading(self, weight_kg):
        raw_kg = float(weight_kg)
        with self.sensor_data_lock:
            buffer = list(getattr(self, "loadcell_filter_buffer", []))
            buffer.append(raw_kg)
            window = max(1, int(LOADCELL_FILTER_WINDOW))
            if len(buffer) > window:
                buffer = buffer[-window:]

            values = np.asarray(buffer, dtype=np.float64)
            filtered_kg = float(np.median(values))
            mad = float(np.median(np.abs(values - filtered_kg))) if values.size else 0.0
            noise_kg = float(1.4826 * mad)
            if values.size >= 3:
                outlier_limit = max(LOADCELL_STABLE_NOISE_KG * 3.0, noise_kg * 6.0, 0.03)
                if abs(raw_kg - filtered_kg) > outlier_limit:
                    filtered_kg = float(np.median(values[:-1])) if values.size > 1 else filtered_kg

            live_zero_kg = float(getattr(self, "loadcell_live_zero_kg", 0.0))
            corrected_kg = filtered_kg - live_zero_kg
            self.loadcell_filter_buffer = buffer
            self.loadcell_filter_count = len(buffer)
            self.loadcell_noise_kg = noise_kg
            self.latest_loadcell_raw_kg = raw_kg
            self.latest_loadcell_filtered_raw_kg = filtered_kg
            self.latest_loadcell_kg = corrected_kg
            self.latest_loadcell_force_n = corrected_kg * FORCE_GRAVITY_MPS2
        self._refresh_force_live_pair()
        self._update_force_estimate()
        self._append_force_graph_sample()
        return corrected_kg

    def _refresh_force_live_pair(self):
        if not hasattr(self, "force_calibration_live_pair_var"):
            return
        with self.sensor_data_lock:
            pressure = self.latest_pressure_hpa
            zero = self.force_pressure_zero_hpa
            weight_kg = self.latest_loadcell_kg

        pressure_text = "-" if pressure is None else f"{pressure:.2f} hPa"
        pressure_delta = self._pressure_delta_from_zero(pressure, zero)
        delta_text = "-" if pressure_delta is None else f"{pressure_delta:+.2f} hPa"
        force_text = "-" if weight_kg is None else f"{weight_kg * FORCE_GRAVITY_MPS2:.3f} N"
        self._set_var(
            self.force_calibration_live_pair_var,
            f"Pressure {pressure_text}, dP {delta_text}, load force {force_text}",
        )

    def _update_force_estimate(self):
        if not hasattr(self, "force_estimate_var"):
            return
        with self.sensor_data_lock:
            pressure = self.latest_pressure_hpa
            zero = self.force_pressure_zero_hpa
            coeffs = self.force_calibration_coeffs

        pressure_delta = self._pressure_delta_from_zero(pressure, zero)

        if pressure_delta is None or coeffs is None:
            with self.sensor_data_lock:
                self.latest_pressure_force_n = None
            self._set_var(self.force_estimate_var, "-")
            return

        slope, intercept = coeffs
        estimated_force = (slope * pressure_delta) + intercept
        with self.sensor_data_lock:
            self.latest_pressure_force_n = float(estimated_force)
        estimated_load_kg = float(estimated_force) / FORCE_GRAVITY_MPS2
        self._set_var(self.force_estimate_var, f"{estimated_load_kg:.4f} kg")

    def _append_force_graph_sample(self):
        now = time.monotonic()
        with self.sensor_data_lock:
            load_force = self.latest_loadcell_force_n
            pressure_force = self.latest_pressure_force_n
            pressure_hpa = self.latest_pressure_hpa

            if load_force is None and pressure_force is None and pressure_hpa is None:
                return

            self.force_graph_history.append(
                (
                    now,
                    None if load_force is None else float(load_force),
                    None if pressure_force is None else float(pressure_force),
                    None if pressure_hpa is None else float(pressure_hpa),
                )
            )

            cutoff = now - FORCE_GRAPH_HISTORY_SECONDS
            self.force_graph_history = [
                point for point in self.force_graph_history if point[0] >= cutoff
            ]
            if len(self.force_graph_history) > FORCE_GRAPH_MAX_POINTS:
                del self.force_graph_history[:-FORCE_GRAPH_MAX_POINTS]

        self._append_force_cycle_sample(now=now)
        self._request_force_graph_redraw()

    @staticmethod
    def _clean_force_cycle_shape_name(value):
        text = str(value or "").strip()
        return text if text else "sample"

    @staticmethod
    def _safe_filename_token(value, default="sample"):
        token = []
        for char in str(value or "").strip().lower():
            if char.isalnum():
                token.append(char)
            elif char in {" ", "-", "_", "."}:
                token.append("_")
        safe = "".join(token).strip("_")
        return (safe or default)[:48]

    def _force_cycle_report_sample_target(self):
        text = str(self.force_cycle_report_sample_count_var.get() or "all").strip().lower()
        if text in {"", "all", "full"}:
            return None
        try:
            value = int(float(text))
        except (TypeError, ValueError):
            return None
        return value if value in {10, 25, 30} else None

    def _force_cycle_target_depth_mm(self):
        return None

    def _force_cycle_report_trial(self):
        text = str(self.force_cycle_trial_var.get() or "1").strip()
        try:
            value = int(float(text))
        except (TypeError, ValueError):
            value = 1
        return int(np.clip(value, 1, 3))

    @staticmethod
    def _downsample_force_cycle_history(history, target_count):
        samples = list(history or [])
        if not target_count or target_count <= 0 or len(samples) <= target_count:
            return samples
        target_count = int(target_count)
        anchor_scores = {}

        def add_anchor(index, score):
            if 0 <= index < len(samples):
                previous = anchor_scores.get(index)
                anchor_scores[index] = score if previous is None else min(previous, score)

        add_anchor(0, 0)
        add_anchor(len(samples) - 1, 0)

        extrema_keys = (
            ("indentation_depth_mm", 1),
            ("load_kg", 2),
            ("pressure_kg_eq", 2),
            ("dot_mean_px", 2),
            ("dot_max_px", 2),
            ("error_kg", 3),
            ("contact_peak", 3),
            ("contact_area_ratio", 3),
        )
        for key, score in extrema_keys:
            points = [
                (idx, float(sample.get(key)))
                for idx, sample in enumerate(samples)
                if AllInOneTesterGUI._finite_number(sample.get(key))
            ]
            if not points:
                continue
            add_anchor(min(points, key=lambda item: item[1])[0], score)
            add_anchor(max(points, key=lambda item: item[1])[0], score)

        selected = set(anchor_scores)
        if len(selected) > target_count:
            priority = sorted(selected, key=lambda idx: (anchor_scores[idx], idx))
            selected = set(priority[:target_count])

        for index in np.linspace(0, len(samples) - 1, target_count):
            if len(selected) >= target_count:
                break
            rounded = int(np.clip(round(float(index)), 0, len(samples) - 1))
            candidates = [rounded]
            for offset in range(1, len(samples)):
                left = rounded - offset
                right = rounded + offset
                if left >= 0:
                    candidates.append(left)
                if right < len(samples):
                    candidates.append(right)
                if left < 0 and right >= len(samples):
                    break
            for candidate in candidates:
                if candidate not in selected:
                    selected.add(candidate)
                    break

        selected_indexes = sorted(selected)[:target_count]
        return [samples[index] for index in selected_indexes]

    @staticmethod
    def _force_cycle_series_min_max(history, key):
        points = []
        for sample in history or []:
            value = sample.get(key)
            elapsed = sample.get("elapsed_s")
            if AllInOneTesterGUI._finite_number(value) and AllInOneTesterGUI._finite_number(elapsed):
                points.append((float(elapsed), float(value)))
        if not points:
            return None
        min_point = min(points, key=lambda point: point[1])
        max_point = max(points, key=lambda point: point[1])
        return {
            "min_elapsed_s": min_point[0],
            "min_value": min_point[1],
            "max_elapsed_s": max_point[0],
            "max_value": max_point[1],
        }

    @staticmethod
    @staticmethod
    def _force_cycle_depth_token(target_depth_mm):
        if target_depth_mm is None:
            return "limit"
        depth = abs(float(target_depth_mm))
        return f"d{depth:.1f}mm".replace(".", "p")

    @staticmethod
    def _force_cycle_report_token(report_sample_target, trial, target_depth_mm=None):
        pieces = []
        if target_depth_mm is not None:
            pieces.append(AllInOneTesterGUI._force_cycle_depth_token(target_depth_mm))
        if report_sample_target:
            pieces.append(f"n{int(report_sample_target)}")
        if trial is not None:
            pieces.append(f"trial{int(trial)}")
        return "" if not pieces else "_" + "_".join(pieces)

    def _force_cycle_depth_mm_locked(self, now):
        if not getattr(self, "force_cycle_capture_active", False):
            return None

        phase = getattr(self, "force_cycle_phase", "idle")
        phase_started_at = getattr(self, "force_cycle_phase_started_at", None)
        down_seconds = getattr(self, "force_cycle_down_seconds", None)
        mm_per_second = float(FORCE_CYCLE_STEPPER_FREQUENCY_HZ) * float(STEPPER_MM_PER_PULSE)

        if phase == "down":
            if phase_started_at is None:
                return 0.0
            elapsed = max(0.0, float(now) - float(phase_started_at))
            return elapsed * mm_per_second

        if phase == "up":
            if down_seconds is None:
                return None
            down_depth = max(0.0, float(down_seconds) * mm_per_second)
            if phase_started_at is None:
                return down_depth
            elapsed = max(0.0, float(now) - float(phase_started_at))
            return max(0.0, down_depth - (elapsed * mm_per_second))

        if phase == "settle" and down_seconds is not None:
            return max(0.0, float(down_seconds) * mm_per_second)

        if phase == "complete":
            return 0.0

        return 0.0

    def _start_force_cycle_capture(
        self,
        started_at,
        shape_name,
        report_sample_target=None,
        trial=1,
        target_depth_mm=None,
    ):
        clean_shape = self._clean_force_cycle_shape_name(shape_name)
        with self.sensor_data_lock:
            self.force_cycle_sample_history = []
            self.force_cycle_capture_active = True
            self.force_cycle_capture_started_at = float(started_at)
            self.force_cycle_capture_last_time = None
            self.force_cycle_phase = "idle"
            self.force_cycle_phase_started_at = float(started_at)
            self.force_cycle_down_seconds = None
            self.force_cycle_shape_name = clean_shape
            self.force_cycle_report_sample_target = report_sample_target
            self.force_cycle_trial = int(trial)
            self.force_cycle_target_depth_mm = target_depth_mm
        target_text = "all" if report_sample_target is None else str(report_sample_target)
        depth_text = "limit" if target_depth_mm is None else f"{target_depth_mm:.1f} mm"
        self.log(
            f"Force cycle capture armed for shape: {clean_shape}, depth={depth_text}, "
            f"n={target_text}, trial={trial}."
        )

    def _set_force_cycle_phase(self, phase, phase_started_at=None, down_seconds=None):
        now = time.monotonic() if phase_started_at is None else float(phase_started_at)
        with self.sensor_data_lock:
            self.force_cycle_phase = str(phase)
            self.force_cycle_phase_started_at = now
            if down_seconds is not None:
                self.force_cycle_down_seconds = max(0.0, float(down_seconds))
        self._append_force_cycle_sample(now=now, force=True)

    def _stop_force_cycle_capture(self):
        with self.sensor_data_lock:
            self.force_cycle_capture_active = False
            self.force_cycle_phase = "idle"
            self.force_cycle_phase_started_at = None

    def _append_force_cycle_sample(self, now=None, force=False):
        now = time.monotonic() if now is None else float(now)
        appended = False
        with self.sensor_data_lock:
            if not getattr(self, "force_cycle_capture_active", False):
                return
            last_time = getattr(self, "force_cycle_capture_last_time", None)
            if (
                not force
                and last_time is not None
                and now - float(last_time) < FORCE_CYCLE_SAMPLE_MIN_INTERVAL_SECONDS
            ):
                return

            started_at = self.force_cycle_capture_started_at
            if started_at is None:
                return

            load_force = self.latest_loadcell_force_n
            pressure_force = self.latest_pressure_force_n
            pressure_hpa = self.latest_pressure_hpa
            zero_hpa = self.force_pressure_zero_hpa
            features = (
                dict(self.camera_deform_latest_features)
                if self.camera_deform_latest_features is not None
                else {}
            )
            pressure_delta = self._pressure_delta_from_zero(pressure_hpa, zero_hpa)
            indentation_depth_mm = self._force_cycle_depth_mm_locked(now)
            load_kg = None if load_force is None else float(load_force) / FORCE_GRAVITY_MPS2
            pressure_kg = None if pressure_force is None else float(pressure_force) / FORCE_GRAVITY_MPS2
            error_kg = (
                None
                if load_force is None or pressure_force is None
                else (float(pressure_force) - float(load_force)) / FORCE_GRAVITY_MPS2
            )

            self.force_cycle_sample_history.append(
                {
                    "elapsed_s": max(0.0, now - float(started_at)),
                    "phase": getattr(self, "force_cycle_phase", "idle"),
                    "shape_name": getattr(self, "force_cycle_shape_name", "sample"),
                    "target_depth_mm": getattr(self, "force_cycle_target_depth_mm", None),
                    "report_sample_target": getattr(self, "force_cycle_report_sample_target", None),
                    "trial": getattr(self, "force_cycle_trial", 1),
                    "indentation_depth_mm": indentation_depth_mm,
                    "load_force_n": None if load_force is None else float(load_force),
                    "load_kg": load_kg,
                    "pressure_force_n": None if pressure_force is None else float(pressure_force),
                    "pressure_kg_eq": pressure_kg,
                    "pressure_hpa": None if pressure_hpa is None else float(pressure_hpa),
                    "pressure_delta_hpa": pressure_delta,
                    "error_kg": error_kg,
                    "dot_mean_px": features.get("dot_mean_px"),
                    "dot_max_px": features.get("dot_max_px"),
                    "missing_ratio": features.get("missing_ratio"),
                    "contact_peak": features.get("contact_peak"),
                    "contact_area_ratio": features.get("contact_area_ratio"),
                    "residual_mean_px": features.get("residual_mean_px"),
                    "residual_peak_px": features.get("residual_peak_px"),
                    "surface_scale_px": features.get("surface_scale_px"),
                }
            )
            self.force_cycle_capture_last_time = now
            appended = True
        if appended:
            self._request_force_graph_redraw()

    @staticmethod
    def _csv_number(value, digits=6):
        if value is None:
            return ""
        try:
            value = float(value)
        except (TypeError, ValueError):
            return ""
        if not np.isfinite(value):
            return ""
        return f"{value:.{digits}f}"

    def _save_force_cycle_graphs(
        self,
        output_dir,
        file_stem,
        shape_name,
        history,
        report_sample_target=None,
        trial=1,
        raw_sample_count=None,
    ):
        try:
            import matplotlib

            matplotlib.use("Agg", force=True)
            import matplotlib.pyplot as plt
        except Exception as exc:
            self.log(f"Force cycle graph export skipped: {exc}")
            return []

        saved_paths = []
        report_text = "all samples" if report_sample_target is None else f"{report_sample_target} samples"
        target_depth = None
        for sample in history:
            target_depth = sample.get("target_depth_mm")
            if self._finite_number(target_depth):
                break
        depth_text = "limit" if target_depth is None else f"{float(target_depth):.1f} mm"
        title_suffix = f"{shape_name} | depth {depth_text} | report {report_text} | trial {trial}"
        if raw_sample_count is not None and raw_sample_count != len(history):
            title_suffix += f" | selected {len(history)}/{raw_sample_count}"

        def finite_series(key):
            points = []
            for sample in history:
                value = sample.get(key)
                elapsed = sample.get("elapsed_s")
                if self._finite_number(value) and self._finite_number(elapsed):
                    points.append((float(elapsed), float(value)))
            return points

        def min_max_points(points):
            if not points:
                return None, None
            return (
                min(points, key=lambda point: point[1]),
                max(points, key=lambda point: point[1]),
            )

        def add_min_max_summary(ax, lines):
            if not lines:
                return
            ax.text(
                0.99,
                0.97,
                "\n".join(lines),
                transform=ax.transAxes,
                ha="right",
                va="top",
                fontsize=8,
                bbox={
                    "boxstyle": "round,pad=0.25",
                    "facecolor": "white",
                    "edgecolor": "#c8d3d8",
                    "alpha": 0.85,
                },
            )

        def mark_min_max(ax, points, label, color, unit):
            min_point, max_point = min_max_points(points)
            if min_point is None or max_point is None:
                return None
            for marker_label, marker, point in (
                ("min", "v", min_point),
                ("max", "^", max_point),
            ):
                ax.scatter(
                    [point[0]],
                    [point[1]],
                    color=color,
                    edgecolor="white",
                    linewidth=0.6,
                    s=38,
                    marker=marker,
                    zorder=5,
                )
                ax.annotate(
                    f"{marker_label} {point[1]:.3f}",
                    xy=point,
                    xytext=(5, 7 if marker_label == "max" else -12),
                    textcoords="offset points",
                    fontsize=7,
                    color=color,
                )
            return f"{label} min/max: {min_point[1]:.3f}/{max_point[1]:.3f} {unit}"

        load_points = finite_series("load_kg")
        pressure_points = finite_series("pressure_kg_eq")
        error_points = finite_series("error_kg")

        if load_points or pressure_points:
            fig, ax = plt.subplots(figsize=(8.5, 4.8))
            ax.set_title(f"Force comparison: {title_suffix}")
            force_summary_lines = []
            if load_points:
                ax.plot(
                    [point[0] for point in load_points],
                    [point[1] for point in load_points],
                    label="load cell kg",
                    color="#139a74",
                    linewidth=1.6,
                )
                summary = mark_min_max(ax, load_points, "load", "#139a74", "kg")
                if summary:
                    force_summary_lines.append(summary)
            if pressure_points:
                ax.plot(
                    [point[0] for point in pressure_points],
                    [point[1] for point in pressure_points],
                    label="pressure kg-eq",
                    color="#f0a000",
                    linewidth=1.4,
                    linestyle="--",
                )
                summary = mark_min_max(ax, pressure_points, "pressure", "#f0a000", "kg")
                if summary:
                    force_summary_lines.append(summary)
            ax.set_xlabel("elapsed (s)")
            ax.set_ylabel("kg")
            ax.grid(True, alpha=0.28)
            add_min_max_summary(ax, force_summary_lines)
            ax.legend(loc="best")
            fig.tight_layout()
            path = output_dir / f"{file_stem}_force_comparison.png"
            fig.savefig(path, dpi=160)
            plt.close(fig)
            saved_paths.append(path)

        if error_points:
            fig, ax = plt.subplots(figsize=(8.5, 4.8))
            ax.set_title(f"Force error: {title_suffix}")
            error_summary_lines = []
            ax.plot(
                [point[0] for point in error_points],
                [point[1] for point in error_points],
                label="pressure kg-eq - load kg",
                color="#a34b2a",
                linewidth=1.5,
            )
            summary = mark_min_max(ax, error_points, "error", "#a34b2a", "kg")
            if summary:
                error_summary_lines.append(summary)
            ax.axhline(0.0, color="#4f9da6", linewidth=1.0, linestyle=":")
            ax.set_xlabel("elapsed (s)")
            ax.set_ylabel("error kg")
            ax.grid(True, alpha=0.28)
            add_min_max_summary(ax, error_summary_lines)
            ax.legend(loc="best")
            fig.tight_layout()
            path = output_dir / f"{file_stem}_force_error.png"
            fig.savefig(path, dpi=160)
            plt.close(fig)
            saved_paths.append(path)

        indent_points = []
        for sample in history:
            depth = sample.get("indentation_depth_mm")
            dot_mean = sample.get("dot_mean_px")
            if self._finite_number(depth) and self._finite_number(dot_mean):
                indent_points.append((float(depth), float(dot_mean), str(sample.get("phase", ""))))
        if indent_points:
            fig, ax = plt.subplots(figsize=(7.2, 4.8))
            indent_summary_lines = []
            colors = [
                "#139a74" if point[2] == "down" else "#d17800" if point[2] == "up" else "#6b879a"
                for point in indent_points
            ]
            ax.plot(
                [point[0] for point in indent_points],
                [point[1] for point in indent_points],
                color="#2f6178",
                linewidth=1.0,
                alpha=0.45,
            )
            ax.scatter(
                [point[0] for point in indent_points],
                [point[1] for point in indent_points],
                c=colors,
                s=18,
                alpha=0.85,
            )
            ax.set_title(f"Indentation vs mean dot displacement: {title_suffix}")
            ax.set_xlabel("indentation depth (mm)")
            ax.set_ylabel("mean dot displacement (px)")
            ax.grid(True, alpha=0.28)
            summary = mark_min_max(
                ax,
                [(point[0], point[1]) for point in indent_points],
                "dot displacement",
                "#2f6178",
                "px",
            )
            if summary:
                indent_summary_lines.append(summary)
            add_min_max_summary(ax, indent_summary_lines)
            fig.tight_layout()
            path = output_dir / f"{file_stem}_indentation_dot.png"
            fig.savefig(path, dpi=160)
            plt.close(fig)
            saved_paths.append(path)

        dot_mean_points = finite_series("dot_mean_px")
        dot_max_points = finite_series("dot_max_px")
        depth_points = finite_series("indentation_depth_mm")
        if dot_mean_points or dot_max_points or depth_points:
            fig, axes = plt.subplots(2, 1, figsize=(8.5, 6.0), sharex=True)
            fig.suptitle(f"Deformation and distance estimate: {title_suffix}")
            deform_summary_lines = []
            distance_summary_lines = []
            if dot_mean_points:
                axes[0].plot(
                    [point[0] for point in dot_mean_points],
                    [point[1] for point in dot_mean_points],
                    label="mean dot displacement",
                    color="#2f6178",
                    linewidth=1.5,
                )
                summary = mark_min_max(axes[0], dot_mean_points, "mean dot", "#2f6178", "px")
                if summary:
                    deform_summary_lines.append(summary)
            if dot_max_points:
                axes[0].plot(
                    [point[0] for point in dot_max_points],
                    [point[1] for point in dot_max_points],
                    label="max dot displacement",
                    color="#f0a000",
                    linewidth=1.2,
                    linestyle="--",
                )
                summary = mark_min_max(axes[0], dot_max_points, "max dot", "#f0a000", "px")
                if summary:
                    deform_summary_lines.append(summary)
            axes[0].set_ylabel("deformation (px)")
            axes[0].grid(True, alpha=0.28)
            if dot_mean_points or dot_max_points:
                axes[0].legend(loc="best")
            add_min_max_summary(axes[0], deform_summary_lines)

            if depth_points:
                axes[1].plot(
                    [point[0] for point in depth_points],
                    [point[1] for point in depth_points],
                    label="stepper indentation depth",
                    color="#139a74",
                    linewidth=1.5,
                )
                summary = mark_min_max(axes[1], depth_points, "depth", "#139a74", "mm")
                if summary:
                    distance_summary_lines.append(summary)
            axes[1].set_xlabel("elapsed (s)")
            axes[1].set_ylabel("distance estimate (mm)")
            axes[1].grid(True, alpha=0.28)
            if depth_points:
                axes[1].legend(loc="best")
            add_min_max_summary(axes[1], distance_summary_lines)
            fig.tight_layout()
            path = output_dir / f"{file_stem}_deformation_distance.png"
            fig.savefig(path, dpi=160)
            plt.close(fig)
            saved_paths.append(path)

        return saved_paths

    def _save_force_cycle_data(self, started_at):
        with self.sensor_data_lock:
            history = list(getattr(self, "force_cycle_sample_history", []))
            shape_name = getattr(self, "force_cycle_shape_name", "sample")
            report_sample_target = getattr(self, "force_cycle_report_sample_target", None)
            trial = getattr(self, "force_cycle_trial", 1)
            target_depth_mm = getattr(self, "force_cycle_target_depth_mm", None)

        if not history:
            self.log("Force cycle finished without force samples to save.")
            return None

        output_dir = Path(__file__).resolve().parent / "logs" / "force_cycles"
        output_dir.mkdir(parents=True, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        shape_token = self._safe_filename_token(shape_name)
        report_history = self._downsample_force_cycle_history(history, report_sample_target)
        report_token = self._force_cycle_report_token(report_sample_target, trial, target_depth_mm)
        file_stem = f"force_cycle_{timestamp}_{shape_token}{report_token}"
        output_path = output_dir / f"{file_stem}.csv"

        def write_cycle_csv(path, rows):
            with path.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.writer(handle)
                writer.writerow(
                    [
                        "elapsed_s",
                        "phase",
                        "shape_name",
                        "target_depth_mm",
                        "report_sample_target",
                        "trial",
                        "indentation_depth_mm",
                        "load_force_n",
                        "load_kg",
                        "pressure_force_n",
                        "pressure_kg_eq",
                        "pressure_hpa",
                        "pressure_delta_hpa",
                        "error_kg",
                        "dot_mean_px",
                        "dot_max_px",
                        "missing_ratio",
                        "contact_peak",
                        "contact_area_ratio",
                        "residual_mean_px",
                        "residual_peak_px",
                        "surface_scale_px",
                    ]
                )
                for sample in rows:
                    writer.writerow(
                        [
                            self._csv_number(sample.get("elapsed_s"), digits=3),
                            sample.get("phase", ""),
                            sample.get("shape_name", shape_name),
                            self._csv_number(sample.get("target_depth_mm"), digits=3),
                            "" if report_sample_target is None else str(report_sample_target),
                            str(trial),
                            self._csv_number(sample.get("indentation_depth_mm"), digits=6),
                            self._csv_number(sample.get("load_force_n"), digits=6),
                            self._csv_number(sample.get("load_kg"), digits=6),
                            self._csv_number(sample.get("pressure_force_n"), digits=6),
                            self._csv_number(sample.get("pressure_kg_eq"), digits=6),
                            self._csv_number(sample.get("pressure_hpa"), digits=4),
                            self._csv_number(sample.get("pressure_delta_hpa"), digits=4),
                            self._csv_number(sample.get("error_kg"), digits=6),
                            self._csv_number(sample.get("dot_mean_px"), digits=6),
                            self._csv_number(sample.get("dot_max_px"), digits=6),
                            self._csv_number(sample.get("missing_ratio"), digits=6),
                            self._csv_number(sample.get("contact_peak"), digits=6),
                            self._csv_number(sample.get("contact_area_ratio"), digits=6),
                            self._csv_number(sample.get("residual_mean_px"), digits=6),
                            self._csv_number(sample.get("residual_peak_px"), digits=6),
                            self._csv_number(sample.get("surface_scale_px"), digits=6),
                        ]
                    )

        write_cycle_csv(output_path, report_history)
        raw_output_path = None
        if report_sample_target is not None and len(report_history) != len(history):
            raw_output_path = output_dir / f"{file_stem}_raw.csv"
            write_cycle_csv(raw_output_path, history)

        summary_path = output_dir / f"{file_stem}_summary.csv"
        with summary_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(
                [
                    "series",
                    "unit",
                    "shape_name",
                    "target_depth_mm",
                    "report_sample_target",
                    "trial",
                    "sample_count",
                    "min_value",
                    "min_elapsed_s",
                    "max_value",
                    "max_elapsed_s",
                ]
            )
            for series_key, label, unit in (
                ("load_kg", "load cell", "kg"),
                ("pressure_kg_eq", "pressure estimate", "kg"),
                ("error_kg", "force error", "kg"),
                ("indentation_depth_mm", "distance estimate", "mm"),
                ("dot_mean_px", "mean dot deformation", "px"),
                ("dot_max_px", "max dot deformation", "px"),
                ("contact_peak", "contact peak", "ratio"),
                ("contact_area_ratio", "contact area", "percent"),
            ):
                stats = self._force_cycle_series_min_max(report_history, series_key)
                if stats is None:
                    continue
                writer.writerow(
                    [
                        label,
                        unit,
                        shape_name,
                        self._csv_number(target_depth_mm, digits=3),
                        "" if report_sample_target is None else str(report_sample_target),
                        str(trial),
                        str(len(report_history)),
                        self._csv_number(stats["min_value"], digits=6),
                        self._csv_number(stats["min_elapsed_s"], digits=3),
                        self._csv_number(stats["max_value"], digits=6),
                        self._csv_number(stats["max_elapsed_s"], digits=3),
                    ]
                )

        graph_paths = self._save_force_cycle_graphs(
            output_dir,
            file_stem,
            shape_name,
            report_history,
            report_sample_target=report_sample_target,
            trial=trial,
            raw_sample_count=len(history),
        )
        saved_names = [output_path.name, summary_path.name]
        if raw_output_path is not None:
            saved_names.append(raw_output_path.name)
        saved_names.extend(path.name for path in graph_paths)
        graph_text = ", ".join(saved_names)
        if report_sample_target is not None and len(report_history) < int(report_sample_target):
            self.log(
                f"Force cycle requested {report_sample_target} report samples but only "
                f"{len(report_history)} were available."
            )
        if graph_text:
            self.log(
                f"Force cycle data saved: {graph_text} ({len(report_history)} report samples; "
                f"{len(history)} raw samples)."
            )
        else:
            self.log(f"Force cycle data saved: {output_path.name} ({len(report_history)} samples).")
        return output_path

    def start_force_cycle(self):
        if self._is_thread_running(self.force_cycle_thread):
            self.log("Force cycle is already running.")
            return
        if self._is_thread_running(self.stepper_thread):
            messagebox.showwarning("Force Cycle", "Stop the current stepper move before starting a force cycle.")
            return
        if not self._ensure_limit_monitor_for_stepper(show_dialog=True):
            return
        if self._get_limit_trigger_state()[1]:
            messagebox.showwarning(
                "Force Cycle",
                "Limit 2 / GPIO27 is already triggered. Move up and set the start height first.",
            )
            self.log("Force cycle blocked because Limit 2 is already triggered at the start height.")
            return
        if self._load_loadcell_calibration() is None:
            messagebox.showwarning("Force Cycle", "Cal Load once before running a force data cycle.")
            return
        with self.sensor_data_lock:
            force_coeffs = self.force_calibration_coeffs
            force_sample_count = len(self.force_calibration_samples)
        if force_coeffs is None:
            messagebox.showwarning(
                "Force Cycle",
                "Calibrate force before running a force data cycle. Capture 2+ force samples and press Fit Calibration.",
            )
            self.force_calibration_status_var.set(
                f"Force cycle blocked: force calibration missing ({force_sample_count} sample(s))."
            )
            return
        if not self._is_thread_running(self.pressure_thread):
            self.start_pressure_read()
        if not self._is_thread_running(self.loadcell_thread):
            self._reset_loadcell_filter(reset_live_zero=True)
            self.start_loadcell_read()

        self.force_cycle_requested_shape_name = self._clean_force_cycle_shape_name(
            self.force_cycle_shape_var.get()
        )
        self.force_cycle_requested_target_depth_mm = None
        self.force_cycle_requested_report_sample_target = self._force_cycle_report_sample_target()
        self.force_cycle_requested_trial = self._force_cycle_report_trial()
        if not self._is_blob_running():
            self.log("Force cycle will save force data only; Start Detection first to include shape displacement.")
        self.force_cycle_stop_event.clear()
        self.force_cycle_thread = threading.Thread(target=self._force_cycle_worker, daemon=True)
        self.force_cycle_thread.start()
        self._set_force_cycle_controls(True)
        self._set_var(self.force_calibration_status_var, "Force cycle running.")
        target_text = (
            "all"
            if self.force_cycle_requested_report_sample_target is None
            else str(self.force_cycle_requested_report_sample_target)
        )
        depth_text = (
            "limit"
            if self.force_cycle_requested_target_depth_mm is None
            else f"{self.force_cycle_requested_target_depth_mm:.1f} mm"
        )
        self.log(
            f"Force cycle started for shape: {self.force_cycle_requested_shape_name}, "
            f"depth={depth_text}, report n={target_text}, trial={self.force_cycle_requested_trial}."
        )

    def stop_force_cycle(self):
        self.force_cycle_stop_event.set()
        if self._is_thread_running(self.stepper_thread):
            self.stop_stepper_move()
        if self.force_cycle_thread is not None and self.force_cycle_thread.is_alive():
            self.force_cycle_thread.join(timeout=1.0)
        self.force_cycle_thread = None
        self._set_force_cycle_controls(False)

    def _force_cycle_wait_stepper_done(self, target_limit_index=None):
        while self._is_thread_running(self.stepper_thread):
            if self.force_cycle_stop_event.is_set():
                self.stop_stepper_move()
                return False
            if target_limit_index is not None and self._get_limit_trigger_state()[target_limit_index]:
                self.stepper_stop_reason = f"Force cycle reached Limit {target_limit_index + 1}"
                self.stepper_stop_event.set()
            time.sleep(0.05)
        return not self.force_cycle_stop_event.is_set()

    def _force_cycle_move_until_limit_timed(self, direction, target_limit_index, label):
        if self._get_limit_trigger_state()[target_limit_index]:
            self.log(f"Force cycle aborted: {label} is already triggered before motion.")
            return None

        self.log(f"Force cycle: moving {direction} toward {label}.")
        started_at = time.monotonic()
        self._set_force_cycle_phase(direction, phase_started_at=started_at)
        if not self._start_stepper_move_command(
            direction,
            FORCE_CYCLE_LIMIT_MAX_SECONDS,
            show_dialog=False,
            frequency_hz=FORCE_CYCLE_STEPPER_FREQUENCY_HZ,
        ):
            return None
        if not self._force_cycle_wait_stepper_done(target_limit_index=target_limit_index):
            return None

        elapsed_seconds = time.monotonic() - started_at
        if not self._get_limit_trigger_state()[target_limit_index]:
            self.log(f"Force cycle stopped before reaching {label}.")
            return None
        self.log(f"Force cycle reached {label} after {elapsed_seconds:.2f}s.")
        if direction == "down":
            self._set_force_cycle_phase("settle", down_seconds=elapsed_seconds)
        return max(0.0, elapsed_seconds)

    def _force_cycle_move_for_seconds(self, direction, seconds, label):
        self.log(f"Force cycle: {label}.")
        phase_started_at = time.monotonic()
        self._set_force_cycle_phase(
            direction,
            phase_started_at=phase_started_at,
            down_seconds=seconds if direction == "up" else None,
        )
        if not self._start_stepper_move_command(
            direction,
            seconds,
            show_dialog=False,
            frequency_hz=FORCE_CYCLE_STEPPER_FREQUENCY_HZ,
        ):
            return False
        return self._force_cycle_wait_stepper_done(target_limit_index=None)

    def _force_cycle_move_for_seconds_timed(self, direction, seconds, label):
        self.log(f"Force cycle: {label}.")
        phase_started_at = time.monotonic()
        self._set_force_cycle_phase(
            direction,
            phase_started_at=phase_started_at,
            down_seconds=seconds if direction == "up" else None,
        )
        if not self._start_stepper_move_command(
            direction,
            seconds,
            show_dialog=False,
            frequency_hz=FORCE_CYCLE_STEPPER_FREQUENCY_HZ,
        ):
            return None
        if not self._force_cycle_wait_stepper_done(target_limit_index=None):
            return None
        elapsed = max(0.0, time.monotonic() - phase_started_at)
        return elapsed

    def _force_cycle_sleep(self):
        if FORCE_CYCLE_SETTLE_SECONDS <= 0:
            return not self.force_cycle_stop_event.is_set()
        deadline = time.monotonic() + FORCE_CYCLE_SETTLE_SECONDS
        while time.monotonic() < deadline:
            if self.force_cycle_stop_event.is_set():
                return False
            time.sleep(0.02)
        return True

    def _force_cycle_wait_sensor_ready(self, timeout=8.0):
        self.log("Force cycle: waiting for pressure/load-cell samples.")
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.force_cycle_stop_event.is_set():
                return False
            with self.sensor_data_lock:
                pressure_ready = self.latest_pressure_hpa is not None
                load_ready = self.latest_loadcell_kg is not None
            if pressure_ready and load_ready:
                return True
            time.sleep(0.05)
        self.log("Force cycle aborted: pressure/load-cell samples were not ready.")
        return False

    def _force_cycle_worker(self):
        started_at = None
        completed = False
        abort_status = None
        try:
            if not self._force_cycle_wait_sensor_ready():
                abort_status = "Force cycle aborted: pressure/load-cell samples were not ready."
                return
            started_at = time.monotonic()
            shape_name = getattr(self, "force_cycle_requested_shape_name", "sample")
            report_sample_target = getattr(self, "force_cycle_requested_report_sample_target", None)
            trial = getattr(self, "force_cycle_requested_trial", 1)
            target_depth_mm = None
            self._start_force_cycle_capture(
                started_at,
                shape_name,
                report_sample_target,
                trial,
                target_depth_mm=target_depth_mm,
            )
            self._append_force_cycle_sample(now=started_at, force=True)

            self._set_var(self.force_calibration_status_var, "Force cycle moving down to Limit 2.")
            down_seconds = self._force_cycle_move_until_limit_timed("down", 1, "Limit 2")
            if down_seconds is None:
                abort_status = "Force cycle aborted during down stroke. Check Limit 2 and stepper movement."
                return
            if down_seconds <= 0.05:
                abort_status = (
                    "Force cycle aborted: down stroke time was too short. "
                    "Limit 2 may already be triggered/noisy."
                )
                self.log(abort_status)
                return
            self._set_force_cycle_phase("settle", down_seconds=down_seconds)
            if not self._force_cycle_sleep():
                abort_status = "Force cycle stopped during settle."
                return
            if not self._force_cycle_move_for_seconds(
                "up",
                down_seconds,
                f"return up for {down_seconds:.2f}s",
            ):
                abort_status = "Force cycle aborted during return up stroke."
                return
            self._set_force_cycle_phase("complete", down_seconds=down_seconds)
            completed = True
            output_path = self._save_force_cycle_data(started_at or time.monotonic())
            status = f"Force cycle complete. Down/up time {down_seconds:.2f}s."
            if output_path is not None:
                status += f" Saved {output_path.name}."
            self._set_var(self.force_calibration_status_var, status)
        except Exception as exc:
            self.log(f"Force cycle error: {exc}")
            self._set_var(self.force_calibration_status_var, f"Force cycle error: {exc}")
        finally:
            if not completed and self.force_cycle_stop_event.is_set():
                self.log("Force cycle stopped by user.")
                self._set_var(self.force_calibration_status_var, "Force cycle stopped.")
            elif not completed:
                self._set_var(self.force_calibration_status_var, abort_status or "Force cycle aborted.")
            self._stop_force_cycle_capture()
            self.force_cycle_stop_event.clear()
            self.force_cycle_thread = None
            self._set_force_cycle_controls(False)

    def _request_force_graph_redraw(self):
        if not self.root.winfo_exists():
            return
        graph_canvases = self._force_graph_canvases()
        if not any(graph_canvases):
            return
        if getattr(self, "force_graph_redraw_pending", False):
            return

        self.force_graph_redraw_pending = True

        def redraw():
            self.force_graph_redraw_pending = False
            self._draw_force_graphs()

        self.root.after(80, redraw)

    def _force_graph_canvases(self):
        compare_canvases = []
        error_canvases = []
        deform_canvases = []
        distance_canvases = []
        seen = set()
        for name, target in (
            ("force_compare_canvas", compare_canvases),
            ("force_error_canvas", error_canvases),
            ("force_deform_canvas", deform_canvases),
            ("force_distance_canvas", distance_canvases),
        ):
            canvas = getattr(self, name, None)
            if canvas is None:
                continue
            if id(canvas) in seen:
                continue
            try:
                if not canvas.winfo_exists():
                    continue
            except tk.TclError:
                continue
            seen.add(id(canvas))
            target.append(canvas)
        return compare_canvases, error_canvases, deform_canvases, distance_canvases

    @staticmethod
    def _finite_number(value):
        return isinstance(value, (int, float)) and bool(np.isfinite(float(value)))

    @staticmethod
    def _graph_bounds(values, include_zero=True):
        finite_values = [float(value) for value in values if AllInOneTesterGUI._finite_number(value)]
        if not finite_values:
            return 0.0, 1.0

        min_value = min(finite_values)
        max_value = max(finite_values)
        if include_zero:
            min_value = min(0.0, min_value)
            max_value = max(0.0, max_value)
        if abs(max_value - min_value) < 1e-9:
            pad = max(1.0, abs(max_value) * 0.1)
        else:
            pad = (max_value - min_value) * 0.12
        return min_value - pad, max_value + pad

    def _draw_force_graphs(self):
        if not self.root.winfo_exists():
            return

        compare_canvases, error_canvases, deform_canvases, distance_canvases = self._force_graph_canvases()
        if not any((compare_canvases, error_canvases, deform_canvases, distance_canvases)):
            return

        for canvas in compare_canvases:
            self._draw_force_time_graph(canvas)
        for canvas in error_canvases:
            self._draw_force_error_graph(canvas)
        for canvas in deform_canvases:
            self._draw_force_cycle_deformation_graph(canvas)
        for canvas in distance_canvases:
            self._draw_force_cycle_distance_graph(canvas)

    def _prepare_graph_canvas(self, canvas, title, x_label, y_label):
        width = max(1, int(canvas.winfo_width()))
        height = max(1, int(canvas.winfo_height()))
        canvas.delete("all")
        canvas.create_rectangle(0, 0, width, height, fill="#f9fbfc", outline="")

        left = 48
        right = max(left + 20, width - 12)
        top = 28
        bottom = max(top + 20, height - 28)
        plot_width = max(1, right - left)
        plot_height = max(1, bottom - top)

        canvas.create_text(
            10,
            8,
            text=title,
            anchor="nw",
            fill=COLOR_TEXT,
            font=("Segoe UI", 9, "bold"),
        )
        canvas.create_text(
            (left + right) / 2,
            height - 5,
            text=x_label,
            anchor="s",
            fill=COLOR_MUTED,
            font=("Segoe UI", 8),
        )
        canvas.create_text(
            6,
            (top + bottom) / 2,
            text=y_label,
            anchor="w",
            fill=COLOR_MUTED,
            font=("Segoe UI", 8),
        )
        canvas.create_rectangle(left, top, right, bottom, outline=COLOR_BORDER)
        return width, height, left, top, right, bottom, plot_width, plot_height

    def _draw_empty_graph(self, canvas, title, message):
        width, height, *_ = self._prepare_graph_canvas(canvas, title, "", "")
        canvas.create_text(
            width / 2,
            height / 2,
            text=message,
            anchor="center",
            fill=COLOR_MUTED,
            font=("Segoe UI", 9),
        )

    def _draw_force_time_graph(self, canvas):
        with self.sensor_data_lock:
            history = list(self.force_graph_history)

        if not history:
            self._draw_empty_graph(
                canvas,
                "Load comparison",
                "Waiting for load cell / pressure data",
            )
            return

        (
            _width,
            _height,
            left,
            top,
            right,
            bottom,
            plot_width,
            plot_height,
        ) = self._prepare_graph_canvas(
            canvas,
            "Load comparison",
            f"last {int(FORCE_GRAPH_HISTORY_SECONDS)} s",
            "kg",
        )

        now = max(point[0] for point in history)
        x_min = now - FORCE_GRAPH_HISTORY_SECONDS
        x_max = now
        y_values = []
        for _timestamp, load_force, pressure_force, _pressure_hpa in history:
            if self._finite_number(load_force):
                y_values.append(float(load_force) / FORCE_GRAVITY_MPS2)
            if self._finite_number(pressure_force):
                y_values.append(float(pressure_force) / FORCE_GRAVITY_MPS2)

        if not y_values:
            canvas.create_text(
                (left + right) / 2,
                (top + bottom) / 2,
                text="Waiting for force calibration",
                anchor="center",
                fill=COLOR_MUTED,
                font=("Segoe UI", 9),
            )
            return

        y_min, y_max = self._graph_bounds(y_values, include_zero=True)

        def x_for(timestamp):
            ratio = (float(timestamp) - x_min) / max(1e-9, x_max - x_min)
            return left + (np.clip(ratio, 0.0, 1.0) * plot_width)

        def y_for(value):
            ratio = (float(value) - y_min) / max(1e-9, y_max - y_min)
            return bottom - (np.clip(ratio, 0.0, 1.0) * plot_height)

        for idx in range(4):
            ratio = idx / 3.0
            y = top + (ratio * plot_height)
            value = y_max - (ratio * (y_max - y_min))
            canvas.create_line(left, y, right, y, fill="#e1e8ea")
            canvas.create_text(
                left - 5,
                y,
                text=f"{value:.1f}",
                anchor="e",
                fill=COLOR_MUTED,
                font=("Segoe UI", 7),
            )

        for idx in range(3):
            ratio = idx / 2.0
            x = left + (ratio * plot_width)
            seconds_ago = int(round((1.0 - ratio) * FORCE_GRAPH_HISTORY_SECONDS))
            label = "now" if seconds_ago == 0 else f"-{seconds_ago}s"
            canvas.create_line(x, top, x, bottom, fill="#edf2f3")
            canvas.create_text(
                x,
                bottom + 4,
                text=label,
                anchor="n",
                fill=COLOR_MUTED,
                font=("Segoe UI", 7),
            )

        def draw_series(value_index, color, dash=None):
            coords = []
            last_point = None
            for point in history:
                value = point[value_index]
                if not self._finite_number(value):
                    if len(coords) >= 4:
                        canvas.create_line(*coords, fill=color, width=2, smooth=True, dash=dash)
                    coords = []
                    continue
                value = float(value) / FORCE_GRAVITY_MPS2
                x = x_for(point[0])
                y = y_for(value)
                coords.extend((x, y))
                last_point = (x, y)
            if len(coords) >= 4:
                canvas.create_line(*coords, fill=color, width=2, smooth=True, dash=dash)
            if last_point is not None:
                x, y = last_point
                canvas.create_oval(x - 3, y - 3, x + 3, y + 3, fill=color, outline=color)

        load_color = COLOR_OK
        pressure_color = COLOR_WARN
        draw_series(1, load_color)
        draw_series(2, pressure_color, dash=(4, 2))

        legend_y = top - 9
        canvas.create_line(right - 150, legend_y, right - 132, legend_y, fill=load_color, width=2)
        canvas.create_text(
            right - 128,
            legend_y,
            text="load cell kg",
            anchor="w",
            fill=COLOR_MUTED,
            font=("Segoe UI", 7),
        )
        canvas.create_line(
            right - 80,
            legend_y,
            right - 62,
            legend_y,
            fill=pressure_color,
            width=2,
            dash=(4, 2),
        )
        canvas.create_text(
            right - 58,
            legend_y,
            text="pressure kg-eq",
            anchor="w",
            fill=COLOR_MUTED,
            font=("Segoe UI", 7),
        )

    def _draw_force_error_graph(self, canvas):
        with self.sensor_data_lock:
            history = list(self.force_graph_history)

        error_points = []
        for timestamp, load_force, pressure_force, _pressure_hpa in history:
            if self._finite_number(load_force) and self._finite_number(pressure_force):
                error_points.append(
                    (
                        float(timestamp),
                        (float(pressure_force) - float(load_force)) / FORCE_GRAVITY_MPS2,
                    )
                )

        if not error_points:
            self._draw_empty_graph(
                canvas,
                "Force error",
                "Waiting for both force lines",
            )
            return

        (
            _width,
            _height,
            left,
            top,
            right,
            bottom,
            plot_width,
            plot_height,
        ) = self._prepare_graph_canvas(
            canvas,
            "Load error (pressure kg-eq - load cell kg)",
            f"last {int(FORCE_GRAPH_HISTORY_SECONDS)} s",
            "kg",
        )

        now = max(point[0] for point in history) if history else error_points[-1][0]
        x_min = now - FORCE_GRAPH_HISTORY_SECONDS
        x_max = now
        y_min, y_max = self._graph_bounds([point[1] for point in error_points], include_zero=True)

        def x_for(timestamp):
            ratio = (float(timestamp) - x_min) / max(1e-9, x_max - x_min)
            return left + (np.clip(ratio, 0.0, 1.0) * plot_width)

        def y_for(value):
            ratio = (float(value) - y_min) / max(1e-9, y_max - y_min)
            return bottom - (np.clip(ratio, 0.0, 1.0) * plot_height)

        for idx in range(4):
            ratio = idx / 3.0
            y = top + (ratio * plot_height)
            value = y_max - (ratio * (y_max - y_min))
            canvas.create_line(left, y, right, y, fill="#e1e8ea")
            canvas.create_text(
                left - 5,
                y,
                text=f"{value:.1f}",
                anchor="e",
                fill=COLOR_MUTED,
                font=("Segoe UI", 7),
            )

        for idx in range(3):
            ratio = idx / 2.0
            x = left + (ratio * plot_width)
            seconds_ago = int(round((1.0 - ratio) * FORCE_GRAPH_HISTORY_SECONDS))
            label = "now" if seconds_ago == 0 else f"-{seconds_ago}s"
            canvas.create_line(x, top, x, bottom, fill="#edf2f3")
            canvas.create_text(
                x,
                bottom + 4,
                text=label,
                anchor="n",
                fill=COLOR_MUTED,
                font=("Segoe UI", 7),
            )

        zero_y = y_for(0.0)
        canvas.create_line(left, zero_y, right, zero_y, fill=COLOR_ACCENT, width=1, dash=(3, 2))

        error_color = "#a34b2a"
        coords = []
        last_point = None
        for timestamp, error_n in error_points:
            x = x_for(timestamp)
            y = y_for(error_n)
            coords.extend((x, y))
            last_point = (x, y, error_n)
        if len(coords) >= 4:
            canvas.create_line(*coords, fill=error_color, width=2, smooth=True)
        if last_point is not None:
            x, y, error_n = last_point
            canvas.create_oval(x - 3, y - 3, x + 3, y + 3, fill=error_color, outline=error_color)
            canvas.create_text(
                right - 6,
                top + 4,
                text=f"latest {error_n:+.3f} kg | abs {abs(error_n):.3f} kg",
                anchor="ne",
                fill=COLOR_MUTED,
                font=("Segoe UI", 7),
            )

        legend_y = top - 9
        canvas.create_line(right - 160, legend_y, right - 142, legend_y, fill=error_color, width=2)
        canvas.create_text(
            right - 138,
            legend_y,
            text="pressure kg-eq - load kg",
            anchor="w",
            fill=COLOR_MUTED,
            font=("Segoe UI", 7),
        )

    def _draw_force_cycle_series_graph(self, canvas, title, y_label, series_specs, empty_message):
        with self.sensor_data_lock:
            history = list(getattr(self, "force_cycle_sample_history", []))

        series_points = []
        y_values = []
        x_values = []
        for key, label, color, dash in series_specs:
            points = []
            for sample in history:
                elapsed = sample.get("elapsed_s")
                value = sample.get(key)
                if self._finite_number(elapsed) and self._finite_number(value):
                    point = (float(elapsed), float(value))
                    points.append(point)
                    x_values.append(point[0])
                    y_values.append(point[1])
            series_points.append((label, color, dash, points))

        if not y_values:
            self._draw_empty_graph(canvas, title, empty_message)
            return

        (
            _width,
            _height,
            left,
            top,
            right,
            bottom,
            plot_width,
            plot_height,
        ) = self._prepare_graph_canvas(canvas, title, "cycle elapsed (s)", y_label)

        x_min = 0.0
        x_max = max(1.0, max(x_values))
        y_min, y_max = self._graph_bounds(y_values, include_zero=True)

        def x_for(value):
            ratio = (float(value) - x_min) / max(1e-9, x_max - x_min)
            return left + (np.clip(ratio, 0.0, 1.0) * plot_width)

        def y_for(value):
            ratio = (float(value) - y_min) / max(1e-9, y_max - y_min)
            return bottom - (np.clip(ratio, 0.0, 1.0) * plot_height)

        for idx in range(4):
            ratio = idx / 3.0
            y = top + (ratio * plot_height)
            value = y_max - (ratio * (y_max - y_min))
            canvas.create_line(left, y, right, y, fill="#e1e8ea")
            canvas.create_text(
                left - 5,
                y,
                text=f"{value:.1f}",
                anchor="e",
                fill=COLOR_MUTED,
                font=("Segoe UI", 7),
            )

        for idx in range(3):
            ratio = idx / 2.0
            x = left + (ratio * plot_width)
            value = x_min + (ratio * (x_max - x_min))
            canvas.create_line(x, top, x, bottom, fill="#edf2f3")
            canvas.create_text(
                x,
                bottom + 4,
                text=f"{value:.1f}",
                anchor="n",
                fill=COLOR_MUTED,
                font=("Segoe UI", 7),
            )

        legend_x = right - 150
        legend_y = top - 9
        for index, (label, color, dash, points) in enumerate(series_points):
            if not points:
                continue
            coords = []
            last_point = None
            for elapsed, value in points:
                x = x_for(elapsed)
                y = y_for(value)
                coords.extend((x, y))
                last_point = (x, y, value)
            if len(coords) >= 4:
                canvas.create_line(*coords, fill=color, width=2, smooth=True, dash=dash)
            if last_point is not None:
                x, y, value = last_point
                canvas.create_oval(x - 3, y - 3, x + 3, y + 3, fill=color, outline=color)
                if index == 0:
                    canvas.create_text(
                        right - 6,
                        top + 4,
                        text=f"latest {value:.3f}",
                        anchor="ne",
                        fill=COLOR_MUTED,
                        font=("Segoe UI", 7),
                    )
            legend_offset = index * 78
            canvas.create_line(
                legend_x + legend_offset,
                legend_y,
                legend_x + legend_offset + 18,
                legend_y,
                fill=color,
                width=2,
                dash=dash,
            )
            canvas.create_text(
                legend_x + legend_offset + 22,
                legend_y,
                text=label,
                anchor="w",
                fill=COLOR_MUTED,
                font=("Segoe UI", 7),
            )

    def _draw_force_cycle_deformation_graph(self, canvas):
        self._draw_force_cycle_series_graph(
            canvas,
            "Cycle deformation monitor",
            "px",
            (
                ("dot_mean_px", "mean dot", COLOR_ACCENT, None),
                ("dot_max_px", "max dot", COLOR_WARN, (4, 2)),
            ),
            "Start Detection, then Run Force Cycle",
        )

    def _draw_force_cycle_distance_graph(self, canvas):
        self._draw_force_cycle_series_graph(
            canvas,
            "Cycle distance estimate",
            "mm",
            (("indentation_depth_mm", "stepper depth", COLOR_OK, None),),
            "Run Force Cycle to estimate indentation depth",
        )

    @staticmethod
    def _force_linear_fit_score(samples, slope, intercept):
        if len(samples) < 2:
            return None
        forces = [float(force_n) for _pressure_delta, force_n in samples]
        mean_force = sum(forces) / len(forces)
        total = sum((force_n - mean_force) ** 2 for force_n in forces)
        if total <= 1e-12:
            return None
        residual = sum(
            (float(force_n) - ((float(pressure_delta) * slope) + intercept)) ** 2
            for pressure_delta, force_n in samples
        )
        return 1.0 - (residual / total)

    def show_force_linear_equation_graph(self):
        with self.sensor_data_lock:
            samples = list(self.force_calibration_samples)
            coeffs = self.force_calibration_coeffs

        if len(samples) >= 2 and coeffs is None:
            self._fit_force_calibration(show_warning=False)

        window = getattr(self, "force_linear_graph_window", None)
        if window is not None and window.winfo_exists():
            window.lift()
            window.focus_force()
            self._refresh_force_linear_equation_graph()
            return

        window = tk.Toplevel(self.root)
        window.title("Force Linear Equation")
        window.geometry("760x520")
        window.minsize(520, 360)
        window.transient(self.root)
        try:
            window.configure(bg=COLOR_BG)
        except tk.TclError:
            pass

        self.force_linear_graph_window = window
        self.force_linear_graph_summary_var = tk.StringVar(value="")

        container = ttk.Frame(window, padding=12, style="App.TFrame")
        container.grid(row=0, column=0, sticky="nsew")
        container.columnconfigure(0, weight=1)
        container.rowconfigure(1, weight=1)
        window.columnconfigure(0, weight=1)
        window.rowconfigure(0, weight=1)

        ttk.Label(
            container,
            text="Linear equation from captured force samples",
            style="SectionTitle.TLabel",
        ).grid(row=0, column=0, sticky="w")
        ttk.Label(
            container,
            textvariable=self.force_linear_graph_summary_var,
            style="SectionHint.TLabel",
            wraplength=700,
            justify="left",
        ).grid(row=0, column=1, sticky="e", padx=(12, 0))

        canvas = tk.Canvas(
            container,
            height=380,
            background="#f9fbfc",
            highlightthickness=1,
            highlightbackground=COLOR_BORDER,
        )
        canvas.grid(row=1, column=0, columnspan=2, sticky="nsew", pady=(10, 0))
        self.force_linear_graph_canvas = canvas

        footer = ttk.Frame(container, style="App.TFrame")
        footer.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(10, 0))
        footer.columnconfigure(0, weight=1)
        ttk.Button(footer, text="Refresh", command=self._refresh_force_linear_equation_graph).grid(
            row=0, column=1, sticky="e", padx=(0, 8)
        )
        close_btn = ttk.Button(footer, text="Close", command=window.destroy)
        close_btn.grid(row=0, column=2, sticky="e")

        def on_close():
            self.force_linear_graph_window = None
            self.force_linear_graph_canvas = None
            window.destroy()

        close_btn.configure(command=on_close)
        window.protocol("WM_DELETE_WINDOW", on_close)
        canvas.bind("<Configure>", lambda _event: self._refresh_force_linear_equation_graph())
        window.after(80, self._refresh_force_linear_equation_graph)

    def _refresh_force_linear_equation_graph(self):
        canvas = getattr(self, "force_linear_graph_canvas", None)
        if canvas is None or not canvas.winfo_exists():
            return
        self._draw_force_linear_equation_graph(canvas)

    def _draw_force_linear_equation_graph(self, canvas):
        with self.sensor_data_lock:
            samples = list(self.force_calibration_samples)
            coeffs = self.force_calibration_coeffs

        if len(samples) >= 2 and coeffs is None:
            coeffs = None

        summary_var = getattr(self, "force_linear_graph_summary_var", None)
        if not samples:
            if summary_var is not None:
                summary_var.set("Capture bubble samples to build the linear equation.")
            self._draw_empty_graph(
                canvas,
                "Force calibration samples",
                "No bubble samples captured yet",
            )
            return

        pressure_deltas = [float(pressure_delta) for pressure_delta, _force_n in samples]
        forces = [float(force_n) for _pressure_delta, force_n in samples]
        x_min, x_max = self._graph_bounds(pressure_deltas, include_zero=True)

        line_points = []
        if coeffs is not None:
            slope, intercept = coeffs
            line_points = [
                (x_min, (slope * x_min) + intercept),
                (x_max, (slope * x_max) + intercept),
            ]
            fit_score = self._force_linear_fit_score(samples, slope, intercept)
            fit_text = "" if fit_score is None else f" | R^2 {fit_score:.3f}"
            if summary_var is not None:
                summary_var.set(
                    f"F(N) = {slope:.6f} * dP(hPa) + {intercept:.3f} | "
                    f"{len(samples)} samples{fit_text}"
                )
            y_values = forces + [point[1] for point in line_points]
        else:
            if summary_var is not None:
                summary_var.set("Need at least 2 samples with distinct dP values to fit a line.")
            y_values = forces

        y_min, y_max = self._graph_bounds(y_values, include_zero=True)

        (
            _width,
            _height,
            left,
            top,
            right,
            bottom,
            plot_width,
            plot_height,
        ) = self._prepare_graph_canvas(
            canvas,
            "Force sample linear fit",
            "pressure delta dP (hPa)",
            "force (N)",
        )

        def x_for(value):
            ratio = (float(value) - x_min) / max(1e-9, x_max - x_min)
            return left + (np.clip(ratio, 0.0, 1.0) * plot_width)

        def y_for(value):
            ratio = (float(value) - y_min) / max(1e-9, y_max - y_min)
            return bottom - (np.clip(ratio, 0.0, 1.0) * plot_height)

        for idx in range(5):
            ratio = idx / 4.0
            y = top + (ratio * plot_height)
            value = y_max - (ratio * (y_max - y_min))
            canvas.create_line(left, y, right, y, fill="#e1e8ea")
            canvas.create_text(
                left - 5,
                y,
                text=f"{value:.2f}",
                anchor="e",
                fill=COLOR_MUTED,
                font=("Segoe UI", 7),
            )

        for idx in range(5):
            ratio = idx / 4.0
            x = left + (ratio * plot_width)
            value = x_min + (ratio * (x_max - x_min))
            canvas.create_line(x, top, x, bottom, fill="#edf2f3")
            canvas.create_text(
                x,
                bottom + 4,
                text=f"{value:.2f}",
                anchor="n",
                fill=COLOR_MUTED,
                font=("Segoe UI", 7),
            )

        if x_min <= 0.0 <= x_max:
            zero_x = x_for(0.0)
            canvas.create_line(zero_x, top, zero_x, bottom, fill=COLOR_BORDER, dash=(3, 2))
        if y_min <= 0.0 <= y_max:
            zero_y = y_for(0.0)
            canvas.create_line(left, zero_y, right, zero_y, fill=COLOR_BORDER, dash=(3, 2))

        if line_points:
            line_color = COLOR_WARN
            canvas.create_line(
                x_for(line_points[0][0]),
                y_for(line_points[0][1]),
                x_for(line_points[1][0]),
                y_for(line_points[1][1]),
                fill=line_color,
                width=2,
            )
            legend_y = top - 9
            canvas.create_line(right - 118, legend_y, right - 98, legend_y, fill=line_color, width=2)
            canvas.create_text(
                right - 94,
                legend_y,
                text="linear fit",
                anchor="w",
                fill=COLOR_MUTED,
                font=("Segoe UI", 7),
            )

        point_color = COLOR_ACCENT
        for idx, (pressure_delta, force_n) in enumerate(samples, start=1):
            x = x_for(pressure_delta)
            y = y_for(force_n)
            canvas.create_oval(x - 4, y - 4, x + 4, y + 4, fill=point_color, outline="#ffffff", width=1)
            if len(samples) <= 12:
                canvas.create_text(
                    x + 6,
                    y - 6,
                    text=str(idx),
                    anchor="sw",
                    fill=COLOR_TEXT,
                    font=("Segoe UI", 7, "bold"),
                )

    def add_force_calibration_sample(self):
        if self._is_thread_running(self.stepper_thread):
            messagebox.showwarning(
                "Calibration Sample",
                "Wait for the stepper to stop and the load-cell reading to settle before capturing.",
            )
            self.force_calibration_status_var.set("Sample blocked: stepper is moving.")
            return
        with self.sensor_data_lock:
            pressure = self.latest_pressure_hpa
            zero = self.force_pressure_zero_hpa
            weight_kg = self.latest_loadcell_kg
            raw_weight_kg = self.latest_loadcell_raw_kg
            loadcell_noise_kg = self.loadcell_noise_kg
            filter_count = self.loadcell_filter_count

        if pressure is None or weight_kg is None:
            messagebox.showwarning(
                "Calibration Sample",
                "Start both pressure and load-cell readings before capturing a sample.",
            )
            return
        if filter_count < max(3, min(LOADCELL_FILTER_WINDOW, 3)):
            messagebox.showwarning(
                "Calibration Sample",
                "Wait for a few load-cell samples before capturing.",
            )
            self.force_calibration_status_var.set("Sample blocked: load cell not settled yet.")
            return
        if loadcell_noise_kg > LOADCELL_STABLE_NOISE_KG:
            messagebox.showwarning(
                "Calibration Sample",
                (
                    "Load-cell reading is still noisy. Hold the rig still and capture again "
                    f"when noise is below {LOADCELL_STABLE_NOISE_KG:.3f} kg."
                ),
            )
            self.force_calibration_status_var.set(
                f"Sample blocked: load noise {loadcell_noise_kg:.3f} kg."
            )
            return
        raw_weight_kg = weight_kg if raw_weight_kg is None else raw_weight_kg
        pressure_delta = self._pressure_delta_from_zero(pressure, zero)
        if pressure_delta is None:
            messagebox.showwarning(
                "Calibration Sample",
                "Set pressure zero before capturing force samples.",
            )
            return
        if abs(float(pressure_delta)) < FORCE_PRESSURE_DELTA_MIN_HPA:
            messagebox.showwarning(
                "Calibration Sample",
                "Press the bubble into the load cell before capturing a sample.",
            )
            self.force_calibration_status_var.set("Need bubble pressure change before capture.")
            return

        force_n = weight_kg * FORCE_GRAVITY_MPS2
        if force_n < FORCE_CALIBRATION_MIN_SAMPLE_FORCE_N:
            messagebox.showwarning(
                "Calibration Sample",
                "Load force is negative. Press Zero Load with no contact, then capture again.",
            )
            self.force_calibration_status_var.set(
                f"Sample blocked: negative load force {force_n:.2f} N."
            )
            return
        with self.sensor_data_lock:
            self.force_calibration_samples.append((float(pressure_delta), float(force_n)))
            count = len(self.force_calibration_samples)

        self.force_calibration_sample_count_var.set(f"{count} sample{'s' if count != 1 else ''}")
        self.force_calibration_status_var.set(
            "Bubble sample captured. Capture another pressure level."
            if count < 2
            else "Bubble sample captured."
        )
        self.log(
            f"Force calibration sample {count}: pressure={pressure:.2f} hPa, "
            f"dP={pressure_delta:+.2f} hPa, force={force_n:.3f} N, "
            f"raw={raw_weight_kg:.4f} kg, noise={loadcell_noise_kg:.3f} kg."
        )

        if count >= 2:
            self._fit_force_calibration(show_warning=False)
        else:
            self._refresh_force_linear_equation_graph()

    def fit_force_calibration(self):
        self._fit_force_calibration(show_warning=True)

    def _fit_force_calibration(self, show_warning=True):
        with self.sensor_data_lock:
            samples = list(self.force_calibration_samples)

        if len(samples) < 2:
            if show_warning:
                messagebox.showwarning("Force Calibration", "Capture at least 2 bubble samples before fitting.")
            self.force_calibration_status_var.set("Need at least 2 bubble samples.")
            return False

        n = float(len(samples))
        sum_x = sum(sample[0] for sample in samples)
        sum_y = sum(sample[1] for sample in samples)
        sum_xx = sum(sample[0] * sample[0] for sample in samples)
        sum_xy = sum(sample[0] * sample[1] for sample in samples)
        denominator = (n * sum_xx) - (sum_x * sum_x)
        if abs(denominator) < 1e-9:
            if show_warning:
                messagebox.showwarning(
                    "Force Calibration",
                    "Pressure values are too similar. Capture samples at different loads.",
                )
            self.force_calibration_status_var.set("Need distinct pressure points.")
            return False

        slope = ((n * sum_xy) - (sum_x * sum_y)) / denominator
        intercept = (sum_y - (slope * sum_x)) / n
        if abs(intercept) > FORCE_CALIBRATION_MAX_ZERO_FORCE_N:
            if show_warning:
                messagebox.showwarning(
                    "Force Calibration",
                    (
                        "Calibration zero offset is too large. Press Zero Load, "
                        "Reset Samples, then capture stable samples again."
                    ),
                )
            self.force_calibration_status_var.set(
                f"Calibration rejected: zero offset {intercept:.2f} N."
            )
            self.log(f"Force calibration rejected: intercept {intercept:.3f} N.")
            return False
        fit_score = self._force_linear_fit_score(samples, slope, intercept)
        if len(samples) >= 4 and fit_score is not None and fit_score < FORCE_CALIBRATION_MIN_R2:
            if show_warning:
                messagebox.showwarning(
                    "Force Calibration",
                    (
                        "Force samples are too scattered for a reliable linear fit. "
                        "Reset samples and capture only after the load cell settles."
                    ),
                )
            self.force_calibration_status_var.set(
                f"Calibration rejected: unstable fit R^2 {fit_score:.3f}."
            )
            self.log(f"Force calibration rejected: R^2 {fit_score:.3f}.")
            return False
        with self.sensor_data_lock:
            self.force_calibration_coeffs = (slope, intercept)

        count = len(samples)
        self.force_calibration_model_var.set(self._format_force_model(slope, intercept))
        self.force_calibration_status_var.set(f"Bubble calibrated with {count} samples.")
        self.log(f"Force calibration fit: F(N) = {slope:.6f} * dP(hPa) + {intercept:.3f}.")
        self._save_force_calibration()
        self._update_force_estimate()
        self._request_force_graph_redraw()
        self._refresh_force_linear_equation_graph()
        return True

    def reset_force_bubble_samples(self):
        with self.sensor_data_lock:
            self.force_calibration_samples.clear()
            self.force_calibration_coeffs = None
            self.latest_pressure_force_n = None
            self.force_pressure_zero_pending = False
            self.force_pressure_zero_samples = []
            if hasattr(self, "force_graph_history"):
                self.force_graph_history.clear()
            zero = self.force_pressure_zero_hpa

        self.force_calibration_sample_count_var.set("0 samples")
        self.force_calibration_model_var.set("F = a*dP + b")
        self.force_estimate_var.set("-")
        self._refresh_force_pressure_zero()
        self._refresh_force_live_pair()
        if zero is None:
            self.force_calibration_status_var.set("Bubble samples reset. Waiting for pressure zero.")
        else:
            self.force_calibration_status_var.set("Bubble samples reset. Pressure zero kept.")
        try:
            self._force_calibration_path().unlink(missing_ok=True)
        except Exception as exc:
            self.log(f"Could not remove saved force calibration: {exc}")
        self.log("Bubble force calibration samples reset.")
        self._request_force_graph_redraw()
        self._refresh_force_linear_equation_graph()

    def clear_force_calibration(self):
        with self.sensor_data_lock:
            self.force_calibration_samples.clear()
            self.force_calibration_coeffs = None
            self.latest_pressure_force_n = None
            self.force_pressure_zero_hpa = FORCE_DEFAULT_PRESSURE_ZERO_HPA
            self.force_pressure_zero_pending = False
            self.force_pressure_zero_samples = []
            if hasattr(self, "force_graph_history"):
                self.force_graph_history.clear()
        self.force_calibration_sample_count_var.set("0 samples")
        self.force_calibration_model_var.set("F = a*dP + b")
        self.force_estimate_var.set("-")
        self._refresh_force_pressure_zero()
        self._refresh_force_live_pair()
        self.force_calibration_status_var.set(
            f"Calibration cleared. Pressure zero reset to {FORCE_DEFAULT_PRESSURE_ZERO_HPA:.2f} hPa."
        )
        try:
            self._force_calibration_path().unlink(missing_ok=True)
        except Exception as exc:
            self.log(f"Could not remove saved force calibration: {exc}")
        self.log("Force calibration cleared.")
        self._request_force_graph_redraw()
        self._refresh_force_linear_equation_graph()

    def _load_hx711_class(self):
        base_path = Path(__file__).resolve().parent
        for hx_path in reversed((base_path / "library" / "hx711py", base_path)):
            if str(hx_path) not in sys.path:
                sys.path.insert(0, str(hx_path))
        from hx711v0_5_1 import HX711

        return HX711

    def _loadcell_calibration_path(self):
        return Path(__file__).resolve().parent / LOADCELL_CALIBRATION_FILENAME

    def _load_loadcell_calibration(self):
        path = self._loadcell_calibration_path()
        if not path.exists():
            return None
        try:
            with path.open("r", encoding="utf-8") as handle:
                payload = json.load(handle)
            return float(payload["counts_per_gram"])
        except Exception as exc:
            self.log(f"Could not load load-cell calibration: {exc}")
            return None

    def _save_loadcell_calibration(self, counts_per_gram, known_weight_grams):
        payload = {
            "counts_per_gram": float(counts_per_gram),
            "known_weight_grams": float(known_weight_grams),
            "dt_pin": int(LOADCELL_DT_PIN),
            "sck_pin": int(LOADCELL_SCK_PIN),
            "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        }
        try:
            with self._loadcell_calibration_path().open("w", encoding="utf-8") as handle:
                json.dump(payload, handle, indent=2)
        except Exception as exc:
            self.log(f"Could not save load-cell calibration: {exc}")

    def reset_loadcell_calibration(self):
        if self.loadcell_thread is not None and self.loadcell_thread.is_alive():
            messagebox.showwarning(
                "Load Cell Calibration",
                "Stop load-cell reading before resetting the saved calibration.",
            )
            return

        try:
            self._loadcell_calibration_path().unlink(missing_ok=True)
        except Exception as exc:
            self.log(f"Could not remove saved load-cell calibration: {exc}")
            messagebox.showerror("Load Cell Calibration", str(exc))
            return

        self._reset_loadcell_filter(reset_live_zero=True)
        self._set_var(self.loadcell_weight_var, "-")
        self._refresh_force_live_pair()
        self._request_force_graph_redraw()
        self.log("Saved load-cell calibration reset.")
        messagebox.showinfo(
            "Load Cell Calibration",
            "Saved load-cell calibration was reset. Press Cal Load to save a new one.",
        )

    @staticmethod
    def _read_average_raw(hx, samples=12, delay=0.025, channel="A", timeout=1.5):
        readings = []
        attempts = max(samples, samples * 3)
        for _ in range(attempts):
            try:
                value = hx.getLong(channel, timeout=timeout)
            except TimeoutError:
                value = None
            if value is not None:
                readings.append(float(value))
                if len(readings) >= samples:
                    break
            time.sleep(delay)

        if not readings:
            raise RuntimeError("No valid load cell readings were captured.")

        return float(np.median(np.array(readings, dtype=np.float64)))

    def _read_weight_grams(self, hx, samples=5):
        raw = self._read_average_raw(hx, samples=samples, delay=0.015, channel="A", timeout=1.2)
        return (raw - hx.offset["A"]) / hx.reference_unit["A"]

    def start_loadcell_read(self):
        if self.loadcell_thread is not None and self.loadcell_thread.is_alive():
            self.log("Load cell read is already running.")
            return

        self._start_loadcell_worker(
            known_weight_grams=None,
            force_calibrate=False,
            state_text="Running",
            log_message="Load cell read started with saved calibration.",
        )

    def calibrate_loadcell(self):
        if self.loadcell_thread is not None and self.loadcell_thread.is_alive():
            self.log("Load cell read is already running.")
            return

        known_weight_grams = self._parse_loadcell_known_weight()
        if known_weight_grams is None:
            return

        self._start_loadcell_worker(
            known_weight_grams=known_weight_grams,
            force_calibrate=True,
            state_text="Calibrating",
            log_message="Load cell calibration started.",
        )

    def _parse_loadcell_known_weight(self):
        try:
            known_weight_grams = float(self.loadcell_known_weight_var.get().strip())
        except ValueError:
            messagebox.showerror("Invalid Weight", "Known weight must be a number.")
            return None

        if known_weight_grams <= 0:
            messagebox.showerror("Invalid Weight", "Known weight must be > 0 grams.")
            return None
        return known_weight_grams

    def _start_loadcell_worker(self, known_weight_grams, force_calibrate, state_text, log_message):
        self.loadcell_stop_event.clear()
        self.loadcell_thread = threading.Thread(
            target=self._loadcell_worker,
            args=(known_weight_grams, force_calibrate),
            daemon=True,
        )
        self.loadcell_thread.start()
        self.loadcell_state_var.set(state_text)
        self._set_loadcell_controls(True)
        self._refresh_system_status()
        self.log(log_message)

    def _loadcell_worker(self, known_weight_grams, force_calibrate=False):
        hx = None
        try:
            HX711 = self._load_hx711_class()
            hx = HX711(LOADCELL_DT_PIN, LOADCELL_SCK_PIN)
            self.loadcell_hx = hx
            self.log(f"Load cell HX711 GPIO backend: {getattr(hx, 'backend_name', 'unknown')}.")

            hx.setReadingFormat("MSB", "MSB")
            hx.reset()

            self.log("Load cell: remove all weight for tare.")
            if not self._wait_with_cancel(2.0, self.loadcell_stop_event):
                return

            offset = self._read_average_raw(hx, channel="A")
            hx.setOffset(offset, "A")
            self.log(f"Load cell tare offset: {offset:.2f}")

            saved_counts_per_gram = self._load_loadcell_calibration()
            if force_calibrate:
                self.log(
                    f"Load cell: place {known_weight_grams:.2f} g now; "
                    f"calibrating in {LOADCELL_CALIBRATION_DELAY_S:.0f}s."
                )
                if not self._wait_with_cancel(LOADCELL_CALIBRATION_DELAY_S, self.loadcell_stop_event):
                    return
                loaded_raw = self._read_average_raw(hx, channel="A")
                delta = loaded_raw - offset
                if abs(delta) < 50.0:
                    raise RuntimeError(f"raw delta is too small ({delta:.2f} counts)")
                counts_per_gram = delta / known_weight_grams
                self._save_loadcell_calibration(counts_per_gram, known_weight_grams)
                self.log(f"Load cell calibrated. counts_per_gram={counts_per_gram:.6f}")
            else:
                if saved_counts_per_gram is None:
                    raise RuntimeError(
                        "No saved load-cell calibration. Enter a known weight and press Calibrate once."
                    )
                counts_per_gram = saved_counts_per_gram
                self.log(
                    "Load cell using saved calibration "
                    f"counts_per_gram={counts_per_gram:.6f}."
                )
            hx.setReferenceUnit(counts_per_gram, "A")
            self._reset_loadcell_filter(reset_live_zero=True)

            read_failures = 0
            while not self.loadcell_stop_event.is_set():
                try:
                    weight_grams = self._read_weight_grams(hx, samples=LOADCELL_READ_SAMPLES)
                    read_failures = 0
                    weight_kg = float(weight_grams) / 1000.0
                    filtered_kg = self._record_loadcell_reading(weight_kg)
                    with self.sensor_data_lock:
                        noise_kg = self.loadcell_noise_kg
                    self._set_var(
                        self.loadcell_weight_var,
                        f"{filtered_kg:.4f} kg (noise {noise_kg:.3f})",
                    )
                except Exception as read_exc:
                    read_failures += 1
                    if read_failures == 1 or read_failures % 5 == 0:
                        self.log(f"Load cell read retry {read_failures}: {read_exc}")
                    if read_failures >= 12:
                        raise
                    self._set_var(self.loadcell_weight_var, "Retrying...")
                time.sleep(0.2)
        except Exception as exc:
            self.log(f"Load cell error: {exc}")
            self._set_var(self.loadcell_weight_var, "Error")
        finally:
            if hx is not None and getattr(hx, "readyCallbackEnabled", False):
                try:
                    hx.disableReadyCallback()
                except RuntimeError:
                    pass
            if hx is not None and hasattr(hx, "close"):
                try:
                    hx.close()
                except Exception:
                    pass

            self.loadcell_hx = None
            self._set_var(self.loadcell_state_var, "Stopped")
            self._set_loadcell_controls(False)
            self._refresh_system_status()

    def stop_loadcell_read(self):
        self.loadcell_stop_event.set()
        if self.loadcell_thread is not None and self.loadcell_thread.is_alive():
            self.loadcell_thread.join(timeout=1.0)
        self.loadcell_thread = None
        self.loadcell_state_var.set("Stopped")
        self._set_loadcell_controls(False)
        self._refresh_system_status()
        self.log("Load cell stop requested.")

    @staticmethod
    def _remote_points_from_payload(payload, key):
        points = []
        for point in payload.get(key, []) or []:
            if point is None or len(point) < 2:
                continue
            points.append((int(point[0]), int(point[1])))
        return points

    @staticmethod
    def _remote_displacements_from_payload(payload):
        displacements = []
        for disp in payload.get("displacements", []) or []:
            if disp is None or len(disp) < 2:
                displacements.append(None)
            else:
                displacements.append((float(disp[0]), float(disp[1])))
        return displacements

    def _post_remote_vision_frame(
        self,
        cv2,
        frame_bgr,
        params,
        reset_reference=False,
        surface_reset=False,
    ):
        view_type = str(params.get("view_type", "overlay")).strip().lower()
        remote_preview = REMOTE_VISION_PREVIEW_ENABLED or view_type in {
            "surface_3d",
            "optical_flow_3d",
            "pointcloud",
            "heatmap",
        }
        include_points = bool(REMOTE_VISION_POINTS_ENABLED or not remote_preview)
        query_params = {
            "reset": "1" if reset_reference else "0",
            "preview": "1" if remote_preview else "0",
            "points": "1" if include_points else "0",
            "view": view_type,
            "surface_reset": "1" if surface_reset else "0",
            "mode": params.get("mode", "auto"),
            "use_roi": "1" if params.get("use_roi", True) else "0",
            "roi_scale": f"{float(params.get('roi_scale', 0.68)):.4f}",
            "min_area": int(float(params.get("min_area", 20))),
            "max_area": int(float(params.get("max_area", 8000))),
            "min_circularity": f"{float(params.get('min_circularity', 0.35)):.4f}",
            "match_dist": f"{float(params.get('match_dist', 9.0)):.4f}",
            "surface_grid": self._parse_surface_int(self.surface_grid_var, 52, 24, 140),
            "surface_gain": f"{self._parse_surface_float(self.surface_gain_var, 1.0, 0.2, 3.0):.4f}",
            "surface_smooth": f"{self._parse_surface_float(self.surface_smooth_var, 1.4, 0.0, 4.0):.4f}",
            "preview_max_width": max(0, int(REMOTE_VISION_PREVIEW_MAX_WIDTH)),
            "surface_proc_width": max(0, int(REMOTE_VISION_SURFACE_PROCESS_WIDTH)),
        }
        transport = REMOTE_VISION_TRANSPORT if REMOTE_VISION_TRANSPORT in {"raw", "jpeg"} else "raw"
        if transport == "raw":
            raw_format = REMOTE_VISION_RAW_FORMAT
            height, width = frame_bgr.shape[:2]
            query_params.update(
                {
                    "width": int(width),
                    "height": int(height),
                    "format": raw_format,
                }
            )
            if raw_format in {"bgr24", "bgr", "bgr888"}:
                body = np.ascontiguousarray(frame_bgr).tobytes()
            elif raw_format in {"rgb24", "rgb", "rgb888"}:
                rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
                body = np.ascontiguousarray(rgb).tobytes()
            elif raw_format in {"gray8", "grey8", "mono8"}:
                gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
                body = np.ascontiguousarray(gray).tobytes()
            elif raw_format in {"i420", "yuv420", "yuv420p"}:
                yuv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2YUV_I420)
                body = np.ascontiguousarray(yuv).tobytes()
            else:
                raise RuntimeError(f"Unsupported remote raw format: {raw_format}")
            endpoint = "process_raw"
            content_type = "application/octet-stream"
        else:
            ok, encoded = cv2.imencode(
                ".jpg",
                frame_bgr,
                [int(cv2.IMWRITE_JPEG_QUALITY), int(np.clip(REMOTE_VISION_JPEG_QUALITY, 35, 92))],
            )
            if not ok:
                raise RuntimeError("Could not encode camera frame for remote vision.")
            body = encoded.tobytes()
            endpoint = "process"
            content_type = "image/jpeg"

        query = urlencode(query_params)
        url = f"{params['remote_vision_url']}/{endpoint}?{query}"
        request = Request(
            url,
            data=body,
            headers={"Content-Type": content_type},
            method="POST",
        )
        started_at = time.perf_counter()
        with urlopen(request, timeout=REMOTE_VISION_TIMEOUT_SECONDS) as response:
            payload = json.loads(response.read().decode("utf-8"))
        roundtrip_ms = (time.perf_counter() - started_at) * 1000.0
        if not payload.get("ok"):
            raise RuntimeError(str(payload.get("error", "remote vision failed")))

        preview_bgr = None
        preview_text = payload.get("preview_jpeg_b64")
        if preview_text:
            preview_bytes = base64.b64decode(preview_text)
            preview_array = np.frombuffer(preview_bytes, dtype=np.uint8)
            preview_bgr = cv2.imdecode(preview_array, cv2.IMREAD_COLOR)

        payload["roundtrip_ms"] = roundtrip_ms
        return payload, preview_bgr

    def start_blob_test(self):
        if self._is_blob_running():
            self.log("Blob detector is already running.")
            return

        if self._is_flow_running():
            self.log("Optical flow is running. Stopping optical flow before detection starts.")
            self.stop_optical_flow()
            if self._is_flow_running():
                self.log("Cannot start detection: optical flow is still shutting down.")
                messagebox.showwarning(
                    "Camera Busy",
                    "Optical flow is still releasing the camera. Please wait a moment and try again.",
                )
                return
            time.sleep(0.2)

        if self.camera_running:
            self.log("Camera preview is running. Stopping preview before Blob test starts.")
            self.stop_camera()

        try:
            self._load_blob_backend()
            self.blob_params = self._collect_blob_params()
        except Exception as exc:
            self.blob_state_var.set("Error")
            self.blob_message_var.set(str(exc))
            self.log(f"Blob setup error: {exc}")
            messagebox.showerror("Blob Test Error", str(exc))
            self._set_blob_controls(False)
            self._refresh_system_status()
            return

        self.blob_stop_event.clear()
        self.blob_snapshot_event.clear()
        self._clear_blob_frame_queue()
        self.blob_dot_count_var.set("-")
        self.blob_pick_mode_var.set("-")
        self.blob_fps_var.set("-")
        self.blob_latency_var.set("-")
        self.blob_mean_disp_var.set("-")
        self.blob_max_disp_var.set("-")
        self.blob_missing_ratio_var.set("-")
        self.blob_new_tracks_var.set("-")
        self.blob_lost_tracks_var.set("-")
        self.blob_photo = None
        self.blob_preview_label.configure(image="", text="Starting preview...")
        self.blob_reset_reference_event.clear()
        self._reset_contact_deform_zero_state("Contact deform zero pending. Press Reset Zero.")
        self._request_pressure_zero_from_camera_open()

        self.blob_thread = threading.Thread(target=self._blob_worker, daemon=True)
        self.blob_thread.start()

        self.blob_state_var.set("Starting")
        self.blob_message_var.set("Opening camera...")
        self._set_blob_controls(True)
        self._refresh_system_status()
        self.log("Blob test start requested.")

    def _blob_worker(self):
        stream = None
        error_message = None
        frame_counter = 0
        fps = 0.0
        fps_start = time.perf_counter()
        reference_centroids = None
        tracks = {}
        next_id = 0
        robust_countdown = 0
        read_fail_count = 0
        max_read_fail_retries = 40
        flow_prev_gray = None
        flow_prev_pts = None
        flow_reseed_counter = 0
        surface_refresh_counter = 0
        surface_display_cache = None
        surface_distance_text_cache = ""
        surface_mean_cache = None
        surface_max_cache = None
        surface_graph_history = []
        contact_zero_pending = False
        contact_zero_ready = False
        contact_zero_samples = []

        try:
            pipe = self.blob_module
            cv2 = self.blob_cv2
            params = dict(self.blob_params)
            remote_backend = params.get("vision_backend") == "remote_cuda"
            remote_reset_reference_pending = remote_backend
            remote_fail_count = 0

            pipe.MIN_BLOB_AREA = int(params["min_area"])
            pipe.MAX_BLOB_AREA = int(params["max_area"])
            pipe.MIN_CIRCULARITY = float(params["min_circularity"])
            if hasattr(pipe, "get_detection_backend_name"):
                self.log(f"Blob detector backend: {pipe.get_detection_backend_name()}")

            stream = pipe.open_camera_stream(
                camera_index=params["camera_index"],
                width=int(params["cam_width"]),
                height=int(params["cam_height"]),
                backend=params["camera_backend"],
                autofocus=params["autofocus"],
                lens_position=params["lens_position"],
                exposure_ev=params["exposure_ev"],
                exposure_time_us=params.get("exposure_time_us"),
                analogue_gain=params.get("analogue_gain"),
                awb_mode=params.get("awb_mode", "auto"),
                colour_gains=params.get("colour_gains"),
                frame_rate=REMOTE_VISION_TARGET_FPS if remote_backend else None,
            )

            self.blob_camera = stream
            active_controls = stream.get("controls") if isinstance(stream, dict) else None
            if active_controls:
                compact_controls = ", ".join(
                    f"{name}={value}" for name, value in sorted(active_controls.items())
                )
                self.log(f"Camera controls: {compact_controls}")
            self._set_var(self.blob_state_var, "Running")
            self._set_var(self.blob_message_var, "Blob detector is running.")
            self._refresh_system_status()
            contact_zero_pending = False
            contact_zero_ready = False
            contact_zero_samples = []

            while not self.blob_stop_event.is_set():
                loop_start = time.perf_counter()

                ok, frame_bgr = pipe.read_camera_frame(stream)
                if not ok or frame_bgr is None:
                    read_fail_count += 1
                    if read_fail_count == 1:
                        self._set_var(self.blob_message_var, "Waiting for camera frames...")
                    if read_fail_count in {1, 10, 20, 30}:
                        self.log(
                            "Camera frame retry "
                            f"{read_fail_count}/{max_read_fail_retries} "
                            f"(backend={stream.get('backend', 'unknown')})."
                        )
                    if read_fail_count >= max_read_fail_retries:
                        raise RuntimeError(
                            "Camera read failed in dot pipeline worker "
                            f"after {max_read_fail_retries} retries."
                        )
                    time.sleep(0.05)
                    continue

                if read_fail_count > 0:
                    self.log(f"Camera stream recovered after {read_fail_count} retries.")
                    self._set_var(self.blob_message_var, "Blob detector is running.")
                    read_fail_count = 0

                if params["proc_scale"] != 1.0:
                    frame_bgr = cv2.resize(
                        frame_bgr,
                        (
                            int(frame_bgr.shape[1] * params["proc_scale"]),
                            int(frame_bgr.shape[0] * params["proc_scale"]),
                        ),
                        interpolation=cv2.INTER_AREA,
                    )

                frame_for_detection = frame_bgr
                if params.get("use_illum_norm", False) and not remote_backend:
                    frame_for_detection = self._apply_illumination_normalization(
                        frame_bgr,
                        cv2,
                        params.get("illum_method", "clahe_bg"),
                    )

                if remote_backend:
                    reset_remote = remote_reset_reference_pending
                    if self.blob_reset_reference_event.is_set():
                        self.blob_reset_reference_event.clear()
                        reset_remote = True
                        self.log("Remote CUDA reference reset requested.")
                    surface_reset_requested = self.surface_reset_zero_event.is_set()
                    try:
                        remote_payload, remote_preview_bgr = self._post_remote_vision_frame(
                            cv2,
                            frame_for_detection,
                            params,
                            reset_reference=reset_remote,
                            surface_reset=surface_reset_requested,
                        )
                        remote_fail_count = 0
                        remote_reset_reference_pending = False
                    except Exception as remote_exc:
                        remote_fail_count += 1
                        if remote_fail_count == 1 or remote_fail_count % 5 == 0:
                            self.log(f"Remote CUDA vision retry {remote_fail_count}: {remote_exc}")
                        self._set_var(
                            self.blob_message_var,
                            f"Remote CUDA retry {remote_fail_count}: {remote_exc}",
                        )
                        if remote_fail_count >= 12:
                            raise RuntimeError(f"Remote CUDA vision failed: {remote_exc}") from remote_exc
                        time.sleep(0.04)
                        continue

                    if surface_reset_requested and (
                        remote_payload.get("surface_zero_accepted")
                        or remote_payload.get("surface_zero_pending")
                        or remote_payload.get("surface_zero_ready")
                    ):
                        self.surface_reset_zero_event.clear()

                    centroids = self._remote_points_from_payload(remote_payload, "centroids")
                    remote_dot_count = int(remote_payload.get("dot_count", len(centroids)) or 0)
                    reference_centroids = self._remote_points_from_payload(
                        remote_payload,
                        "reference_centroids",
                    )
                    displacements = self._remote_displacements_from_payload(remote_payload)
                    mean_disp = float(remote_payload.get("mean_disp", 0.0) or 0.0)
                    max_disp = float(remote_payload.get("max_disp", 0.0) or 0.0)
                    missing_ratio = float(remote_payload.get("missing_ratio", 0.0) or 0.0)
                    new_count = int(remote_payload.get("new_tracks", 0) or 0)
                    lost_count = int(remote_payload.get("lost_tracks", 0) or 0)
                    if remote_payload.get("reference_reset"):
                        self.surface_scale_ema = None
                        self.surface_height_ema = None
                        self.surface_reference_height_map = None
                        self.surface_contact_ema = None
                        self.surface_contact_baseline = None
                        self.surface_contact_display_ema = None
                        self.surface_zero_ready = False
                        contact_zero_pending = False
                        contact_zero_ready = False
                        contact_zero_samples.clear()
                        surface_display_cache = None
                        surface_distance_text_cache = ""
                        surface_mean_cache = None
                        surface_max_cache = None
                        surface_graph_history.clear()
                        self._set_var(self.surface_status_var, "Remote reference reset. Press Reset Zero.")

                    if self.surface_reset_zero_event.is_set():
                        if centroids:
                            self.surface_reset_zero_event.clear()
                            self.surface_reference_height_map = None
                            self.surface_contact_ema = None
                            self.surface_contact_baseline = None
                            self.surface_contact_display_ema = None
                            self.surface_zero_ready = False
                            contact_zero_pending = True
                            contact_zero_ready = False
                            contact_zero_samples.clear()
                            surface_display_cache = None
                            surface_distance_text_cache = ""
                            surface_mean_cache = None
                            surface_max_cache = None
                            surface_graph_history.clear()
                            self._set_var(self.surface_status_var, "Capturing remote contact deform zero...")
                            self.log("Remote CUDA contact deform zero capture requested.")
                        else:
                            self._set_var(self.surface_status_var, "Waiting for remote dots to reset contact zero.")

                    contact_zero_active = bool(centroids) and contact_zero_pending
                    view_type = params.get("view_type", "auto")
                    display_mode = view_type
                    if view_type == "auto":
                        if params["use_mosaic"]:
                            display_mode = "mosaic"
                        elif params["use_pointcloud"]:
                            display_mode = "pointcloud"
                        else:
                            display_mode = "overlay"

                    distance_text = (
                        f"Remote CUDA: mean {mean_disp:.3f}px | "
                        f"max {max_disp:.3f}px | dots {remote_dot_count}"
                    )
                    display_bgr = None
                    flow_distance_mean = mean_disp
                    flow_distance_max = max_disp
                    deform_contact_stats = None
                    deform_tactile_frame = None
                    deform_surface_scale = None
                    deform_gate = None
                    remote_surface_rendered = (
                        display_mode in {"surface_3d", "optical_flow_3d"}
                        and remote_preview_bgr is not None
                        and bool(remote_payload.get("surface_rendered"))
                    )

                    if remote_surface_rendered:
                        display_bgr = remote_preview_bgr
                        center_payload = remote_payload.get("contact_center")
                        center = (
                            None
                            if center_payload is None or len(center_payload) < 2
                            else (float(center_payload[0]), float(center_payload[1]))
                        )
                        deform_contact_stats = {
                            "ready": bool(remote_payload.get("contact_ready")),
                            "peak": float(remote_payload.get("contact_peak", 0.0) or 0.0),
                            "top_mean": float(remote_payload.get("contact_top_mean", 0.0) or 0.0),
                            "area_ratio": float(remote_payload.get("contact_area_ratio", 0.0) or 0.0),
                            "center": center,
                        }
                        deform_surface_scale = float(remote_payload.get("surface_scale_px", 0.0) or 0.0)
                        status_text = str(
                            remote_payload.get("surface_status")
                            or "Remote surface running."
                        )
                        self._set_var(self.surface_status_var, status_text)
                        if deform_contact_stats["ready"]:
                            camera_depth_mm = self._estimate_camera_depth_mm(deform_contact_stats)
                            self._refresh_contact_depth_readouts(
                                camera_depth_mm,
                                update_camera=True,
                            )
                            flow_distance_mean = deform_contact_stats["top_mean"]
                            flow_distance_max = deform_contact_stats["peak"]
                            distance_text = (
                                f"Remote CUDA surface: contact "
                                f"{deform_contact_stats['peak'] * 100.0:.0f}% | "
                                f"area {deform_contact_stats['area_ratio']:.1f}%"
                            )
                        else:
                            self._refresh_contact_depth_readouts(None, update_camera=True)
                            distance_text = status_text
                    elif display_mode in {"surface_3d", "optical_flow_3d"}:
                        flow_gray = cv2.cvtColor(frame_for_detection, cv2.COLOR_BGR2GRAY)
                        height_map, ellipse_mask, scale = self._build_surface_height_map(
                            centroids,
                            reference_centroids,
                            displacements,
                            frame_for_detection.shape,
                            params,
                            cv2,
                        )
                        if height_map is not None:
                            tactile_frame = self.surface_tactile_frame
                            tactile_config = self._build_tactile_contact_config(params)
                            raw_contact_map = (
                                tactile_frame.contact_map
                                if tactile_frame is not None
                                else None
                            )
                            base_map = tactile_frame.base_map if tactile_frame is not None else None
                            contact_stats = None
                            gate = self._pressure_contact_gate_snapshot()
                            gate_blocks_contact = False
                            if raw_contact_map is not None and base_map is not None:
                                gate_blocks_contact = bool(gate["enabled"]) and not bool(gate["contact"])
                                if gate_blocks_contact:
                                    contact_zero_pending = False
                                    contact_zero_ready = False
                                    contact_zero_active = False
                                    contact_zero_samples.clear()
                                    self.surface_contact_baseline = None
                                    self.surface_contact_display_ema = None
                                    self.surface_zero_ready = False
                                    contact_map_for_display = np.zeros_like(raw_contact_map, dtype=np.float32)
                                else:
                                    if gate["enabled"] and not contact_zero_ready and not contact_zero_pending:
                                        contact_zero_pending = True
                                        contact_zero_samples.clear()
                                        self.surface_contact_baseline = None
                                        self.surface_contact_display_ema = None
                                        self.surface_zero_ready = False
                                    contact_zero_active = bool(centroids) and contact_zero_pending

                                if not gate_blocks_contact and contact_zero_active:
                                    contact_zero_samples.append(raw_contact_map.astype(np.float32).copy())
                                    if len(contact_zero_samples) >= CONTACT_ZERO_SAMPLE_COUNT:
                                        self.surface_contact_baseline = np.median(
                                            np.stack(
                                                contact_zero_samples[-CONTACT_ZERO_SAMPLE_COUNT:],
                                                axis=0,
                                            ),
                                            axis=0,
                                        ).astype(np.float32)
                                        self.surface_contact_display_ema = np.zeros_like(
                                            raw_contact_map,
                                            dtype=np.float32,
                                        )
                                        contact_zero_samples.clear()
                                        contact_zero_pending = False
                                        contact_zero_ready = True
                                        self.surface_zero_ready = True
                                        self.log("Remote CUDA contact zero captured from averaged contact maps.")
                                    contact_map_for_display = np.zeros_like(raw_contact_map, dtype=np.float32)
                                elif not gate_blocks_contact and (
                                    contact_zero_ready
                                    and self.surface_contact_baseline is not None
                                    and self.surface_contact_baseline.shape == raw_contact_map.shape
                                ):
                                    contact_map_for_display = np.maximum(
                                        raw_contact_map - self.surface_contact_baseline,
                                        0.0,
                                    ).astype(np.float32)
                                    if (
                                        self.surface_contact_display_ema is None
                                        or self.surface_contact_display_ema.shape != contact_map_for_display.shape
                                    ):
                                        self.surface_contact_display_ema = contact_map_for_display
                                    else:
                                        self.surface_contact_display_ema = cv2.addWeighted(
                                            self.surface_contact_display_ema.astype(np.float32),
                                            0.68,
                                            contact_map_for_display,
                                            0.32,
                                            0.0,
                                        )
                                    contact_map_for_display = self.surface_contact_display_ema
                                else:
                                    contact_map_for_display = np.zeros_like(raw_contact_map, dtype=np.float32)

                                height_map = surface_from_contact_map(
                                    contact_map_for_display,
                                    base_map,
                                    ellipse_mask,
                                    tactile_config,
                                    cv2=cv2,
                                )
                                contact_stats = summarize_contact_map(
                                    contact_map_for_display,
                                    ellipse_mask,
                                    tactile_config.contact_area_threshold,
                                )

                            deform_contact_stats = contact_stats
                            deform_tactile_frame = tactile_frame
                            deform_surface_scale = scale
                            deform_gate = gate
                            if contact_stats is not None and contact_stats["ready"]:
                                surface_graph_history.append(
                                    (
                                        float(contact_stats["peak"]),
                                        float(contact_stats["top_mean"]),
                                    )
                                )
                            else:
                                surface_graph_history.append((None, None))
                            if len(surface_graph_history) > 180:
                                del surface_graph_history[:-180]

                            self._set_var(
                                self.surface_status_var,
                                (
                                    (
                                        (
                                            f"Remote contact zeroing {len(contact_zero_samples)}/"
                                            f"{CONTACT_ZERO_SAMPLE_COUNT}..."
                                        )
                                        if contact_zero_pending
                                        else "Remote contact zero set."
                                    )
                                    if contact_zero_active
                                    else (
                                        (
                                            "Remote surface running."
                                            if not gate["enabled"]
                                            else gate["status"]
                                        )
                                        if contact_zero_ready
                                        else (
                                            gate["status"]
                                            if gate_blocks_contact
                                            else "Press Reset Zero to set contact deform zero."
                                        )
                                    )
                                ),
                            )

                            display_bgr = self._build_flow_3d_plot(
                                flow_gray,
                                height_map,
                                cv2,
                                roi_mask=ellipse_mask,
                                camera_gray_frame=flow_gray,
                                surface_history=surface_graph_history,
                            )
                            if contact_stats is not None and contact_stats["ready"]:
                                camera_depth_mm = self._estimate_camera_depth_mm(contact_stats)
                                if gate_blocks_contact:
                                    camera_depth_mm = 0.0
                                stepper_depth_mm = self._stepper_contact_depth_mm()
                                self._refresh_contact_depth_readouts(
                                    camera_depth_mm,
                                    update_camera=True,
                                )
                                flow_distance_mean = contact_stats["top_mean"]
                                flow_distance_max = contact_stats["peak"]
                                peak_pct = contact_stats["peak"] * 100.0
                                top_pct = contact_stats["top_mean"] * 100.0
                                stepper_text = (
                                    f"{stepper_depth_mm:.3f} mm"
                                    if stepper_depth_mm is not None
                                    else ("manual" if not STEPPER_DEPTH_ENABLED else "-")
                                )
                                camera_text = (
                                    f"{camera_depth_mm:.2f} mm approx"
                                    if camera_depth_mm is not None
                                    else "-"
                                )
                                distance_text = (
                                    f"Depth stepper: {stepper_text} | "
                                    f"camera: {camera_text} | contact {peak_pct:.0f}%"
                                )
                                overlay_lines = [
                                    "Remote CUDA surface",
                                    f"Pressure gate: {gate['status']}",
                                    (
                                        f"Contact intensity peak {peak_pct:.0f}% | "
                                        f"top {top_pct:.0f}%"
                                    ),
                                    (
                                        f"Area {contact_stats['area_ratio']:.1f}% | "
                                        f"residual floor {tactile_frame.residual_floor_px:.3f} px"
                                    ),
                                ]
                            else:
                                self._refresh_contact_depth_readouts(None, update_camera=True)
                                distance_text = "Press Reset Zero to set contact deform zero."
                                overlay_lines = [
                                    "Remote CUDA surface",
                                    "Press Reset Zero to set contact deform zero.",
                                    f"Disp mean {mean_disp:.3f} px | scale {scale:.3f}",
                                ]

                            for line_index, overlay_line in enumerate(overlay_lines):
                                cv2.putText(
                                    display_bgr,
                                    overlay_line,
                                    (12, 50 + (line_index * 20)),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    0.54,
                                    (250, 245, 85),
                                    2,
                                )
                        else:
                            self._set_var(self.surface_status_var, "Remote surface waiting for dots.")
                            display_bgr = frame_for_detection.copy()
                    elif remote_preview_bgr is not None and display_mode in {"pointcloud", "heatmap"}:
                        display_bgr = remote_preview_bgr
                    elif display_mode == "pointcloud":
                        display_bgr = pipe.build_pointcloud_view(
                            frame_for_detection.shape,
                            reference_centroids,
                            displacements,
                        )
                    else:
                        vector_vis = pipe.draw_displacement_vectors(
                            frame_for_detection.copy(),
                            reference_centroids,
                            displacements,
                        )
                        if display_mode == "heatmap":
                            heatmap = pipe.build_displacement_heatmap(
                                frame_for_detection.shape,
                                reference_centroids,
                                displacements,
                            )
                            display_bgr = pipe.overlay_heatmap(frame_for_detection, heatmap)
                        elif remote_preview_bgr is not None:
                            display_bgr = remote_preview_bgr
                        else:
                            display_bgr = vector_vis

                    if display_bgr is None:
                        display_bgr = frame_for_detection.copy()
                    cv2.putText(
                        display_bgr,
                        distance_text,
                        (12, 28),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.58,
                        (255, 245, 80),
                        2,
                    )
                    frame_rgb_preview = cv2.cvtColor(display_bgr, cv2.COLOR_BGR2RGB)

                    frame_counter += 1
                    now = time.perf_counter()
                    elapsed = now - fps_start
                    if elapsed >= 0.5:
                        fps = frame_counter / elapsed
                        frame_counter = 0
                        fps_start = now
                    frame_ms = (time.perf_counter() - loop_start) * 1000.0

                    self._update_camera_deform_live_features(
                        mean_disp=flow_distance_mean,
                        max_disp=flow_distance_max,
                        missing_ratio=missing_ratio,
                        contact_stats=deform_contact_stats,
                        tactile_frame=deform_tactile_frame,
                        scale_px=deform_surface_scale,
                        gate=deform_gate,
                    )
                    self._set_var(
                        self.blob_message_var,
                        (
                            f"Remote CUDA running "
                            f"({remote_payload.get('roundtrip_ms', 0.0):.1f} ms network, "
                            f"{remote_payload.get('process_backend', 'remote')}, "
                            f"{remote_payload.get('input_transport', REMOTE_VISION_TRANSPORT)}:"
                            f"{remote_payload.get('input_format', REMOTE_VISION_RAW_FORMAT)})."
                        ),
                    )

                    self._enqueue_blob_frame(
                        {
                            "frame_rgb": frame_rgb_preview,
                            "dot_count": remote_dot_count,
                            "picked_mode": f"remote CUDA | {display_mode}",
                            "fps": fps,
                            "frame_ms": frame_ms,
                            "mean_disp": flow_distance_mean,
                            "max_disp": flow_distance_max,
                            "missing_ratio": missing_ratio,
                            "new_tracks": new_count,
                            "lost_tracks": lost_count,
                            "distance_text": distance_text,
                        }
                    )

                    if self.blob_snapshot_event.is_set():
                        self.blob_snapshot_event.clear()
                        try:
                            binary = cv2.cvtColor(display_bgr, cv2.COLOR_BGR2GRAY)
                            self._save_blob_snapshot(cv2, display_bgr, binary)
                        except Exception as snapshot_exc:
                            self.log(f"Blob snapshot error: {snapshot_exc}")

                    if REMOTE_VISION_TARGET_FPS > 0:
                        min_period = 1.0 / max(0.1, REMOTE_VISION_TARGET_FPS)
                        remaining = min_period - (time.perf_counter() - loop_start)
                        if remaining > 0:
                            time.sleep(min(remaining, 0.2))

                    continue

                pre = pipe.preprocess(frame_for_detection)
                binary = pipe.threshold_image(pre, polarity=params["mode"])

                if params["use_roi"]:
                    h, w = binary.shape[:2]
                    roi_mask = np.zeros((h, w), dtype=np.uint8)
                    cx, cy = w // 2, h // 2
                    rx = max(10, int((w * params["roi_scale"]) * 0.5))
                    ry = max(10, int((h * params["roi_scale"]) * 0.5))
                    cv2.ellipse(roi_mask, (cx, cy), (rx, ry), 0, 0, 360, 255, -1)
                    binary = cv2.bitwise_and(binary, roi_mask)

                centroids = pipe.detect_centroids_from_binary(
                    binary,
                    min_area=params["min_area"],
                    max_area=params["max_area"],
                    min_circularity=params["min_circularity"],
                )
                if not centroids:
                    keypoints = pipe.detect_blobs(binary)
                    centroids = pipe.extract_centroids(keypoints)

                if (reference_centroids is None or len(reference_centroids) == 0) and centroids:
                    reference_centroids = list(centroids)
                    self.surface_scale_ema = None
                    self.surface_height_ema = None
                    self.surface_reference_height_map = None
                    self.surface_contact_ema = None
                    self.surface_contact_baseline = None
                    self.surface_contact_display_ema = None
                    self.surface_zero_ready = False
                    contact_zero_pending = False
                    contact_zero_ready = False
                    contact_zero_samples.clear()
                    surface_display_cache = None
                    surface_distance_text_cache = ""
                    surface_mean_cache = None
                    surface_max_cache = None
                    surface_graph_history.clear()
                    self._set_var(self.surface_status_var, "Contact deform zero pending. Press Reset Zero.")

                if reference_centroids is None:
                    reference_centroids = []

                if self.blob_reset_reference_event.is_set():
                    self.blob_reset_reference_event.clear()
                    reference_centroids = list(centroids)
                    tracks = {}
                    next_id = 0
                    self.surface_scale_ema = None
                    self.surface_height_ema = None
                    self.surface_reference_height_map = None
                    self.surface_contact_ema = None
                    self.surface_contact_baseline = None
                    self.surface_contact_display_ema = None
                    self.surface_zero_ready = False
                    contact_zero_pending = False
                    contact_zero_ready = False
                    contact_zero_samples.clear()
                    surface_display_cache = None
                    surface_distance_text_cache = ""
                    surface_mean_cache = None
                    surface_max_cache = None
                    surface_graph_history.clear()
                    self._set_var(self.surface_status_var, "Reference reset. Press Reset Zero.")
                    self.log("Dot pipeline reference reset to current centroids.")

                if self.surface_reset_zero_event.is_set():
                    if centroids:
                        self.surface_reset_zero_event.clear()
                        self.surface_reference_height_map = None
                        self.surface_contact_ema = None
                        self.surface_contact_baseline = None
                        self.surface_contact_display_ema = None
                        self.surface_zero_ready = False
                        contact_zero_pending = True
                        contact_zero_ready = False
                        contact_zero_samples.clear()
                        surface_display_cache = None
                        surface_distance_text_cache = ""
                        surface_mean_cache = None
                        surface_max_cache = None
                        surface_graph_history.clear()
                        self._set_var(self.surface_status_var, "Capturing contact deform zero...")
                        self.log("Contact deform zero capture requested.")
                    else:
                        self._set_var(self.surface_status_var, "Waiting for dots to reset contact deform zero.")

                contact_zero_active = bool(centroids) and contact_zero_pending

                displacements, _unmatched_ref = pipe.compute_displacements(
                    reference_centroids,
                    centroids,
                    params["match_dist"],
                )

                tracks, _assigned, new_count, lost_count, next_id = pipe.update_tracks(
                    tracks,
                    centroids,
                    params["match_dist"],
                    next_id,
                )

                disp_vals = [np.hypot(d[0], d[1]) for d in displacements if d is not None]
                mean_disp = float(np.mean(disp_vals)) if disp_vals else 0.0
                max_disp = float(np.max(disp_vals)) if disp_vals else 0.0
                missing_ratio = (
                    len([d for d in displacements if d is None]) / len(reference_centroids)
                    if reference_centroids
                    else 0.0
                )

                view_type = params.get("view_type", "auto")
                display_mode = view_type
                if view_type == "auto":
                    if params["use_mosaic"]:
                        display_mode = "mosaic"
                    elif params["use_pointcloud"]:
                        display_mode = "pointcloud"
                    else:
                        display_mode = "overlay"

                distance_text = ""
                flow_distance_mean = None
                flow_distance_max = None

                blob_vis = None
                vector_vis = None
                heatmap_vis = None
                pointcloud_vis = None
                deform_contact_stats = None
                deform_tactile_frame = None
                deform_surface_scale = None
                deform_gate = None

                if display_mode in {"blob", "vector", "overlay", "mosaic"}:
                    blob_vis = pipe.build_blob_view(binary, frame_bgr)
                if display_mode in {"vector", "overlay", "mosaic"}:
                    vector_vis = pipe.draw_displacement_vectors(blob_vis, reference_centroids, displacements)
                if display_mode in {"heatmap", "overlay", "mosaic"}:
                    heatmap = pipe.build_displacement_heatmap(frame_bgr.shape, reference_centroids, displacements)
                    heatmap_vis = pipe.overlay_heatmap(frame_bgr, heatmap)
                if display_mode in {"pointcloud", "mosaic"}:
                    pointcloud_vis = pipe.build_pointcloud_view(
                        frame_bgr.shape,
                        reference_centroids,
                        displacements,
                    )

                if display_mode == "binary":
                    display_bgr = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
                elif display_mode == "blob":
                    display_bgr = blob_vis
                elif display_mode == "vector":
                    display_bgr = vector_vis
                elif display_mode == "heatmap":
                    display_bgr = heatmap_vis
                elif display_mode == "pointcloud":
                    display_bgr = pointcloud_vis
                elif display_mode in {"optical_flow_2d", "surface_3d"}:
                    flow_gray = cv2.cvtColor(frame_for_detection, cv2.COLOR_BGR2GRAY)
                    display_bgr = frame_bgr.copy()

                    if display_mode == "optical_flow_2d":
                        flow_reseed_counter += 1
                        flow_roi_mask = None

                        if params["use_roi"]:
                            fh, fw = flow_gray.shape[:2]
                            flow_roi_mask = np.zeros((fh, fw), dtype=np.uint8)
                            fcx, fcy = fw // 2, fh // 2
                            frx = max(10, int((fw * params["roi_scale"]) * 0.5))
                            fry = max(10, int((fh * params["roi_scale"]) * 0.5))
                            cv2.ellipse(flow_roi_mask, (fcx, fcy), (frx, fry), 0, 0, 360, 255, -1)

                        need_reseed = (
                            flow_prev_gray is None
                            or flow_prev_pts is None
                            or len(flow_prev_pts) < 10
                            or flow_reseed_counter >= 12
                        )

                        if need_reseed:
                            flow_prev_pts = cv2.goodFeaturesToTrack(
                                flow_prev_gray if flow_prev_gray is not None else flow_gray,
                                maxCorners=220,
                                qualityLevel=0.02,
                                minDistance=7,
                                mask=flow_roi_mask,
                            )
                            flow_reseed_counter = 0

                        if flow_prev_gray is not None and flow_prev_pts is not None:
                            next_pts, status, _err = cv2.calcOpticalFlowPyrLK(
                                flow_prev_gray,
                                flow_gray,
                                flow_prev_pts,
                                None,
                            )
                            if next_pts is not None and status is not None:
                                good_old = flow_prev_pts[status.flatten() == 1]
                                good_new = next_pts[status.flatten() == 1]

                                flow_mags = []

                                for old_pt, new_pt in zip(good_old, good_new):
                                    ox, oy = old_pt.ravel()
                                    nx, ny = new_pt.ravel()
                                    dx = float(nx - ox)
                                    dy = float(ny - oy)
                                    mag = float(np.hypot(dx, dy))
                                    flow_mags.append(mag)

                                    cv2.arrowedLine(
                                        display_bgr,
                                        (int(ox), int(oy)),
                                        (int(nx), int(ny)),
                                        (0, 255, 255),
                                        1,
                                        tipLength=0.25,
                                    )
                                    cv2.circle(display_bgr, (int(nx), int(ny)), 2, (0, 220, 0), -1)

                                if flow_mags:
                                    scale = float(params.get("distance_scale", 1.0))
                                    unit = str(params.get("distance_unit", "px"))
                                    flow_distance_mean = float(np.mean(flow_mags)) * scale
                                    flow_distance_max = float(np.max(flow_mags)) * scale
                                    distance_text = (
                                        f"Distance mean: {flow_distance_mean:.3f} {unit} | "
                                        f"max: {flow_distance_max:.3f} {unit}"
                                    )
                                    cv2.putText(
                                        display_bgr,
                                        distance_text,
                                        (12, 28),
                                        cv2.FONT_HERSHEY_SIMPLEX,
                                        0.62,
                                        (255, 245, 80),
                                        2,
                                    )
                                else:
                                    distance_text = "Distance mean: - | max: -"

                                flow_prev_pts = (
                                    good_new.reshape(-1, 1, 2).astype(np.float32)
                                    if len(good_new) > 0
                                    else None
                                )
                            else:
                                flow_prev_pts = None
                    else:
                        surface_refresh_counter += 1
                        refresh_surface = surface_display_cache is None or surface_refresh_counter >= 2

                        if refresh_surface:
                            surface_refresh_counter = 0

                            height_map, ellipse_mask, scale = self._build_surface_height_map(
                                centroids,
                                reference_centroids,
                                displacements,
                                frame_bgr.shape,
                                params,
                                cv2,
                            )

                            if height_map is not None:
                                tactile_frame = self.surface_tactile_frame
                                tactile_config = self._build_tactile_contact_config(params)
                                raw_contact_map = (
                                    tactile_frame.contact_map
                                    if tactile_frame is not None
                                    else None
                                )
                                base_map = tactile_frame.base_map if tactile_frame is not None else None
                                contact_stats = None
                                gate = self._pressure_contact_gate_snapshot()
                                gate_blocks_contact = False
                                if raw_contact_map is not None and base_map is not None:
                                    gate_blocks_contact = bool(gate["enabled"]) and not bool(gate["contact"])
                                    if gate_blocks_contact:
                                        contact_zero_pending = False
                                        contact_zero_ready = False
                                        contact_zero_active = False
                                        contact_zero_samples.clear()
                                        self.surface_contact_baseline = None
                                        self.surface_contact_display_ema = None
                                        self.surface_zero_ready = False
                                        contact_map_for_display = np.zeros_like(raw_contact_map, dtype=np.float32)
                                    else:
                                        if gate["enabled"] and not contact_zero_ready and not contact_zero_pending:
                                            contact_zero_pending = True
                                            contact_zero_samples.clear()
                                            self.surface_contact_baseline = None
                                            self.surface_contact_display_ema = None
                                            self.surface_zero_ready = False
                                        contact_zero_active = bool(centroids) and contact_zero_pending

                                    if not gate_blocks_contact and contact_zero_active:
                                        contact_zero_samples.append(raw_contact_map.astype(np.float32).copy())
                                        if len(contact_zero_samples) >= CONTACT_ZERO_SAMPLE_COUNT:
                                            self.surface_contact_baseline = np.median(
                                                np.stack(
                                                    contact_zero_samples[-CONTACT_ZERO_SAMPLE_COUNT:],
                                                    axis=0,
                                                ),
                                                axis=0,
                                            ).astype(np.float32)
                                            self.surface_contact_display_ema = np.zeros_like(
                                                raw_contact_map,
                                                dtype=np.float32,
                                            )
                                            contact_zero_samples.clear()
                                            contact_zero_pending = False
                                            contact_zero_ready = True
                                            self.surface_zero_ready = True
                                            self.log("Contact zero captured from averaged contact maps.")
                                        contact_map_for_display = np.zeros_like(raw_contact_map, dtype=np.float32)
                                    elif not gate_blocks_contact and (
                                        contact_zero_ready
                                        and self.surface_contact_baseline is not None
                                        and self.surface_contact_baseline.shape == raw_contact_map.shape
                                    ):
                                        contact_map_for_display = np.maximum(
                                            raw_contact_map - self.surface_contact_baseline,
                                            0.0,
                                        ).astype(np.float32)
                                        if (
                                            self.surface_contact_display_ema is None
                                            or self.surface_contact_display_ema.shape != contact_map_for_display.shape
                                        ):
                                            self.surface_contact_display_ema = contact_map_for_display
                                        else:
                                            self.surface_contact_display_ema = cv2.addWeighted(
                                                self.surface_contact_display_ema.astype(np.float32),
                                                0.68,
                                                contact_map_for_display,
                                                0.32,
                                                0.0,
                                            )
                                        contact_map_for_display = self.surface_contact_display_ema
                                    else:
                                        contact_map_for_display = np.zeros_like(raw_contact_map, dtype=np.float32)

                                    height_map = surface_from_contact_map(
                                        contact_map_for_display,
                                        base_map,
                                        ellipse_mask,
                                        tactile_config,
                                        cv2=cv2,
                                    )
                                    contact_stats = summarize_contact_map(
                                        contact_map_for_display,
                                        ellipse_mask,
                                        tactile_config.contact_area_threshold,
                                    )
                                deform_contact_stats = contact_stats
                                deform_tactile_frame = tactile_frame
                                deform_surface_scale = scale
                                deform_gate = gate
                                if contact_stats is not None and contact_stats["ready"]:
                                    surface_graph_history.append(
                                        (
                                            float(contact_stats["peak"]),
                                            float(contact_stats["top_mean"]),
                                        )
                                    )
                                else:
                                    surface_graph_history.append((None, None))
                                if len(surface_graph_history) > 180:
                                    del surface_graph_history[:-180]
                                self._set_var(
                                    self.surface_status_var,
                                    (
                                        (
                                            (
                                                f"Contact deform zeroing {len(contact_zero_samples)}/"
                                                f"{CONTACT_ZERO_SAMPLE_COUNT}..."
                                            )
                                            if contact_zero_pending
                                            else "Contact deform zero set."
                                        )
                                        if contact_zero_active
                                        else (
                                            (
                                                "Surface running."
                                                if not gate["enabled"]
                                                else gate["status"]
                                            )
                                            if contact_zero_ready
                                            else (
                                                gate["status"]
                                                if gate_blocks_contact
                                                else "Press Reset Zero to set contact deform zero."
                                            )
                                        )
                                    ),
                                )
                                display_bgr = self._build_flow_3d_plot(
                                    flow_gray,
                                    height_map,
                                    cv2,
                                    roi_mask=ellipse_mask,
                                    camera_gray_frame=flow_gray,
                                    surface_history=surface_graph_history,
                                )

                                flow_distance_mean = mean_disp
                                flow_distance_max = max_disp
                                if contact_stats is not None and contact_stats["ready"]:
                                    camera_depth_mm = self._estimate_camera_depth_mm(contact_stats)
                                    if gate_blocks_contact:
                                        camera_depth_mm = 0.0
                                    stepper_depth_mm = self._stepper_contact_depth_mm()
                                    self._refresh_contact_depth_readouts(
                                        camera_depth_mm,
                                        update_camera=True,
                                    )
                                    flow_distance_mean = contact_stats["top_mean"]
                                    flow_distance_max = contact_stats["peak"]
                                    peak_pct = contact_stats["peak"] * 100.0
                                    top_pct = contact_stats["top_mean"] * 100.0
                                    stepper_text = (
                                        f"{stepper_depth_mm:.3f} mm"
                                        if stepper_depth_mm is not None
                                        else ("manual" if not STEPPER_DEPTH_ENABLED else "-")
                                    )
                                    camera_text = (
                                        f"{camera_depth_mm:.2f} mm approx"
                                        if camera_depth_mm is not None
                                        else "-"
                                    )
                                    distance_text = (
                                        f"Depth stepper: {stepper_text} | "
                                        f"camera: {camera_text} | contact {peak_pct:.0f}%"
                                    )
                                    overlay_lines = [
                                        f"Pressure gate: {gate['status']}",
                                        (
                                            f"Contact intensity peak {peak_pct:.0f}% | "
                                            f"top {top_pct:.0f}%"
                                        ),
                                        (
                                            f"Area {contact_stats['area_ratio']:.1f}% | "
                                            f"residual floor {tactile_frame.residual_floor_px:.3f} px"
                                        ),
                                    ]
                                    overlay_lines.append(
                                        f"Residual mean {tactile_frame.residual_mean_px:.3f} px | "
                                        f"peak {tactile_frame.residual_peak_px:.3f} px | "
                                        f"scale {scale:.3f}"
                                    )
                                    overlay_lines.append(
                                        (
                                            f"Depth stepper {stepper_depth_mm:.3f} mm"
                                            if stepper_depth_mm is not None
                                            else (
                                                "Depth stepper unavailable (manual)"
                                                if not STEPPER_DEPTH_ENABLED
                                                else "Depth stepper -"
                                            )
                                        )
                                    )
                                    overlay_lines[-1] += (
                                        f" | camera approx {camera_depth_mm:.2f} mm"
                                        if camera_depth_mm is not None
                                        else " | camera approx -"
                                    )
                                else:
                                    self._refresh_contact_depth_readouts(None, update_camera=True)
                                    distance_text = "Press Reset Zero to set contact deform zero."
                                    overlay_lines = [
                                        "Press Reset Zero to set contact deform zero.",
                                        f"Disp mean {mean_disp:.3f} px | scale {scale:.3f}",
                                    ]
                                for line_index, overlay_line in enumerate(overlay_lines):
                                    cv2.putText(
                                        display_bgr,
                                        overlay_line,
                                        (12, 50 + (line_index * 20)),
                                        cv2.FONT_HERSHEY_SIMPLEX,
                                        0.54,
                                        (250, 245, 85),
                                        2,
                                    )
                                surface_display_cache = display_bgr
                                surface_distance_text_cache = distance_text
                                surface_mean_cache = flow_distance_mean
                                surface_max_cache = flow_distance_max
                            else:
                                self._set_var(
                                    self.surface_status_var,
                                    (
                                        "Waiting for valid surface to set contact deform zero."
                                        if contact_zero_pending
                                        else "Surface waiting for dots."
                                    ),
                                )

                        display_bgr = surface_display_cache
                        distance_text = surface_distance_text_cache
                        flow_distance_mean = surface_mean_cache
                        flow_distance_max = surface_max_cache
                        if display_bgr is None:
                            display_bgr = cv2.cvtColor(flow_gray, cv2.COLOR_GRAY2BGR)

                    flow_prev_gray = flow_gray
                elif display_mode == "mosaic":
                    display_bgr = pipe.build_mosaic(
                        pre,
                        binary,
                        vector_vis,
                        heatmap_vis,
                        pointcloud_vis=pointcloud_vis,
                        scale=params["mosaic_scale"],
                    )
                elif display_mode == "overlay":
                    display_bgr = cv2.addWeighted(vector_vis, 0.65, heatmap_vis, 0.35, 0)
                else:
                    display_bgr = cv2.addWeighted(vector_vis, 0.65, heatmap_vis, 0.35, 0)

                if params["run_robust_test"]:
                    robust_countdown += 1
                    if robust_countdown >= 45:
                        robust_countdown = 0
                        robust_summary = pipe.robustness_test(frame_bgr, params["match_dist"])
                        summary_text = ", ".join([f"{name}:{count}" for name, count in robust_summary])
                        self.log(f"Robustness: {summary_text}")

                frame_bgr = display_bgr
                frame_rgb_preview = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

                frame_counter += 1
                now = time.perf_counter()
                elapsed = now - fps_start
                if elapsed >= 0.5:
                    fps = frame_counter / elapsed
                    frame_counter = 0
                    fps_start = now

                frame_ms = (time.perf_counter() - loop_start) * 1000.0

                payload_mean_disp = mean_disp if flow_distance_mean is None else flow_distance_mean
                payload_max_disp = max_disp if flow_distance_max is None else flow_distance_max
                payload_mode = f"{params['mode']} | {display_mode}"
                self._update_camera_deform_live_features(
                    mean_disp=mean_disp,
                    max_disp=max_disp,
                    missing_ratio=missing_ratio,
                    contact_stats=deform_contact_stats,
                    tactile_frame=deform_tactile_frame,
                    scale_px=deform_surface_scale,
                    gate=deform_gate,
                )

                self._enqueue_blob_frame(
                    {
                        "frame_rgb": frame_rgb_preview,
                        "dot_count": len(centroids),
                        "picked_mode": payload_mode,
                        "fps": fps,
                        "frame_ms": frame_ms,
                        "mean_disp": payload_mean_disp,
                        "max_disp": payload_max_disp,
                        "missing_ratio": missing_ratio,
                        "new_tracks": new_count,
                        "lost_tracks": lost_count,
                        "distance_text": distance_text,
                    }
                )

                if self.blob_snapshot_event.is_set():
                    self.blob_snapshot_event.clear()
                    try:
                        self._save_blob_snapshot(cv2, display_bgr, binary)
                    except Exception as snapshot_exc:
                        self.log(f"Blob snapshot error: {snapshot_exc}")

        except Exception as exc:
            error_message = str(exc)
            self.log(f"Blob worker error: {exc}")
            self._set_var(self.blob_state_var, "Error")
            self._set_var(self.blob_message_var, error_message)
        finally:
            if stream is not None:
                try:
                    pipe.close_camera_stream(stream)
                except Exception:
                    pass
            self._close_stream_device(stream)

            self.blob_camera = None
            self.blob_thread = None

            if error_message is None:
                self._set_var(self.blob_state_var, "Stopped")
                self._set_var(self.blob_message_var, "Blob detector stopped.")

            self._set_blob_controls(False)
            self._refresh_system_status()
            self.log("Blob test stopped.")

    def stop_blob_test(self):
        was_running = self._is_blob_running()
        self.blob_stop_event.set()
        self.blob_snapshot_event.clear()

        if not was_running:
            self.blob_state_var.set("Stopped")
            self.blob_message_var.set("Blob detector is not running.")
            self._set_blob_controls(False)
            self._refresh_system_status()
            return

        self.blob_state_var.set("Stopping")
        self.blob_message_var.set("Stopping blob detector...")
        self._set_button_enabled(self.blob_start_btn, False)
        self._set_button_enabled(self.blob_stop_btn, False)
        self._set_button_enabled(self.blob_snapshot_btn, False)
        self.log("Blob stop requested.")

        if self.blob_thread is not None and self.blob_thread.is_alive():
            self.blob_thread.join(timeout=1.5)
            if self.blob_thread is not None and self.blob_thread.is_alive():
                self.log("Blob worker is taking longer than expected to stop.")
                self._close_stream_device(self.blob_camera)
                stopped = self._wait_worker_stop(lambda: self.blob_thread, timeout=2.0)
                if not stopped:
                    self.log("Blob worker still active after forced camera release.")

    def request_blob_reference_reset(self):
        if not self._is_blob_running():
            self.log("Reference reset ignored: Dot pipeline is not running.")
            return
        self.blob_reset_reference_event.set()
        self.log("Reference reset requested.")

    def request_blob_snapshot(self):
        if not self._is_blob_running():
            self.log("Snapshot ignored: Blob detector is not running.")
            return

        self.blob_snapshot_event.set()
        self.log("Blob snapshot requested.")

    def _save_blob_snapshot(self, cv2, frame_bgr, binary):
        output_dir = Path(__file__).resolve().parent / "logs" / "blob"
        output_dir.mkdir(parents=True, exist_ok=True)

        timestamp = time.strftime("%Y%m%d_%H%M%S")
        millis = int((time.time() % 1) * 1000)
        raw_path = output_dir / f"blob_debug_raw_{timestamp}_{millis:03d}.png"
        bin_path = output_dir / f"blob_debug_bin_{timestamp}_{millis:03d}.png"

        raw_ok = cv2.imwrite(str(raw_path), frame_bgr)
        bin_ok = cv2.imwrite(str(bin_path), binary)
        if not raw_ok or not bin_ok:
            raise RuntimeError("Failed to save snapshot images.")

        self.log(f"Blob snapshot saved: {raw_path.name}, {bin_path.name}")

    def on_close(self):
        self.stop_force_cycle()
        self.stop_optical_flow()
        self.stop_blob_test()
        self.stop_camera()
        self.stop_limit_monitor()
        self.stop_stepper_move()
        self.stop_pressure_read()
        self.stop_loadcell_read()

        if GPIO is not None:
            try:
                GPIO.cleanup()
            except Exception:
                pass

        self.root.destroy()


def main(use_legacy=False):
    if not use_legacy:
        try:
            sys.modules.setdefault("all_in_one_gui", sys.modules[__name__])
            from python_mission_control_gui import main as modern_main

            modern_main()
            return
        except Exception as exc:
            try:
                import traceback

                log_path = Path(__file__).resolve().parent / "launch_errors.log"
                with log_path.open("a", encoding="utf-8") as handle:
                    handle.write("\n[all_in_one_gui_modern_redirect]\n")
                    handle.write(
                        "".join(traceback.format_exception(type(exc), exc, exc.__traceback__))
                    )
            except Exception:
                pass

    root = tk.Tk()
    app = AllInOneTesterGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
