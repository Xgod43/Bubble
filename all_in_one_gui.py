import queue
import sys
import threading
import time
from pathlib import Path

import tkinter as tk
import numpy as np
from tkinter import filedialog, messagebox, ttk
from tkinter.scrolledtext import ScrolledText

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

LOADCELL_DT_PIN = 5
LOADCELL_SCK_PIN = 6

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


class AllInOneTesterGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Soft Gripper bubble hardware console")
        self.root.geometry("1100x820")
        self.root.minsize(980, 720)

        self.root.configure(bg=COLOR_BG)
        self._configure_styles()

        self.log_queue = queue.Queue()

        self.camera = None
        self.picam2 = None
        self.camera_running = False

        self.limit_thread = None
        self.limit_stop_event = threading.Event()

        self.stepper_thread = None
        self.stepper_stop_event = threading.Event()

        self.pressure_thread = None
        self.pressure_stop_event = threading.Event()

        self.loadcell_thread = None
        self.loadcell_stop_event = threading.Event()
        self.loadcell_hx = None

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
        self.blob_start_btn = None
        self.blob_stop_btn = None
        self.blob_snapshot_btn = None
        self.blob_reset_ref_btn = None
        self.blob_reset_reference_event = threading.Event()

        self.flow_thread = None
        self.flow_stop_event = threading.Event()
        self.flow_frame_queue = queue.Queue(maxsize=2)
        self.flow_photo = None
        self.flow_params = None
        self.flow_camera = None
        self.flow_start_btn = None
        self.flow_stop_btn = None
        self.flow_settings_btn = None

        self._build_ui()
        self._setup_gpio_once()
        self.root.after(100, self._drain_log_queue)
        self.root.after(60, self._drain_blob_frame_queue)
        self.root.after(60, self._drain_flow_frame_queue)
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
        self._build_optical_flow_tab(notebook)
        self._build_camera_tab(notebook)
        self._build_limit_tab(notebook)
        self._build_stepper_tab(notebook)
        self._build_pressure_tab(notebook)
        self._build_loadcell_tab(notebook)
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

    def _build_camera_tab(self, notebook):
        tab = ttk.Frame(notebook, padding=12)
        notebook.add(tab, text="3) Camera Test")
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
        notebook.add(tab, text="4) Limit Switch Test")
        tab.columnconfigure(2, weight=1)

        self.limit1_status_var = tk.StringVar(value="not triggered")
        self.limit2_status_var = tk.StringVar(value="not triggered")
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
        notebook.add(tab, text="5) Stepper Test")
        tab.columnconfigure(4, weight=1)

        self.stepper_direction_var = tk.StringVar(value="up")
        self.stepper_seconds_var = tk.StringVar(value="1.0")
        self.stepper_state_var = tk.StringVar(value="Idle")

        ttk.Label(
            tab,
            text=(
                "Command-style move: select up or down, enter seconds, then click Move. "
                "(Pins: PUL=GPIO24, DIR=GPIO23, ENA=GPIO26, pulses/rev=1600)"
            ),
        ).grid(row=0, column=0, columnspan=4, sticky="w", pady=(0, 8))

        ttk.Label(tab, text="Direction:").grid(row=1, column=0, sticky="w")
        direction_combo = ttk.Combobox(
            tab,
            textvariable=self.stepper_direction_var,
            values=("up", "down"),
            state="readonly",
            width=10,
        )
        direction_combo.grid(row=1, column=1, sticky="w")

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
        self._set_stepper_controls(False)

    def _build_pressure_tab(self, notebook):
        tab = ttk.Frame(notebook, padding=12)
        notebook.add(tab, text="6) Pressure Test")
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
        self._set_pressure_controls(False)

    def _build_loadcell_tab(self, notebook):
        tab = ttk.Frame(notebook, padding=12)
        notebook.add(tab, text="7) Load Cell Test")
        tab.columnconfigure(3, weight=1)

        self.loadcell_known_weight_var = tk.StringVar(value="500")
        self.loadcell_weight_var = tk.StringVar(value="-")
        self.loadcell_state_var = tk.StringVar(value="Stopped")

        ttk.Label(
            tab,
            text=(
                "Calibrates with a known weight (grams), then shows measured load in kg. "
                "(Pins: DT=GPIO5, SCK=GPIO6)"
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

        ttk.Label(tab, text="State:").grid(row=3, column=0, sticky="w", pady=(10, 0))
        ttk.Label(tab, textvariable=self.loadcell_state_var).grid(
            row=3, column=1, sticky="w", pady=(10, 0)
        )

        ttk.Label(tab, text="Weight:").grid(row=4, column=0, sticky="w", pady=(10, 0))
        ttk.Label(tab, textvariable=self.loadcell_weight_var).grid(
            row=4, column=1, sticky="w", pady=(10, 0)
        )
        self._set_loadcell_controls(False)

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
        self.blob_cam_height_var = tk.StringVar(value="720")
        self.blob_autofocus_var = tk.StringVar(value="continuous")
        self.blob_lens_position_var = tk.StringVar(value="1.0")
        self.blob_exposure_ev_var = tk.StringVar(value="0.8")

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
        right_panel.rowconfigure(1, weight=1)

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
            values=("fast", "balanced", "precision"),
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
            values=("auto", "mosaic", "overlay", "vector", "heatmap", "pointcloud", "binary", "blob", "optical_flow_2d", "optical_flow_3d"),
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

        action_row = ttk.Frame(controls)
        action_row.grid(row=8, column=0, columnspan=4, sticky="ew", pady=(10, 0))
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

        metrics = ttk.LabelFrame(right_panel, text="Live Metrics", padding=8)
        metrics.grid(row=1, column=0, sticky="nsew", pady=(10, 0))
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

        ttk.Label(
            metrics,
            textvariable=self.blob_message_var,
            wraplength=300,
            justify="left",
        ).grid(row=5, column=0, columnspan=4, sticky="w", pady=(10, 0))

        self._set_blob_controls(False)

    def _apply_blob_preset(self):
        preset = self.blob_profile_var.get().strip().lower()
        if preset == "fast":
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
        else:
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
    def _build_flow_3d_plot(gray_frame, mag_map, cv2, roi_mask=None):
        h, w = gray_frame.shape[:2]
        canvas = cv2.cvtColor(gray_frame, cv2.COLOR_GRAY2BGR)
        canvas = cv2.convertScaleAbs(canvas, alpha=0.55, beta=0)

        if mag_map is None or mag_map.size == 0:
            return canvas

        mag_work = mag_map.astype(np.float32)
        if roi_mask is not None:
            roi_binary = (roi_mask > 0)
            if np.any(roi_binary):
                mag_vals = mag_work[roi_binary]
            else:
                mag_vals = mag_work.reshape(-1)
        else:
            roi_binary = None
            mag_vals = mag_work.reshape(-1)

        if mag_vals.size == 0:
            return canvas

        robust_scale = float(np.percentile(mag_vals, 95.0))
        robust_scale = max(robust_scale, 0.08)
        mag_norm = np.clip(mag_work / robust_scale, 0.0, 1.0)

        if roi_binary is not None:
            mag_norm = mag_norm * roi_binary.astype(np.float32)

        step = max(8, min(16, w // 80))
        x_scale = 0.95
        y_scale = 0.62
        z_scale = 95.0
        horizon = int(h * 0.72)
        center_x = w * 0.5

        # Draw wireframe rows (Y lines)
        for gy in range(0, h, step):
            prev = None
            for gx in range(0, w, step):
                z = float(mag_norm[gy, gx])
                px = int((gx - center_x) * x_scale + center_x + z * 28.0)
                py = int((gy - h * 0.5) * y_scale + horizon - z * z_scale)
                cur = (int(np.clip(px, 0, w - 1)), int(np.clip(py, 0, h - 1)))
                if prev is not None:
                    color = (70, int(120 + z * 110), int(180 + z * 70))
                    cv2.line(canvas, prev, cur, color, 1)
                prev = cur

        # Draw wireframe columns (X lines)
        for gx in range(0, w, step):
            prev = None
            for gy in range(0, h, step):
                z = float(mag_norm[gy, gx])
                px = int((gx - center_x) * x_scale + center_x + z * 28.0)
                py = int((gy - h * 0.5) * y_scale + horizon - z * z_scale)
                cur = (int(np.clip(px, 0, w - 1)), int(np.clip(py, 0, h - 1)))
                if prev is not None:
                    color = (55, int(110 + z * 120), int(170 + z * 85))
                    cv2.line(canvas, prev, cur, color, 1)
                prev = cur

        if roi_binary is not None:
            roi_uint8 = roi_binary.astype(np.uint8) * 255
            contours, _ = cv2.findContours(roi_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(canvas, contours, -1, (40, 180, 220), 1)

        cv2.putText(canvas, "3D Deformation Plot", (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 230, 80), 2)
        return canvas

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
        self.flow_cam_height_var = tk.StringVar(value="720")
        self.flow_proc_scale_var = tk.StringVar(value="1.0")
        self.flow_roi_scale_var = tk.StringVar(value="0.96")
        self.flow_3d_roi_scale_var = tk.StringVar(value="0.40")
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

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

    def _require_gpio(self):
        if GPIO is not None:
            return True
        self.log("GPIO test requested, but RPi.GPIO is unavailable.")
        messagebox.showerror(
            "GPIO Unavailable",
            "RPi.GPIO is not available in this environment.",
        )
        return False

    def _set_var(self, variable, value):
        if self.root.winfo_exists():
            self.root.after(0, variable.set, value)

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

    def _set_camera_controls(self, running):
        self._set_start_stop_controls(self.camera_start_btn, self.camera_stop_btn, running)

    def _set_limit_controls(self, running):
        self._set_start_stop_controls(self.limit_start_btn, self.limit_stop_btn, running)

    def _set_stepper_controls(self, running):
        self._set_start_stop_controls(self.stepper_start_btn, self.stepper_stop_btn, running)

    def _set_pressure_controls(self, running):
        self._set_start_stop_controls(self.pressure_start_btn, self.pressure_stop_btn, running)

    def _set_loadcell_controls(self, running):
        self._set_start_stop_controls(self.loadcell_start_btn, self.loadcell_stop_btn, running)

    def _set_blob_controls(self, running):
        self._set_start_stop_controls(self.blob_start_btn, self.blob_stop_btn, running)
        self._set_button_enabled(self.blob_snapshot_btn, running)
        self._set_button_enabled(self.blob_reset_ref_btn, running)
        self._set_button_enabled(self.blob_settings_btn, not running)

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
        max_width = min(BLOB_PREVIEW_MAX_WIDTH, max(320, self.blob_preview_label.winfo_width()))
        max_height = min(BLOB_PREVIEW_MAX_HEIGHT, max(240, self.blob_preview_label.winfo_height()))
        pil_image.thumbnail((max_width, max_height), resample_mode)

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
        max_width = min(BLOB_PREVIEW_MAX_WIDTH, max(320, self.flow_preview_label.winfo_width()))
        max_height = min(BLOB_PREVIEW_MAX_HEIGHT, max(240, self.flow_preview_label.winfo_height()))
        pil_image.thumbnail((max_width, max_height), resample_mode)

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
            values=("auto", "mosaic", "overlay", "vector", "heatmap", "pointcloud", "binary", "blob", "optical_flow_2d", "optical_flow_3d"),
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

    def _load_blob_backend(self):
        if self.blob_module is not None and self.blob_cv2 is not None:
            return

        if Image is None or ImageTk is None:
            raise RuntimeError("Pillow is required for embedded preview. Install with: pip install pillow")

        try:
            import cv2
        except Exception as exc:
            raise RuntimeError(f"OpenCV import failed: {exc}") from exc

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

    def _collect_blob_params(self):
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
        if view_type == "optical_flow":
            view_type = "optical_flow_2d"
        valid_view_types = {"auto", "mosaic", "overlay", "vector", "heatmap", "pointcloud", "binary", "blob", "optical_flow_2d", "optical_flow_3d"}
        if view_type not in valid_view_types:
            raise ValueError(
                "Output type must be one of: auto, mosaic, overlay, vector, heatmap, pointcloud, binary, blob, optical_flow_2d, optical_flow_3d."
            )

        illum_method = self.blob_illum_method_var.get().strip().lower()
        if illum_method not in {"clahe", "clahe_bg"}:
            raise ValueError("Illumination method must be clahe or clahe_bg.")

        camera_index = int(self.blob_camera_index_var.get().strip())
        cam_width = int(self.blob_cam_width_var.get().strip())
        cam_height = int(self.blob_cam_height_var.get().strip())
        lens_position = float(self.blob_lens_position_var.get().strip())
        exposure_ev = float(self.blob_exposure_ev_var.get().strip())

        min_area = float(self.blob_min_area_var.get().strip())
        max_area = float(self.blob_max_area_var.get().strip())
        min_circularity = float(self.blob_min_circularity_var.get().strip())
        roi_percent = float(self.blob_roi_percent_var.get().strip())
        proc_scale = float(self.blob_proc_scale_var.get().strip())
        match_dist = float(self.blob_match_dist_var.get().strip())
        mosaic_scale = float(self.blob_mosaic_scale_var.get().strip())
        distance_scale = float(self.blob_distance_scale_var.get().strip())
        distance_unit = self.blob_distance_unit_var.get().strip()

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
            "proc_scale": proc_scale,
            "match_dist": match_dist,
            "mosaic_scale": mosaic_scale,
            "use_pointcloud": bool(self.blob_use_pointcloud_var.get()),
            "use_mosaic": bool(self.blob_use_mosaic_var.get()),
            "run_robust_test": bool(self.blob_run_robust_test_var.get()),
            "distance_scale": distance_scale,
            "distance_unit": distance_unit,
        }
        params["mode"] = mode
        params["min_area"] = min_area
        params["max_area"] = max_area
        params["min_circularity"] = min_circularity
        params["use_roi"] = bool(self.blob_use_roi_var.get())
        params["roi_scale"] = roi_percent / 100.0
        params["view_type"] = view_type
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
        if not self._require_gpio():
            return
        if self.limit_thread is not None and self.limit_thread.is_alive():
            self.log("Limit monitor is already running.")
            return

        GPIO.setup(LIMIT_1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(LIMIT_2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        self.limit_stop_event.clear()
        self.limit_thread = threading.Thread(target=self._limit_worker, daemon=True)
        self.limit_thread.start()
        self.limit_monitor_state_var.set("Running")
        self._set_limit_controls(True)
        self._refresh_system_status()
        self.log(
            f"Limit monitor started on GPIO{LIMIT_1_PIN} and GPIO{LIMIT_2_PIN}."
        )

    def _limit_worker(self):
        last_state = (None, None)
        try:
            while not self.limit_stop_event.is_set():
                l1_triggered = GPIO.input(LIMIT_1_PIN) == GPIO_LOW
                l2_triggered = GPIO.input(LIMIT_2_PIN) == GPIO_LOW
                current_state = (l1_triggered, l2_triggered)

                if current_state != last_state:
                    l1_text = "TRIGGERED" if l1_triggered else "not triggered"
                    l2_text = "TRIGGERED" if l2_triggered else "not triggered"
                    self._set_var(self.limit1_status_var, l1_text)
                    self._set_var(self.limit2_status_var, l2_text)

                    if l1_triggered:
                        self.log("Limit 1 triggered.")
                    if l2_triggered:
                        self.log("Limit 2 triggered.")
                    if not l1_triggered and not l2_triggered:
                        self.log("Both limits are not triggered.")

                    last_state = current_state

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
        if not self._require_gpio():
            return
        if self.stepper_thread is not None and self.stepper_thread.is_alive():
            self.log("Stepper is already moving.")
            return

        direction = self.stepper_direction_var.get().strip().lower()
        if direction not in {"up", "down"}:
            messagebox.showerror("Invalid Direction", "Direction must be up or down.")
            return

        try:
            seconds = float(self.stepper_seconds_var.get().strip())
        except ValueError:
            messagebox.showerror("Invalid Seconds", "Seconds must be a number.")
            return

        if seconds <= 0:
            messagebox.showerror("Invalid Seconds", "Seconds must be greater than 0.")
            return

        GPIO.setup(STEPPER_PUL_PIN, GPIO.OUT, initial=GPIO_LOW)
        GPIO.setup(STEPPER_DIR_PIN, GPIO.OUT, initial=DIR_DOWN_STATE)
        GPIO.setup(STEPPER_ENA_PIN, GPIO.OUT, initial=STEPPER_ENA_INACTIVE_STATE)

        # Keep command naming consistent with existing stepper script wiring.
        direction_state = DIR_DOWN_STATE if direction == "up" else DIR_UP_STATE

        self.stepper_stop_event.clear()
        self.stepper_thread = threading.Thread(
            target=self._stepper_worker,
            args=(direction, direction_state, seconds),
            daemon=True,
        )
        self.stepper_thread.start()
        self.stepper_state_var.set(f"Running ({direction} for {seconds:.2f}s)")
        self._set_stepper_controls(True)
        self._refresh_system_status()
        self.log(f"Stepper command: {direction} for {seconds:.2f} seconds.")

    def _stepper_worker(self, direction_name, direction_state, seconds):
        try:
            GPIO.output(STEPPER_ENA_PIN, STEPPER_ENA_ACTIVE_STATE)
            GPIO.output(STEPPER_DIR_PIN, direction_state)
            time.sleep(0.002)

            end_time = time.perf_counter() + seconds
            while time.perf_counter() < end_time and not self.stepper_stop_event.is_set():
                GPIO.output(STEPPER_PUL_PIN, GPIO_HIGH)
                time.sleep(STEPPER_PULSE_DELAY)
                GPIO.output(STEPPER_PUL_PIN, GPIO_LOW)
                time.sleep(STEPPER_PULSE_DELAY)

            if self.stepper_stop_event.is_set():
                self.log("Stepper movement stopped by user.")
            else:
                self.log(f"Stepper movement complete: {direction_name}.")
        except Exception as exc:
            self.log(f"Stepper error: {exc}")
        finally:
            try:
                GPIO.output(STEPPER_PUL_PIN, GPIO_LOW)
                GPIO.output(STEPPER_ENA_PIN, STEPPER_ENA_INACTIVE_STATE)
            except Exception:
                pass
            self._set_var(self.stepper_state_var, "Idle")
            self._set_stepper_controls(False)
            self._refresh_system_status()

    def stop_stepper_move(self):
        self.stepper_stop_event.set()
        if self.stepper_thread is not None and self.stepper_thread.is_alive():
            self.stepper_thread.join(timeout=1.0)
        self.stepper_thread = None
        self.stepper_state_var.set("Idle")
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

    def _pressure_worker(self):
        try:
            import board
            import adafruit_mprls

            i2c = board.I2C()
            sensor = adafruit_mprls.MPRLS(i2c, psi_min=0, psi_max=25)
            self.log("MPRLS pressure sensor initialized.")

            while not self.pressure_stop_event.is_set():
                pressure_hpa = float(sensor.pressure)
                self._set_var(self.pressure_value_var, f"{pressure_hpa:.2f} hPa")
                time.sleep(0.5)
        except Exception as exc:
            self.log(f"Pressure sensor error: {exc}")
            self._set_var(self.pressure_value_var, "Error")
        finally:
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

    def _load_hx711_class(self):
        hx_path = Path(__file__).resolve().parent / "library" / "hx711py"
        if str(hx_path) not in sys.path:
            sys.path.insert(0, str(hx_path))
        from hx711v0_5_1 import HX711

        return HX711

    @staticmethod
    def _read_average_raw(hx, samples=20, delay=0.05, channel="A"):
        readings = []
        for _ in range(samples):
            value = hx.getLong(channel)
            if value is not None:
                readings.append(float(value))
            time.sleep(delay)

        if not readings:
            raise RuntimeError("No valid load cell readings were captured.")

        return sum(readings) / len(readings)

    def start_loadcell_read(self):
        if not self._require_gpio():
            return
        if self.loadcell_thread is not None and self.loadcell_thread.is_alive():
            self.log("Load cell read is already running.")
            return

        try:
            known_weight_grams = float(self.loadcell_known_weight_var.get().strip())
        except ValueError:
            messagebox.showerror("Invalid Weight", "Known weight must be a number.")
            return

        if known_weight_grams <= 0:
            messagebox.showerror("Invalid Weight", "Known weight must be > 0 grams.")
            return

        self.loadcell_stop_event.clear()
        self.loadcell_thread = threading.Thread(
            target=self._loadcell_worker,
            args=(known_weight_grams,),
            daemon=True,
        )
        self.loadcell_thread.start()
        self.loadcell_state_var.set("Running")
        self._set_loadcell_controls(True)
        self._refresh_system_status()
        self.log("Load cell read started.")

    def _loadcell_worker(self, known_weight_grams):
        hx = None
        try:
            HX711 = self._load_hx711_class()
            hx = HX711(LOADCELL_DT_PIN, LOADCELL_SCK_PIN)
            self.loadcell_hx = hx

            hx.setReadingFormat("MSB", "MSB")
            hx.reset()

            self.log("Load cell: remove all weight for tare.")
            if not self._wait_with_cancel(2.0, self.loadcell_stop_event):
                return

            offset = self._read_average_raw(hx, channel="A")
            hx.setOffset(offset, "A")
            self.log(f"Load cell tare offset: {offset:.2f}")

            self.log(f"Load cell: place {known_weight_grams:.2f} g for calibration.")
            if not self._wait_with_cancel(3.0, self.loadcell_stop_event):
                return

            loaded_raw = self._read_average_raw(hx, channel="A")
            delta = loaded_raw - offset
            if abs(delta) < 1e-9:
                raise RuntimeError("Calibration failed: raw delta is too small.")

            counts_per_gram = delta / known_weight_grams
            hx.setReferenceUnit(counts_per_gram, "A")
            self.log(f"Load cell calibrated. counts_per_gram={counts_per_gram:.6f}")

            while not self.loadcell_stop_event.is_set():
                weight_grams = hx.getWeight("A")
                if weight_grams is not None:
                    weight_kg = float(weight_grams) / 1000.0
                    self._set_var(self.loadcell_weight_var, f"{weight_kg:.4f} kg")
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
        flow_mag_ema = None
        flow_3d_refresh_counter = 0
        flow_3d_display_cache = None
        flow_3d_distance_text_cache = ""
        flow_3d_mean_cache = None
        flow_3d_max_cache = None

        try:
            pipe = self.blob_module
            cv2 = self.blob_cv2
            params = dict(self.blob_params)

            pipe.MIN_BLOB_AREA = int(params["min_area"])
            pipe.MAX_BLOB_AREA = int(params["max_area"])
            pipe.MIN_CIRCULARITY = float(params["min_circularity"])

            stream = pipe.open_camera_stream(
                camera_index=params["camera_index"],
                width=int(params["cam_width"]),
                height=int(params["cam_height"]),
                backend=params["camera_backend"],
                autofocus=params["autofocus"],
                lens_position=params["lens_position"],
                exposure_ev=params["exposure_ev"],
            )

            self.blob_camera = stream
            self._set_var(self.blob_state_var, "Running")
            self._set_var(self.blob_message_var, "Blob detector is running.")
            self._refresh_system_status()

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
                if params.get("use_illum_norm", False):
                    frame_for_detection = self._apply_illumination_normalization(
                        frame_bgr,
                        cv2,
                        params.get("illum_method", "clahe_bg"),
                    )

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
                )
                if not centroids:
                    keypoints = pipe.detect_blobs(binary)
                    centroids = pipe.extract_centroids(keypoints)

                if (reference_centroids is None or len(reference_centroids) == 0) and centroids:
                    reference_centroids = list(centroids)

                if reference_centroids is None:
                    reference_centroids = []

                if self.blob_reset_reference_event.is_set():
                    self.blob_reset_reference_event.clear()
                    reference_centroids = list(centroids)
                    tracks = {}
                    next_id = 0
                    self.log("Dot pipeline reference reset to current centroids.")

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
                elif display_mode in {"optical_flow_2d", "optical_flow_3d"}:
                    flow_gray = cv2.cvtColor(frame_for_detection, cv2.COLOR_BGR2GRAY)
                    display_bgr = frame_bgr.copy()
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

                                if view_type == "optical_flow_2d":
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
                                if view_type == "optical_flow_2d":
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

                            flow_prev_pts = good_new.reshape(-1, 1, 2).astype(np.float32) if len(good_new) > 0 else None
                        else:
                            flow_prev_pts = None

                    if view_type == "optical_flow_3d" and flow_prev_gray is not None:
                        flow_3d_refresh_counter += 1
                        refresh_3d = flow_3d_display_cache is None or flow_3d_refresh_counter >= 3

                        if refresh_3d:
                            flow_3d_refresh_counter = 0

                            dense_prev_gray = flow_prev_gray
                            dense_gray = flow_gray
                            dense_roi_mask = flow_roi_mask

                            if dense_gray.shape[0] >= 360 or dense_gray.shape[1] >= 640:
                                dense_prev_gray = cv2.resize(
                                    flow_prev_gray,
                                    None,
                                    fx=0.5,
                                    fy=0.5,
                                    interpolation=cv2.INTER_AREA,
                                )
                                dense_gray = cv2.resize(
                                    flow_gray,
                                    None,
                                    fx=0.5,
                                    fy=0.5,
                                    interpolation=cv2.INTER_AREA,
                                )
                                if flow_roi_mask is not None:
                                    dense_roi_mask = cv2.resize(
                                        flow_roi_mask,
                                        (dense_gray.shape[1], dense_gray.shape[0]),
                                        interpolation=cv2.INTER_NEAREST,
                                    )

                            dense_roi_scale = float(params.get("roi_3d_scale", params.get("roi_scale", 1.0)))
                            dense_h, dense_w = dense_gray.shape[:2]
                            dense_cx = dense_w // 2
                            dense_cy = dense_h // 2
                            dense_rx = max(8, int((dense_w * dense_roi_scale) * 0.5))
                            dense_ry = max(8, int((dense_h * dense_roi_scale) * 0.5))
                            dense_x0 = max(0, dense_cx - dense_rx)
                            dense_x1 = min(dense_w, dense_cx + dense_rx)
                            dense_y0 = max(0, dense_cy - dense_ry)
                            dense_y1 = min(dense_h, dense_cy + dense_ry)

                            dense_prev_gray = dense_prev_gray[dense_y0:dense_y1, dense_x0:dense_x1]
                            dense_gray = dense_gray[dense_y0:dense_y1, dense_x0:dense_x1]
                            if dense_roi_mask is not None:
                                dense_roi_mask = dense_roi_mask[dense_y0:dense_y1, dense_x0:dense_x1]

                            dense_flow = cv2.calcOpticalFlowFarneback(
                                dense_prev_gray,
                                dense_gray,
                                None,
                                0.5,
                                2,
                                15,
                                2,
                                5,
                                1.1,
                                cv2.OPTFLOW_FARNEBACK_GAUSSIAN,
                            )

                            dense_dx = dense_flow[..., 0].astype(np.float32)
                            dense_dy = dense_flow[..., 1].astype(np.float32)
                            valid_mask = np.ones(dense_gray.shape, dtype=bool)

                            if dense_roi_mask is not None:
                                roi_valid = dense_roi_mask > 0
                                valid_mask &= roi_valid
                                dense_dx = np.where(roi_valid, dense_dx, 0.0)
                                dense_dy = np.where(roi_valid, dense_dy, 0.0)

                            # Suppress low-texture regions where dense optical flow is unstable.
                            gx = cv2.Sobel(dense_gray, cv2.CV_32F, 1, 0, ksize=3)
                            gy = cv2.Sobel(dense_gray, cv2.CV_32F, 0, 1, ksize=3)
                            grad_mag = cv2.magnitude(gx, gy)
                            grad_pool = grad_mag[valid_mask] if np.any(valid_mask) else grad_mag.reshape(-1)
                            grad_thresh = float(np.percentile(grad_pool, 45.0)) if grad_pool.size > 0 else 0.0
                            texture_mask = grad_mag >= grad_thresh
                            valid_mask &= texture_mask

                            if np.any(valid_mask):
                                # Remove global camera jitter so remaining magnitude better reflects deformation.
                                global_dx = float(np.median(dense_dx[valid_mask]))
                                global_dy = float(np.median(dense_dy[valid_mask]))
                                dense_dx = dense_dx - global_dx
                                dense_dy = dense_dy - global_dy

                            dense_mag = cv2.magnitude(dense_dx, dense_dy)
                            dense_mag = np.where(valid_mask, dense_mag, 0.0)

                            valid_mag = dense_mag[valid_mask] if np.any(valid_mask) else dense_mag.reshape(-1)
                            if valid_mag.size > 0:
                                clip_hi = float(np.percentile(valid_mag, 98.0))
                                if clip_hi > 0:
                                    dense_mag = np.clip(dense_mag, 0.0, clip_hi)

                            dense_mag = cv2.GaussianBlur(dense_mag, (0, 0), sigmaX=1.4, sigmaY=1.4)

                            if flow_mag_ema is None or flow_mag_ema.shape != dense_mag.shape:
                                flow_mag_ema = dense_mag
                            else:
                                flow_mag_ema = cv2.addWeighted(flow_mag_ema, 0.80, dense_mag, 0.20, 0.0)

                            display_bgr = self._build_flow_3d_plot(dense_gray, flow_mag_ema, cv2, roi_mask=dense_roi_mask)

                            scale = float(params.get("distance_scale", 1.0))
                            unit = str(params.get("distance_unit", "px"))
                            ema_valid_mag = flow_mag_ema[valid_mask] if np.any(valid_mask) else flow_mag_ema.reshape(-1)
                            if ema_valid_mag.size > 0:
                                flow_distance_mean = float(np.mean(ema_valid_mag)) * scale
                                flow_distance_max = float(np.max(ema_valid_mag)) * scale
                            else:
                                flow_distance_mean = 0.0
                                flow_distance_max = 0.0
                            distance_text = (
                                f"Distance mean: {flow_distance_mean:.3f} {unit} | "
                                f"max: {flow_distance_max:.3f} {unit}"
                            )
                            cv2.putText(
                                display_bgr,
                                distance_text,
                                (12, 50),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.60,
                                (250, 245, 85),
                                2,
                            )
                            flow_3d_display_cache = display_bgr
                            flow_3d_distance_text_cache = distance_text
                            flow_3d_mean_cache = flow_distance_mean
                            flow_3d_max_cache = flow_distance_max

                        display_bgr = flow_3d_display_cache
                        distance_text = flow_3d_distance_text_cache
                        flow_distance_mean = flow_3d_mean_cache
                        flow_distance_max = flow_3d_max_cache
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


def main():
    root = tk.Tk()
    app = AllInOneTesterGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()