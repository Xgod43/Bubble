from __future__ import annotations

import time
import tkinter as tk
from tkinter import ttk

from all_in_one_gui import (
    AllInOneTesterGUI,
    COLOR_ACCENT,
    COLOR_ACCENT_SOFT,
    COLOR_OK,
    COLOR_WARN,
    LIMIT_1_PIN,
    LIMIT_2_PIN,
)


COLOR_BG_DARK = "#071018"
COLOR_SURFACE = "#0c1722"
COLOR_SURFACE_ALT = "#0f1f2e"
COLOR_PANEL = "#122232"
COLOR_BORDER_DARK = "#22384d"
COLOR_TEXT_BRIGHT = "#eef5fb"
COLOR_TEXT_MUTED = "#93a9bc"
COLOR_DANGER = "#c25b5b"


class MissionControlGUI(AllInOneTesterGUI):
    def __init__(self, root):
        super().__init__(root)
        self.root.title("Soft Gripper Bubble Console")
        screen_w = self.root.winfo_screenwidth()
        screen_h = self.root.winfo_screenheight()
        initial_w = max(1024, min(1520, screen_w - 60))
        initial_h = max(680, min(960, screen_h - 90))
        self.root.geometry(f"{initial_w}x{initial_h}")
        self.root.minsize(1024, 680)
        self.log("Mission control console loaded.")

    def _configure_styles(self):
        style = ttk.Style(self.root)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass

        style.configure(".", background=COLOR_BG_DARK, foreground=COLOR_TEXT_BRIGHT)
        style.configure("TFrame", background=COLOR_BG_DARK)
        style.configure("App.TFrame", background=COLOR_BG_DARK)
        style.configure("Surface.TFrame", background=COLOR_SURFACE)
        style.configure("Panel.TFrame", background=COLOR_PANEL)
        style.configure("HeaderCard.TFrame", background=COLOR_SURFACE_ALT)
        style.configure(
            "HeaderTitle.TLabel",
            background=COLOR_SURFACE_ALT,
            foreground=COLOR_TEXT_BRIGHT,
            font=("Segoe UI", 23, "bold"),
        )
        style.configure(
            "HeaderSubtitle.TLabel",
            background=COLOR_SURFACE_ALT,
            foreground=COLOR_TEXT_MUTED,
            font=("Segoe UI", 10),
        )
        style.configure(
            "SectionHint.TLabel",
            background=COLOR_SURFACE,
            foreground=COLOR_TEXT_MUTED,
            font=("Segoe UI", 9),
        )
        style.configure(
            "SectionTitle.TLabel",
            background=COLOR_SURFACE,
            foreground=COLOR_TEXT_BRIGHT,
            font=("Segoe UI", 12, "bold"),
        )
        style.configure(
            "MetricLabel.TLabel",
            background=COLOR_PANEL,
            foreground=COLOR_TEXT_MUTED,
            font=("Segoe UI", 9),
        )
        style.configure(
            "MetricValue.TLabel",
            background=COLOR_PANEL,
            foreground=COLOR_TEXT_BRIGHT,
            font=("Segoe UI", 14, "bold"),
        )
        style.configure(
            "StatusLabel.TLabel",
            background=COLOR_SURFACE_ALT,
            foreground=COLOR_TEXT_MUTED,
            font=("Segoe UI", 9, "bold"),
        )
        style.configure(
            "StatusReady.TLabel",
            background="#16354b",
            foreground="#7de1ff",
            font=("Segoe UI", 10, "bold"),
            padding=(12, 5),
        )
        style.configure(
            "StatusActive.TLabel",
            background="#113b2f",
            foreground=COLOR_OK,
            font=("Segoe UI", 10, "bold"),
            padding=(12, 5),
        )
        style.configure(
            "StatusWarn.TLabel",
            background="#4a3316",
            foreground=COLOR_WARN,
            font=("Segoe UI", 10, "bold"),
            padding=(12, 5),
        )
        style.configure(
            "Clock.TLabel",
            background=COLOR_SURFACE_ALT,
            foreground=COLOR_TEXT_MUTED,
            font=("Consolas", 10, "bold"),
        )

        style.configure(
            "TLabel",
            background=COLOR_BG_DARK,
            foreground=COLOR_TEXT_BRIGHT,
            font=("Segoe UI", 10),
        )
        style.configure(
            "TButton",
            padding=(10, 7),
            font=("Segoe UI", 10, "bold"),
            background=COLOR_PANEL,
            foreground=COLOR_TEXT_BRIGHT,
            bordercolor=COLOR_BORDER_DARK,
        )
        style.map(
            "TButton",
            background=[("active", "#193248"), ("disabled", "#1b2732")],
            foreground=[("disabled", "#6d8191")],
        )
        style.configure("Accent.TButton", background=COLOR_ACCENT, foreground="#071018")
        style.map("Accent.TButton", background=[("active", "#84e2ff")])
        style.configure("Danger.TButton", background=COLOR_DANGER, foreground="#ffffff")
        style.map("Danger.TButton", background=[("active", "#d47373")])

        style.configure(
            "TLabelframe",
            background=COLOR_SURFACE,
            bordercolor=COLOR_BORDER_DARK,
            relief="solid",
        )
        style.configure(
            "TLabelframe.Label",
            background=COLOR_SURFACE,
            foreground=COLOR_TEXT_BRIGHT,
            font=("Segoe UI", 10, "bold"),
        )

        style.configure("TNotebook", background=COLOR_BG_DARK, borderwidth=0)
        style.configure(
            "TNotebook.Tab",
            font=("Segoe UI", 10, "bold"),
            padding=(14, 8),
            background=COLOR_PANEL,
            foreground=COLOR_TEXT_MUTED,
        )
        style.map(
            "TNotebook.Tab",
            background=[("selected", COLOR_SURFACE_ALT), ("!selected", COLOR_PANEL)],
            foreground=[("selected", COLOR_TEXT_BRIGHT), ("!selected", COLOR_TEXT_MUTED)],
        )

        style.configure("Split.TPanedwindow", background=COLOR_BG_DARK)

        style.configure(
            "TEntry",
            fieldbackground="#0a131b",
            foreground=COLOR_TEXT_BRIGHT,
            insertcolor=COLOR_TEXT_BRIGHT,
            bordercolor=COLOR_BORDER_DARK,
        )
        style.configure(
            "TCombobox",
            fieldbackground="#0a131b",
            foreground=COLOR_TEXT_BRIGHT,
            bordercolor=COLOR_BORDER_DARK,
            arrowsize=16,
        )
        style.map(
            "TCombobox",
            fieldbackground=[("readonly", "#0a131b")],
            selectbackground=[("readonly", "#0a131b")],
            selectforeground=[("readonly", COLOR_TEXT_BRIGHT)],
        )
        style.configure(
            "TCheckbutton",
            background=COLOR_SURFACE,
            foreground=COLOR_TEXT_BRIGHT,
            font=("Segoe UI", 10),
        )

    def _build_ui(self):
        main = ttk.Frame(self.root, padding=14, style="App.TFrame")
        main.pack(fill="both", expand=True)
        main.columnconfigure(0, weight=1)
        main.rowconfigure(2, weight=1)
        self._layout_sashes_initialized = False

        self._build_header(main)
        self._build_action_row(main)

        workspace = ttk.Panedwindow(main, orient="vertical", style="Split.TPanedwindow")
        workspace.grid(row=2, column=0, sticky="nsew")
        self.workspace_pane = workspace

        body_shell = ttk.Frame(workspace, style="App.TFrame")
        body_shell.columnconfigure(0, weight=1)
        body_shell.rowconfigure(0, weight=1)
        workspace.add(body_shell, weight=6)

        self._build_body(body_shell)
        self._build_log_panel_modern(workspace)
        self.root.after(120, self._apply_initial_split_positions)

    def _build_header(self, parent):
        header = ttk.Frame(parent, padding=18, style="HeaderCard.TFrame")
        header.grid(row=0, column=0, sticky="ew", pady=(0, 12))
        header.columnconfigure(0, weight=1)
        header.columnconfigure(1, weight=1)

        left = ttk.Frame(header, style="HeaderCard.TFrame")
        left.grid(row=0, column=0, sticky="w")

        ttk.Label(
            left,
            text="Soft Gripper Bubble Console",
            style="HeaderTitle.TLabel",
        ).grid(row=0, column=0, sticky="w")
        ttk.Label(
            left,
            text="Operate the rig from one screen: live detection, motion, pressure, load, and fault visibility.",
            style="HeaderSubtitle.TLabel",
        ).grid(row=1, column=0, sticky="w", pady=(4, 0))

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

    def _build_action_row(self, parent):
        actions = ttk.Frame(parent, style="App.TFrame")
        actions.grid(row=1, column=0, sticky="ew", pady=(0, 12))
        actions.columnconfigure(0, weight=1)

        ttk.Label(actions, text="Operator actions", style="SectionHint.TLabel").grid(
            row=0, column=0, sticky="w", padx=(2, 0)
        )

        quick = ttk.Frame(actions, style="App.TFrame")
        quick.grid(row=0, column=1, sticky="e")

        ttk.Button(
            quick,
            text="Hold All",
            style="Danger.TButton",
            command=self.stop_all_tests,
        ).grid(row=0, column=0, padx=(0, 8))
        ttk.Button(
            quick,
            text="Clear Log",
            command=self.clear_log,
        ).grid(row=0, column=1, padx=(0, 8))
        ttk.Button(
            quick,
            text="Save Log",
            style="Accent.TButton",
            command=self.save_log,
        ).grid(row=0, column=2, padx=(0, 8))

        self.log_panel_visible = True
        self.log_toggle_btn = ttk.Button(quick, text="Hide Log", command=self.toggle_log_panel)
        self.log_toggle_btn.grid(row=0, column=3)

    def _build_body(self, parent):
        self._init_hardware_defaults()
        self._init_flow_defaults()

        body = ttk.Panedwindow(parent, orient="horizontal", style="Split.TPanedwindow")
        body.grid(row=0, column=0, sticky="nsew")
        self.body_pane = body

        left = ttk.Frame(body, style="App.TFrame", padding=(0, 0, 6, 0))
        left.columnconfigure(0, weight=1)
        left.rowconfigure(0, weight=1)

        right = ttk.Frame(body, style="App.TFrame", padding=(6, 0, 0, 0))
        right.columnconfigure(0, weight=1)
        right.rowconfigure(1, weight=1)

        body.add(left, weight=3)
        body.add(right, weight=2)

        self._build_detection_workspace(left)
        self._build_sidebar(right)

    def _apply_initial_split_positions(self):
        if self._layout_sashes_initialized:
            return
        if not self.root.winfo_exists():
            return

        body_width = self.body_pane.winfo_width() if hasattr(self, "body_pane") else 0
        workspace_height = self.workspace_pane.winfo_height() if hasattr(self, "workspace_pane") else 0
        if body_width < 320 or workspace_height < 320:
            self.root.after(120, self._apply_initial_split_positions)
            return

        try:
            self.body_pane.sashpos(0, int(body_width * 0.64))
            if self.log_panel_visible:
                self.workspace_pane.sashpos(0, int(workspace_height * 0.72))
            self._layout_sashes_initialized = True
        except tk.TclError:
            self.root.after(120, self._apply_initial_split_positions)

    def _init_blob_defaults(self):
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
        self.blob_message_var = tk.StringVar(value="Tune the live detection profile and press Start Detection.")

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

    def _init_hardware_defaults(self):
        self.camera_status_var = tk.StringVar(value="Stopped")
        self.limit1_status_var = tk.StringVar(value="not triggered")
        self.limit2_status_var = tk.StringVar(value="not triggered")
        self.limit_monitor_state_var = tk.StringVar(value="Stopped")
        self.stepper_direction_var = tk.StringVar(value="up")
        self.stepper_seconds_var = tk.StringVar(value="1.0")
        self.stepper_state_var = tk.StringVar(value="Idle")
        self.pressure_value_var = tk.StringVar(value="-")
        self.pressure_state_var = tk.StringVar(value="Stopped")
        self.loadcell_known_weight_var = tk.StringVar(value="500")
        self.loadcell_weight_var = tk.StringVar(value="-")
        self.loadcell_state_var = tk.StringVar(value="Stopped")

    def _init_flow_defaults(self):
        self.flow_state_var = tk.StringVar(value="Stopped")
        self.flow_points_var = tk.StringVar(value="-")
        self.flow_mean_disp_var = tk.StringVar(value="-")
        self.flow_max_disp_var = tk.StringVar(value="-")
        self.flow_fps_var = tk.StringVar(value="-")
        self.flow_message_var = tk.StringVar(value="Start optical flow to inspect live motion.")

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

    def _build_detection_workspace(self, parent):
        self._init_blob_defaults()

        card = ttk.LabelFrame(parent, text="Live Detection", padding=12)
        card.grid(row=0, column=0, sticky="nsew")
        card.columnconfigure(0, weight=1)
        card.rowconfigure(1, weight=1, minsize=240)

        ttk.Label(
            card,
            text="Primary run surface for dot tracking and deformation monitoring. Advanced camera and tracking tuning stays behind Settings.",
            style="SectionHint.TLabel",
        ).grid(row=0, column=0, sticky="w", pady=(0, 10))

        preview_shell = ttk.Frame(card, style="Surface.TFrame")
        preview_shell.grid(row=1, column=0, sticky="nsew")
        preview_shell.columnconfigure(0, weight=1)
        preview_shell.rowconfigure(0, weight=1)

        self.blob_preview_label = ttk.Label(
            preview_shell,
            text="Detection output stopped",
            anchor="center",
            background="#0a131b",
            foreground=COLOR_TEXT_MUTED,
            font=("Segoe UI", 14),
        )
        self.blob_preview_label.grid(row=0, column=0, sticky="nsew")

        metrics = ttk.Frame(card, style="App.TFrame")
        metrics.grid(row=2, column=0, sticky="ew", pady=(12, 10))
        for col in range(4):
            metrics.columnconfigure(col, weight=1)

        self._metric_cell(metrics, 0, 0, "State", self.blob_state_var)
        self._metric_cell(metrics, 0, 1, "Mode", self.blob_pick_mode_var)
        self._metric_cell(metrics, 0, 2, "Dots", self.blob_dot_count_var)
        self._metric_cell(metrics, 0, 3, "FPS", self.blob_fps_var, pad_right=0)
        self._metric_cell(metrics, 1, 0, "Frame", self.blob_latency_var)
        self._metric_cell(metrics, 1, 1, "Mean disp", self.blob_mean_disp_var)
        self._metric_cell(metrics, 1, 2, "Max disp", self.blob_max_disp_var)
        self._metric_cell(metrics, 1, 3, "Missing", self.blob_missing_ratio_var, pad_right=0)

        control_card = ttk.LabelFrame(card, text="Detection Command Deck", padding=12)
        control_card.grid(row=3, column=0, sticky="ew")
        for col in range(4):
            control_card.columnconfigure(col, weight=1)

        ttk.Label(control_card, text="Preset").grid(row=0, column=0, sticky="w")
        ttk.Combobox(
            control_card,
            textvariable=self.blob_profile_var,
            values=("fast", "balanced", "precision"),
            state="readonly",
        ).grid(row=1, column=0, sticky="ew", padx=(0, 8))

        ttk.Label(control_card, text="Backend").grid(row=0, column=1, sticky="w")
        ttk.Combobox(
            control_card,
            textvariable=self.blob_camera_backend_var,
            values=("auto", "picamera2", "opencv"),
            state="readonly",
        ).grid(row=1, column=1, sticky="ew", padx=(0, 8))

        ttk.Label(control_card, text="Output").grid(row=0, column=2, sticky="w")
        ttk.Combobox(
            control_card,
            textvariable=self.blob_view_type_var,
            values=("overlay", "auto", "mosaic", "vector", "heatmap", "binary", "blob", "optical_flow_2d", "optical_flow_3d"),
            state="readonly",
        ).grid(row=1, column=2, sticky="ew", padx=(0, 8))

        ttk.Label(control_card, text="Polarity").grid(row=0, column=3, sticky="w")
        ttk.Combobox(
            control_card,
            textvariable=self.blob_mode_var,
            values=("auto", "dark", "bright"),
            state="readonly",
        ).grid(row=1, column=3, sticky="ew")

        ttk.Label(control_card, text="Cam width").grid(row=2, column=0, sticky="w", pady=(10, 0))
        ttk.Entry(control_card, textvariable=self.blob_cam_width_var).grid(
            row=3, column=0, sticky="ew", padx=(0, 8)
        )

        ttk.Label(control_card, text="Cam height").grid(row=2, column=1, sticky="w", pady=(10, 0))
        ttk.Entry(control_card, textvariable=self.blob_cam_height_var).grid(
            row=3, column=1, sticky="ew", padx=(0, 8)
        )

        ttk.Label(control_card, text="Proc scale").grid(row=2, column=2, sticky="w", pady=(10, 0))
        ttk.Entry(control_card, textvariable=self.blob_proc_scale_var).grid(
            row=3, column=2, sticky="ew", padx=(0, 8)
        )

        ttk.Label(control_card, text="ROI (%)").grid(row=2, column=3, sticky="w", pady=(10, 0))
        ttk.Entry(control_card, textvariable=self.blob_roi_percent_var).grid(
            row=3, column=3, sticky="ew"
        )

        toggles = ttk.Frame(control_card, style="Surface.TFrame")
        toggles.grid(row=4, column=0, columnspan=4, sticky="ew", pady=(10, 0))
        ttk.Checkbutton(toggles, text="Use ROI", variable=self.blob_use_roi_var).grid(row=0, column=0, sticky="w")
        ttk.Checkbutton(toggles, text="Point Cloud", variable=self.blob_use_pointcloud_var).grid(row=0, column=1, sticky="w", padx=(12, 0))
        ttk.Checkbutton(toggles, text="Mosaic", variable=self.blob_use_mosaic_var).grid(row=0, column=2, sticky="w", padx=(12, 0))
        ttk.Checkbutton(
            toggles,
            text="Illumination normalization",
            variable=self.blob_use_illum_norm_var,
        ).grid(row=0, column=3, sticky="w", padx=(12, 0))

        actions = ttk.Frame(control_card, style="Surface.TFrame")
        actions.grid(row=5, column=0, columnspan=4, sticky="ew", pady=(12, 0))
        for col in range(5):
            actions.columnconfigure(col, weight=1)

        ttk.Button(actions, text="Apply Preset", command=self._apply_blob_preset).grid(
            row=0, column=0, sticky="ew", padx=(0, 8)
        )
        self.blob_start_btn = ttk.Button(actions, text="Start Detection", style="Accent.TButton", command=self.start_blob_test)
        self.blob_start_btn.grid(row=0, column=1, sticky="ew", padx=(0, 8))
        self.blob_stop_btn = ttk.Button(actions, text="Stop", command=self.stop_blob_test)
        self.blob_stop_btn.grid(row=0, column=2, sticky="ew", padx=(0, 8))
        self.blob_snapshot_btn = ttk.Button(actions, text="Snapshot", command=self.request_blob_snapshot)
        self.blob_snapshot_btn.grid(row=0, column=3, sticky="ew", padx=(0, 8))
        self.blob_reset_ref_btn = ttk.Button(actions, text="Reset Ref", command=self.request_blob_reference_reset)
        self.blob_reset_ref_btn.grid(row=0, column=4, sticky="ew")

        self.blob_settings_btn = ttk.Button(
            control_card,
            text="Advanced Detection Settings",
            command=self.open_blob_settings_dialog,
        )
        self.blob_settings_btn.grid(row=6, column=0, columnspan=4, sticky="ew", pady=(10, 0))

        blob_message_label = ttk.Label(
            card,
            textvariable=self.blob_message_var,
            style="SectionHint.TLabel",
            wraplength=920,
            justify="left",
        )
        blob_message_label.grid(row=4, column=0, sticky="w", pady=(10, 0))
        self._bind_dynamic_wrap(blob_message_label, margin=18, min_wrap=320)

        self._set_blob_controls(False)

    def _metric_cell(self, parent, row, column, label, variable, pad_right=8):
        panel = ttk.Frame(parent, padding=10, style="Panel.TFrame")
        panel.grid(row=row, column=column, sticky="nsew", padx=(0, pad_right), pady=(0, 8))
        ttk.Label(panel, text=label, style="MetricLabel.TLabel").pack(anchor="w")
        ttk.Label(panel, textvariable=variable, style="MetricValue.TLabel").pack(anchor="w", pady=(6, 0))

    def _build_scrollable_tab(self, notebook, padding=12):
        shell = ttk.Frame(notebook, style="App.TFrame")
        shell.columnconfigure(0, weight=1)
        shell.rowconfigure(0, weight=1)

        canvas = tk.Canvas(
            shell,
            background=COLOR_BG_DARK,
            highlightthickness=0,
            bd=0,
            relief="flat",
        )
        scrollbar = ttk.Scrollbar(shell, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.grid(row=0, column=0, sticky="nsew")
        scrollbar.grid(row=0, column=1, sticky="ns")

        content = ttk.Frame(canvas, padding=padding, style="App.TFrame")
        window_id = canvas.create_window((0, 0), window=content, anchor="nw")

        def _sync_scroll_region(_event):
            canvas.configure(scrollregion=canvas.bbox("all"))

        def _sync_width(event):
            canvas.itemconfigure(window_id, width=event.width)

        content.bind("<Configure>", _sync_scroll_region)
        canvas.bind("<Configure>", _sync_width)
        return shell, content

    @staticmethod
    def _bind_dynamic_wrap(label, margin=26, min_wrap=180):
        def _apply_wrap(_event):
            width = max(min_wrap, label.winfo_width() - margin)
            label.configure(wraplength=width)

        label.bind("<Configure>", _apply_wrap)

    def _build_sidebar(self, parent):
        overview = ttk.LabelFrame(parent, text="Run Readiness", padding=12)
        overview.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        overview.columnconfigure(0, weight=1)
        overview.columnconfigure(1, weight=1)

        self._build_status_card(overview, 0, 0, "Detection", self.blob_state_var, "Native dot pipeline posture")
        self._build_status_card(overview, 0, 1, "Camera", self.camera_status_var, "Preview channel")
        self._build_status_card(overview, 1, 0, "Pressure", self.pressure_state_var, "Sensor sampling")
        self._build_status_card(overview, 1, 1, "Load Cell", self.loadcell_state_var, "Load verification")

        notebook = ttk.Notebook(parent)
        notebook.grid(row=1, column=0, sticky="nsew")

        ops_tab = ttk.Frame(notebook, padding=12)
        hardware_tab_shell, hardware_tab = self._build_scrollable_tab(notebook, padding=12)
        flow_tab = ttk.Frame(notebook, padding=12)

        notebook.add(ops_tab, text="Operations")
        notebook.add(hardware_tab_shell, text="Hardware")
        notebook.add(flow_tab, text="Optical Flow")

        self._build_operations_tab(ops_tab)
        self._build_hardware_tab(hardware_tab)
        self._build_flow_tab(flow_tab)

    def _build_status_card(self, parent, row, column, title, variable, detail):
        card = ttk.Frame(parent, padding=10, style="Panel.TFrame")
        card.grid(row=row, column=column, sticky="nsew", padx=(0, 8), pady=(0, 8))
        ttk.Label(card, text=title, style="MetricLabel.TLabel").pack(anchor="w")
        ttk.Label(card, textvariable=variable, style="MetricValue.TLabel").pack(anchor="w", pady=(4, 0))
        ttk.Label(card, text=detail, style="MetricLabel.TLabel").pack(anchor="w", pady=(6, 0))

    def _build_operations_tab(self, tab):
        tab.columnconfigure(0, weight=1)

        summary = ttk.LabelFrame(tab, text="Operator Summary", padding=12)
        summary.grid(row=0, column=0, sticky="ew")
        for col in range(2):
            summary.columnconfigure(col, weight=1)

        self._status_line(summary, 0, "Detection state", self.blob_state_var)
        self._status_line(summary, 1, "Picked output", self.blob_pick_mode_var)
        self._status_line(summary, 2, "Frame time", self.blob_latency_var)
        self._status_line(summary, 3, "Track health", self.blob_missing_ratio_var)

        notes = ttk.LabelFrame(tab, text="Run Notes", padding=12)
        notes.grid(row=1, column=0, sticky="ew", pady=(10, 0))
        notes_label = ttk.Label(
            notes,
            text=(
                "Use Start Detection for the main run path. Keep an eye on missing ratio and the session log; "
                "if tracking drifts, reset the reference after lighting or focus changes."
            ),
            style="SectionHint.TLabel",
            wraplength=420,
            justify="left",
        )
        notes_label.grid(row=0, column=0, sticky="w")
        self._bind_dynamic_wrap(notes_label, margin=18, min_wrap=220)

    def _status_line(self, parent, row, label, variable):
        ttk.Label(parent, text=label, style="MetricLabel.TLabel").grid(
            row=row, column=0, sticky="w", pady=(0 if row == 0 else 8, 0)
        )
        ttk.Label(parent, textvariable=variable).grid(
            row=row, column=1, sticky="w", pady=(0 if row == 0 else 8, 0)
        )

    def _build_hardware_tab(self, tab):
        self.hardware_tab = tab
        tab.columnconfigure(0, weight=1)
        tab.columnconfigure(1, weight=1)

        camera = ttk.LabelFrame(tab, text="Camera Preview", padding=12)
        camera.grid(row=0, column=0, sticky="nsew", padx=(0, 8), pady=(0, 8))
        camera.columnconfigure(1, weight=1)
        self.camera_start_btn = ttk.Button(camera, text="Open", command=self.start_camera)
        self.camera_start_btn.grid(row=0, column=0, sticky="ew")
        self.camera_stop_btn = ttk.Button(camera, text="Stop", command=self.stop_camera)
        self.camera_stop_btn.grid(row=0, column=1, sticky="ew", padx=(8, 0))
        ttk.Label(camera, text="Status", style="MetricLabel.TLabel").grid(row=1, column=0, sticky="w", pady=(10, 0))
        ttk.Label(camera, textvariable=self.camera_status_var).grid(row=1, column=1, sticky="w", pady=(10, 0))

        limits = ttk.LabelFrame(tab, text="Limit Switches", padding=12)
        limits.grid(row=0, column=1, sticky="nsew", pady=(0, 8))
        limits.columnconfigure(1, weight=1)
        self.limit_start_btn = ttk.Button(limits, text="Start", command=self.start_limit_monitor)
        self.limit_start_btn.grid(row=0, column=0, sticky="ew")
        self.limit_stop_btn = ttk.Button(limits, text="Stop", command=self.stop_limit_monitor)
        self.limit_stop_btn.grid(row=0, column=1, sticky="ew", padx=(8, 0))
        self._status_line(limits, 1, "Monitor", self.limit_monitor_state_var)
        self._status_line(limits, 2, f"GPIO{LIMIT_1_PIN}", self.limit1_status_var)
        self._status_line(limits, 3, f"GPIO{LIMIT_2_PIN}", self.limit2_status_var)

        stepper = ttk.LabelFrame(tab, text="Motion Axis", padding=12)
        stepper.grid(row=1, column=0, sticky="nsew", padx=(0, 8), pady=(0, 8))
        stepper.columnconfigure(1, weight=1)
        stepper.columnconfigure(3, weight=1)
        ttk.Label(stepper, text="Direction").grid(row=0, column=0, sticky="w")
        ttk.Combobox(stepper, textvariable=self.stepper_direction_var, values=("up", "down"), state="readonly").grid(
            row=0, column=1, sticky="ew", padx=(0, 8)
        )
        ttk.Label(stepper, text="Seconds").grid(row=0, column=2, sticky="w")
        ttk.Entry(stepper, textvariable=self.stepper_seconds_var).grid(row=0, column=3, sticky="ew")
        self.stepper_start_btn = ttk.Button(stepper, text="Move", command=self.start_stepper_move)
        self.stepper_start_btn.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(10, 0), padx=(0, 8))
        self.stepper_stop_btn = ttk.Button(stepper, text="Stop", command=self.stop_stepper_move)
        self.stepper_stop_btn.grid(row=1, column=2, columnspan=2, sticky="ew", pady=(10, 0))
        self._status_line(stepper, 2, "State", self.stepper_state_var)

        pressure = ttk.LabelFrame(tab, text="Pressure", padding=12)
        pressure.grid(row=1, column=1, sticky="nsew", pady=(0, 8))
        pressure.columnconfigure(1, weight=1)
        self.pressure_start_btn = ttk.Button(pressure, text="Start", command=self.start_pressure_read)
        self.pressure_start_btn.grid(row=0, column=0, sticky="ew")
        self.pressure_stop_btn = ttk.Button(pressure, text="Stop", command=self.stop_pressure_read)
        self.pressure_stop_btn.grid(row=0, column=1, sticky="ew", padx=(8, 0))
        self._status_line(pressure, 1, "State", self.pressure_state_var)
        self._status_line(pressure, 2, "Reading", self.pressure_value_var)

        load = ttk.LabelFrame(tab, text="Load Cell", padding=12)
        load.grid(row=2, column=0, columnspan=2, sticky="nsew")
        load.columnconfigure(1, weight=1)
        load.columnconfigure(3, weight=1)
        ttk.Label(load, text="Known weight (g)").grid(row=0, column=0, sticky="w")
        ttk.Entry(load, textvariable=self.loadcell_known_weight_var).grid(row=0, column=1, sticky="ew", padx=(0, 8))
        self.loadcell_start_btn = ttk.Button(load, text="Start", command=self.start_loadcell_read)
        self.loadcell_start_btn.grid(row=0, column=2, sticky="ew", padx=(0, 8))
        self.loadcell_stop_btn = ttk.Button(load, text="Stop", command=self.stop_loadcell_read)
        self.loadcell_stop_btn.grid(row=0, column=3, sticky="ew")
        self._status_line(load, 1, "State", self.loadcell_state_var)
        self._status_line(load, 2, "Weight", self.loadcell_weight_var)

        self._set_camera_controls(False)
        self._set_limit_controls(False)
        self._set_stepper_controls(False)
        self._set_pressure_controls(False)
        self._set_loadcell_controls(False)

        self.hw_camera_card = camera
        self.hw_limits_card = limits
        self.hw_stepper_card = stepper
        self.hw_pressure_card = pressure
        self.hw_load_card = load

        tab.bind("<Configure>", self._on_hardware_tab_resize)
        self.root.after_idle(self._arrange_hardware_tab_cards)

    def _on_hardware_tab_resize(self, event):
        self._arrange_hardware_tab_cards(event.width)

    def _arrange_hardware_tab_cards(self, width=None):
        if not hasattr(self, "hardware_tab"):
            return

        if width is None:
            width = self.hardware_tab.winfo_width()
        compact = width < 980

        if compact:
            self.hardware_tab.columnconfigure(0, weight=1)
            self.hardware_tab.columnconfigure(1, weight=0)
            self.hw_camera_card.grid_configure(row=0, column=0, columnspan=1, padx=(0, 0), pady=(0, 8), sticky="ew")
            self.hw_limits_card.grid_configure(row=1, column=0, columnspan=1, padx=(0, 0), pady=(0, 8), sticky="ew")
            self.hw_stepper_card.grid_configure(row=2, column=0, columnspan=1, padx=(0, 0), pady=(0, 8), sticky="ew")
            self.hw_pressure_card.grid_configure(row=3, column=0, columnspan=1, padx=(0, 0), pady=(0, 8), sticky="ew")
            self.hw_load_card.grid_configure(row=4, column=0, columnspan=1, padx=(0, 0), pady=(0, 0), sticky="ew")
        else:
            self.hardware_tab.columnconfigure(0, weight=1)
            self.hardware_tab.columnconfigure(1, weight=1)
            self.hw_camera_card.grid_configure(row=0, column=0, columnspan=1, padx=(0, 8), pady=(0, 8), sticky="nsew")
            self.hw_limits_card.grid_configure(row=0, column=1, columnspan=1, padx=(0, 0), pady=(0, 8), sticky="nsew")
            self.hw_stepper_card.grid_configure(row=1, column=0, columnspan=1, padx=(0, 8), pady=(0, 8), sticky="nsew")
            self.hw_pressure_card.grid_configure(row=1, column=1, columnspan=1, padx=(0, 0), pady=(0, 8), sticky="nsew")
            self.hw_load_card.grid_configure(row=2, column=0, columnspan=2, padx=(0, 0), pady=(0, 0), sticky="nsew")

    def _build_flow_tab(self, tab):
        tab.columnconfigure(0, weight=1)
        tab.rowconfigure(1, weight=1, minsize=220)

        top = ttk.Frame(tab, style="Surface.TFrame")
        top.grid(row=0, column=0, sticky="ew")
        top.columnconfigure(0, weight=1)
        top.columnconfigure(1, weight=1)
        top.columnconfigure(2, weight=1)

        self.flow_start_btn = ttk.Button(top, text="Start Flow", style="Accent.TButton", command=self.start_optical_flow)
        self.flow_start_btn.grid(row=0, column=0, sticky="ew", padx=(0, 8))
        self.flow_stop_btn = ttk.Button(top, text="Stop", command=self.stop_optical_flow)
        self.flow_stop_btn.grid(row=0, column=1, sticky="ew", padx=(0, 8))
        self.flow_settings_btn = ttk.Button(top, text="Advanced Settings", command=self.open_flow_settings_dialog)
        self.flow_settings_btn.grid(row=0, column=2, sticky="ew")

        preview = ttk.Frame(tab, style="Surface.TFrame", padding=8)
        preview.grid(row=1, column=0, sticky="nsew", pady=(10, 0))
        preview.columnconfigure(0, weight=1)
        preview.rowconfigure(0, weight=1)
        self.flow_preview_label = ttk.Label(
            preview,
            text="Optical flow preview stopped",
            anchor="center",
            background="#0a131b",
            foreground=COLOR_TEXT_MUTED,
        )
        self.flow_preview_label.grid(row=0, column=0, sticky="nsew")

        metrics = ttk.Frame(tab, style="App.TFrame")
        metrics.grid(row=2, column=0, sticky="ew", pady=(10, 0))
        for col in range(5):
            metrics.columnconfigure(col, weight=1)

        self._metric_cell(metrics, 0, 0, "State", self.flow_state_var)
        self._metric_cell(metrics, 0, 1, "Points", self.flow_points_var)
        self._metric_cell(metrics, 0, 2, "Mean disp", self.flow_mean_disp_var)
        self._metric_cell(metrics, 0, 3, "Max disp", self.flow_max_disp_var)
        self._metric_cell(metrics, 0, 4, "FPS", self.flow_fps_var, pad_right=0)

        flow_message_label = ttk.Label(
            tab,
            textvariable=self.flow_message_var,
            style="SectionHint.TLabel",
            wraplength=420,
            justify="left",
        )
        flow_message_label.grid(row=3, column=0, sticky="w", pady=(10, 0))
        self._bind_dynamic_wrap(flow_message_label, margin=18, min_wrap=220)

        self._set_flow_controls(False)

    def _build_log_panel_modern(self, parent):
        panel = ttk.LabelFrame(parent, text="Session Log", padding=10)
        panel.columnconfigure(0, weight=1)
        panel.rowconfigure(1, weight=1)
        self.log_panel = panel
        parent.add(panel, weight=2)

        toolbar = ttk.Frame(panel, style="Surface.TFrame")
        toolbar.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        toolbar.columnconfigure(0, weight=1)

        ttk.Label(
            toolbar,
            text="Live event log for hardware state, camera access, and detection faults.",
            style="SectionHint.TLabel",
        ).grid(row=0, column=0, sticky="w")
        ttk.Button(toolbar, text="Save...", command=self.save_log).grid(row=0, column=1, padx=(8, 0))
        ttk.Button(toolbar, text="Clear", command=self.clear_log).grid(row=0, column=2, padx=(8, 0))

        from tkinter.scrolledtext import ScrolledText

        self.log_text = ScrolledText(panel, height=7, state="disabled", wrap="word")
        self.log_text.configure(
            font=("Consolas", 10),
            bg="#0a131b",
            fg=COLOR_TEXT_BRIGHT,
            insertbackground=COLOR_TEXT_BRIGHT,
            relief="flat",
        )
        self.log_text.grid(row=1, column=0, sticky="nsew")

    def toggle_log_panel(self):
        if self.log_panel_visible:
            self.workspace_pane.forget(self.log_panel)
            self.log_toggle_btn.configure(text="Show Log")
            self.log_panel_visible = False
            self.log("Session log hidden.")
            return

        self.workspace_pane.add(self.log_panel, weight=2)
        self._layout_sashes_initialized = False
        self.root.after(80, self._apply_initial_split_positions)
        self.log_toggle_btn.configure(text="Hide Log")
        self.log_panel_visible = True
        self.log("Session log shown.")


def main():
    root = tk.Tk()
    app = MissionControlGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
