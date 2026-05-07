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
        initial_w = max(980, min(1420, screen_w - 70))
        initial_h = max(620, min(900, screen_h - 100))
        self.root.geometry(f"{initial_w}x{initial_h}")
        self.root.minsize(940, 620)
        self.log("Mission control console loaded.")
        self.log("Force tab layout active; bubble calibration force estimate enabled.")

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
            font=("Segoe UI", 18, "bold"),
        )
        style.configure(
            "HeaderSubtitle.TLabel",
            background=COLOR_SURFACE_ALT,
            foreground=COLOR_TEXT_MUTED,
            font=("Segoe UI", 9),
        )
        style.configure(
            "SectionHint.TLabel",
            background=COLOR_SURFACE,
            foreground=COLOR_TEXT_MUTED,
            font=("Segoe UI", 8),
        )
        style.configure(
            "SectionTitle.TLabel",
            background=COLOR_SURFACE,
            foreground=COLOR_TEXT_BRIGHT,
            font=("Segoe UI", 10, "bold"),
        )
        style.configure(
            "MetricLabel.TLabel",
            background=COLOR_PANEL,
            foreground=COLOR_TEXT_MUTED,
            font=("Segoe UI", 8),
        )
        style.configure(
            "MetricValue.TLabel",
            background=COLOR_PANEL,
            foreground=COLOR_TEXT_BRIGHT,
            font=("Segoe UI", 11, "bold"),
        )
        style.configure(
            "StatusLabel.TLabel",
            background=COLOR_SURFACE_ALT,
            foreground=COLOR_TEXT_MUTED,
            font=("Segoe UI", 8, "bold"),
        )
        style.configure(
            "StatusReady.TLabel",
            background="#16354b",
            foreground="#7de1ff",
            font=("Segoe UI", 9, "bold"),
            padding=(9, 3),
        )
        style.configure(
            "StatusActive.TLabel",
            background="#113b2f",
            foreground=COLOR_OK,
            font=("Segoe UI", 9, "bold"),
            padding=(9, 3),
        )
        style.configure(
            "StatusWarn.TLabel",
            background="#4a3316",
            foreground=COLOR_WARN,
            font=("Segoe UI", 9, "bold"),
            padding=(9, 3),
        )
        style.configure(
            "Clock.TLabel",
            background=COLOR_SURFACE_ALT,
            foreground=COLOR_TEXT_MUTED,
            font=("Consolas", 9, "bold"),
        )

        style.configure(
            "TLabel",
            background=COLOR_BG_DARK,
            foreground=COLOR_TEXT_BRIGHT,
            font=("Segoe UI", 9),
        )
        style.configure(
            "TButton",
            padding=(7, 4),
            font=("Segoe UI", 9, "bold"),
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
            font=("Segoe UI", 9, "bold"),
        )

        style.configure("TNotebook", background=COLOR_BG_DARK, borderwidth=0)
        style.configure(
            "TNotebook.Tab",
            font=("Segoe UI", 9, "bold"),
            padding=(10, 5),
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
            font=("Segoe UI", 9),
        )

    def _build_ui(self):
        main = ttk.Frame(self.root, padding=8, style="App.TFrame")
        main.pack(fill="both", expand=True)
        main.columnconfigure(0, weight=1)
        main.rowconfigure(2, weight=1)
        self._layout_sashes_initialized = False

        self._build_header(main)
        self._build_action_row(main)

        workspace = ttk.Frame(main, style="App.TFrame")
        workspace.grid(row=2, column=0, sticky="nsew")
        workspace.columnconfigure(0, weight=1)
        workspace.rowconfigure(0, weight=1)
        self.workspace = workspace

        body_shell = ttk.Frame(workspace, style="App.TFrame")
        body_shell.grid(row=0, column=0, sticky="nsew")
        body_shell.columnconfigure(0, weight=1)
        body_shell.rowconfigure(0, weight=1)

        self._build_body(body_shell)
        self._build_log_panel_modern(workspace)
        self.root.after(120, self._apply_initial_split_positions)

    def _build_header(self, parent):
        header = ttk.Frame(parent, padding=10, style="HeaderCard.TFrame")
        header.grid(row=0, column=0, sticky="ew", pady=(0, 6))
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
            row=1, column=1, sticky="e", pady=(4, 0)
        )

    def _build_action_row(self, parent):
        actions = ttk.Frame(parent, style="App.TFrame")
        actions.grid(row=1, column=0, sticky="ew", pady=(0, 6))
        actions.columnconfigure(0, weight=1)

        ttk.Label(actions, text="Operator actions", style="SectionHint.TLabel").grid(
            row=0, column=0, sticky="w", padx=(2, 0)
        )

        quick = ttk.Frame(actions, style="App.TFrame")
        quick.grid(row=0, column=1, sticky="e")

        ttk.Button(
            quick,
            text="Hold",
            style="Danger.TButton",
            command=self.stop_all_tests,
        ).grid(row=0, column=0, padx=(0, 6))
        ttk.Button(
            quick,
            text="Clear",
            command=self.clear_log,
        ).grid(row=0, column=1, padx=(0, 6))
        ttk.Button(
            quick,
            text="Save",
            style="Accent.TButton",
            command=self.save_log,
        ).grid(row=0, column=2, padx=(0, 6))

        self.log_panel_visible = True
        self.log_toggle_btn = ttk.Button(quick, text="Hide Log", command=self.toggle_log_panel)
        self.log_toggle_btn.grid(row=0, column=3)

    def _build_body(self, parent):
        self._init_hardware_defaults()
        self._init_flow_defaults()

        body = ttk.Panedwindow(parent, orient="horizontal", style="Split.TPanedwindow")
        body.grid(row=0, column=0, sticky="nsew")
        self.body_pane = body

        left = ttk.Frame(body, style="App.TFrame", padding=(0, 0, 4, 0))
        left.columnconfigure(0, weight=1)
        left.rowconfigure(0, weight=1)

        right = ttk.Frame(body, style="App.TFrame", padding=(4, 0, 0, 0))
        right.columnconfigure(0, weight=1)
        right.rowconfigure(1, weight=1)

        body.add(left, weight=4)
        body.add(right, weight=1)

        self._build_primary_workspace(left)
        self._build_sidebar(right)

    def _apply_initial_split_positions(self):
        if self._layout_sashes_initialized:
            return
        if not self.root.winfo_exists():
            return

        body_width = self.body_pane.winfo_width() if hasattr(self, "body_pane") else 0
        if body_width < 320:
            self.root.after(120, self._apply_initial_split_positions)
            return

        try:
            self.body_pane.sashpos(0, int(body_width * 0.72))
            self._layout_sashes_initialized = True
        except tk.TclError:
            self.root.after(120, self._apply_initial_split_positions)

    def _build_primary_workspace(self, parent):
        notebook = ttk.Notebook(parent)
        notebook.grid(row=0, column=0, sticky="nsew")

        live_tab = ttk.Frame(notebook, style="App.TFrame")
        live_tab.columnconfigure(0, weight=1)
        live_tab.rowconfigure(0, weight=1)

        force_tab = ttk.Frame(notebook, style="App.TFrame", padding=8)
        force_tab.columnconfigure(0, weight=1)
        force_tab.rowconfigure(1, weight=1)

        notebook.add(live_tab, text="Live Detection")
        notebook.add(force_tab, text="Force")

        self._build_detection_workspace(live_tab)
        self._build_force_workspace(force_tab)

    def _build_force_workspace(self, parent):
        ttk.Label(
            parent,
            text="Pressure-derived force, load-cell comparison, and live error tracking.",
            style="SectionHint.TLabel",
        ).grid(row=0, column=0, sticky="w", pady=(0, 6))

        pane = ttk.Panedwindow(parent, orient="horizontal", style="Split.TPanedwindow")
        pane.grid(row=1, column=0, sticky="nsew")

        graph_area = ttk.Frame(pane, style="App.TFrame", padding=(0, 0, 8, 0))
        graph_area.columnconfigure(0, weight=1)
        graph_area.rowconfigure(0, weight=1)
        graph_area.rowconfigure(1, weight=1)

        side_shell, side_content = self._build_scrollable_tab(pane, padding=8)

        pane.add(graph_area, weight=4)
        pane.add(side_shell, weight=2)

        compare = ttk.LabelFrame(graph_area, text="Force comparison", padding=8)
        compare.grid(row=0, column=0, sticky="nsew", pady=(0, 6))
        compare.columnconfigure(0, weight=1)
        compare.rowconfigure(0, weight=1)
        self.force_compare_canvas = tk.Canvas(
            compare,
            height=240,
            background="#0a131b",
            highlightthickness=1,
            highlightbackground=COLOR_BORDER_DARK,
        )
        self.force_compare_canvas.grid(row=0, column=0, sticky="nsew")
        self.force_compare_canvas.bind("<Configure>", lambda _event: self._draw_force_graphs())

        error = ttk.LabelFrame(graph_area, text="Force error", padding=8)
        error.grid(row=1, column=0, sticky="nsew", pady=(6, 0))
        error.columnconfigure(0, weight=1)
        error.rowconfigure(0, weight=1)
        self.force_error_canvas = tk.Canvas(
            error,
            height=220,
            background="#0a131b",
            highlightthickness=1,
            highlightbackground=COLOR_BORDER_DARK,
        )
        self.force_error_canvas.grid(row=0, column=0, sticky="nsew")
        self.force_error_canvas.bind("<Configure>", lambda _event: self._draw_force_graphs())

        self._build_force_measurement_tab(side_content)
        self.root.after(120, self._draw_force_graphs)

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
        self.stepper_position_var = tk.StringVar(value="0.000 mm")
        self.contact_depth_stepper_var = tk.StringVar(value="unavailable (manual)")
        self.contact_depth_camera_var = tk.StringVar(value="-")
        self.contact_gate_var = tk.StringVar(value="Pressure gate waiting")
        self.blob_message_var = tk.StringVar(value="Tune the live detection profile and press Start Detection.")

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
        self._init_force_calibration_state()

    def _init_flow_defaults(self):
        self.flow_state_var = tk.StringVar(value="Merged into Live Detection")
        self.flow_points_var = tk.StringVar(value="-")
        self.flow_mean_disp_var = tk.StringVar(value="-")
        self.flow_max_disp_var = tk.StringVar(value="-")
        self.flow_fps_var = tk.StringVar(value="-")
        self.flow_message_var = tk.StringVar(value="Use Live Detection output type: optical_flow_2d.")

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

    def _build_detection_workspace(self, parent):
        self._init_blob_defaults()

        scroll_shell = ttk.Frame(parent, style="App.TFrame")
        scroll_shell.grid(row=0, column=0, sticky="nsew")
        scroll_shell.columnconfigure(0, weight=1)
        scroll_shell.rowconfigure(0, weight=1)

        canvas = tk.Canvas(
            scroll_shell,
            background=COLOR_BG_DARK,
            highlightthickness=0,
            bd=0,
            relief="flat",
        )
        scrollbar = ttk.Scrollbar(scroll_shell, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.grid(row=0, column=0, sticky="nsew")
        scrollbar.grid(row=0, column=1, sticky="ns")

        content = ttk.Frame(canvas, style="App.TFrame")
        content.columnconfigure(0, weight=1)
        window_id = canvas.create_window((0, 0), window=content, anchor="nw")

        def _sync_scroll_region(_event):
            canvas.configure(scrollregion=canvas.bbox("all"))

        def _sync_width(event):
            canvas.itemconfigure(window_id, width=event.width)

        content.bind("<Configure>", _sync_scroll_region)
        canvas.bind("<Configure>", _sync_width)

        card = ttk.LabelFrame(content, text="Live Detection", padding=8)
        card.grid(row=0, column=0, sticky="nsew")
        card.columnconfigure(0, weight=1)
        card.rowconfigure(2, weight=1, minsize=180)

        ttk.Label(
            card,
            text="Primary run surface for dot tracking and deformation monitoring. Advanced camera and tracking tuning stays behind Settings.",
            style="SectionHint.TLabel",
        ).grid(row=0, column=0, sticky="w", pady=(0, 6))

        command_bar = ttk.Frame(card, style="Surface.TFrame")
        command_bar.grid(row=1, column=0, sticky="ew", pady=(0, 6))
        for col in range(7):
            command_bar.columnconfigure(col, weight=1)

        ttk.Button(command_bar, text="Reset", command=self._apply_blob_preset).grid(
            row=0, column=0, sticky="ew", padx=(0, 6)
        )
        self.blob_start_btn = ttk.Button(
            command_bar,
            text="Start Detection",
            style="Accent.TButton",
            command=self.start_blob_test,
        )
        self.blob_start_btn.grid(row=0, column=1, sticky="ew", padx=(0, 6))
        self.blob_stop_btn = ttk.Button(command_bar, text="Stop", command=self.stop_blob_test)
        self.blob_stop_btn.grid(row=0, column=2, sticky="ew", padx=(0, 6))
        self.blob_snapshot_btn = ttk.Button(
            command_bar,
            text="Snapshot",
            command=self.request_blob_snapshot,
        )
        self.blob_snapshot_btn.grid(row=0, column=3, sticky="ew", padx=(0, 6))
        self.blob_reset_ref_btn = ttk.Button(
            command_bar,
            text="Reset Ref",
            command=self.request_blob_reference_reset,
        )
        self.blob_reset_ref_btn.grid(row=0, column=4, sticky="ew", padx=(0, 6))
        self.surface_reset_btn = ttk.Button(
            command_bar,
            text="Reset Zero",
            command=self.reset_surface_baseline,
        )
        self.surface_reset_btn.grid(row=0, column=5, sticky="ew", padx=(0, 6))
        self.blob_settings_btn = ttk.Button(
            command_bar,
            text="Settings",
            command=self.open_blob_settings_dialog,
        )
        self.blob_settings_btn.grid(row=0, column=6, sticky="ew")

        preview_shell = ttk.Frame(card, style="Surface.TFrame")
        preview_shell.grid(row=2, column=0, sticky="nsew")
        preview_shell.columnconfigure(0, weight=1)
        preview_shell.rowconfigure(0, weight=1)

        self.blob_preview_label = ttk.Label(
            preview_shell,
            text="Detection output stopped",
            anchor="center",
            background="#0a131b",
            foreground=COLOR_TEXT_MUTED,
            font=("Segoe UI", 11),
        )
        self.blob_preview_label.grid(row=0, column=0, sticky="nsew")

        metrics = ttk.Frame(card, style="App.TFrame")
        metrics.grid(row=3, column=0, sticky="ew", pady=(8, 6))
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
        self._metric_cell(metrics, 2, 0, "Stepper depth", self.contact_depth_stepper_var)
        self._metric_cell(metrics, 2, 1, "Camera depth", self.contact_depth_camera_var)
        self._metric_cell(metrics, 2, 2, "Stepper pos", self.stepper_position_var)
        self._metric_cell(metrics, 2, 3, "Contact gate", self.contact_gate_var, pad_right=0)

        control_card = ttk.LabelFrame(card, text="Detection Command Deck", padding=8)
        control_card.grid(row=4, column=0, sticky="ew")
        for col in range(4):
            control_card.columnconfigure(col, weight=1)

        ttk.Label(control_card, text="Preset").grid(row=0, column=0, sticky="w")
        ttk.Combobox(
            control_card,
            textvariable=self.blob_profile_var,
            values=("fast", "balanced", "precision", "measurement"),
            state="readonly",
        ).grid(row=1, column=0, sticky="ew", padx=(0, 6))

        ttk.Label(control_card, text="Backend").grid(row=0, column=1, sticky="w")
        ttk.Combobox(
            control_card,
            textvariable=self.blob_camera_backend_var,
            values=("auto", "picamera2", "opencv"),
            state="readonly",
        ).grid(row=1, column=1, sticky="ew", padx=(0, 6))

        ttk.Label(control_card, text="Output").grid(row=0, column=2, sticky="w")
        ttk.Combobox(
            control_card,
            textvariable=self.blob_view_type_var,
            values=("overlay", "auto", "mosaic", "vector", "heatmap", "binary", "blob", "optical_flow_2d", "surface_3d"),
            state="readonly",
        ).grid(row=1, column=2, sticky="ew", padx=(0, 6))

        ttk.Label(control_card, text="Polarity").grid(row=0, column=3, sticky="w")
        ttk.Combobox(
            control_card,
            textvariable=self.blob_mode_var,
            values=("auto", "dark", "bright"),
            state="readonly",
        ).grid(row=1, column=3, sticky="ew")

        ttk.Label(control_card, text="Cam width").grid(row=2, column=0, sticky="w", pady=(4, 0))
        ttk.Entry(control_card, textvariable=self.blob_cam_width_var).grid(
            row=3, column=0, sticky="ew", padx=(0, 6)
        )

        ttk.Label(control_card, text="Cam height").grid(row=2, column=1, sticky="w", pady=(4, 0))
        ttk.Entry(control_card, textvariable=self.blob_cam_height_var).grid(
            row=3, column=1, sticky="ew", padx=(0, 6)
        )

        ttk.Label(control_card, text="Proc scale").grid(row=2, column=2, sticky="w", pady=(4, 0))
        ttk.Entry(control_card, textvariable=self.blob_proc_scale_var).grid(
            row=3, column=2, sticky="ew", padx=(0, 6)
        )

        ttk.Label(control_card, text="ROI (%)").grid(row=2, column=3, sticky="w", pady=(4, 0))
        ttk.Entry(control_card, textvariable=self.blob_roi_percent_var).grid(
            row=3, column=3, sticky="ew"
        )

        toggles = ttk.Frame(control_card, style="Surface.TFrame")
        toggles.grid(row=4, column=0, columnspan=4, sticky="ew", pady=(4, 0))
        ttk.Checkbutton(toggles, text="Use ROI", variable=self.blob_use_roi_var).grid(row=0, column=0, sticky="w")
        ttk.Checkbutton(toggles, text="Point Cloud", variable=self.blob_use_pointcloud_var).grid(row=0, column=1, sticky="w", padx=(8, 0))
        ttk.Checkbutton(toggles, text="Mosaic", variable=self.blob_use_mosaic_var).grid(row=0, column=2, sticky="w", padx=(8, 0))
        ttk.Checkbutton(
            toggles,
            text="Illumination normalization",
            variable=self.blob_use_illum_norm_var,
        ).grid(row=0, column=3, sticky="w", padx=(8, 0))

        blob_message_label = ttk.Label(
            card,
            textvariable=self.blob_message_var,
            style="SectionHint.TLabel",
            wraplength=920,
            justify="left",
        )
        blob_message_label.grid(row=5, column=0, sticky="w", pady=(4, 0))
        self._bind_dynamic_wrap(blob_message_label, margin=18, min_wrap=320)

        self._set_blob_controls(False)

    def _metric_cell(self, parent, row, column, label, variable, pad_right=8):
        panel = ttk.Frame(parent, padding=6, style="Panel.TFrame")
        panel.grid(row=row, column=column, sticky="nsew", padx=(0, pad_right), pady=(0, 5))
        ttk.Label(panel, text=label, style="MetricLabel.TLabel").pack(anchor="w")
        ttk.Label(panel, textvariable=variable, style="MetricValue.TLabel").pack(anchor="w", pady=(3, 0))

    def _build_scrollable_tab(self, notebook, padding=8):
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
        overview = ttk.LabelFrame(parent, text="Run Readiness", padding=8)
        overview.grid(row=0, column=0, sticky="ew", pady=(0, 6))
        overview.columnconfigure(0, weight=1)
        overview.columnconfigure(1, weight=1)

        self._build_status_card(overview, 0, 0, "Detection", self.blob_state_var, "Native dot pipeline posture")
        self._build_status_card(overview, 0, 1, "Camera", self.camera_status_var, "Preview channel")
        self._build_status_card(overview, 1, 0, "Pressure", self.pressure_state_var, "Sensor sampling")
        self._build_status_card(overview, 1, 1, "Load Cell", self.loadcell_state_var, "Load verification")

        notebook = ttk.Notebook(parent)
        notebook.grid(row=1, column=0, sticky="nsew")

        ops_tab = ttk.Frame(notebook, padding=8)
        hardware_tab_shell, hardware_tab = self._build_scrollable_tab(notebook, padding=8)
        system_tab_shell, system_tab = self._build_scrollable_tab(notebook, padding=8)

        notebook.add(ops_tab, text="Operations")
        notebook.add(hardware_tab_shell, text="Hardware")
        notebook.add(system_tab_shell, text="System Tests")

        self._build_operations_tab(ops_tab)
        self._build_hardware_tab(hardware_tab)
        self._build_system_tests_tab(system_tab)

    def _build_status_card(self, parent, row, column, title, variable, detail):
        card = ttk.Frame(parent, padding=6, style="Panel.TFrame")
        card.grid(row=row, column=column, sticky="nsew", padx=(0, 6), pady=(0, 6))
        ttk.Label(card, text=title, style="MetricLabel.TLabel").pack(anchor="w")
        ttk.Label(card, textvariable=variable, style="MetricValue.TLabel").pack(anchor="w", pady=(2, 0))
        ttk.Label(card, text=detail, style="MetricLabel.TLabel").pack(anchor="w", pady=(3, 0))

    def _build_operations_tab(self, tab):
        tab.columnconfigure(0, weight=1)

        summary = ttk.LabelFrame(tab, text="Operator Summary", padding=8)
        summary.grid(row=0, column=0, sticky="ew")
        for col in range(2):
            summary.columnconfigure(col, weight=1)

        self._status_line(summary, 0, "Detection state", self.blob_state_var)
        self._status_line(summary, 1, "Picked output", self.blob_pick_mode_var)
        self._status_line(summary, 2, "Frame time", self.blob_latency_var)
        self._status_line(summary, 3, "Track health", self.blob_missing_ratio_var)
        self._status_line(summary, 4, "Estimated force", self.force_estimate_var)
        self._status_line(summary, 5, "Stepper depth", self.contact_depth_stepper_var)
        self._status_line(summary, 6, "Camera depth", self.contact_depth_camera_var)
        self._status_line(summary, 7, "Contact gate", self.contact_gate_var)

        notes = ttk.LabelFrame(tab, text="Run Notes", padding=8)
        notes.grid(row=1, column=0, sticky="ew", pady=(4, 0))
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
            row=row, column=0, sticky="w", pady=(0 if row == 0 else 5, 0)
        )
        ttk.Label(parent, textvariable=variable).grid(
            row=row, column=1, sticky="w", pady=(0 if row == 0 else 5, 0)
        )

    def _build_hardware_tab(self, tab):
        self.hardware_tab = tab
        tab.columnconfigure(0, weight=1)
        tab.columnconfigure(1, weight=1)

        camera = ttk.LabelFrame(tab, text="Camera Preview", padding=8)
        camera.grid(row=0, column=0, sticky="nsew", padx=(0, 6), pady=(0, 6))
        camera.columnconfigure(1, weight=1)
        self.camera_start_btn = ttk.Button(camera, text="Open", command=self.start_camera)
        self.camera_start_btn.grid(row=0, column=0, sticky="ew")
        self.camera_stop_btn = ttk.Button(camera, text="Stop", command=self.stop_camera)
        self.camera_stop_btn.grid(row=0, column=1, sticky="ew", padx=(6, 0))
        ttk.Label(camera, text="Status", style="MetricLabel.TLabel").grid(row=1, column=0, sticky="w", pady=(4, 0))
        ttk.Label(camera, textvariable=self.camera_status_var).grid(row=1, column=1, sticky="w", pady=(4, 0))

        limits = ttk.LabelFrame(tab, text="Limit Switches", padding=8)
        limits.grid(row=0, column=1, sticky="nsew", pady=(0, 6))
        limits.columnconfigure(1, weight=1)
        self.limit_start_btn = ttk.Button(limits, text="Start", command=self.start_limit_monitor)
        self.limit_start_btn.grid(row=0, column=0, sticky="ew")
        self.limit_stop_btn = ttk.Button(limits, text="Stop", command=self.stop_limit_monitor)
        self.limit_stop_btn.grid(row=0, column=1, sticky="ew", padx=(6, 0))
        self._status_line(limits, 1, "Monitor", self.limit_monitor_state_var)
        self._status_line(limits, 2, f"GPIO{LIMIT_1_PIN}", self.limit1_status_var)
        self._status_line(limits, 3, f"GPIO{LIMIT_2_PIN}", self.limit2_status_var)

        stepper = ttk.LabelFrame(tab, text="Motion Axis", padding=8)
        stepper.grid(row=1, column=0, sticky="nsew", padx=(0, 6), pady=(0, 6))
        stepper.columnconfigure(1, weight=1)
        stepper.columnconfigure(3, weight=1)
        ttk.Label(stepper, text="Direction").grid(row=0, column=0, sticky="w")
        ttk.Combobox(stepper, textvariable=self.stepper_direction_var, values=("up", "down"), state="readonly").grid(
            row=0, column=1, sticky="ew", padx=(0, 6)
        )
        ttk.Label(stepper, text="Seconds").grid(row=0, column=2, sticky="w")
        ttk.Entry(stepper, textvariable=self.stepper_seconds_var).grid(row=0, column=3, sticky="ew")
        self.stepper_start_btn = ttk.Button(stepper, text="Move", command=self.start_stepper_move)
        self.stepper_start_btn.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(4, 0), padx=(0, 6))
        self.stepper_stop_btn = ttk.Button(stepper, text="Stop", command=self.stop_stepper_move)
        self.stepper_stop_btn.grid(row=1, column=2, columnspan=2, sticky="ew", pady=(4, 0))
        self._status_line(stepper, 2, "State", self.stepper_state_var)

        pressure = ttk.LabelFrame(tab, text="Pressure", padding=8)
        pressure.grid(row=1, column=1, sticky="nsew", pady=(0, 6))
        pressure.columnconfigure(1, weight=1)
        self.pressure_start_btn = ttk.Button(pressure, text="Start", command=self.start_pressure_read)
        self.pressure_start_btn.grid(row=0, column=0, sticky="ew")
        self.pressure_stop_btn = ttk.Button(pressure, text="Stop", command=self.stop_pressure_read)
        self.pressure_stop_btn.grid(row=0, column=1, sticky="ew", padx=(6, 0))
        self._status_line(pressure, 1, "State", self.pressure_state_var)
        self._status_line(pressure, 2, "Reading", self.pressure_value_var)

        load = ttk.LabelFrame(tab, text="Load Cell", padding=8)
        load.grid(row=2, column=0, columnspan=2, sticky="nsew")
        load.columnconfigure(1, weight=1)
        load.columnconfigure(3, weight=1)
        ttk.Label(load, text="Known weight (g)").grid(row=0, column=0, sticky="w")
        ttk.Entry(load, textvariable=self.loadcell_known_weight_var).grid(row=0, column=1, sticky="ew", padx=(0, 6))
        self.loadcell_start_btn = ttk.Button(load, text="Start", command=self.start_loadcell_read)
        self.loadcell_start_btn.grid(row=0, column=2, sticky="ew", padx=(0, 6))
        self.loadcell_stop_btn = ttk.Button(load, text="Stop", command=self.stop_loadcell_read)
        self.loadcell_stop_btn.grid(row=0, column=3, sticky="ew")
        self._status_line(load, 1, "State", self.loadcell_state_var)
        self._status_line(load, 2, "Weight", self.loadcell_weight_var)

        force = ttk.LabelFrame(tab, text="Force Calibration", padding=8)
        force.grid(row=3, column=0, columnspan=2, sticky="ew", pady=(4, 0))
        self._build_force_calibration_panel(force)

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
        self.hw_force_card = force

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
            self.hw_camera_card.grid_configure(row=0, column=0, columnspan=1, padx=(0, 0), pady=(0, 6), sticky="ew")
            self.hw_limits_card.grid_configure(row=1, column=0, columnspan=1, padx=(0, 0), pady=(0, 6), sticky="ew")
            self.hw_stepper_card.grid_configure(row=2, column=0, columnspan=1, padx=(0, 0), pady=(0, 6), sticky="ew")
            self.hw_pressure_card.grid_configure(row=3, column=0, columnspan=1, padx=(0, 0), pady=(0, 6), sticky="ew")
            self.hw_load_card.grid_configure(row=4, column=0, columnspan=1, padx=(0, 0), pady=(0, 6), sticky="ew")
            self.hw_force_card.grid_configure(row=5, column=0, columnspan=1, padx=(0, 0), pady=(0, 0), sticky="ew")
        else:
            self.hardware_tab.columnconfigure(0, weight=1)
            self.hardware_tab.columnconfigure(1, weight=1)
            self.hw_camera_card.grid_configure(row=0, column=0, columnspan=1, padx=(0, 6), pady=(0, 6), sticky="nsew")
            self.hw_limits_card.grid_configure(row=0, column=1, columnspan=1, padx=(0, 0), pady=(0, 6), sticky="nsew")
            self.hw_stepper_card.grid_configure(row=1, column=0, columnspan=1, padx=(0, 6), pady=(0, 6), sticky="nsew")
            self.hw_pressure_card.grid_configure(row=1, column=1, columnspan=1, padx=(0, 0), pady=(0, 6), sticky="nsew")
            self.hw_load_card.grid_configure(row=2, column=0, columnspan=2, padx=(0, 0), pady=(0, 6), sticky="nsew")
            self.hw_force_card.grid_configure(row=3, column=0, columnspan=2, padx=(0, 0), pady=(0, 0), sticky="ew")

    def _build_force_measurement_tab(self, tab):
        tab.columnconfigure(0, weight=1)

        live = ttk.LabelFrame(tab, text="Force From Pressure", padding=8)
        live.grid(row=0, column=0, sticky="ew")
        live.columnconfigure(0, weight=1)
        live.columnconfigure(1, weight=1)

        self._force_metric_cell(live, 0, 0, "Estimated force", self.force_estimate_var)
        self._force_metric_cell(live, 0, 1, "Pressure", self.pressure_value_var)
        self._force_metric_cell(live, 1, 0, "Load cell", self.loadcell_weight_var)
        self._force_metric_cell(live, 1, 1, "Model", self.force_calibration_model_var)

        sensor = ttk.LabelFrame(tab, text="Sensor Run", padding=8)
        sensor.grid(row=1, column=0, sticky="ew", pady=(6, 0))
        sensor.columnconfigure(1, weight=1)
        sensor.columnconfigure(3, weight=1)

        ttk.Label(sensor, text="Pressure").grid(row=0, column=0, sticky="w")
        ttk.Label(sensor, textvariable=self.pressure_state_var).grid(row=0, column=1, sticky="w")
        self.force_pressure_start_btn = ttk.Button(sensor, text="Start", command=self.start_pressure_read)
        self.force_pressure_start_btn.grid(row=1, column=0, sticky="ew", pady=(4, 0), padx=(0, 6))
        self.force_pressure_stop_btn = ttk.Button(sensor, text="Stop", command=self.stop_pressure_read)
        self.force_pressure_stop_btn.grid(row=1, column=1, sticky="ew", pady=(4, 0))

        ttk.Label(sensor, text="Known weight (g)").grid(row=2, column=0, sticky="w", pady=(8, 0))
        ttk.Entry(sensor, textvariable=self.loadcell_known_weight_var).grid(
            row=2,
            column=1,
            sticky="ew",
            pady=(8, 0),
        )
        ttk.Label(sensor, text="Load cell").grid(row=3, column=0, sticky="w", pady=(8, 0))
        ttk.Label(sensor, textvariable=self.loadcell_state_var).grid(row=3, column=1, sticky="w", pady=(8, 0))
        self.force_loadcell_start_btn = ttk.Button(sensor, text="Start", command=self.start_loadcell_read)
        self.force_loadcell_start_btn.grid(row=4, column=0, sticky="ew", pady=(4, 0), padx=(0, 6))
        self.force_loadcell_stop_btn = ttk.Button(sensor, text="Stop", command=self.stop_loadcell_read)
        self.force_loadcell_stop_btn.grid(row=4, column=1, sticky="ew", pady=(4, 0))

        both = ttk.Frame(sensor, style="Surface.TFrame")
        both.grid(row=5, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        both.columnconfigure(0, weight=1)
        both.columnconfigure(1, weight=1)
        ttk.Button(both, text="Start Both", style="Accent.TButton", command=self.start_force_measurement).grid(
            row=0,
            column=0,
            sticky="ew",
            padx=(0, 6),
        )
        ttk.Button(both, text="Stop Both", command=self.stop_force_measurement).grid(
            row=0,
            column=1,
            sticky="ew",
        )

        calibration = ttk.LabelFrame(tab, text="Pressure To Force Calibration", padding=8)
        calibration.grid(row=2, column=0, sticky="ew", pady=(6, 0))
        self._build_force_calibration_panel(calibration)

        self._set_pressure_controls(self._is_thread_running(self.pressure_thread))
        self._set_loadcell_controls(self._is_thread_running(self.loadcell_thread))

    def _force_metric_cell(self, parent, row, column, label, variable):
        panel = ttk.Frame(parent, padding=8, style="Panel.TFrame")
        panel.grid(row=row, column=column, sticky="nsew", padx=(0 if column == 1 else 0, 6 if column == 0 else 0), pady=(0, 6))
        ttk.Label(panel, text=label, style="MetricLabel.TLabel").pack(anchor="w")
        ttk.Label(panel, textvariable=variable, style="MetricValue.TLabel").pack(anchor="w", pady=(3, 0))

    def start_force_measurement(self):
        if not self._is_thread_running(self.pressure_thread):
            self.start_pressure_read()
        if not self._is_thread_running(self.loadcell_thread):
            self.start_loadcell_read()

    def stop_force_measurement(self):
        self.stop_loadcell_read()
        self.stop_pressure_read()

    def _set_pressure_controls(self, running):
        super()._set_pressure_controls(running)
        if hasattr(self, "force_pressure_start_btn"):
            self._set_start_stop_controls(
                self.force_pressure_start_btn,
                self.force_pressure_stop_btn,
                running,
            )

    def _set_loadcell_controls(self, running):
        super()._set_loadcell_controls(running)
        if hasattr(self, "force_loadcell_start_btn"):
            self._set_start_stop_controls(
                self.force_loadcell_start_btn,
                self.force_loadcell_stop_btn,
                running,
            )

    def _build_system_tests_tab(self, tab):
        tab.columnconfigure(0, weight=1)

        vision = ttk.LabelFrame(tab, text="Vision Tests", padding=8)
        vision.grid(row=0, column=0, sticky="ew")
        vision.columnconfigure(1, weight=1)
        vision.columnconfigure(2, weight=1)

        self._test_row(
            vision,
            0,
            "Camera",
            self.camera_status_var,
            (("Open", self.start_camera), ("Stop", self.stop_camera)),
        )
        self._test_row(
            vision,
            1,
            "Dot pipeline",
            self.blob_state_var,
            (("Start", self.start_blob_test), ("Stop", self.stop_blob_test)),
        )
        hardware = ttk.LabelFrame(tab, text="Hardware Tests", padding=8)
        hardware.grid(row=1, column=0, sticky="ew", pady=(4, 0))
        hardware.columnconfigure(1, weight=1)
        hardware.columnconfigure(2, weight=1)

        self._test_row(
            hardware,
            0,
            "Limit switches",
            self.limit_monitor_state_var,
            (("Start", self.start_limit_monitor), ("Stop", self.stop_limit_monitor)),
        )
        self._test_row(
            hardware,
            1,
            "Stepper",
            self.stepper_state_var,
            (("Move", self.start_stepper_move), ("Stop", self.stop_stepper_move)),
        )
        self._test_row(
            hardware,
            2,
            "Pressure",
            self.pressure_state_var,
            (("Start", self.start_pressure_read), ("Stop", self.stop_pressure_read)),
        )
        self._test_row(
            hardware,
            3,
            "Load cell",
            self.loadcell_state_var,
            (("Start", self.start_loadcell_read), ("Stop", self.stop_loadcell_read)),
        )

        readout = ttk.LabelFrame(tab, text="Live Readouts", padding=8)
        readout.grid(row=2, column=0, sticky="ew", pady=(4, 0))
        for col in range(2):
            readout.columnconfigure(col, weight=1)
        self._status_line(readout, 0, "Pressure", self.pressure_value_var)
        self._status_line(readout, 1, "Estimated force", self.force_estimate_var)
        self._status_line(readout, 2, "Load cell", self.loadcell_weight_var)
        self._status_line(readout, 3, f"Limit GPIO{LIMIT_1_PIN}", self.limit1_status_var)
        self._status_line(readout, 4, f"Limit GPIO{LIMIT_2_PIN}", self.limit2_status_var)

        ttk.Button(
            tab,
            text="Stop All",
            style="Danger.TButton",
            command=self.stop_all_tests,
        ).grid(row=3, column=0, sticky="ew", pady=(4, 0))

    def _test_row(self, parent, row, label, status_var, actions):
        row_frame = ttk.Frame(parent, padding=6, style="Panel.TFrame")
        row_frame.grid(row=row, column=0, columnspan=3, sticky="ew", pady=(0 if row == 0 else 5, 0))
        row_frame.columnconfigure(0, weight=1)

        header = ttk.Frame(row_frame, style="Panel.TFrame")
        header.grid(row=0, column=0, sticky="ew")
        header.columnconfigure(1, weight=1)
        ttk.Label(header, text=label, style="MetricLabel.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Label(header, textvariable=status_var, style="MetricValue.TLabel").grid(
            row=0,
            column=1,
            sticky="e",
            padx=(8, 0),
        )

        buttons = ttk.Frame(row_frame, style="Panel.TFrame")
        buttons.grid(row=1, column=0, sticky="ew", pady=(5, 0))
        for idx, (text, command) in enumerate(actions):
            buttons.columnconfigure(idx, weight=1)
            ttk.Button(buttons, text=text, command=command).grid(
                row=0,
                column=idx,
                sticky="ew",
                padx=(0 if idx == 0 else 5, 0),
            )

    def _build_flow_tab(self, tab):
        tab.columnconfigure(0, weight=1)
        tab.rowconfigure(1, weight=1, minsize=220)

        top = ttk.Frame(tab, style="Surface.TFrame")
        top.grid(row=0, column=0, sticky="ew")
        top.columnconfigure(0, weight=1)
        top.columnconfigure(1, weight=1)
        top.columnconfigure(2, weight=1)

        self.flow_start_btn = ttk.Button(top, text="Start Flow", style="Accent.TButton", command=self.start_optical_flow)
        self.flow_start_btn.grid(row=0, column=0, sticky="ew", padx=(0, 6))
        self.flow_stop_btn = ttk.Button(top, text="Stop", command=self.stop_optical_flow)
        self.flow_stop_btn.grid(row=0, column=1, sticky="ew", padx=(0, 6))
        self.flow_settings_btn = ttk.Button(top, text="Advanced Settings", command=self.open_flow_settings_dialog)
        self.flow_settings_btn.grid(row=0, column=2, sticky="ew")

        preview = ttk.Frame(tab, style="Surface.TFrame", padding=8)
        preview.grid(row=1, column=0, sticky="nsew", pady=(4, 0))
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
        metrics.grid(row=2, column=0, sticky="ew", pady=(4, 0))
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
        flow_message_label.grid(row=3, column=0, sticky="w", pady=(4, 0))
        self._bind_dynamic_wrap(flow_message_label, margin=18, min_wrap=220)

        self._set_flow_controls(False)

    def _build_log_panel_modern(self, parent):
        panel = ttk.LabelFrame(parent, text="Session Log", padding=10)
        panel.columnconfigure(0, weight=1)
        panel.rowconfigure(1, weight=1)
        self.log_panel = panel
        panel.grid(row=1, column=0, sticky="ew", pady=(12, 0))

        toolbar = ttk.Frame(panel, style="Surface.TFrame")
        toolbar.grid(row=0, column=0, sticky="ew", pady=(0, 6))
        toolbar.columnconfigure(0, weight=1)

        ttk.Label(
            toolbar,
            text="Live event log for hardware state, camera access, and detection faults.",
            style="SectionHint.TLabel",
        ).grid(row=0, column=0, sticky="w")
        ttk.Button(toolbar, text="Save...", command=self.save_log).grid(row=0, column=1, padx=(6, 0))
        ttk.Button(toolbar, text="Clear", command=self.clear_log).grid(row=0, column=2, padx=(6, 0))

        from tkinter.scrolledtext import ScrolledText

        self.log_text = ScrolledText(panel, height=5, state="disabled", wrap="word")
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
            self.log_panel.grid_remove()
            self.log_toggle_btn.configure(text="Show Log")
            self.log_panel_visible = False
            self.log("Session log hidden.")
            return

        self.log_panel.grid()
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
