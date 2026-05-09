from __future__ import annotations

import argparse
import base64
import json
import os
import sys
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np


ROOT_DIR = Path(__file__).resolve().parent
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from JNR import dot_pipeline as pipe
from tactile_contact_pipeline import (
    TactileContactConfig,
    build_tactile_contact_frame,
    summarize_contact_map,
    surface_from_contact_map,
)


SERVICE_BACKEND = "remote-cuda-opencv"
CUDA_PREPROCESS_ENABLED = os.environ.get(
    "BUBBLE_REMOTE_VISION_CUDA_PREPROCESS",
    "1",
).strip().lower() in {"1", "true", "yes", "on"}
REMOTE_RENDER_JPEG_QUALITY = int(os.environ.get("BUBBLE_REMOTE_VISION_RENDER_JPEG_QUALITY", "62"))
REMOTE_SURFACE_RENDER_MODE = os.environ.get("BUBBLE_REMOTE_SURFACE_RENDER_MODE", "fast").strip().lower()
BUBBLE_WIDTH_MM = 120.0
BUBBLE_HEIGHT_MM = 80.0
BUBBLE_MAX_HEIGHT_MM = 25.0
CONTACT_ZERO_SAMPLE_COUNT = 8


class RemoteVisionState:
    def __init__(self):
        self.reference_centroids = None
        self.last_frame_shape = None
        self.last_frame_at = None
        self.last_process_ms = None
        self.last_process_backend = "not-started"
        self.cuda_error = None
        self.surface_scale_ema = None
        self.surface_height_ema = None
        self.surface_contact_baseline = None
        self.surface_contact_display_ema = None
        self.surface_zero_ready = False
        self.surface_zero_pending = False
        self.surface_zero_samples = []
        self.surface_history = []
        self.frame_count = 0
        self.started_at = time.perf_counter()

    def reset(self):
        self.reference_centroids = None
        self.last_frame_shape = None
        self.last_frame_at = None
        self.last_process_ms = None
        self.last_process_backend = "not-started"
        self.cuda_error = None
        self.reset_surface()
        self.frame_count = 0
        self.started_at = time.perf_counter()

    def reset_surface(self):
        self.surface_scale_ema = None
        self.surface_height_ema = None
        self.surface_contact_baseline = None
        self.surface_contact_display_ema = None
        self.surface_zero_ready = False
        self.surface_zero_pending = False
        self.surface_zero_samples = []
        self.surface_history = []


STATE = RemoteVisionState()
_CUDA_DEVICE_COUNT = None
_CUDA_FILTERS = {}


def _cuda_device_count():
    global _CUDA_DEVICE_COUNT
    if _CUDA_DEVICE_COUNT is not None:
        return _CUDA_DEVICE_COUNT
    try:
        _CUDA_DEVICE_COUNT = int(cv2.cuda.getCudaEnabledDeviceCount())
    except Exception:
        _CUDA_DEVICE_COUNT = 0
    return _CUDA_DEVICE_COUNT


def _cuda_preprocess(frame_bgr):
    if not CUDA_PREPROCESS_ENABLED or _cuda_device_count() <= 0:
        return None

    try:
        gpu_frame = cv2.cuda_GpuMat()
        gpu_frame.upload(frame_bgr)
        gpu_gray = cv2.cuda.cvtColor(gpu_frame, cv2.COLOR_BGR2GRAY)

        if pipe.MEDIAN_BLUR_KSIZE >= 3 and pipe.MEDIAN_BLUR_KSIZE % 2 == 1:
            median_key = ("median", int(pipe.MEDIAN_BLUR_KSIZE))
            median_filter = _CUDA_FILTERS.get(median_key)
            if median_filter is None:
                median_filter = cv2.cuda.createMedianFilter(
                    cv2.CV_8UC1,
                    int(pipe.MEDIAN_BLUR_KSIZE),
                )
                _CUDA_FILTERS[median_key] = median_filter
            gpu_gray = median_filter.apply(gpu_gray)

        gauss_key = ("gaussian", 3, 3)
        gauss_filter = _CUDA_FILTERS.get(gauss_key)
        if gauss_filter is None:
            gauss_filter = cv2.cuda.createGaussianFilter(
                cv2.CV_8UC1,
                cv2.CV_8UC1,
                (3, 3),
                0,
            )
            _CUDA_FILTERS[gauss_key] = gauss_filter
        return gauss_filter.apply(gpu_gray).download()
    except Exception as exc:
        STATE.cuda_error = str(exc)
        return None


def _preprocess(frame_bgr):
    pre = _cuda_preprocess(frame_bgr)
    if pre is not None:
        return pre, "cuda-preprocess"
    return pipe.preprocess(frame_bgr), "opencv-cpu"


def _warm_cuda_preprocess():
    if not CUDA_PREPROCESS_ENABLED or _cuda_device_count() <= 0:
        return
    started_at = time.perf_counter()
    sample = np.zeros((64, 64, 3), dtype=np.uint8)
    pre = _cuda_preprocess(sample)
    elapsed_ms = (time.perf_counter() - started_at) * 1000.0
    if pre is None:
        print(f"CUDA preprocess warmup skipped: {STATE.cuda_error}")
    else:
        print(f"CUDA preprocess warmup complete in {elapsed_ms:.1f} ms")


def _draw_surface_history_graph(width, height, history):
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
    y_max = max(1.0, float(np.max(valid)) * 1.15) if valid.size else 1.0

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
    return graph


def _build_surface_render(gray_frame, height_map, roi_mask=None, history=None):
    out_h, out_w = gray_frame.shape[:2]
    show_graph = history is not None
    graph_h = max(72, min(128, int(out_h * 0.18))) if show_graph and out_h > 220 else 0
    camera_h = max(96, int(out_h * (0.42 if show_graph else 0.58)))
    min_plot_h = max(72, int(out_h * (0.26 if show_graph else 0.18)))
    if camera_h + graph_h + min_plot_h > out_h:
        camera_h = max(1, out_h - graph_h - min_plot_h)
    camera_h = min(camera_h, max(1, out_h - graph_h - 1))
    plot_h = max(1, out_h - camera_h - graph_h)

    camera_canvas = cv2.cvtColor(gray_frame, cv2.COLOR_GRAY2BGR)
    camera_canvas = cv2.convertScaleAbs(camera_canvas, alpha=0.78, beta=0)
    camera_view = cv2.resize(camera_canvas, (out_w, camera_h), interpolation=cv2.INTER_AREA)
    output = np.zeros((out_h, out_w, 3), dtype=np.uint8)
    output[:camera_h, :, :] = camera_view
    cv2.line(output, (0, camera_h - 1), (out_w - 1, camera_h - 1), (45, 73, 96), 1)

    plot_canvas = np.zeros((plot_h, out_w, 3), dtype=np.uint8)
    plot_canvas[:, :, :] = (9, 22, 32)
    graph_canvas = _draw_surface_history_graph(out_w, graph_h, history) if graph_h > 0 else None
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

    height = cv2.resize(height_work, (out_w, plot_h), interpolation=cv2.INTER_AREA).astype(np.float32)
    plot_roi_binary = None
    if roi_binary is not None:
        plot_roi_binary = cv2.resize(
            roi_binary.astype(np.uint8),
            (out_w, plot_h),
            interpolation=cv2.INTER_NEAREST,
        ).astype(bool)
    height = cv2.GaussianBlur(height, (0, 0), sigmaX=1.6, sigmaY=1.4)
    if plot_roi_binary is not None:
        soft_mask = cv2.GaussianBlur(plot_roi_binary.astype(np.float32), (0, 0), sigmaX=2.0, sigmaY=2.0)
        height = height * np.clip(soft_mask, 0.0, 1.0)
    height = np.clip(height, 0.0, 1.0)

    color_map = cv2.applyColorMap((height * 255.0).astype(np.uint8), cv2.COLORMAP_TURBO)
    dx = cv2.Sobel(height, cv2.CV_32F, 1, 0, ksize=3)
    dy = cv2.Sobel(height, cv2.CV_32F, 0, 1, ksize=3)
    nx = -dx * 1.5
    ny = -dy * 1.5
    nz = 1.0
    norm = np.sqrt(nx * nx + ny * ny + nz * nz)
    norm = np.where(norm < 1e-6, 1.0, norm)
    shade = np.clip((nx * 0.25 + ny * -0.35 + nz) / norm, 0.0, 1.0)
    shade = 0.35 + 0.65 * shade

    if REMOTE_SURFACE_RENDER_MODE != "mesh":
        bg_color = np.array([9, 22, 32], dtype=np.uint8)
        fast_panel = np.zeros((plot_h, out_w, 3), dtype=np.uint8)
        fast_panel[:, :, :] = bg_color
        shaded_color = np.clip(color_map.astype(np.float32) * shade[..., None], 0, 255).astype(np.uint8)
        if plot_roi_binary is not None:
            active_mask = plot_roi_binary & (height > 0.015)
            outline_mask = plot_roi_binary.astype(np.uint8) * 255
            contours, _ = cv2.findContours(outline_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(fast_panel, contours, -1, (58, 95, 112), 1, lineType=cv2.LINE_AA)
        else:
            active_mask = height > 0.015
        fast_panel = np.where(active_mask[..., None], shaded_color, fast_panel)
        contour = ((height * 8.0).astype(np.int32) % 2) == 0
        edge = contour & (height > 0.04)
        if np.any(edge):
            fast_panel[edge] = cv2.addWeighted(
                fast_panel[edge],
                0.72,
                np.full_like(fast_panel[edge], (255, 255, 255)),
                0.28,
                0,
            )
        cv2.putText(
            fast_panel,
            "bubble deformation",
            (12, 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (255, 230, 80),
            1,
        )
        if not np.any(active_mask):
            cv2.putText(
                fast_panel,
                "no contact deformation",
                (12, 44),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                (130, 170, 186),
                1,
                cv2.LINE_AA,
            )
        output[camera_h:camera_h + plot_h, :, :] = fast_panel
        if graph_canvas is not None:
            cv2.line(output, (0, camera_h + plot_h - 1), (out_w - 1, camera_h + plot_h - 1), (45, 73, 96), 1)
            output[camera_h + plot_h:, :, :] = graph_canvas
        return output

    step = max(12, min(24, out_w // 45))
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
        cv2.fillConvexPoly(plot_canvas, polygon, color, lineType=cv2.LINE_AA)

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


def _query_float(query, name, default):
    try:
        return float(query.get(name, [default])[0])
    except (TypeError, ValueError):
        return default


def _query_int(query, name, default):
    try:
        return int(float(query.get(name, [default])[0]))
    except (TypeError, ValueError):
        return default


def _query_bool(query, name, default=False):
    value = str(query.get(name, [str(int(default))])[0]).strip().lower()
    return value in {"1", "true", "yes", "on"}


def _surface_config(query):
    return TactileContactConfig(
        bubble_width_mm=BUBBLE_WIDTH_MM,
        bubble_height_mm=BUBBLE_HEIGHT_MM,
        bubble_max_height_mm=BUBBLE_MAX_HEIGHT_MM,
        grid_width=int(np.clip(_query_int(query, "surface_grid", 52), 24, 140)),
        roi_scale=float(np.clip(_query_float(query, "roi_scale", 1.0), 0.1, 1.0)),
        use_roi=_query_bool(query, "use_roi", True),
        gain=float(np.clip(_query_float(query, "surface_gain", 1.0), 0.2, 3.0)),
        smooth=float(np.clip(_query_float(query, "surface_smooth", 1.4), 0.0, 4.0)),
    )


def _surface_payload_from_stats(stats):
    stats = stats or {}
    center = stats.get("center")
    return {
        "contact_ready": bool(stats.get("ready")),
        "contact_peak": float(stats.get("peak", 0.0) or 0.0),
        "contact_top_mean": float(stats.get("top_mean", 0.0) or 0.0),
        "contact_area_ratio": float(stats.get("area_ratio", 0.0) or 0.0),
        "contact_center": None if center is None else [float(center[0]), float(center[1])],
    }


def _process_surface_preview(frame_bgr, reference, displacements, query):
    reset_zero = _query_bool(query, "surface_reset", False)
    config = _surface_config(query)
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    frame = build_tactile_contact_frame(
        reference,
        displacements,
        frame_bgr.shape,
        config,
        cv2=cv2,
        previous_scale_px=STATE.surface_scale_ema,
    )

    if reset_zero:
        STATE.surface_zero_pending = True
        STATE.surface_zero_ready = False
        STATE.surface_zero_samples = []
        STATE.surface_contact_baseline = None
        STATE.surface_contact_display_ema = None
        STATE.surface_history = []

    if frame is None:
        preview = _build_surface_render(gray, None, history=STATE.surface_history)
        return {
            "preview": preview,
            "payload": {
                "surface_rendered": True,
                "surface_status": "Remote surface waiting for dots.",
                "surface_zero_pending": bool(STATE.surface_zero_pending),
                "surface_zero_ready": bool(STATE.surface_zero_ready),
                "surface_zero_count": len(STATE.surface_zero_samples),
                "surface_scale_px": 0.0,
                "residual_floor_px": 0.0,
                "residual_peak_px": 0.0,
                "residual_mean_px": 0.0,
                **_surface_payload_from_stats(None),
            },
        }

    STATE.surface_scale_ema = frame.scale_px
    raw_contact_map = frame.contact_map.astype(np.float32)
    base_map = frame.base_map.astype(np.float32)
    ellipse_mask = frame.ellipse_mask

    if STATE.surface_zero_pending:
        STATE.surface_zero_samples.append(raw_contact_map.copy())
        if len(STATE.surface_zero_samples) >= CONTACT_ZERO_SAMPLE_COUNT:
            STATE.surface_contact_baseline = np.median(
                np.stack(STATE.surface_zero_samples[-CONTACT_ZERO_SAMPLE_COUNT:], axis=0),
                axis=0,
            ).astype(np.float32)
            STATE.surface_contact_display_ema = np.zeros_like(raw_contact_map, dtype=np.float32)
            STATE.surface_zero_samples = []
            STATE.surface_zero_pending = False
            STATE.surface_zero_ready = True
        contact_map_for_display = np.zeros_like(raw_contact_map, dtype=np.float32)
    elif (
        STATE.surface_zero_ready
        and STATE.surface_contact_baseline is not None
        and STATE.surface_contact_baseline.shape == raw_contact_map.shape
    ):
        contact_map_for_display = np.maximum(
            raw_contact_map - STATE.surface_contact_baseline,
            0.0,
        ).astype(np.float32)
        if (
            STATE.surface_contact_display_ema is None
            or STATE.surface_contact_display_ema.shape != contact_map_for_display.shape
        ):
            STATE.surface_contact_display_ema = contact_map_for_display
        else:
            STATE.surface_contact_display_ema = cv2.addWeighted(
                STATE.surface_contact_display_ema.astype(np.float32),
                0.68,
                contact_map_for_display,
                0.32,
                0.0,
            )
        contact_map_for_display = STATE.surface_contact_display_ema
    else:
        contact_map_for_display = np.zeros_like(raw_contact_map, dtype=np.float32)

    height_map = surface_from_contact_map(
        contact_map_for_display,
        base_map,
        ellipse_mask,
        config,
        cv2=cv2,
    )
    deformation_map = np.clip(base_map.astype(np.float32) - height_map.astype(np.float32), 0.0, 1.0)
    deformation_map = np.where(ellipse_mask, deformation_map, 0.0).astype(np.float32)
    stats = summarize_contact_map(
        contact_map_for_display,
        ellipse_mask,
        config.contact_area_threshold,
    )
    if stats is not None and stats.get("ready"):
        STATE.surface_history.append((float(stats["peak"]), float(stats["top_mean"])))
    else:
        STATE.surface_history.append((None, None))
    if len(STATE.surface_history) > 180:
        del STATE.surface_history[:-180]

    preview = _build_surface_render(gray, deformation_map, roi_mask=ellipse_mask, history=STATE.surface_history)
    if STATE.surface_zero_pending:
        status = f"Remote contact zeroing {len(STATE.surface_zero_samples)}/{CONTACT_ZERO_SAMPLE_COUNT}..."
    elif STATE.surface_zero_ready:
        status = "Remote surface running."
    else:
        status = "Press Reset Zero to set contact deform zero."

    payload = {
        "surface_rendered": True,
        "surface_status": status,
        "surface_zero_accepted": bool(reset_zero),
        "surface_zero_pending": bool(STATE.surface_zero_pending),
        "surface_zero_ready": bool(STATE.surface_zero_ready),
        "surface_zero_count": len(STATE.surface_zero_samples),
        "surface_scale_px": float(frame.scale_px),
        "residual_floor_px": float(frame.residual_floor_px),
        "residual_peak_px": float(frame.residual_peak_px),
        "residual_mean_px": float(frame.residual_mean_px),
        **_surface_payload_from_stats(stats),
    }

    overlay_lines = ["Remote CUDA surface", status]
    if payload["contact_ready"]:
        overlay_lines.append(
            f"Contact peak {payload['contact_peak'] * 100.0:.0f}% | "
            f"top {payload['contact_top_mean'] * 100.0:.0f}% | "
            f"area {payload['contact_area_ratio']:.1f}%"
        )
    else:
        overlay_lines.append(f"Disp scale {frame.scale_px:.3f}px | residual {frame.residual_mean_px:.3f}px")

    for line_index, overlay_line in enumerate(overlay_lines):
        cv2.putText(
            preview,
            overlay_line,
            (12, 50 + (line_index * 20)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.54,
            (250, 245, 85),
            2,
        )
    return {"preview": preview, "payload": payload}


def _detect_centroids(frame_bgr, query):
    polarity = str(query.get("mode", ["auto"])[0]).strip().lower()
    min_area = _query_int(query, "min_area", pipe.MIN_BLOB_AREA)
    max_area = _query_int(query, "max_area", pipe.MAX_BLOB_AREA)
    min_circularity = _query_float(query, "min_circularity", pipe.MIN_CIRCULARITY)
    match_dist = _query_float(query, "match_dist", 9.0)
    use_roi = _query_bool(query, "use_roi", True)
    roi_scale = float(np.clip(_query_float(query, "roi_scale", 0.68), 0.1, 1.0))

    pre, process_backend = _preprocess(frame_bgr)
    binary = pipe.threshold_image(pre, polarity=polarity)

    if use_roi:
        height, width = binary.shape[:2]
        roi_mask = np.zeros((height, width), dtype=np.uint8)
        cx, cy = width // 2, height // 2
        rx = max(10, int((width * roi_scale) * 0.5))
        ry = max(10, int((height * roi_scale) * 0.5))
        cv2.ellipse(roi_mask, (cx, cy), (rx, ry), 0, 0, 360, 255, -1)
        binary = cv2.bitwise_and(binary, roi_mask)

    centroids = pipe.detect_centroids_from_binary(
        binary,
        min_area=min_area,
        max_area=max_area,
        min_circularity=min_circularity,
    )
    if not centroids:
        centroids = pipe.extract_centroids(pipe.detect_blobs(binary))

    return binary, centroids, match_dist, process_backend


def _process_frame(frame_bgr, query):
    reset_reference = _query_bool(query, "reset", False)
    include_preview = _query_bool(query, "preview", False)
    include_points = _query_bool(query, "points", False)
    view_type = str(query.get("view", ["overlay"])[0]).strip().lower()

    started_at = time.perf_counter()
    binary, centroids, match_dist, process_backend = _detect_centroids(frame_bgr, query)

    if reset_reference or not STATE.reference_centroids:
        STATE.reference_centroids = list(centroids)
        STATE.reset_surface()
        STATE.started_at = time.perf_counter()

    reference = STATE.reference_centroids or []
    displacements, missing = pipe.compute_displacements(reference, centroids, match_dist)
    mags = [
        float(np.hypot(dx, dy))
        for disp in displacements
        if disp is not None
        for dx, dy in (disp,)
    ]

    mean_disp = float(np.mean(mags)) if mags else 0.0
    max_disp = float(np.max(mags)) if mags else 0.0
    missing_ratio = float(len(missing) / max(1, len(reference)))

    STATE.frame_count += 1
    STATE.last_frame_shape = frame_bgr.shape[:2]
    STATE.last_frame_at = time.perf_counter()
    STATE.last_process_ms = (STATE.last_frame_at - started_at) * 1000.0
    STATE.last_process_backend = process_backend
    elapsed = max(1e-6, time.perf_counter() - STATE.started_at)

    payload = {
        "ok": True,
        "backend": SERVICE_BACKEND,
        "processor_backend": pipe.get_detection_backend_name(),
        "process_backend": process_backend,
        "process_ms": STATE.last_process_ms,
        "cuda_device_count": _cuda_device_count(),
        "cuda_preprocess_enabled": CUDA_PREPROCESS_ENABLED,
        "frame_count": STATE.frame_count,
        "fps_est": STATE.frame_count / elapsed,
        "dot_count": len(centroids),
        "reference_count": len(reference),
        "mean_disp": mean_disp,
        "max_disp": max_disp,
        "missing_ratio": missing_ratio,
        "new_tracks": 0,
        "lost_tracks": len(missing),
        "height": int(frame_bgr.shape[0]),
        "width": int(frame_bgr.shape[1]),
        "reference_reset": bool(reset_reference),
    }

    if include_points:
        payload["centroids"] = [[int(x), int(y)] for x, y in centroids]
        payload["reference_centroids"] = [[int(x), int(y)] for x, y in reference]
        payload["displacements"] = [
            None if disp is None else [float(disp[0]), float(disp[1])]
            for disp in displacements
        ]

    if include_preview:
        if view_type in {"surface_3d", "optical_flow_3d"}:
            surface_result = _process_surface_preview(frame_bgr, reference, displacements, query)
            payload.update(surface_result["payload"])
            preview_bgr = surface_result["preview"]
        elif view_type == "pointcloud":
            preview_bgr = pipe.build_pointcloud_view(frame_bgr.shape, reference, displacements)
        elif view_type == "heatmap":
            heatmap = pipe.build_displacement_heatmap(frame_bgr.shape, reference, displacements)
            preview_bgr = pipe.overlay_heatmap(frame_bgr, heatmap)
        else:
            blob_vis = pipe.build_blob_view(binary, frame_bgr)
            preview_bgr = pipe.draw_displacement_vectors(blob_vis, reference, displacements)

        ok, encoded = cv2.imencode(
            ".jpg",
            preview_bgr,
            [int(cv2.IMWRITE_JPEG_QUALITY), int(np.clip(REMOTE_RENDER_JPEG_QUALITY, 35, 92))],
        )
        if ok:
            payload["preview_jpeg_b64"] = base64.b64encode(encoded.tobytes()).decode("ascii")

    total_ms = (time.perf_counter() - started_at) * 1000.0
    payload["total_ms"] = total_ms
    STATE.last_process_ms = total_ms
    return payload


class RemoteVisionHandler(BaseHTTPRequestHandler):
    server_version = "BubbleRemoteVision/0.1"

    def _send_json(self, status_code, payload):
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status_code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path != "/health":
            self._send_json(404, {"ok": False, "error": "unknown endpoint"})
            return
        self._send_json(
            200,
            {
                "ok": True,
                "service": "remote_vision_server",
                "backend": SERVICE_BACKEND,
                "processor_backend": pipe.get_detection_backend_name(),
                "process_backend": STATE.last_process_backend,
                "process_ms": STATE.last_process_ms,
                "cuda_device_count": _cuda_device_count(),
                "cuda_preprocess_enabled": CUDA_PREPROCESS_ENABLED,
                "cuda_error": STATE.cuda_error,
                "frame_count": STATE.frame_count,
                "reference_count": len(STATE.reference_centroids or []),
                "last_frame_age_s": (
                    None
                    if STATE.last_frame_at is None
                    else max(0.0, time.perf_counter() - STATE.last_frame_at)
                ),
            },
        )

    def do_POST(self):
        parsed = urlparse(self.path)
        query = parse_qs(parsed.query)

        if parsed.path == "/reset":
            STATE.reset()
            self._send_json(200, {"ok": True, "reset": True})
            return

        if parsed.path != "/process":
            self._send_json(404, {"ok": False, "error": "unknown endpoint"})
            return

        try:
            content_length = int(self.headers.get("Content-Length", "0"))
            if content_length <= 0:
                raise ValueError("empty request body")
            raw = self.rfile.read(content_length)
            image_data = np.frombuffer(raw, dtype=np.uint8)
            frame_bgr = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
            if frame_bgr is None:
                raise ValueError("request body is not a valid JPEG/PNG image")
            payload = _process_frame(frame_bgr, query)
            self._send_json(200, payload)
        except Exception as exc:
            self._send_json(500, {"ok": False, "error": str(exc)})

    def log_message(self, fmt, *args):
        sys.stderr.write("[%s] %s\n" % (self.log_date_time_string(), fmt % args))


def main():
    parser = argparse.ArgumentParser(description="Temporary CUDA remote vision server.")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()

    server = ThreadingHTTPServer((args.host, args.port), RemoteVisionHandler)
    print(f"Remote vision server listening on http://{args.host}:{args.port}")
    print("Health check: GET /health")
    print("Process frame: POST JPEG bytes to /process")
    _warm_cuda_preprocess()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nRemote vision server stopped.")


if __name__ == "__main__":
    main()
