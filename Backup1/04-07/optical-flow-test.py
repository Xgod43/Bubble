import argparse
import csv
import math
import os
import sys
import time
from datetime import datetime

import cv2
import numpy as np
from picamera2 import Picamera2

try:
	from libcamera import controls as libcamera_controls
except Exception:
	libcamera_controls = None


FLOW_WINDOW = "Optical Flow"
MASK_WINDOW = "Dot Mask"
DEPTH_WINDOW = "Depth Graph"
DEPTH3D_WINDOW = "Depth Graph 3D"

LK_PARAMS = dict(
	winSize=(21, 21),
	maxLevel=3,
	criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.03),
)


def build_parser() -> argparse.ArgumentParser:
	parser = argparse.ArgumentParser(description="Realtime dot-pattern optical flow for Pi Camera v3")
	parser.add_argument("--width", type=int, default=1280, help="Preview width")
	parser.add_argument("--height", type=int, default=720, help="Preview height")
	parser.add_argument(
		"--window-scale",
		type=float,
		default=0.62,
		help="Display scale for OpenCV windows (smaller makes all windows more compact)",
	)
	parser.add_argument("--fps", type=float, default=60.0, help="Requested camera frame rate")
	parser.add_argument("--roi-scale", type=float, default=0.96, help="Ellipse ROI scale in [0.1, 1.0]")
	parser.add_argument("--max-points", type=int, default=260, help="Max points kept after reseed")
	parser.add_argument("--min-track-points", type=int, default=30, help="Force reseed below this point count")
	parser.add_argument("--reseed-interval", type=int, default=90, help="Periodic reseed interval in frames, 0 disables")
	parser.add_argument("--min-area", type=float, default=15.0, help="Minimum dot blob area")
	parser.add_argument("--max-area", type=float, default=3000.0, help="Maximum dot blob area")
	parser.add_argument("--min-circularity", type=float, default=0.40, help="Minimum contour circularity")
	parser.add_argument("--fb-thresh", type=float, default=1.7, help="Forward-backward flow error threshold")
	parser.add_argument("--ransac-thresh", type=float, default=3.2, help="RANSAC reprojection threshold")
	parser.add_argument(
		"--focus-mode",
		type=str,
		default="manual",
		choices=["manual", "continuous", "auto"],
		help="Focus mode: manual lock is recommended for deformation tracking",
	)
	parser.add_argument(
		"--lens-position",
		type=float,
		default=4.5,
		help="Manual lens position (higher is closer focus)",
	)
	parser.add_argument(
		"--focus-step",
		type=float,
		default=0.20,
		help="Lens position step for [ and ] hotkeys",
	)
	parser.add_argument(
		"--focus-sweep-min",
		type=float,
		default=0.0,
		help="Minimum lens position for auto focus sweep",
	)
	parser.add_argument(
		"--focus-sweep-max",
		type=float,
		default=10.0,
		help="Maximum lens position for auto focus sweep",
	)
	parser.add_argument(
		"--focus-sweep-step",
		type=float,
		default=0.25,
		help="Lens step used by auto focus sweep",
	)
	parser.add_argument(
		"--focus-sweep-frames",
		type=int,
		default=2,
		help="Number of frames averaged at each lens position during sweep",
	)
	parser.add_argument(
		"--focus-sweep-settle-ms",
		type=int,
		default=70,
		help="Settle time at each lens position during sweep (ms)",
	)
	parser.add_argument(
		"--focus-sweep-center",
		type=float,
		default=0.60,
		help="Center ROI ratio used for focus scoring during sweep",
	)
	parser.add_argument(
		"--focus-sweep-on-start",
		action="store_true",
		help="Run auto focus sweep once at startup and lock best lens position",
	)
	parser.add_argument(
		"--focus-target-radius",
		type=int,
		default=140,
		help="Radius in pixels for focus target region set by mouse click",
	)
	parser.add_argument(
		"--hotspot-focus-span",
		type=float,
		default=0.9,
		help="Half-range around current lens position used by hotspot autofocus (key h)",
	)
	parser.add_argument(
		"--hotspot-confirm-frames",
		type=int,
		default=3,
		help="Consecutive frames required to confirm hotspot marker",
	)
	parser.add_argument(
		"--hotspot-sigma-thresh",
		type=float,
		default=3.0,
		help="Hotspot outlier threshold in robust sigma units",
	)
	parser.add_argument(
		"--hotspot-roi-guard",
		type=float,
		default=0.92,
		help="Keep hotspot away from ellipse edge (normalized radius^2 limit)",
	)
	parser.add_argument(
		"--hotspot-ema",
		type=float,
		default=0.65,
		help="EMA smoothing factor for hotspot marker position",
	)
	parser.add_argument(
		"--auto-hotspot-focus",
		action="store_true",
		help="Auto-run hotspot focus sweep when deformation max reaches a new peak",
	)
	parser.add_argument(
		"--auto-hotspot-thresh-mm",
		type=float,
		default=0.35,
		help="Minimum local max deformation (mm) to allow auto hotspot focus",
	)
	parser.add_argument(
		"--auto-hotspot-min-rise-mm",
		type=float,
		default=0.08,
		help="Required rise over last auto trigger peak (mm)",
	)
	parser.add_argument(
		"--auto-hotspot-cooldown",
		type=int,
		default=80,
		help="Minimum frame gap between auto hotspot focus sweeps",
	)
	parser.add_argument(
		"--exposure-us",
		type=int,
		default=0,
		help="Manual exposure time in microseconds, 0 means auto",
	)
	parser.add_argument(
		"--analogue-gain",
		type=float,
		default=0.0,
		help="Manual analogue gain, 0 means auto",
	)
	parser.add_argument(
		"--nn-distance-mm",
		type=float,
		default=7.071067811865,
		help="Known nearest-neighbor dot spacing in mm (ScreenType1 ~= 7.07 mm)",
	)
	parser.add_argument(
		"--auto-grid-calib",
		action="store_true",
		help="Auto-update mm/px from detected nearest-neighbor spacing on reseed",
	)
	parser.add_argument(
		"--calib-distance-mm",
		type=float,
		default=10.0,
		help="Known movement distance used by calibration key c (mm)",
	)
	parser.add_argument(
		"--px-to-mm",
		type=float,
		default=1.0,
		help="Scale factor to convert px to mm (set from your calibration)",
	)
	parser.add_argument(
		"--depth-graph-height",
		type=int,
		default=280,
		help="Depth graph window height in pixels",
	)
	parser.add_argument(
		"--depth-graph-max-mm",
		type=float,
		default=0.0,
		help="Fixed Y-axis max for depth graph in mm, 0 = auto",
	)
	parser.add_argument(
		"--depth-graph-3d-height",
		type=int,
		default=360,
		help="Depth 3D graph window height in pixels",
	)
	parser.add_argument(
		"--depth-graph-3d-z-scale",
		type=float,
		default=1.0,
		help="Vertical exaggeration for 3D depth graph",
	)
	parser.add_argument(
		"--depth-graph-3d-grid-x",
		type=int,
		default=24,
		help="Grid resolution X for 3D depth graph",
	)
	parser.add_argument(
		"--depth-graph-3d-grid-y",
		type=int,
		default=14,
		help="Grid resolution Y for 3D depth graph",
	)
	parser.add_argument(
		"--bubble-major-mm",
		type=float,
		default=110.0,
		help="Real bubble major axis in mm",
	)
	parser.add_argument(
		"--bubble-minor-mm",
		type=float,
		default=70.0,
		help="Real bubble minor axis in mm",
	)
	parser.add_argument(
		"--bubble-depth-mm",
		type=float,
		default=35.0,
		help="Real bubble max depth in mm",
	)
	parser.add_argument(
		"--depth-graph-3d-show-reference",
		action="store_true",
		help="Overlay reference dome from bubble geometry on 3D graph",
	)
	parser.add_argument(
		"--depth-graph-3d-reference-shape",
		type=str,
		default="ellipsoid",
		choices=["ellipsoid", "paraboloid"],
		help="Reference dome shape for 3D graph",
	)
	parser.add_argument(
		"--auto-origin",
		action=argparse.BooleanOptionalAction,
		default=True,
		help="Automatically set origin after stable tracking window",
	)
	parser.add_argument(
		"--auto-origin-stable-frames",
		type=int,
		default=18,
		help="Number of consecutive stable frames required before auto origin set",
	)
	parser.add_argument(
		"--auto-origin-min-points",
		type=int,
		default=48,
		help="Minimum active points required for auto origin",
	)
	parser.add_argument(
		"--auto-origin-max-frame-px",
		type=float,
		default=0.75,
		help="Maximum frame translation in px for stability check",
	)
	parser.add_argument(
		"--auto-origin-max-frame-mm",
		type=float,
		default=0.08,
		help="Maximum frame translation in mm for stability check when scale is calibrated",
	)
	parser.add_argument(
		"--auto-origin-max-rot-deg",
		type=float,
		default=0.28,
		help="Maximum frame rotation in degrees for stability check",
	)
	parser.add_argument(
		"--auto-origin-cooldown",
		type=int,
		default=140,
		help="Minimum frame gap between automatic origin attempts",
	)
	parser.add_argument(
		"--auto-origin-require-calibrated",
		action=argparse.BooleanOptionalAction,
		default=False,
		help="Require calibrated mm scale before auto origin can trigger",
	)
	parser.add_argument("--draw-points", type=int, default=140, help="Maximum vectors drawn per frame")
	parser.add_argument(
		"--export-fullfield",
		action="store_true",
		help="Enable per-point CSV export (mode B)",
	)
	parser.add_argument(
		"--export-every",
		type=int,
		default=2,
		help="Write per-point CSV every N frames",
	)
	parser.add_argument(
		"--export-dir",
		type=str,
		default="logs",
		help="Directory for per-point CSV export",
	)
	parser.add_argument(
		"--export-prefix",
		type=str,
		default="flow_points",
		help="Prefix for per-point CSV file",
	)
	return parser


def clamp(value: float, lo: float, hi: float) -> float:
	return max(lo, min(hi, value))


def build_roi_mask(shape, roi_scale: float) -> np.ndarray:
	h, w = shape[:2]
	roi_scale = clamp(roi_scale, 0.1, 1.0)
	mask = np.zeros((h, w), dtype=np.uint8)
	center = (w // 2, h // 2)
	axes = (max(10, int(w * roi_scale * 0.5)), max(10, int(h * roi_scale * 0.5)))
	cv2.ellipse(mask, center, axes, 0, 0, 360, 255, -1)
	return mask


def preprocess_gray(frame_rgb: np.ndarray, clahe: cv2.CLAHE) -> np.ndarray:
	gray = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY)
	gray = cv2.GaussianBlur(gray, (3, 3), 0)
	return clahe.apply(gray)


def choose_dot_mask(gray: np.ndarray) -> np.ndarray:
	blur = cv2.GaussianBlur(gray, (5, 5), 0)
	_ret_bright, bright = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
	_ret_dark, dark = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

	def score(mask: np.ndarray) -> float:
		ratio = float(np.count_nonzero(mask)) / float(mask.size)
		if ratio < 0.002 or ratio > 0.70:
			return 10.0 + ratio
		return abs(ratio - 0.12)

	chosen = bright if score(bright) <= score(dark) else dark
	kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
	return cv2.morphologyEx(chosen, cv2.MORPH_OPEN, kernel, iterations=1)


def detect_dot_centers(
	gray: np.ndarray,
	roi_mask: np.ndarray,
	min_area: float,
	max_area: float,
	min_circularity: float,
	max_points: int,
):
	binary = choose_dot_mask(gray)
	if roi_mask is not None:
		binary = cv2.bitwise_and(binary, roi_mask)

	contours, _hier = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	centers = []
	for cnt in contours:
		area = cv2.contourArea(cnt)
		if area < min_area or area > max_area:
			continue

		perimeter = cv2.arcLength(cnt, True)
		if perimeter <= 1e-6:
			continue

		circularity = 4.0 * np.pi * area / (perimeter * perimeter)
		if circularity < min_circularity:
			continue

		moments = cv2.moments(cnt)
		if abs(moments["m00"]) <= 1e-6:
			continue

		cx = float(moments["m10"] / moments["m00"])
		cy = float(moments["m01"] / moments["m00"])
		centers.append((cx, cy))

	centers.sort(key=lambda p: (p[1], p[0]))
	detected_count = len(centers)

	if detected_count > max_points:
		pick_idx = np.linspace(0, detected_count - 1, max_points, dtype=np.int32)
		centers = [centers[int(i)] for i in pick_idx]

	if not centers:
		return None, binary, detected_count

	points = np.array(centers, dtype=np.float32).reshape(-1, 1, 2)
	return points, binary, detected_count


def open_camera(
	width: int,
	height: int,
	fps: float,
	focus_mode: str,
	lens_position: float,
	exposure_us: int,
	analogue_gain: float,
) -> Picamera2:
	cam = Picamera2()
	config = cam.create_preview_configuration(main={"size": (width, height), "format": "RGB888"})
	cam.configure(config)
	cam.start()

	controls_payload = {"FrameRate": float(max(1.0, fps))}
	if exposure_us > 0:
		controls_payload["ExposureTime"] = int(exposure_us)
	if analogue_gain > 0:
		controls_payload["AnalogueGain"] = float(analogue_gain)

	if libcamera_controls is not None:
		if focus_mode == "manual":
			controls_payload["AfMode"] = libcamera_controls.AfModeEnum.Manual
			controls_payload["LensPosition"] = float(max(0.0, lens_position))
		elif focus_mode == "auto":
			controls_payload["AfMode"] = libcamera_controls.AfModeEnum.Auto
		else:
			controls_payload["AfMode"] = libcamera_controls.AfModeEnum.Continuous
	else:
		mode_map = {"manual": 0, "auto": 1, "continuous": 2}
		controls_payload["AfMode"] = mode_map.get(focus_mode, 0)
		if focus_mode == "manual":
			controls_payload["LensPosition"] = float(max(0.0, lens_position))
	try:
		cam.set_controls(controls_payload)
	except Exception:
		pass

	time.sleep(0.25)
	return cam


def read_lens_position(cam: Picamera2, fallback: float) -> float:
	try:
		md = cam.capture_metadata()
		lp = md.get("LensPosition")
		if lp is None:
			return fallback
		return float(lp)
	except Exception:
		return fallback


def set_manual_focus(cam: Picamera2, lens_position: float) -> float:
	new_pos = float(max(0.0, lens_position))
	payload = {"LensPosition": new_pos}
	if libcamera_controls is not None:
		payload["AfMode"] = libcamera_controls.AfModeEnum.Manual
	else:
		payload["AfMode"] = 0
	try:
		cam.set_controls(payload)
	except Exception:
		pass
	return new_pos


def set_continuous_focus(cam: Picamera2) -> None:
	payload = {}
	if libcamera_controls is not None:
		payload["AfMode"] = libcamera_controls.AfModeEnum.Continuous
	else:
		payload["AfMode"] = 2
	try:
		cam.set_controls(payload)
	except Exception:
		pass


def trigger_autofocus_and_lock(cam: Picamera2, fallback_lens: float) -> float:
	if libcamera_controls is None:
		return fallback_lens
	try:
		cam.set_controls({"AfMode": libcamera_controls.AfModeEnum.Auto})
		trigger_enum = getattr(libcamera_controls, "AfTriggerEnum", None)
		if trigger_enum is not None:
			cam.set_controls({"AfTrigger": trigger_enum.Start})
		time.sleep(0.35)
		best_lp = read_lens_position(cam, fallback_lens)
		return set_manual_focus(cam, best_lp)
	except Exception:
		return fallback_lens


def compute_focus_score(gray: np.ndarray) -> float:
	return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def build_focus_target_mask(shape, focus_target: dict | None) -> np.ndarray | None:
	if focus_target is None:
		return None
	x = focus_target.get("x")
	y = focus_target.get("y")
	r = int(max(10, focus_target.get("r", 100)))
	if x is None or y is None:
		return None

	h, w = shape[:2]
	cx = int(clamp(int(x), 0, w - 1))
	cy = int(clamp(int(y), 0, h - 1))
	mask = np.zeros((h, w), dtype=np.uint8)
	cv2.circle(mask, (cx, cy), r, 255, -1)
	return mask


def compute_focus_score_from_rgb(frame_rgb: np.ndarray, center_ratio: float, focus_target: dict | None = None) -> float:
	gray = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY)
	ratio = clamp(center_ratio, 0.2, 1.0)
	center_mask = build_roi_mask(gray.shape, ratio)
	target_mask = build_focus_target_mask(gray.shape, focus_target)
	base_mask = target_mask if target_mask is not None else center_mask
	tone_mask = cv2.inRange(gray, 25, 235)
	valid_mask = cv2.bitwise_and(base_mask, tone_mask)

	# Prefer sharpness where the dot pattern exists, not bright glare regions.
	dot_mask = choose_dot_mask(gray)
	dot_mask = cv2.bitwise_and(dot_mask, base_mask)

	lap_abs = np.abs(cv2.Laplacian(gray, cv2.CV_64F))

	def robust_score(mask: np.ndarray) -> float | None:
		vals = lap_abs[mask > 0]
		if vals.size < 200:
			return None
		return float(np.percentile(vals, 92.0))

	s_valid = robust_score(valid_mask)
	s_dot = robust_score(dot_mask)

	if s_dot is not None and s_valid is not None:
		return 0.70 * s_dot + 0.30 * s_valid
	if s_dot is not None:
		return s_dot
	if s_valid is not None:
		return s_valid
	return compute_focus_score(gray)


def run_focus_sweep(
	cam: Picamera2,
	lens_min: float,
	lens_max: float,
	lens_step: float,
	frames_per_pos: int,
	settle_ms: int,
	center_ratio: float,
	focus_target: dict | None = None,
) -> tuple[float, float, list[tuple[float, float]]]:
	lo = max(0.0, float(min(lens_min, lens_max)))
	hi = max(0.0, float(max(lens_min, lens_max)))
	step = max(0.02, float(abs(lens_step)))
	frames_n = max(1, int(frames_per_pos))
	wait_s = max(0.0, float(settle_ms) / 1000.0)

	positions = np.arange(lo, hi + 0.5 * step, step, dtype=np.float64)
	best_pos = float(lo)
	best_score = -1.0
	results = []

	for pos in positions:
		cur = set_manual_focus(cam, float(pos))
		if wait_s > 0:
			time.sleep(wait_s)

		score_sum = 0.0
		for _i in range(frames_n):
			fr = cam.capture_array()
			score_sum += compute_focus_score_from_rgb(fr, center_ratio, focus_target)

		score_avg = score_sum / float(frames_n)
		results.append((float(cur), float(score_avg)))
		if score_avg > best_score:
			best_score = float(score_avg)
			best_pos = float(cur)

	best_pos = set_manual_focus(cam, best_pos)
	return best_pos, best_score, results


def run_hotspot_focus_sweep(
	cam: Picamera2,
	lens_position: float,
	hotspot_xy: tuple[int, int],
	hotspot_id: int | None,
	focus_target: dict,
	sweep_min: float,
	sweep_max: float,
	sweep_step: float,
	sweep_frames: int,
	sweep_settle_ms: int,
	sweep_center: float,
	hotspot_focus_span: float,
	trigger_label: str,
) -> tuple[float, str, bool]:
	# วิธีใช้: ฟังก์ชันนี้ทำงานเหมือนกดปุ่ม h โดยจะย้าย focus target ไปที่ hotspot แล้ว sweep หาเลนส์ที่คมที่สุด
	hx, hy = hotspot_xy
	focus_target["x"] = int(hx)
	focus_target["y"] = int(hy)

	span = max(0.10, float(hotspot_focus_span))
	sweep_lo = max(float(sweep_min), float(lens_position - span))
	sweep_hi = min(float(sweep_max), float(lens_position + span))
	if sweep_hi - sweep_lo < max(0.02, float(sweep_step)):
		sweep_lo = float(sweep_min)
		sweep_hi = float(sweep_max)

	print(
		f"{trigger_label} hotspot focus sweep: "
		f"id={hotspot_id} at {hx},{hy} range=({sweep_lo:.2f},{sweep_hi:.2f})"
	)
	lens_position, sweep_score, sweep_results = run_focus_sweep(
		cam,
		sweep_lo,
		sweep_hi,
		sweep_step,
		sweep_frames,
		sweep_settle_ms,
		sweep_center,
		focus_target,
	)
	focus_note = (
		f"hotspot id={hotspot_id} lens={lens_position:.2f} "
		f"sharp={sweep_score:.1f} n={len(sweep_results)}"
	)

	edge_range = False
	if len(sweep_results) >= 2:
		sweep_scores = np.array([s for _lp, s in sweep_results], dtype=np.float64)
		best_i = int(np.argmax(sweep_scores))
		if best_i == 0 or best_i == len(sweep_results) - 1:
			edge_range = True
			focus_note += " edge-range"

	return lens_position, focus_note, edge_range


def make_focus_mouse_callback(focus_target: dict):
	def _callback(event, x, y, _flags, _param):
		if event == cv2.EVENT_LBUTTONDOWN:
			focus_target["x"] = int(x)
			focus_target["y"] = int(y)
		if event == cv2.EVENT_RBUTTONDOWN:
			focus_target["x"] = None
			focus_target["y"] = None

	return _callback


def put_lines(frame: np.ndarray, lines) -> None:
	y = 26
	for line in lines:
		cv2.putText(frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 255), 2, cv2.LINE_AA)
		y += 24


def build_depth_graph_image(
	current_xy: np.ndarray | None,
	depth_mm: np.ndarray | None,
	width: int,
	height: int,
	max_mm_hint: float,
) -> np.ndarray:
	# หมายเหตุ: กราฟนี้เป็น "depth proxy" จาก local deformation (ไม่ใช่ความลึกจริงแบบ 3D)
	graph_h = int(max(180, height))
	graph_w = int(max(420, width))
	img = np.full((graph_h, graph_w, 3), 18, dtype=np.uint8)

	left = 56
	right = 16
	top = 26
	bottom = 34
	plot_w = max(10, graph_w - left - right)
	plot_h = max(10, graph_h - top - bottom)

	cv2.rectangle(img, (left, top), (left + plot_w, top + plot_h), (60, 60, 60), 1, cv2.LINE_AA)
	cv2.putText(
		img,
		"Depth Graph (proxy from local deformation)",
		(12, 18),
		cv2.FONT_HERSHEY_SIMPLEX,
		0.52,
		(220, 220, 220),
		1,
		cv2.LINE_AA,
	)

	if current_xy is None or depth_mm is None or len(depth_mm) < 2:
		cv2.putText(
			img,
			"Depth proxy unavailable: set origin (o) and keep stable tracked points",
			(12, graph_h - 12),
			cv2.FONT_HERSHEY_SIMPLEX,
			0.48,
			(170, 170, 170),
			1,
			cv2.LINE_AA,
		)
		return img

	depth_vals = depth_mm.astype(np.float64)
	xy_vals = current_xy.astype(np.float64)
	order = np.argsort(xy_vals[:, 0])
	depth_sorted = depth_vals[order]
	if len(depth_sorted) >= 5:
		# Smooth the 1D profile so idle-state noise does not look like sharp zig-zag.
		kernel = np.ones(5, dtype=np.float64) / 5.0
		depth_sorted = np.convolve(depth_sorted, kernel, mode="same")

	auto_max = float(np.percentile(depth_sorted, 98.0))
	auto_max = max(1e-6, auto_max)
	if max_mm_hint > 0:
		y_max = max(float(max_mm_hint), auto_max)
	else:
		y_max = auto_max

	pts = []
	n = len(depth_sorted)
	for i, val in enumerate(depth_sorted):
		x = left + int((plot_w - 1) * i / max(1, n - 1))
		r = float(np.clip(val / y_max, 0.0, 1.0))
		y = top + int((1.0 - r) * (plot_h - 1))
		pts.append((x, y))

	if len(pts) >= 2:
		cv2.polylines(img, [np.array(pts, dtype=np.int32)], False, (70, 220, 255), 2, cv2.LINE_AA)

	step = max(1, len(pts) // 80)
	for i in range(0, len(pts), step):
		cv2.circle(img, pts[i], 2, (0, 180, 255), -1, cv2.LINE_AA)

	mean_v = float(np.mean(depth_sorted))
	p95_v = float(np.percentile(depth_sorted, 95.0))
	max_v = float(np.max(depth_sorted))

	cv2.putText(img, f"{y_max:.3f} mm", (8, top + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (190, 190, 190), 1, cv2.LINE_AA)
	cv2.putText(img, "0", (30, top + plot_h), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (190, 190, 190), 1, cv2.LINE_AA)
	cv2.putText(
		img,
		f"mean={mean_v:.3f}mm p95={p95_v:.3f}mm max={max_v:.3f}mm n={len(depth_sorted)}",
		(12, graph_h - 12),
		cv2.FONT_HERSHEY_SIMPLEX,
		0.50,
		(210, 210, 210),
		1,
		cv2.LINE_AA,
	)

	return img


def estimate_bubble_ellipse_pose(points_xy: np.ndarray | None):
	if points_xy is None or len(points_xy) < 5:
		return None

	try:
		pts = points_xy.astype(np.float32).reshape(-1, 1, 2)
		(center_xy, axes_wh, angle_deg) = cv2.fitEllipse(pts)
	except Exception:
		return None

	cx, cy = float(center_xy[0]), float(center_xy[1])
	w, h = float(axes_wh[0]), float(axes_wh[1])
	if w <= 1e-6 or h <= 1e-6:
		return None

	if w >= h:
		a_px = 0.5 * w
		b_px = 0.5 * h
		theta_deg = angle_deg
	else:
		a_px = 0.5 * h
		b_px = 0.5 * w
		theta_deg = angle_deg + 90.0

	return {
		"cx": cx,
		"cy": cy,
		"a_px": float(max(1e-6, a_px)),
		"b_px": float(max(1e-6, b_px)),
		"theta": float(math.radians(theta_deg)),
	}


def point_xy_to_bubble_uv(points_xy: np.ndarray, bubble_pose: dict) -> np.ndarray:
	dx = points_xy[:, 0] - float(bubble_pose["cx"])
	dy = points_xy[:, 1] - float(bubble_pose["cy"])
	ct = math.cos(float(bubble_pose["theta"]))
	st = math.sin(float(bubble_pose["theta"]))

	xr = ct * dx + st * dy
	yr = -st * dx + ct * dy

	u = xr / float(max(1e-6, bubble_pose["a_px"]))
	v = yr / float(max(1e-6, bubble_pose["b_px"]))
	return np.column_stack([u, v])


def fill_nan_cells(z_grid: np.ndarray, passes: int = 5) -> np.ndarray:
	out = z_grid.copy()
	h, w = out.shape[:2]
	for _ in range(max(1, int(passes))):
		nan_idx = np.argwhere(~np.isfinite(out))
		if len(nan_idx) == 0:
			break

		changed = 0
		for y, x in nan_idx:
			y0 = max(0, int(y) - 1)
			y1 = min(h, int(y) + 2)
			x0 = max(0, int(x) - 1)
			x1 = min(w, int(x) + 2)
			neigh = out[y0:y1, x0:x1]
			vals = neigh[np.isfinite(neigh)]
			if vals.size > 0:
				out[y, x] = float(np.mean(vals))
				changed += 1

		if changed == 0:
			break

	return out


def build_depth_graph_3d_image(
	current_xy: np.ndarray | None,
	depth_mm: np.ndarray | None,
	frame_width: int,
	frame_height: int,
	graph_height: int,
	max_mm_hint: float,
	z_scale: float,
	grid_x: int,
	grid_y: int,
	bubble_pose: dict | None,
	bubble_major_mm: float,
	bubble_minor_mm: float,
	bubble_depth_mm: float,
	show_reference: bool,
	reference_shape: str,
) -> np.ndarray:
	# หมายเหตุ: กราฟนี้เป็น 3D visualization ของ depth proxy จาก deformation (ไม่ใช่ depth จริงจากเซนเซอร์เชิงลึก)
	graph_h = int(max(220, graph_height))
	graph_w = int(max(560, frame_width))
	img = np.full((graph_h, graph_w, 3), 16, dtype=np.uint8)

	left = 14
	right = 14
	top = 30
	bottom = 36
	plot_w = max(20, graph_w - left - right)
	plot_h = max(20, graph_h - top - bottom)

	cv2.putText(
		img,
		"Depth Graph 3D (proxy surface, bubble-scaled)",
		(14, 20),
		cv2.FONT_HERSHEY_SIMPLEX,
		0.56,
		(220, 220, 220),
		1,
		cv2.LINE_AA,
	)

	if current_xy is None or depth_mm is None or len(depth_mm) < 8:
		cv2.putText(
			img,
			"Depth proxy unavailable: set origin (o) and keep stable tracked points",
			(14, graph_h - 12),
			cv2.FONT_HERSHEY_SIMPLEX,
			0.50,
			(170, 170, 170),
			1,
			cv2.LINE_AA,
		)
		return img

	gx = int(max(6, grid_x))
	gy = int(max(4, grid_y))
	b_major = float(max(1e-6, bubble_major_mm))
	b_minor = float(max(1e-6, bubble_minor_mm))
	b_depth = float(max(1e-6, bubble_depth_mm))

	xy = current_xy.astype(np.float64)
	z = depth_mm.astype(np.float64)

	if bubble_pose is not None:
		uv = point_xy_to_bubble_uv(xy, bubble_pose)
	else:
		xn = (xy[:, 0] / float(max(1, frame_width - 1))) * 2.0 - 1.0
		yn = (xy[:, 1] / float(max(1, frame_height - 1))) * 2.0 - 1.0
		uv = np.column_stack([xn, yn])

	inside = np.sum(uv * uv, axis=1) <= 1.25
	if np.count_nonzero(inside) >= 8:
		uv_use = uv[inside]
		z_use = z[inside]
	else:
		uv_use = uv
		z_use = z

	ix = np.clip(np.round((uv_use[:, 0] + 1.0) * 0.5 * (gx - 1)).astype(np.int32), 0, gx - 1)
	iy = np.clip(np.round((uv_use[:, 1] + 1.0) * 0.5 * (gy - 1)).astype(np.int32), 0, gy - 1)

	z_sum = np.zeros((gy, gx), dtype=np.float64)
	z_cnt = np.zeros((gy, gx), dtype=np.int32)
	for k in range(len(z_use)):
		cx = int(ix[k])
		cy = int(iy[k])
		z_sum[cy, cx] += float(z_use[k])
		z_cnt[cy, cx] += 1

	z_mean = np.full((gy, gx), np.nan, dtype=np.float64)
	mask = z_cnt > 0
	z_mean[mask] = z_sum[mask] / z_cnt[mask]
	z_fill = fill_nan_cells(z_mean, passes=6)

	xy_inside = np.zeros((gy, gx), dtype=bool)
	for j in range(gy):
		for i in range(gx):
			xn = 0.0 if gx <= 1 else (float(i) / float(gx - 1)) * 2.0 - 1.0
			yn = 0.0 if gy <= 1 else (float(j) / float(gy - 1)) * 2.0 - 1.0
			xy_inside[j, i] = (xn * xn + yn * yn) <= 1.0

	# Suppress isolated spikes and smooth the surface so pre-press frame looks flatter.
	inside_vals = z_fill[xy_inside & np.isfinite(z_fill)]
	if inside_vals.size >= 8:
		hi_clip = float(np.percentile(inside_vals, 98.5))
		z_fill = np.minimum(z_fill, hi_clip)
	z_fill = cv2.GaussianBlur(
		z_fill.astype(np.float32),
		ksize=(0, 0),
		sigmaX=1.05,
		sigmaY=1.05,
		borderType=cv2.BORDER_REPLICATE,
	).astype(np.float64)

	auto_max = float(np.nanpercentile(z_fill, 98.0)) if np.any(np.isfinite(z_fill)) else 1e-6
	auto_max = max(1e-6, auto_max)
	z_max_mm = auto_max
	if max_mm_hint > 0:
		z_max_mm = max(z_max_mm, float(max_mm_hint))
	if show_reference:
		z_max_mm = max(z_max_mm, b_depth)
	z_max_mm = max(1e-6, z_max_mm)

	cx0 = left + plot_w * 0.50
	cy0 = top + plot_h * 0.76
	scale_xy = min(plot_w, plot_h) * 0.53
	scale_z = min(plot_h, plot_w) * 0.55 * max(0.2, float(z_scale))

	def project(i: int, j: int, zr: float) -> tuple[int, int]:
		xn = 0.0 if gx <= 1 else float(i) / float(gx - 1)
		yn = 0.0 if gy <= 1 else float(j) / float(gy - 1)
		xw = (xn - 0.5) * 2.0
		yw = (yn - 0.5) * 2.0
		sx = int(cx0 + (xw - yw) * scale_xy * 0.62)
		sy = int(cy0 + (xw + yw) * scale_xy * 0.28 - float(np.clip(zr, 0.0, 1.0)) * scale_z)
		return sx, sy

	def color_from_ratio(r: float) -> tuple[int, int, int]:
		r = float(np.clip(r, 0.0, 1.0))
		b = int(255 * (1.0 - r))
		g = int(190 * (1.0 - abs(2.0 * r - 1.0)) + 50)
		rr = int(255 * r)
		return (b, g, rr)

	def ref_ratio_for_cell(i: int, j: int) -> float:
		xn = 0.0 if gx <= 1 else (float(i) / float(gx - 1)) * 2.0 - 1.0
		yn = 0.0 if gy <= 1 else (float(j) / float(gy - 1)) * 2.0 - 1.0
		r2 = xn * xn + yn * yn
		if r2 >= 1.0:
			return 0.0
		if reference_shape == "paraboloid":
			zr = 1.0 - r2
		else:
			zr = math.sqrt(max(0.0, 1.0 - r2))
		return float(np.clip((zr * b_depth) / z_max_mm, 0.0, 1.0))

	base_quad = [
		project(0, 0, 0.0),
		project(gx - 1, 0, 0.0),
		project(gx - 1, gy - 1, 0.0),
		project(0, gy - 1, 0.0),
	]
	cv2.polylines(img, [np.array(base_quad, dtype=np.int32)], True, (60, 60, 60), 1, cv2.LINE_AA)

	if show_reference:
		for j in range(gy):
			prev_p = None
			for i in range(gx):
				if not xy_inside[j, i]:
					prev_p = None
					continue
				p = project(i, j, ref_ratio_for_cell(i, j))
				if prev_p is not None:
					cv2.line(img, prev_p, p, (80, 80, 80), 1, cv2.LINE_AA)
				prev_p = p

		for i in range(gx):
			prev_p = None
			for j in range(gy):
				if not xy_inside[j, i]:
					prev_p = None
					continue
				p = project(i, j, ref_ratio_for_cell(i, j))
				if prev_p is not None:
					cv2.line(img, prev_p, p, (80, 80, 80), 1, cv2.LINE_AA)
				prev_p = p

	peak_val = -1.0
	peak_ij = None

	for j in range(gy):
		prev_p = None
		prev_ratio = 0.0
		for i in range(gx):
			if not xy_inside[j, i] or not np.isfinite(z_fill[j, i]):
				prev_p = None
				continue
			val = float(z_fill[j, i])
		ratio = float(np.clip(val / z_max_mm, 0.0, 1.0))
		p = project(i, j, ratio)
		if prev_p is not None:
			c = color_from_ratio(0.5 * (prev_ratio + ratio))
			cv2.line(img, prev_p, p, c, 1, cv2.LINE_AA)
		prev_p = p
		prev_ratio = ratio
		if val > peak_val:
			peak_val = val
			peak_ij = (i, j, ratio)

	for i in range(gx):
		prev_p = None
		prev_ratio = 0.0
		for j in range(gy):
			if not xy_inside[j, i] or not np.isfinite(z_fill[j, i]):
				prev_p = None
				continue
			val = float(z_fill[j, i])
			ratio = float(np.clip(val / z_max_mm, 0.0, 1.0))
			p = project(i, j, ratio)
			if prev_p is not None:
				c = color_from_ratio(0.5 * (prev_ratio + ratio))
				cv2.line(img, prev_p, p, c, 1, cv2.LINE_AA)
			prev_p = p
			prev_ratio = ratio

	if peak_ij is not None:
		pi, pj, pr = peak_ij
		p_peak = project(int(pi), int(pj), float(pr))
		cv2.circle(img, p_peak, 5, (0, 0, 255), 2, cv2.LINE_AA)

	z_axis_bottom = project(0, gy - 1, 0.0)
	z_axis_top = project(0, gy - 1, 1.0)
	cv2.arrowedLine(img, z_axis_bottom, z_axis_top, (170, 170, 170), 1, cv2.LINE_AA, tipLength=0.08)

	mean_v = float(np.mean(z))
	p95_v = float(np.percentile(z, 95.0))
	max_v = float(np.max(z))
	cv2.putText(
		img,
		f"bubble={b_major:.1f}x{b_minor:.1f}mm depth={b_depth:.1f}mm  z-max={z_max_mm:.3f}mm",
		(14, graph_h - 32),
		cv2.FONT_HERSHEY_SIMPLEX,
		0.48,
		(190, 190, 190),
		1,
		cv2.LINE_AA,
	)
	cv2.putText(
		img,
		f"measured proxy: mean={mean_v:.3f}mm  p95={p95_v:.3f}mm  max={max_v:.3f}mm",
		(14, graph_h - 12),
		cv2.FONT_HERSHEY_SIMPLEX,
		0.50,
		(210, 210, 210),
		1,
		cv2.LINE_AA,
	)

	return img


def rigid_transform_from_affine_2x3(affine_2x3: np.ndarray) -> np.ndarray:
	a = float(affine_2x3[0, 0])
	b = float(affine_2x3[1, 0])
	norm = max(1e-9, math.hypot(a, b))
	cos_theta = a / norm
	sin_theta = b / norm

	out = np.eye(3, dtype=np.float64)
	out[0, 0] = cos_theta
	out[0, 1] = -sin_theta
	out[1, 0] = sin_theta
	out[1, 1] = cos_theta
	out[0, 2] = float(affine_2x3[0, 2])
	out[1, 2] = float(affine_2x3[1, 2])
	return out


def translation_matrix(dx: float, dy: float) -> np.ndarray:
	out = np.eye(3, dtype=np.float64)
	out[0, 2] = float(dx)
	out[1, 2] = float(dy)
	return out


def angle_deg_from_matrix(tf: np.ndarray) -> float:
	return math.degrees(math.atan2(float(tf[1, 0]), float(tf[0, 0])))


def apply_transform_to_points(tf: np.ndarray, points_xy: np.ndarray) -> np.ndarray:
	ones = np.ones((points_xy.shape[0], 1), dtype=np.float64)
	homo = np.hstack([points_xy.astype(np.float64), ones])
	warped = (tf @ homo.T).T
	return warped[:, :2]


def compute_local_deformation(
	origin_xy: np.ndarray,
	current_xy: np.ndarray,
	ransac_thresh: float,
) -> tuple[np.ndarray, np.ndarray]:
	if len(origin_xy) < 4 or len(current_xy) < 4:
		residual_xy = current_xy - origin_xy
		residual_mag_px = np.linalg.norm(residual_xy, axis=1)
		return residual_xy, residual_mag_px

	affine, _inlier_mask = cv2.estimateAffinePartial2D(
		origin_xy,
		current_xy,
		method=cv2.RANSAC,
		ransacReprojThreshold=ransac_thresh,
		maxIters=500,
		confidence=0.99,
		refineIters=10,
	)
	if affine is None:
		pred_xy = origin_xy
	else:
		rigid_tf = rigid_transform_from_affine_2x3(affine)
		pred_xy = apply_transform_to_points(rigid_tf, origin_xy)

	residual_xy = current_xy - pred_xy
	residual_mag_px = np.linalg.norm(residual_xy, axis=1)
	return residual_xy, residual_mag_px


def estimate_nn_px(points: np.ndarray) -> float | None:
	if points is None or len(points) < 4:
		return None

	xy = points.reshape(-1, 2).astype(np.float64)
	diff = xy[:, None, :] - xy[None, :, :]
	dist = np.sqrt(np.sum(diff * diff, axis=2))
	n = dist.shape[0]
	dist[np.arange(n), np.arange(n)] = np.inf
	nn = np.min(dist, axis=1)
	nn = nn[np.isfinite(nn)]
	if len(nn) < 4:
		return None

	q1, q3 = np.percentile(nn, [25, 75])
	iqr = max(1e-9, float(q3 - q1))
	lo = float(q1 - 1.5 * iqr)
	hi = float(q3 + 1.5 * iqr)
	keep = (nn >= lo) & (nn <= hi)
	filtered = nn[keep]
	if len(filtered) < 4:
		filtered = nn

	return float(np.median(filtered))


def estimate_mm_per_px_from_grid(points: np.ndarray, nn_distance_mm: float) -> float | None:
	nn_px = estimate_nn_px(points)
	if nn_px is None or nn_px <= 1e-9:
		return None
	return float(nn_distance_mm / nn_px)


def open_fullfield_export(export_dir: str, export_prefix: str):
	os.makedirs(export_dir, exist_ok=True)
	ts = datetime.now().strftime("%Y%m%d_%H%M%S")
	path = os.path.join(export_dir, f"{export_prefix}_{ts}.csv")
	fh = open(path, "w", newline="", encoding="utf-8")
	writer = csv.writer(fh)
	writer.writerow(
		[
			"time_s",
			"frame",
			"point_id",
			"origin_x_px",
			"origin_y_px",
			"current_x_px",
			"current_y_px",
			"raw_dx_px",
			"raw_dy_px",
			"raw_disp_px",
			"local_dx_px",
			"local_dy_px",
			"local_disp_px",
			"raw_dx_mm",
			"raw_dy_mm",
			"raw_disp_mm",
			"local_dx_mm",
			"local_dy_mm",
			"local_disp_mm",
		]
	)
	return fh, writer, path


def write_fullfield_rows(
	writer,
	time_s: float,
	frame_index: int,
	point_ids: np.ndarray,
	origin_xy: np.ndarray,
	current_xy: np.ndarray,
	raw_xy: np.ndarray,
	raw_mag_px: np.ndarray,
	local_xy: np.ndarray,
	local_mag_px: np.ndarray,
	px_to_mm: float,
) -> None:
	for idx in range(len(point_ids)):
		writer.writerow(
			[
				f"{time_s:.6f}",
				int(frame_index),
				int(point_ids[idx]),
				f"{origin_xy[idx, 0]:.4f}",
				f"{origin_xy[idx, 1]:.4f}",
				f"{current_xy[idx, 0]:.4f}",
				f"{current_xy[idx, 1]:.4f}",
				f"{raw_xy[idx, 0]:.6f}",
				f"{raw_xy[idx, 1]:.6f}",
				f"{raw_mag_px[idx]:.6f}",
				f"{local_xy[idx, 0]:.6f}",
				f"{local_xy[idx, 1]:.6f}",
				f"{local_mag_px[idx]:.6f}",
				f"{raw_xy[idx, 0] * px_to_mm:.6f}",
				f"{raw_xy[idx, 1] * px_to_mm:.6f}",
				f"{raw_mag_px[idx] * px_to_mm:.6f}",
				f"{local_xy[idx, 0] * px_to_mm:.6f}",
				f"{local_xy[idx, 1] * px_to_mm:.6f}",
				f"{local_mag_px[idx] * px_to_mm:.6f}",
			]
		)


def snapshot_origin(
	global_transform: np.ndarray,
	points: np.ndarray | None,
	point_ids: np.ndarray | None,
) -> tuple[np.ndarray, np.ndarray | None, np.ndarray | None, bool]:
	origin_inv_transform = np.linalg.inv(global_transform)
	if points is None or point_ids is None or len(points) < 4:
		return origin_inv_transform, None, None, False
	return origin_inv_transform, points.copy(), point_ids.copy(), True


def main() -> None:
	args = build_parser().parse_args()

	clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
	roi_scale = clamp(args.roi_scale, 0.1, 1.0)
	focus_mode = str(args.focus_mode)
	lens_position = float(max(0.0, args.lens_position))
	focus_step = float(max(0.01, args.focus_step))

	cam = open_camera(
		args.width,
		args.height,
		args.fps,
		focus_mode,
		lens_position,
		args.exposure_us,
		args.analogue_gain,
	)
	window_scale = clamp(float(args.window_scale), 0.20, 1.00)
	ui_w = int(max(420, round(args.width * window_scale)))
	ui_h = int(max(240, round(args.height * window_scale)))
	depth_ui_h = int(max(220, round(args.depth_graph_height * window_scale)))
	depth3d_ui_h = int(max(220, round(args.depth_graph_3d_height * window_scale)))
	cv2.namedWindow(FLOW_WINDOW, cv2.WINDOW_NORMAL)
	cv2.resizeWindow(FLOW_WINDOW, ui_w, ui_h)
	cv2.namedWindow(MASK_WINDOW, cv2.WINDOW_NORMAL)
	cv2.resizeWindow(MASK_WINDOW, ui_w, ui_h)
	cv2.namedWindow(DEPTH_WINDOW, cv2.WINDOW_NORMAL)
	cv2.resizeWindow(DEPTH_WINDOW, ui_w, depth_ui_h)
	cv2.namedWindow(DEPTH3D_WINDOW, cv2.WINDOW_NORMAL)
	cv2.resizeWindow(DEPTH3D_WINDOW, ui_w, depth3d_ui_h)
	focus_target = {
		"x": None,
		"y": None,
		"r": int(max(20, args.focus_target_radius)),
	}
	cv2.setMouseCallback(FLOW_WINDOW, make_focus_mouse_callback(focus_target))

	prev_gray = None
	prev_pts = None
	prev_ids = None
	origin_pts = None
	origin_ids = None
	origin_valid = False
	roi_mask = None
	last_mask = None

	frame_index = 0
	last_seed_frame = -1
	manual_reseed = True

	px_to_mm = float(max(1e-9, args.px_to_mm))
	px_to_mm_from_cli = "--px-to-mm" in sys.argv[1:]
	scale_is_calibrated = bool(px_to_mm_from_cli)
	global_transform = np.eye(3, dtype=np.float64)
	origin_inv_transform = np.eye(3, dtype=np.float64)
	calibration_start_global_t = None
	if scale_is_calibrated:
		calibration_info = f"scale={px_to_mm:.6f} mm/px (cli)"
	else:
		calibration_info = f"scale={px_to_mm:.6f} mm/px"
	auto_hotspot_last_focus_frame = -1_000_000_000
	auto_hotspot_last_trigger_peak_mm = 0.0
	hotspot_lock_id = None
	hotspot_lock_count = 0
	hotspot_lock_xy = None
	auto_origin_stable_count = 0
	auto_origin_last_set_frame = -1_000_000_000
	next_point_id = 0
	export_every = max(1, int(args.export_every))
	export_enabled = bool(args.export_fullfield)
	export_file = None
	export_writer = None
	export_path = ""
	lut_img = cv2.applyColorMap(np.arange(256, dtype=np.uint8).reshape(-1, 1), cv2.COLORMAP_TURBO)
	turbo_lut = lut_img[:, 0, :]

	smooth_fps = 0.0
	last_t = time.perf_counter()

	print("Running realtime optical flow.")
	print(
		"Keys: q/ESC=quit, r=reseed, o=set origin, c=move-calib, g=grid-calib, "
		"m=manual focus, a=continuous AF, f=AF+lock, z=sweep+lock, [/] lens step, "
		h="focus hotspot, x=clear focus target, e=export, s=save"
	)
	print(
		"Bubble geometry: "
		f"{float(args.bubble_major_mm):.1f}x{float(args.bubble_minor_mm):.1f} mm, "
		f"depth={float(args.bubble_depth_mm):.1f} mm"
	)
	print("Focus target: left-click Optical Flow window to set ROI, right-click to clear.")
	# วิธีใช้ (TH): ตั้ง baseline ด้วยปุ่ม o, คาลิเบรต mm/px ด้วยปุ่ม g หรือ c,
	# แล้วใช้ h เพื่อโฟกัส hotspot ทันที หรือเปิด --auto-hotspot-focus ให้ระบบสั่งเองอัตโนมัติ
	if args.auto_origin:
		print(
			"Auto origin ON: "
			f"stable={int(max(1, args.auto_origin_stable_frames))}f "
			f"min_pts={int(max(4, args.auto_origin_min_points))} "
			f"max_px={float(max(1e-6, args.auto_origin_max_frame_px)):.3f} "
			f"max_rot={float(max(1e-6, args.auto_origin_max_rot_deg)):.3f}deg"
		)
	else:
		print("Auto origin OFF (use key o to set manually).")
	if args.auto_hotspot_focus:
		print(
			"Auto hotspot focus ON: triggers focus sweep on new deformation peak "
			f"(th={args.auto_hotspot_thresh_mm:.3f}mm, rise={args.auto_hotspot_min_rise_mm:.3f}mm, cd={int(args.auto_hotspot_cooldown)}f)."
		)

	focus_note = ""
	if args.focus_sweep_on_start:
		print("Running startup focus sweep...")
		lens_position, sweep_score, sweep_results = run_focus_sweep(
			cam,
			args.focus_sweep_min,
			args.focus_sweep_max,
			args.focus_sweep_step,
			args.focus_sweep_frames,
			args.focus_sweep_settle_ms,
			args.focus_sweep_center,
			focus_target,
		)
		focus_mode = "manual"
		focus_note = f"sweep best={lens_position:.2f} sharp={sweep_score:.1f} n={len(sweep_results)}"
		if len(sweep_results) >= 2:
			sweep_scores = np.array([s for _lp, s in sweep_results], dtype=np.float64)
			best_i = int(np.argmax(sweep_scores))
			if best_i == 0 or best_i == len(sweep_results) - 1:
				focus_note += " edge-range"
				print(
					"Sweep warning: best focus at range edge. "
					"Try widening --focus-sweep-min/max around your working distance."
				)
		print(f"Startup sweep done. {focus_note}")

	if export_enabled:
		export_file, export_writer, export_path = open_fullfield_export(args.export_dir, args.export_prefix)
		print(f"Per-point export enabled: {export_path}")

	try:
		while True:
			frame_rgb = cam.capture_array()
			gray = preprocess_gray(frame_rgb, clahe)
			focus_score = compute_focus_score(gray)
			if frame_index % 15 == 0:
				lens_position = read_lens_position(cam, lens_position)

			if roi_mask is None or roi_mask.shape != gray.shape:
				roi_mask = build_roi_mask(gray.shape, roi_scale)

			frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
			h, w = frame_bgr.shape[:2]
			if focus_target.get("x") is not None and focus_target.get("y") is not None:
				cx = int(clamp(int(focus_target["x"]), 0, w - 1))
				cy = int(clamp(int(focus_target["y"]), 0, h - 1))
				rad = int(max(10, focus_target.get("r", 100)))
				focus_target["x"] = cx
				focus_target["y"] = cy
				cv2.circle(frame_bgr, (cx, cy), rad, (255, 140, 0), 1, cv2.LINE_AA)
				cv2.drawMarker(frame_bgr, (cx, cy), (255, 140, 0), cv2.MARKER_CROSS, 14, 1, cv2.LINE_AA)

			now = time.perf_counter()
			dt = max(1e-6, now - last_t)
			last_t = now
			inst_fps = 1.0 / dt
			smooth_fps = inst_fps if smooth_fps <= 0 else 0.90 * smooth_fps + 0.10 * inst_fps

			frame_index += 1
			frame_dx = 0.0
			frame_dy = 0.0
			frame_angle = 0.0
			step_transform = None
			tracked_count = 0
			inlier_count = 0
			status = "tracking"
			local_mean_mm = 0.0
			local_p95_mm = 0.0
			local_max_mm = 0.0
			hotspot_text = "none"
			hotspot_xy_for_focus = None
			hotspot_id_for_focus = None
			hotspot_candidate_xy = None
			hotspot_candidate_id = None
			hotspot_candidate_score = 0.0
			hotspot_candidate_z = 0.0
			hotspot_candidate_ok = False
			origin_state = "origin:unset (press o)"
			export_state = "export:off"
			depth_graph_xy = None
			depth_graph_mm = None
			bubble_pose_for_3d = None

			need_reseed = False
			reseed_reason = ""
			if manual_reseed:
				need_reseed = True
				reseed_reason = "manual"
			elif prev_pts is None or len(prev_pts) < args.min_track_points:
				need_reseed = True
				reseed_reason = "low-points"
			elif (
				not origin_valid
				and args.reseed_interval > 0
				and (frame_index - last_seed_frame) >= args.reseed_interval
			):
				need_reseed = True
				reseed_reason = "periodic"

			if need_reseed:
				points, last_mask, detected_count = detect_dot_centers(
					gray,
					roi_mask,
					min_area=args.min_area,
					max_area=args.max_area,
					min_circularity=args.min_circularity,
					max_points=args.max_points,
				)

				if points is not None and len(points) >= args.min_track_points:
					prev_pts = points
					n_pts = len(prev_pts)
					prev_ids = np.arange(next_point_id, next_point_id + n_pts, dtype=np.int64)
					next_point_id += n_pts
					if args.auto_grid_calib:
						est_scale = estimate_mm_per_px_from_grid(prev_pts, args.nn_distance_mm)
						if est_scale is not None:
							px_to_mm = est_scale
							scale_is_calibrated = True
							calibration_info = f"scale={px_to_mm:.6f} mm/px (auto-grid)"
					if origin_valid:
						origin_pts = None
						origin_ids = None
						origin_valid = False
					prev_gray = gray
					last_seed_frame = frame_index
					status = f"reseed:{reseed_reason} det={detected_count} use={len(prev_pts)}"
					if reseed_reason != "manual":
						status += " origin-reset"
					for pt in prev_pts:
						x, y = pt[0]
						cv2.circle(frame_bgr, (int(x), int(y)), 2, (255, 255, 0), -1, cv2.LINE_AA)
				else:
					prev_pts = None
					prev_ids = None
					origin_pts = None
					origin_ids = None
					origin_valid = False
					prev_gray = gray
					status = f"reseed-failed:{reseed_reason} det={detected_count}"

				manual_reseed = False
			elif prev_gray is not None and prev_pts is not None and len(prev_pts) >= 4:
				origin_xy = None
				if (
					origin_valid
					and origin_pts is not None
					and origin_ids is not None
					and prev_ids is not None
					and len(origin_pts) == len(prev_pts)
					and len(origin_ids) == len(prev_ids)
				):
					origin_xy = origin_pts.reshape(-1, 2)

				next_pts, st, _err = cv2.calcOpticalFlowPyrLK(prev_gray, gray, prev_pts, None, **LK_PARAMS)
				if next_pts is None or st is None:
					prev_pts = None
					prev_ids = None
					origin_pts = None
					origin_ids = None
					origin_valid = False
					prev_gray = gray
					status = "lk-failed"
				else:
					ok = st.reshape(-1).astype(bool)
					good_prev = prev_pts.reshape(-1, 2)[ok]
					good_next = next_pts.reshape(-1, 2)[ok]
					good_ids = prev_ids[ok] if prev_ids is not None else None
					good_origin = origin_xy[ok] if origin_xy is not None else None
					good_origin_ids = origin_ids[ok] if origin_ids is not None else None
					tracked_count = int(len(good_next))

					if tracked_count >= 4:
						back_pts, back_st, _back_err = cv2.calcOpticalFlowPyrLK(
							gray,
							prev_gray,
							good_next.reshape(-1, 1, 2),
							None,
							**LK_PARAMS,
						)
						if back_pts is not None and back_st is not None:
							fb_ok = back_st.reshape(-1).astype(bool)
							good_prev = good_prev[fb_ok]
							good_next = good_next[fb_ok]
							if good_ids is not None:
								good_ids = good_ids[fb_ok]
							if good_origin is not None:
								good_origin = good_origin[fb_ok]
							if good_origin_ids is not None:
								good_origin_ids = good_origin_ids[fb_ok]
							back_xy = back_pts.reshape(-1, 2)[fb_ok]
							fb_error = np.linalg.norm(good_prev - back_xy, axis=1)
							keep = fb_error <= args.fb_thresh
							good_prev = good_prev[keep]
							good_next = good_next[keep]
							if good_ids is not None:
								good_ids = good_ids[keep]
							if good_origin is not None:
								good_origin = good_origin[keep]
							if good_origin_ids is not None:
								good_origin_ids = good_origin_ids[keep]
							tracked_count = int(len(good_next))

					if tracked_count >= 4:
						affine, inlier_mask = cv2.estimateAffinePartial2D(
							good_prev,
							good_next,
							method=cv2.RANSAC,
							ransacReprojThreshold=args.ransac_thresh,
							maxIters=500,
							confidence=0.99,
							refineIters=10,
						)

						if affine is not None and inlier_mask is not None:
							inliers = inlier_mask.reshape(-1).astype(bool)
							in_prev = good_prev[inliers]
							in_next = good_next[inliers]
							in_ids = good_ids[inliers] if good_ids is not None else None
							in_origin = good_origin[inliers] if good_origin is not None else None
							in_origin_ids = good_origin_ids[inliers] if good_origin_ids is not None else None
							inlier_count = int(len(in_next))

							if inlier_count >= 3:
								frame_dx = float(affine[0, 2])
								frame_dy = float(affine[1, 2])
								frame_angle = math.degrees(math.atan2(float(affine[1, 0]), float(affine[0, 0])))
								step_transform = rigid_transform_from_affine_2x3(affine)

								draw_step = max(1, len(in_next) // max(1, args.draw_points))
								for p0, p1 in zip(in_prev[::draw_step], in_next[::draw_step]):
									x0, y0 = int(p0[0]), int(p0[1])
									x1, y1 = int(p1[0]), int(p1[1])
									cv2.line(frame_bgr, (x0, y0), (x1, y1), (0, 160, 255), 1, cv2.LINE_AA)
									cv2.circle(frame_bgr, (x1, y1), 2, (0, 255, 0), -1, cv2.LINE_AA)

								prev_pts = in_next.reshape(-1, 1, 2).astype(np.float32)
								prev_ids = in_ids.copy() if in_ids is not None else None
								if in_origin is not None:
									origin_pts = in_origin.reshape(-1, 1, 2).astype(np.float32)
									origin_ids = in_origin_ids.copy() if in_origin_ids is not None else None
								else:
									origin_pts = None
									origin_ids = None
									origin_valid = False
								prev_gray = gray
								status = "tracking"
							else:
								prev_pts = None
								prev_ids = None
								origin_pts = None
								origin_ids = None
								origin_valid = False
								prev_gray = gray
								status = "ransac-low-inliers"
						else:
							flow = good_next - good_prev
							frame_dx = float(np.median(flow[:, 0]))
							frame_dy = float(np.median(flow[:, 1]))
							step_transform = translation_matrix(frame_dx, frame_dy)

							draw_step = max(1, len(good_next) // max(1, args.draw_points))
							for p0, p1 in zip(good_prev[::draw_step], good_next[::draw_step]):
								x0, y0 = int(p0[0]), int(p0[1])
								x1, y1 = int(p1[0]), int(p1[1])
								cv2.line(frame_bgr, (x0, y0), (x1, y1), (0, 110, 255), 1, cv2.LINE_AA)
								cv2.circle(frame_bgr, (x1, y1), 2, (80, 255, 80), -1, cv2.LINE_AA)

							prev_pts = good_next.reshape(-1, 1, 2).astype(np.float32)
							prev_ids = good_ids.copy() if good_ids is not None else None
							if good_origin is not None:
								origin_pts = good_origin.reshape(-1, 1, 2).astype(np.float32)
								origin_ids = good_origin_ids.copy() if good_origin_ids is not None else None
							else:
								origin_pts = None
								origin_ids = None
								origin_valid = False
							prev_gray = gray
							inlier_count = tracked_count
							status = "tracking-no-ransac"
					else:
						prev_pts = None
						prev_ids = None
						origin_pts = None
						origin_ids = None
						origin_valid = False
						prev_gray = gray
						status = "too-few-good-points"
			else:
				prev_gray = gray
				status = "waiting-for-seed"

			if step_transform is not None:
				global_transform = step_transform @ global_transform

			absolute_transform = global_transform @ origin_inv_transform
			absolute_dx = float(absolute_transform[0, 2])
			absolute_dy = float(absolute_transform[1, 2])
			absolute_angle = angle_deg_from_matrix(absolute_transform)

			h, w = frame_bgr.shape[:2]
			if roi_scale < 0.999:
				roi_axes = (max(10, int(w * roi_scale * 0.5)), max(10, int(h * roi_scale * 0.5)))
				cv2.ellipse(frame_bgr, (w // 2, h // 2), roi_axes, 0, 0, 360, (255, 180, 0), 1, cv2.LINE_AA)

			frame_mm_x = frame_dx * px_to_mm
			frame_mm_y = frame_dy * px_to_mm
			absolute_mm_x = absolute_dx * px_to_mm
			absolute_mm_y = absolute_dy * px_to_mm
			export_payload = None

			if (
				origin_valid
				and prev_pts is not None
				and prev_ids is not None
				and origin_pts is not None
				and origin_ids is not None
				and len(prev_pts) == len(origin_pts)
				and len(prev_ids) == len(origin_ids)
				and len(prev_pts) >= 4
			):
				current_xy = prev_pts.reshape(-1, 2).astype(np.float64)
				current_ids = prev_ids.astype(np.int64)
				origin_xy = origin_pts.reshape(-1, 2).astype(np.float64)
				id_same = np.array_equal(current_ids, origin_ids.astype(np.int64))
				if not id_same:
					common_ids, idx_curr, idx_org = np.intersect1d(
						current_ids,
						origin_ids.astype(np.int64),
						return_indices=True,
					)
					current_ids = common_ids
					current_xy = current_xy[idx_curr]
					origin_xy = origin_xy[idx_org]

				if len(origin_xy) >= 5:
					bubble_pose_for_3d = estimate_bubble_ellipse_pose(origin_xy)
				elif len(current_xy) >= 5:
					bubble_pose_for_3d = estimate_bubble_ellipse_pose(current_xy)

				_residual_xy, residual_mag_px = compute_local_deformation(
					origin_xy,
					current_xy,
					args.ransac_thresh,
				)

				if len(residual_mag_px) > 0:
					raw_xy = current_xy - origin_xy
					raw_mag_px = np.linalg.norm(raw_xy, axis=1)
					residual_mag_mm = residual_mag_px * px_to_mm
					depth_graph_xy = current_xy
					# Remove per-frame floor noise so idle surface appears flatter before pressing.
					noise_floor = float(np.percentile(residual_mag_mm, 55.0))
					depth_proxy_mm = np.maximum(residual_mag_mm - noise_floor, 0.0)
					if len(depth_proxy_mm) >= 8:
						hi_clip = float(np.percentile(depth_proxy_mm, 99.0))
						depth_proxy_mm = np.minimum(depth_proxy_mm, hi_clip)
					depth_graph_mm = depth_proxy_mm
					local_mean_mm = float(np.mean(depth_proxy_mm))
					local_p95_mm = float(np.percentile(depth_proxy_mm, 95))
					hot_idx = int(np.argmax(depth_proxy_mm))
					local_max_mm = float(depth_proxy_mm[hot_idx])
					hot_xy = current_xy[hot_idx]
					hot_id = int(current_ids[hot_idx])

					proxy_med = float(np.median(depth_proxy_mm))
					proxy_mad = float(np.median(np.abs(depth_proxy_mm - proxy_med)))
					proxy_sigma = max(1e-9, 1.4826 * proxy_mad)
					hot_z = float((local_max_mm - proxy_med) / proxy_sigma)
					proxy_spread = float(np.percentile(depth_proxy_mm, 95) - np.percentile(depth_proxy_mm, 50))

					cx_roi = 0.5 * float(w - 1)
					cy_roi = 0.5 * float(h - 1)
					ax_roi = max(10.0, float(w) * roi_scale * 0.5)
					ay_roi = max(10.0, float(h) * roi_scale * 0.5)
					nx = (float(hot_xy[0]) - cx_roi) / ax_roi
					ny = (float(hot_xy[1]) - cy_roi) / ay_roi
					edge_ok = nx * nx + ny * ny <= float(clamp(args.hotspot_roi_guard, 0.55, 0.99))
					z_ok = hot_z >= float(max(0.5, args.hotspot_sigma_thresh))
					spread_ok = proxy_spread > 1e-9

					hotspot_candidate_ok = edge_ok and z_ok and spread_ok and (local_max_mm > 0.0)
					hotspot_candidate_id = hot_id
					hotspot_candidate_xy = (float(hot_xy[0]), float(hot_xy[1]))
					hotspot_candidate_score = local_max_mm
					hotspot_candidate_z = hot_z

					draw_norm = max(1e-9, float(np.percentile(residual_mag_px, 95)))
					draw_step = max(1, len(current_xy) // max(1, args.draw_points))
					for pt_xy, mag_px in zip(current_xy[::draw_step], residual_mag_px[::draw_step]):
						lut_idx = int(np.clip((mag_px / draw_norm) * 255.0, 0, 255))
						bgr = turbo_lut[lut_idx]
						cv2.circle(
							frame_bgr,
							(int(pt_xy[0]), int(pt_xy[1])),
							3,
							(int(bgr[0]), int(bgr[1]), int(bgr[2])),
							-1,
							cv2.LINE_AA,
						)

					export_payload = (
						current_ids,
						origin_xy,
						current_xy,
						raw_xy,
						raw_mag_px,
						_residual_xy,
						residual_mag_px,
					)

				origin_state = f"origin:locked pts={len(current_xy)}"

			if export_enabled and export_writer is not None:
				export_state = f"export:on/{export_every}"
				if export_payload is not None and (frame_index % export_every == 0):
					(
						ids_for_export,
						origin_xy_export,
						current_xy_export,
						raw_xy_export,
						raw_mag_export,
						local_xy_export,
						local_mag_export,
					) = export_payload
					write_fullfield_rows(
						export_writer,
						time.time(),
						frame_index,
						ids_for_export,
						origin_xy_export,
						current_xy_export,
						raw_xy_export,
						raw_mag_export,
						local_xy_export,
						local_mag_export,
						px_to_mm,
					)
					if frame_index % max(30, export_every * 10) == 0:
						export_file.flush()
			elif export_enabled:
				export_state = "export:on(no-file)"

			req_confirm = int(max(1, args.hotspot_confirm_frames))
			ema_alpha = float(clamp(args.hotspot_ema, 0.0, 1.0))
			if hotspot_candidate_ok and hotspot_candidate_id is not None and hotspot_candidate_xy is not None:
				cand_id = int(hotspot_candidate_id)
				cand_xy = hotspot_candidate_xy

				if hotspot_lock_id == cand_id:
					hotspot_lock_count += 1
				else:
					hotspot_lock_id = cand_id
					hotspot_lock_count = 1
					hotspot_lock_xy = cand_xy

				if hotspot_lock_xy is None:
					hotspot_lock_xy = cand_xy
				else:
					hotspot_lock_xy = (
						(1.0 - ema_alpha) * float(hotspot_lock_xy[0]) + ema_alpha * float(cand_xy[0]),
						(1.0 - ema_alpha) * float(hotspot_lock_xy[1]) + ema_alpha * float(cand_xy[1]),
					)

				if hotspot_lock_count >= req_confirm and hotspot_lock_xy is not None:
					px = int(clamp(round(hotspot_lock_xy[0]), 0, w - 1))
					py = int(clamp(round(hotspot_lock_xy[1]), 0, h - 1))
					hotspot_xy_for_focus = (px, py)
					hotspot_id_for_focus = cand_id
					hotspot_text = f"id={cand_id}@{px},{py} z={hotspot_candidate_z:.1f}"
					cv2.circle(frame_bgr, (px, py), 9, (0, 0, 255), 2, cv2.LINE_AA)
				elif hotspot_lock_xy is not None:
					px = int(clamp(round(hotspot_lock_xy[0]), 0, w - 1))
					py = int(clamp(round(hotspot_lock_xy[1]), 0, h - 1))
					hotspot_text = f"pending {hotspot_lock_count}/{req_confirm} id={cand_id} z={hotspot_candidate_z:.1f}"
					cv2.circle(frame_bgr, (px, py), 7, (0, 165, 255), 1, cv2.LINE_AA)
			else:
				if hotspot_lock_count > 0:
					hotspot_lock_count -= 1
				if hotspot_lock_count <= 0:
					hotspot_lock_count = 0
					hotspot_lock_id = None
					hotspot_lock_xy = None

			# โหมดอัตโนมัติ: ทำงานเหมือนกดปุ่ม h เมื่อ hotspot ใหม่แรงขึ้นและพ้นช่วง cooldown
			if (
				args.auto_hotspot_focus
				and hotspot_xy_for_focus is not None
				and local_max_mm >= float(args.auto_hotspot_thresh_mm)
				and local_max_mm >= (auto_hotspot_last_trigger_peak_mm + float(args.auto_hotspot_min_rise_mm))
				and (frame_index - auto_hotspot_last_focus_frame) >= int(max(1, args.auto_hotspot_cooldown))
			):
				focus_mode = "manual"
				lens_position, focus_note, edge_range = run_hotspot_focus_sweep(
					cam,
					lens_position,
					hotspot_xy_for_focus,
					hotspot_id_for_focus,
					focus_target,
					args.focus_sweep_min,
					args.focus_sweep_max,
					args.focus_sweep_step,
					args.focus_sweep_frames,
					args.focus_sweep_settle_ms,
					args.focus_sweep_center,
					args.hotspot_focus_span,
					"AUTO",
				)
				if edge_range:
					print(
						"Sweep warning: best focus at range edge. "
						"Try widening --focus-sweep-min/max or --hotspot-focus-span."
					)
				manual_reseed = True
				prev_gray = None
				prev_pts = None
				prev_ids = None
				auto_hotspot_last_focus_frame = frame_index
				auto_hotspot_last_trigger_peak_mm = local_max_mm
				status = "auto-hotspot-focus"
				print(f"Auto hotspot focus done. {focus_note}")

			active_points = 0 if prev_pts is None else int(len(prev_pts))
			frame_shift_px = float(math.hypot(frame_dx, frame_dy))
			frame_shift_mm = float(frame_shift_px * px_to_mm)
			auto_origin_state = "auto-origin:off"

			if args.auto_origin:
				auto_origin_state = "auto-origin:wait"
				if origin_valid:
					auto_origin_state = "auto-origin:locked"
					auto_origin_stable_count = 0
				else:
					if args.auto_origin_require_calibrated and not scale_is_calibrated:
						auto_origin_state = "auto-origin:wait-scale"
						auto_origin_stable_count = 0
					else:
						motion_ok = frame_shift_px <= float(max(1e-6, args.auto_origin_max_frame_px))
						if scale_is_calibrated:
							motion_ok = motion_ok and (
								frame_shift_mm <= float(max(1e-6, args.auto_origin_max_frame_mm))
							)
						motion_ok = motion_ok and (
							abs(frame_angle) <= float(max(1e-6, args.auto_origin_max_rot_deg))
						)
						tracking_ok = status.startswith("tracking") or status.startswith("reseed")
						points_ok = active_points >= int(max(4, args.auto_origin_min_points))
						cooldown_ok = (
							(frame_index - auto_origin_last_set_frame)
							>= int(max(1, args.auto_origin_cooldown))
						)

						if motion_ok and tracking_ok and points_ok and cooldown_ok:
							auto_origin_stable_count += 1
						else:
							auto_origin_stable_count = 0

						req_frames = int(max(1, args.auto_origin_stable_frames))
						auto_origin_state = f"auto-origin:{auto_origin_stable_count}/{req_frames}"

						if auto_origin_stable_count >= req_frames:
							(
								origin_inv_transform,
								origin_pts,
								origin_ids,
								origin_valid,
							) = snapshot_origin(global_transform, prev_pts, prev_ids)
							auto_origin_last_set_frame = frame_index
							auto_origin_stable_count = 0
							auto_hotspot_last_trigger_peak_mm = 0.0
							if origin_valid and origin_pts is not None:
								auto_origin_state = "auto-origin:set"
								status = "auto-origin"
								print("Auto origin set: ABS reset and deformation baseline captured.")
							else:
								auto_origin_state = "auto-origin:failed"
								print("Auto origin failed: low points.")

			if focus_target.get("x") is None:
				focus_target_label = "center"
			else:
				focus_target_label = f"{focus_target['x']},{focus_target['y']}"

			put_lines(
				frame_bgr,
				[
					f"FPS:{smooth_fps:5.1f} status:{status}",
					f"points tracked={tracked_count} inliers={inlier_count} active={(0 if prev_pts is None else len(prev_pts))}",
					(
						f"focus={focus_mode} lens={lens_position:4.2f} sharp={focus_score:8.1f} "
						f"target={focus_target_label} {focus_note}"
					),
					f"dX={frame_dx:+6.2f}px dY={frame_dy:+6.2f}px rot={frame_angle:+5.2f}deg",
					f"ABS=({absolute_mm_x:+8.3f},{absolute_mm_y:+8.3f}) mm  ABS_rot={absolute_angle:+6.2f}deg",
					f"deform(mm) mean={local_mean_mm:6.3f} p95={local_p95_mm:6.3f} max={local_max_mm:6.3f}",
					f"{origin_state} hotspot(px)={hotspot_text}  {calibration_info}",
					f"frame_mm=({frame_mm_x:+6.3f},{frame_mm_y:+6.3f})  {export_state}  {auto_origin_state}",
					"keys: r/o/c/g | m/a/f/z/h/[/] focus | x clear-target | e export | s save | q quit",
				],
			)

			warning_lines = []
			# เตือนสถานะที่ยังไม่พร้อมสำหรับการวัดเป็นหน่วย mm
			if not origin_valid:
				warning_lines.append("WARNING: origin is not set (press o)")
			if not scale_is_calibrated:
				warning_lines.append("WARNING: mm scale is not calibrated (press g or c)")
			if warning_lines:
				warn_y = 246
				for warn in warning_lines:
					cv2.putText(
						frame_bgr,
						warn,
						(10, warn_y),
						cv2.FONT_HERSHEY_SIMPLEX,
						0.68,
						(0, 0, 255),
						2,
						cv2.LINE_AA,
					)
					warn_y += 28

			if bubble_pose_for_3d is None and prev_pts is not None and len(prev_pts) >= 5:
				bubble_pose_for_3d = estimate_bubble_ellipse_pose(prev_pts.reshape(-1, 2).astype(np.float64))

			depth_graph_img = build_depth_graph_image(
				depth_graph_xy,
				depth_graph_mm,
				args.width,
				args.depth_graph_height,
				float(max(0.0, args.depth_graph_max_mm)),
			)
			depth_graph_3d_img = build_depth_graph_3d_image(
				depth_graph_xy,
				depth_graph_mm,
				args.width,
				args.height,
				args.depth_graph_3d_height,
				float(max(0.0, args.depth_graph_max_mm)),
				float(max(0.1, args.depth_graph_3d_z_scale)),
				int(max(6, args.depth_graph_3d_grid_x)),
				int(max(4, args.depth_graph_3d_grid_y)),
				bubble_pose_for_3d,
				float(max(1e-6, args.bubble_major_mm)),
				float(max(1e-6, args.bubble_minor_mm)),
				float(max(1e-6, args.bubble_depth_mm)),
				bool(args.depth_graph_3d_show_reference),
				str(args.depth_graph_3d_reference_shape),
			)

			cv2.imshow(FLOW_WINDOW, frame_bgr)
			if last_mask is not None:
				cv2.imshow(MASK_WINDOW, last_mask)
			cv2.imshow(DEPTH_WINDOW, depth_graph_img)
			cv2.imshow(DEPTH3D_WINDOW, depth_graph_3d_img)

			key = cv2.waitKey(1) & 0xFF
			if key in (ord("q"), 27):
				break
			if key == ord("r"):
				manual_reseed = True
			if key == ord("m"):
				focus_mode = "manual"
				lens_position = set_manual_focus(cam, lens_position)
				focus_note = ""
				print(f"Focus mode manual, lens={lens_position:.2f}")
			if key == ord("a"):
				focus_mode = "continuous"
				set_continuous_focus(cam)
				focus_note = ""
				print("Focus mode continuous AF")
			if key == ord("f"):
				lens_position = trigger_autofocus_and_lock(cam, lens_position)
				focus_mode = "manual"
				focus_note = ""
				print(f"One-shot AF done, locked manual at lens={lens_position:.2f}")
			if key == ord("z"):
				focus_mode = "manual"
				print("Running focus sweep...")
				lens_position, sweep_score, sweep_results = run_focus_sweep(
					cam,
					args.focus_sweep_min,
					args.focus_sweep_max,
					args.focus_sweep_step,
					args.focus_sweep_frames,
					args.focus_sweep_settle_ms,
					args.focus_sweep_center,
					focus_target,
				)
				focus_note = f"sweep best={lens_position:.2f} sharp={sweep_score:.1f} n={len(sweep_results)}"
				if len(sweep_results) >= 2:
					sweep_scores = np.array([s for _lp, s in sweep_results], dtype=np.float64)
					best_i = int(np.argmax(sweep_scores))
					if best_i == 0 or best_i == len(sweep_results) - 1:
						focus_note += " edge-range"
						print(
							"Sweep warning: best focus at range edge. "
							"Try widening --focus-sweep-min/max around your working distance."
						)
				manual_reseed = True
				prev_gray = None
				prev_pts = None
				prev_ids = None
				print(f"Focus sweep done. {focus_note}")
			if key == ord("h"):
				if hotspot_xy_for_focus is None:
					print("Hotspot focus unavailable: no hotspot detected in current frame.")
				else:
					focus_mode = "manual"
					lens_position, focus_note, edge_range = run_hotspot_focus_sweep(
						cam,
						lens_position,
						hotspot_xy_for_focus,
						hotspot_id_for_focus,
						focus_target,
						args.focus_sweep_min,
						args.focus_sweep_max,
						args.focus_sweep_step,
						args.focus_sweep_frames,
						args.focus_sweep_settle_ms,
						args.focus_sweep_center,
						args.hotspot_focus_span,
						"MANUAL",
					)
					if edge_range:
						print(
							"Sweep warning: best focus at range edge. "
							"Try widening --focus-sweep-min/max or --hotspot-focus-span."
						)
					manual_reseed = True
					prev_gray = None
					prev_pts = None
					prev_ids = None
					auto_hotspot_last_focus_frame = frame_index
					auto_hotspot_last_trigger_peak_mm = max(
						auto_hotspot_last_trigger_peak_mm,
						local_max_mm,
					)
					print(f"Hotspot focus done. {focus_note}")
			if key == ord("["):
				focus_mode = "manual"
				lens_position = set_manual_focus(cam, lens_position - focus_step)
				focus_note = ""
				print(f"LensPosition -> {lens_position:.2f}")
			if key == ord("]"):
				focus_mode = "manual"
				lens_position = set_manual_focus(cam, lens_position + focus_step)
				focus_note = ""
				print(f"LensPosition -> {lens_position:.2f}")
			if key == ord("x"):
				focus_target["x"] = None
				focus_target["y"] = None
				focus_note = ""
				print("Focus target cleared. Sweep will use center ROI.")
			if key == ord("o"):
				(
					origin_inv_transform,
					origin_pts,
					origin_ids,
					origin_valid,
				) = snapshot_origin(global_transform, prev_pts, prev_ids)
				auto_hotspot_last_trigger_peak_mm = 0.0
				auto_origin_last_set_frame = frame_index
				auto_origin_stable_count = 0
				if origin_valid and origin_pts is not None:
					print("Origin set. ABS reset and local deformation baseline captured.")
				else:
					print("Origin set for ABS only. Press r, wait tracking, then press o again.")
			if key == ord("c"):
				global_t = np.array([global_transform[0, 2], global_transform[1, 2]], dtype=np.float64)
				if calibration_start_global_t is None:
					calibration_start_global_t = global_t
					calibration_info = (
						f"calib armed: move {args.calib_distance_mm:.3f} mm then press c again"
					)
					print(
						f"Calibration armed. Move by {args.calib_distance_mm:.3f} mm and press c again to finish."
					)
				else:
					delta = global_t - calibration_start_global_t
					delta_px = float(np.linalg.norm(delta))
					if delta_px < 1.0:
						calibration_info = "calib failed: move too small"
						print("Calibration failed: measured movement < 1 px.")
					else:
						px_to_mm = float(args.calib_distance_mm / delta_px)
						scale_is_calibrated = True
						calibration_info = f"scale={px_to_mm:.6f} mm/px (calibrated)"
						print(
							"Calibration done: "
							f"{args.calib_distance_mm:.3f} mm / {delta_px:.3f} px = {px_to_mm:.6f} mm/px"
						)
					calibration_start_global_t = None
			if key == ord("g"):
				if prev_pts is None or len(prev_pts) < 4:
					print("Grid calibration failed: not enough active points.")
					calibration_info = "grid-calib failed: low points"
				else:
					est_scale = estimate_mm_per_px_from_grid(prev_pts, args.nn_distance_mm)
					if est_scale is None:
						print("Grid calibration failed: could not estimate nearest-neighbor px distance.")
						calibration_info = "grid-calib failed: no nn estimate"
					else:
						px_to_mm = est_scale
						scale_is_calibrated = True
						nn_px = estimate_nn_px(prev_pts)
						calibration_info = f"scale={px_to_mm:.6f} mm/px (grid {args.nn_distance_mm:.3f}mm)"
						if nn_px is None:
							print(f"Grid calibration done: scale={px_to_mm:.6f} mm/px")
						else:
							print(
								"Grid calibration done: "
								f"{args.nn_distance_mm:.3f} mm / {nn_px:.3f} px = {px_to_mm:.6f} mm/px"
							)
			if key == ord("e"):
				export_enabled = not export_enabled
				if export_enabled:
					export_file, export_writer, export_path = open_fullfield_export(args.export_dir, args.export_prefix)
					print(f"Per-point export enabled: {export_path}")
				else:
					if export_file is not None:
						export_file.close()
					export_file = None
					export_writer = None
					export_path = ""
					print("Per-point export disabled.")
			if key == ord("s"):
				ts = datetime.now().strftime("%Y%m%d_%H%M%S")
				frame_name = f"flow_frame_{ts}.png"
				mask_name = f"flow_mask_{ts}.png"
				depth_name = f"flow_depth_{ts}.png"
				depth3d_name = f"flow_depth3d_{ts}.png"
				cv2.imwrite(frame_name, frame_bgr)
				if last_mask is not None:
					cv2.imwrite(mask_name, last_mask)
				cv2.imwrite(depth_name, depth_graph_img)
				cv2.imwrite(depth3d_name, depth_graph_3d_img)
				print(f"Saved {frame_name}, {mask_name}, {depth_name}, and {depth3d_name}")
	finally:
		if export_file is not None:
			export_file.close()
		cam.stop()
		cv2.destroyAllWindows()


if __name__ == "__main__":
	main()
