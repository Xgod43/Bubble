import argparse
import csv
import math
import os
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

LK_PARAMS = dict(
	winSize=(21, 21),
	maxLevel=3,
	criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.03),
)


def build_parser() -> argparse.ArgumentParser:
	parser = argparse.ArgumentParser(description="Realtime dot-pattern optical flow for Pi Camera v3")
	parser.add_argument("--width", type=int, default=1280, help="Preview width")
	parser.add_argument("--height", type=int, default=720, help="Preview height")
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


def put_lines(frame: np.ndarray, lines) -> None:
	y = 26
	for line in lines:
		cv2.putText(frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 255), 2, cv2.LINE_AA)
		y += 24


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
	cv2.namedWindow(FLOW_WINDOW, cv2.WINDOW_NORMAL)
	cv2.resizeWindow(FLOW_WINDOW, args.width, args.height)
	cv2.namedWindow(MASK_WINDOW, cv2.WINDOW_NORMAL)
	cv2.resizeWindow(MASK_WINDOW, max(420, args.width // 2), max(240, args.height // 2))

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
	global_transform = np.eye(3, dtype=np.float64)
	origin_inv_transform = np.eye(3, dtype=np.float64)
	calibration_start_global_t = None
	calibration_info = f"scale={px_to_mm:.6f} mm/px"
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
		"m=manual focus, a=continuous AF, f=AF+lock, [/] lens step, e=export, s=save"
	)

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
			origin_state = "origin:unset (press o)"
			export_state = "export:off"

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

				_residual_xy, residual_mag_px = compute_local_deformation(
					origin_xy,
					current_xy,
					args.ransac_thresh,
				)

				if len(residual_mag_px) > 0:
					raw_xy = current_xy - origin_xy
					raw_mag_px = np.linalg.norm(raw_xy, axis=1)
					residual_mag_mm = residual_mag_px * px_to_mm
					local_mean_mm = float(np.mean(residual_mag_mm))
					local_p95_mm = float(np.percentile(residual_mag_mm, 95))
					hot_idx = int(np.argmax(residual_mag_mm))
					local_max_mm = float(residual_mag_mm[hot_idx])
					hot_xy = current_xy[hot_idx]
					hot_id = int(current_ids[hot_idx])
					hotspot_text = f"id={hot_id}@{int(hot_xy[0])},{int(hot_xy[1])}"

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

					cv2.circle(frame_bgr, (int(hot_xy[0]), int(hot_xy[1])), 9, (0, 0, 255), 2, cv2.LINE_AA)

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

			put_lines(
				frame_bgr,
				[
					f"FPS:{smooth_fps:5.1f} status:{status}",
					f"points tracked={tracked_count} inliers={inlier_count} active={(0 if prev_pts is None else len(prev_pts))}",
					f"focus={focus_mode} lens={lens_position:4.2f} sharp={focus_score:8.1f}",
					f"dX={frame_dx:+6.2f}px dY={frame_dy:+6.2f}px rot={frame_angle:+5.2f}deg",
					f"ABS=({absolute_mm_x:+8.3f},{absolute_mm_y:+8.3f}) mm  ABS_rot={absolute_angle:+6.2f}deg",
					f"deform(mm) mean={local_mean_mm:6.3f} p95={local_p95_mm:6.3f} max={local_max_mm:6.3f}",
					f"{origin_state} hotspot(px)={hotspot_text}  {calibration_info}",
					f"frame_mm=({frame_mm_x:+6.3f},{frame_mm_y:+6.3f})  {export_state}",
					"keys: r/o/c/g | m/a/f/[/] focus | e export | s save | q quit",
				],
			)

			cv2.imshow(FLOW_WINDOW, frame_bgr)
			if last_mask is not None:
				cv2.imshow(MASK_WINDOW, last_mask)

			key = cv2.waitKey(1) & 0xFF
			if key in (ord("q"), 27):
				break
			if key == ord("r"):
				manual_reseed = True
			if key == ord("m"):
				focus_mode = "manual"
				lens_position = set_manual_focus(cam, lens_position)
				print(f"Focus mode manual, lens={lens_position:.2f}")
			if key == ord("a"):
				focus_mode = "continuous"
				set_continuous_focus(cam)
				print("Focus mode continuous AF")
			if key == ord("f"):
				lens_position = trigger_autofocus_and_lock(cam, lens_position)
				focus_mode = "manual"
				print(f"One-shot AF done, locked manual at lens={lens_position:.2f}")
			if key == ord("["):
				focus_mode = "manual"
				lens_position = set_manual_focus(cam, lens_position - focus_step)
				print(f"LensPosition -> {lens_position:.2f}")
			if key == ord("]"):
				focus_mode = "manual"
				lens_position = set_manual_focus(cam, lens_position + focus_step)
				print(f"LensPosition -> {lens_position:.2f}")
			if key == ord("o"):
				origin_inv_transform = np.linalg.inv(global_transform)
				if prev_pts is not None and prev_ids is not None and len(prev_pts) >= 4:
					origin_pts = prev_pts.copy()
					origin_ids = prev_ids.copy()
					origin_valid = True
					print("Origin set. ABS reset and local deformation baseline captured.")
				else:
					origin_pts = None
					origin_ids = None
					origin_valid = False
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
				cv2.imwrite(frame_name, frame_bgr)
				if last_mask is not None:
					cv2.imwrite(mask_name, last_mask)
				print(f"Saved {frame_name} and {mask_name}")
	finally:
		if export_file is not None:
			export_file.close()
		cam.stop()
		cv2.destroyAllWindows()


if __name__ == "__main__":
	main()
