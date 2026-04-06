import argparse
import math
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
		"--px-to-mm",
		type=float,
		default=1.0,
		help="Scale factor to convert px to mm (set from your calibration)",
	)
	parser.add_argument("--draw-points", type=int, default=140, help="Maximum vectors drawn per frame")
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


def open_camera(width: int, height: int, fps: float) -> Picamera2:
	cam = Picamera2()
	config = cam.create_preview_configuration(main={"size": (width, height), "format": "RGB888"})
	cam.configure(config)
	cam.start()

	controls_payload = {"FrameRate": float(max(1.0, fps))}
	if libcamera_controls is not None:
		controls_payload["AfMode"] = libcamera_controls.AfModeEnum.Continuous
	try:
		cam.set_controls(controls_payload)
	except Exception:
		pass

	time.sleep(0.25)
	return cam


def put_lines(frame: np.ndarray, lines) -> None:
	y = 26
	for line in lines:
		cv2.putText(frame, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 255), 2, cv2.LINE_AA)
		y += 24


def main() -> None:
	args = build_parser().parse_args()

	clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
	roi_scale = clamp(args.roi_scale, 0.1, 1.0)

	cam = open_camera(args.width, args.height, args.fps)
	cv2.namedWindow(FLOW_WINDOW, cv2.WINDOW_NORMAL)
	cv2.resizeWindow(FLOW_WINDOW, args.width, args.height)
	cv2.namedWindow(MASK_WINDOW, cv2.WINDOW_NORMAL)
	cv2.resizeWindow(MASK_WINDOW, max(420, args.width // 2), max(240, args.height // 2))

	prev_gray = None
	prev_pts = None
	roi_mask = None
	last_mask = None

	frame_index = 0
	last_seed_frame = -1
	manual_reseed = True

	cumulative_dx = 0.0
	cumulative_dy = 0.0
	smooth_fps = 0.0
	last_t = time.perf_counter()

	print("Running realtime optical flow. Keys: q/ESC=quit, r=reseed, s=save frame")

	try:
		while True:
			frame_rgb = cam.capture_array()
			gray = preprocess_gray(frame_rgb, clahe)

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
			tracked_count = 0
			inlier_count = 0
			status = "tracking"

			need_reseed = False
			reseed_reason = ""
			if manual_reseed:
				need_reseed = True
				reseed_reason = "manual"
			elif prev_pts is None or len(prev_pts) < args.min_track_points:
				need_reseed = True
				reseed_reason = "low-points"
			elif args.reseed_interval > 0 and (frame_index - last_seed_frame) >= args.reseed_interval:
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
					prev_gray = gray
					last_seed_frame = frame_index
					status = f"reseed:{reseed_reason} det={detected_count} use={len(prev_pts)}"
					for pt in prev_pts:
						x, y = pt[0]
						cv2.circle(frame_bgr, (int(x), int(y)), 2, (255, 255, 0), -1, cv2.LINE_AA)
				else:
					prev_pts = None
					prev_gray = gray
					status = f"reseed-failed:{reseed_reason} det={detected_count}"

				manual_reseed = False

			elif prev_gray is not None and prev_pts is not None and len(prev_pts) >= 4:
				next_pts, st, _err = cv2.calcOpticalFlowPyrLK(prev_gray, gray, prev_pts, None, **LK_PARAMS)
				if next_pts is None or st is None:
					prev_pts = None
					prev_gray = gray
					status = "lk-failed"
				else:
					ok = st.reshape(-1).astype(bool)
					good_prev = prev_pts.reshape(-1, 2)[ok]
					good_next = next_pts.reshape(-1, 2)[ok]
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
							back_xy = back_pts.reshape(-1, 2)[fb_ok]
							fb_error = np.linalg.norm(good_prev - back_xy, axis=1)
							keep = fb_error <= args.fb_thresh
							good_prev = good_prev[keep]
							good_next = good_next[keep]
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
							inlier_count = int(len(in_next))

							if inlier_count >= 3:
								frame_dx = float(affine[0, 2])
								frame_dy = float(affine[1, 2])
								frame_angle = math.degrees(math.atan2(float(affine[1, 0]), float(affine[0, 0])))
								cumulative_dx += frame_dx
								cumulative_dy += frame_dy

								draw_step = max(1, len(in_next) // max(1, args.draw_points))
								for p0, p1 in zip(in_prev[::draw_step], in_next[::draw_step]):
									x0, y0 = int(p0[0]), int(p0[1])
									x1, y1 = int(p1[0]), int(p1[1])
									cv2.line(frame_bgr, (x0, y0), (x1, y1), (0, 160, 255), 1, cv2.LINE_AA)
									cv2.circle(frame_bgr, (x1, y1), 2, (0, 255, 0), -1, cv2.LINE_AA)

								prev_pts = in_next.reshape(-1, 1, 2).astype(np.float32)
								prev_gray = gray
								status = "tracking"
							else:
								prev_pts = None
								prev_gray = gray
								status = "ransac-low-inliers"
						else:
							flow = good_next - good_prev
							frame_dx = float(np.median(flow[:, 0]))
							frame_dy = float(np.median(flow[:, 1]))
							cumulative_dx += frame_dx
							cumulative_dy += frame_dy

							draw_step = max(1, len(good_next) // max(1, args.draw_points))
							for p0, p1 in zip(good_prev[::draw_step], good_next[::draw_step]):
								x0, y0 = int(p0[0]), int(p0[1])
								x1, y1 = int(p1[0]), int(p1[1])
								cv2.line(frame_bgr, (x0, y0), (x1, y1), (0, 110, 255), 1, cv2.LINE_AA)
								cv2.circle(frame_bgr, (x1, y1), 2, (80, 255, 80), -1, cv2.LINE_AA)

							prev_pts = good_next.reshape(-1, 1, 2).astype(np.float32)
							prev_gray = gray
							inlier_count = tracked_count
							status = "tracking-no-ransac"
					else:
						prev_pts = None
						prev_gray = gray
						status = "too-few-good-points"
			else:
				prev_gray = gray
				status = "waiting-for-seed"

			h, w = frame_bgr.shape[:2]
			if roi_scale < 0.999:
				roi_axes = (max(10, int(w * roi_scale * 0.5)), max(10, int(h * roi_scale * 0.5)))
				cv2.ellipse(frame_bgr, (w // 2, h // 2), roi_axes, 0, 0, 360, (255, 180, 0), 1, cv2.LINE_AA)

			frame_mm_x = frame_dx * args.px_to_mm
			frame_mm_y = frame_dy * args.px_to_mm
			cumulative_mm_x = cumulative_dx * args.px_to_mm
			cumulative_mm_y = cumulative_dy * args.px_to_mm

			put_lines(
				frame_bgr,
				[
					f"FPS:{smooth_fps:5.1f} status:{status}",
					f"points tracked={tracked_count} inliers={inlier_count} active={(0 if prev_pts is None else len(prev_pts))}",
					f"dX={frame_dx:+6.2f}px dY={frame_dy:+6.2f}px rot={frame_angle:+5.2f}deg",
					f"dX={frame_mm_x:+6.3f}mm dY={frame_mm_y:+6.3f}mm cum=({cumulative_mm_x:+7.3f},{cumulative_mm_y:+7.3f})mm",
					"keys: q/ESC quit | r reseed | s save",
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
			if key == ord("s"):
				ts = datetime.now().strftime("%Y%m%d_%H%M%S")
				frame_name = f"flow_frame_{ts}.png"
				mask_name = f"flow_mask_{ts}.png"
				cv2.imwrite(frame_name, frame_bgr)
				if last_mask is not None:
					cv2.imwrite(mask_name, last_mask)
				print(f"Saved {frame_name} and {mask_name}")
	finally:
		cam.stop()
		cv2.destroyAllWindows()


if __name__ == "__main__":
	main()
