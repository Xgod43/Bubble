import argparse
import csv
import json
import os
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

ROOT_DIR = Path(__file__).resolve().parents[1]
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

try:
    from backend.vision_native import NativeBlobDetector
except Exception:
    NativeBlobDetector = None

try:
    from picamera2 import Picamera2
except Exception:
    Picamera2 = None

try:
    from libcamera import controls as libcamera_controls
except Exception:
    libcamera_controls = None

MIN_BLOB_AREA = 6
MAX_BLOB_AREA = 8000
MIN_CIRCULARITY = 0.35
MEDIAN_BLUR_KSIZE = 5
MORPH_ITERATIONS = 1

_NATIVE_DETECTOR = NativeBlobDetector() if NativeBlobDetector is not None else None
_NATIVE_DETECTOR_ERROR = None


def native_detector_available() -> bool:
    return bool(_NATIVE_DETECTOR is not None and _NATIVE_DETECTOR.available)


def get_detection_backend_name() -> str:
    return "native-c" if native_detector_available() else "opencv-python"


def preprocess(image: np.ndarray) -> np.ndarray:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    if MEDIAN_BLUR_KSIZE >= 3 and MEDIAN_BLUR_KSIZE % 2 == 1:
        gray = cv2.medianBlur(gray, MEDIAN_BLUR_KSIZE)
    blur = cv2.GaussianBlur(gray, (3, 3), 0)
    return blur


def _build_binary(preprocessed: np.ndarray, invert: bool) -> np.ndarray:
    thresh_type = cv2.THRESH_BINARY_INV if invert else cv2.THRESH_BINARY
    adaptive = cv2.adaptiveThreshold(
        preprocessed,
        255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        thresh_type,
        21,
        3,
    )

    _ret, otsu = cv2.threshold(
        preprocessed,
        0,
        255,
        thresh_type + cv2.THRESH_OTSU,
    )

    thresh = cv2.bitwise_or(adaptive, otsu)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=MORPH_ITERATIONS)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=1)
    return thresh


def _estimate_blob_score(binary: np.ndarray) -> int:
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    count = 0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if MIN_BLOB_AREA <= area <= MAX_BLOB_AREA:
            count += 1
    return count


def threshold_image(preprocessed: np.ndarray, polarity: str = "auto") -> np.ndarray:
    polarity = polarity.lower()
    dark_binary = _build_binary(preprocessed, invert=True)
    if polarity == "dark":
        return dark_binary

    bright_binary = _build_binary(preprocessed, invert=False)
    if polarity == "bright":
        return bright_binary

    dark_score = _estimate_blob_score(dark_binary)
    bright_score = _estimate_blob_score(bright_binary)
    return dark_binary if dark_score >= bright_score else bright_binary


def detect_blobs(binary: np.ndarray):
    params = cv2.SimpleBlobDetector_Params()
    params.filterByArea = True
    params.minArea = MIN_BLOB_AREA
    params.maxArea = MAX_BLOB_AREA
    params.filterByCircularity = True
    params.minCircularity = MIN_CIRCULARITY
    params.filterByConvexity = True
    params.minConvexity = 0.7
    params.filterByInertia = True
    params.minInertiaRatio = 0.3
    params.filterByColor = True
    params.blobColor = 255

    detector = cv2.SimpleBlobDetector_create(params)
    keypoints = detector.detect(binary)
    return keypoints


def extract_centroids(keypoints):
    return [(int(k.pt[0]), int(k.pt[1])) for k in keypoints]


def detect_centroids_from_binary(
    binary: np.ndarray,
    min_area=MIN_BLOB_AREA,
    max_area=MAX_BLOB_AREA,
    min_circularity: float = MIN_CIRCULARITY,
):
    global _NATIVE_DETECTOR_ERROR
    if native_detector_available():
        try:
            blobs = _NATIVE_DETECTOR.detect(
                binary,
                min_area=int(min_area),
                max_area=int(max_area),
                min_circularity=float(min_circularity),
            )
            return [(blob.x, blob.y) for blob in blobs]
        except Exception as exc:
            _NATIVE_DETECTOR_ERROR = str(exc)

    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    centroids = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area or area > max_area:
            continue
        perimeter = cv2.arcLength(cnt, True)
        if perimeter <= 0:
            continue
        circularity = 4 * np.pi * (area / (perimeter * perimeter))
        if circularity < min_circularity:
            continue
        m = cv2.moments(cnt)
        if m["m00"] == 0:
            continue
        cx = int(m["m10"] / m["m00"])
        cy = int(m["m01"] / m["m00"])
        centroids.append((cx, cy))
    return centroids


def build_blob_view(binary: np.ndarray, base_image: np.ndarray) -> np.ndarray:
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blob_vis = base_image.copy()
    filtered = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < MIN_BLOB_AREA or area > MAX_BLOB_AREA:
            continue
        perimeter = cv2.arcLength(cnt, True)
        if perimeter <= 0:
            continue
        circularity = 4 * np.pi * (area / (perimeter * perimeter))
        if circularity < MIN_CIRCULARITY:
            continue
        filtered.append(cnt)
    cv2.drawContours(blob_vis, filtered, -1, (0, 255, 0), 2)
    return blob_vis


def draw_displacement_vectors(image, reference_centroids, displacements):
    vis = image.copy()
    if not reference_centroids:
        return vis
    for (x, y), disp in zip(reference_centroids, displacements):
        if disp is None:
            continue
        dx, dy = disp
        end_pt = (int(x + dx), int(y + dy))
        cv2.circle(vis, (int(x), int(y)), 3, (0, 255, 0), -1)
        cv2.arrowedLine(vis, (int(x), int(y)), end_pt, (0, 0, 255), 1, tipLength=0.3)
    return vis


def build_displacement_heatmap(image_shape, reference_centroids, displacements):
    h, w = image_shape[:2]
    value_map = np.zeros((h, w), dtype=np.float32)
    if not reference_centroids:
        return cv2.applyColorMap(value_map.astype(np.uint8), cv2.COLORMAP_JET)
    for (x, y), disp in zip(reference_centroids, displacements):
        if disp is None:
            continue
        dx, dy = disp
        mag = float(np.hypot(dx, dy))
        if 0 <= x < w and 0 <= y < h:
            value_map[int(y), int(x)] = mag

    if np.count_nonzero(value_map) > 0:
        value_map = cv2.GaussianBlur(value_map, (0, 0), 7)
        value_map = cv2.normalize(value_map, None, 0, 255, cv2.NORM_MINMAX)

    heatmap = cv2.applyColorMap(value_map.astype(np.uint8), cv2.COLORMAP_JET)
    return heatmap


def overlay_heatmap(image, heatmap, alpha=0.45):
    return cv2.addWeighted(image, 1 - alpha, heatmap, alpha, 0)


def build_pointcloud_view(
    image_shape,
    reference_centroids: List[Tuple[int, int]],
    displacements: List[Optional[Tuple[float, float]]],
) -> np.ndarray:
    h, w = image_shape[:2]
    panel = np.zeros((h, w, 3), dtype=np.uint8)

    if not reference_centroids:
        cv2.putText(panel, "No points", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (180, 180, 180), 2)
        return panel

    mags = []
    for disp in displacements:
        if disp is None:
            mags.append(0.0)
        else:
            mags.append(float(np.hypot(disp[0], disp[1])))
    max_mag = max(mags) if mags else 1.0
    if max_mag < 1e-6:
        max_mag = 1.0

    for (x, y), mag in zip(reference_centroids, mags):
        z = mag / max_mag
        px = int(np.clip(x + z * 40.0, 0, w - 1))
        py = int(np.clip(y - z * 60.0, 0, h - 1))
        color = (int(255 * z), 255 - int(180 * z), 255)
        cv2.circle(panel, (px, py), 2, color, -1)

    cv2.line(panel, (40, h - 40), (120, h - 40), (100, 100, 255), 2)
    cv2.line(panel, (40, h - 40), (40, h - 120), (100, 255, 100), 2)
    cv2.putText(panel, "X", (125, h - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 255), 1)
    cv2.putText(panel, "Y", (30, h - 125), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 255, 100), 1)
    return panel


def build_mosaic(pre, thresh, blob_vis, heatmap_vis, pointcloud_vis=None, scale=0.6):
    h, w = pre.shape[:2]
    scale = max(0.1, float(scale))
    tile_w = max(1, int(w * scale))
    tile_h = max(1, int(h * scale))

    pre_bgr = cv2.cvtColor(pre, cv2.COLOR_GRAY2BGR)
    thresh_bgr = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)

    def resize(img):
        return cv2.resize(img, (tile_w, tile_h), interpolation=cv2.INTER_AREA)

    if pointcloud_vis is None:
        top = np.hstack([resize(pre_bgr), resize(thresh_bgr)])
        bottom = np.hstack([resize(blob_vis), resize(heatmap_vis)])
        mosaic = np.vstack([top, bottom])
    else:
        blank = np.zeros_like(pointcloud_vis)
        top = np.hstack([resize(pre_bgr), resize(thresh_bgr), resize(pointcloud_vis)])
        bottom = np.hstack([resize(blob_vis), resize(heatmap_vis), resize(blank)])
        mosaic = np.vstack([top, bottom])

    font_scale = 0.6 if scale < 0.75 else 0.8
    thickness = 2 if scale >= 0.75 else 1
    cv2.putText(mosaic, "01 Preprocessed", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 255, 0), thickness)
    cv2.putText(mosaic, "02 Threshold", (tile_w + 10, 25), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 255, 0), thickness)
    if pointcloud_vis is not None:
        cv2.putText(mosaic, "03 PointCloud", (2 * tile_w + 10, 25), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 255, 0), thickness)
    cv2.putText(mosaic, "04 Blobs", (10, tile_h + 25), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 255, 0), thickness)
    cv2.putText(mosaic, "05 Heatmap", (tile_w + 10, tile_h + 25), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 255, 0), thickness)
    return mosaic


def load_metadata(metadata_path: Optional[Path]) -> Dict[str, List[Tuple[int, int]]]:
    if metadata_path is None or not metadata_path.exists():
        return {}
    data = json.loads(metadata_path.read_text(encoding="utf-8"))
    mapping: Dict[str, List[Tuple[int, int]]] = {}
    for item in data.get("images", []):
        name = item.get("file")
        centroids = [tuple(c) for c in item.get("centroids", [])]
        if name:
            mapping[name] = centroids
    return mapping


def match_centroids(pred: List[Tuple[int, int]], gt: List[Tuple[int, int]], max_dist: float):
    if not pred or not gt:
        return [], set(range(len(gt)))

    candidates = []
    for i, p in enumerate(pred):
        for j, g in enumerate(gt):
            d = np.hypot(p[0] - g[0], p[1] - g[1])
            if d <= max_dist:
                candidates.append((d, i, j))

    candidates.sort(key=lambda x: x[0])
    matched_pred = set()
    matched_gt = set()
    matches = []
    for d, i, j in candidates:
        if i in matched_pred or j in matched_gt:
            continue
        matched_pred.add(i)
        matched_gt.add(j)
        matches.append((i, j, d))

    unmatched_gt = set(range(len(gt))) - matched_gt
    return matches, unmatched_gt


def compute_displacements(reference_centroids, current_centroids, max_dist: float):
    if not reference_centroids:
        return [], []
    matches, unmatched_ref = match_centroids(current_centroids, reference_centroids, max_dist)
    displacements: List[Optional[Tuple[float, float]]] = [None] * len(reference_centroids)
    for cur_idx, ref_idx, _dist in matches:
        cx, cy = current_centroids[cur_idx]
        rx, ry = reference_centroids[ref_idx]
        displacements[ref_idx] = (float(cx - rx), float(cy - ry))
    return displacements, list(unmatched_ref)


def update_tracks(
    tracks: Dict[int, Tuple[int, int]],
    centroids: List[Tuple[int, int]],
    max_dist: float,
    next_id: int,
    smooth_alpha: float = 0.6,
):
    if not tracks:
        for c in centroids:
            tracks[next_id] = c
            next_id += 1
        return tracks, {}, len(centroids), 0, next_id

    track_ids = list(tracks.keys())
    track_points = [tracks[t] for t in track_ids]
    matches, unmatched_tracks = match_centroids(centroids, track_points, max_dist)

    assigned = {}
    for cur_idx, track_idx, _dist in matches:
        tid = track_ids[track_idx]
        assigned[tid] = centroids[cur_idx]

    matched_indices = {m[0] for m in matches}
    new_count = 0
    for i, c in enumerate(centroids):
        if i not in matched_indices:
            tracks[next_id] = c
            next_id += 1
            new_count += 1

    for tid, pos in assigned.items():
        old = tracks.get(tid, pos)
        new_x = int(old[0] * (1 - smooth_alpha) + pos[0] * smooth_alpha)
        new_y = int(old[1] * (1 - smooth_alpha) + pos[1] * smooth_alpha)
        tracks[tid] = (new_x, new_y)

    lost_count = len(unmatched_tracks)
    return tracks, assigned, new_count, lost_count, next_id


def robustness_test(image, max_dist: float):
    variants = []
    variants.append(("base", image))
    noise = image.astype(np.float32) + np.random.normal(0, 8, image.shape)
    noise = np.clip(noise, 0, 255).astype(np.uint8)
    variants.append(("noise", noise))
    blur = cv2.GaussianBlur(image, (5, 5), 0)
    variants.append(("blur", blur))
    bright = cv2.convertScaleAbs(image, alpha=1.1, beta=10)
    variants.append(("bright", bright))

    results = []
    for name, img in variants:
        pre = preprocess(img)
        thresh = threshold_image(pre)
        keypoints = detect_blobs(thresh)
        centroids = extract_centroids(keypoints)
        results.append((name, len(centroids)))
    return results


def init_timeseries_log(log_path: Optional[Path]):
    if log_path is None:
        return None
    log_path.parent.mkdir(parents=True, exist_ok=True)
    f = log_path.open("w", newline="", encoding="utf-8")
    writer = csv.writer(f)
    writer.writerow([
        "frame",
        "timestamp",
        "num_dots",
        "mean_disp",
        "max_disp",
        "missing_ratio",
        "new_tracks",
        "lost_tracks",
    ])
    return f, writer


def evaluate_detection(pred: List[Tuple[int, int]], gt: List[Tuple[int, int]], max_dist: float):
    matches, unmatched_gt = match_centroids(pred, gt, max_dist)
    tp = len(matches)
    fp = len(pred) - tp
    fn = len(unmatched_gt)
    precision = tp / (tp + fp) if (tp + fp) > 0 else 0.0
    recall = tp / (tp + fn) if (tp + fn) > 0 else 0.0
    loc_err = np.mean([m[2] for m in matches]) if matches else 0.0
    return precision, recall, loc_err, tp, fp, fn


def iter_images(input_path: Path) -> List[Path]:
    if input_path.is_file():
        return [input_path]
    images = []
    for ext in ("*.png", "*.jpg", "*.jpeg", "*.bmp"):
        images.extend(sorted(input_path.glob(ext)))
    return images


def process_image(
    image_path: Path,
    gt_map: Dict[str, List[Tuple[int, int]]],
    max_dist: float,
    reference_centroids: Optional[List[Tuple[int, int]]] = None,
    proc_scale: float = 1.0,
    dot_polarity: str = "auto",
):
    image = cv2.imread(str(image_path))
    if image is None:
        raise ValueError(f"Failed to read image: {image_path}")

    if proc_scale != 1.0:
        image = cv2.resize(
            image,
            (int(image.shape[1] * proc_scale), int(image.shape[0] * proc_scale)),
            interpolation=cv2.INTER_AREA,
        )

    pre = preprocess(image)
    thresh = threshold_image(pre, dot_polarity)
    centroids = detect_centroids_from_binary(thresh)
    if not centroids:
        keypoints = detect_blobs(thresh)
        centroids = extract_centroids(keypoints)
    if (reference_centroids is None or len(reference_centroids) == 0) and centroids:
        reference_centroids = centroids
    displacements, unmatched_ref = compute_displacements(reference_centroids, centroids, max_dist)
    vector_vis = draw_displacement_vectors(image, reference_centroids, displacements)
    heatmap = build_displacement_heatmap(image.shape, reference_centroids, displacements)
    heatmap_vis = overlay_heatmap(image, heatmap)
    blob_vis = build_blob_view(thresh, image)

    metrics = None
    gt = gt_map.get(image_path.name)
    if gt is not None:
        metrics = evaluate_detection(centroids, gt, max_dist)

    return (
        image,
        pre,
        thresh,
        blob_vis,
        heatmap_vis,
        centroids,
        metrics,
        reference_centroids,
        unmatched_ref,
        displacements,
    )


def process_frame(
    frame: np.ndarray,
    max_dist: float,
    reference_centroids: Optional[List[Tuple[int, int]]],
    proc_scale: float = 1.0,
    dot_polarity: str = "auto",
):
    if proc_scale != 1.0:
        frame = cv2.resize(
            frame,
            (int(frame.shape[1] * proc_scale), int(frame.shape[0] * proc_scale)),
            interpolation=cv2.INTER_AREA,
        )
    pre = preprocess(frame)
    thresh = threshold_image(pre, dot_polarity)
    centroids = detect_centroids_from_binary(thresh)
    if not centroids:
        keypoints = detect_blobs(thresh)
        centroids = extract_centroids(keypoints)

    if (reference_centroids is None or len(reference_centroids) == 0) and centroids:
        reference_centroids = centroids
    displacements, unmatched_ref = compute_displacements(reference_centroids, centroids, max_dist)
    heatmap = build_displacement_heatmap(frame.shape, reference_centroids, displacements)
    heatmap_vis = overlay_heatmap(frame, heatmap)
    blob_vis = build_blob_view(thresh, frame)

    return pre, thresh, blob_vis, heatmap_vis, centroids, reference_centroids, unmatched_ref, displacements


def open_camera_stream(
    camera_index: int,
    width: int,
    height: int,
    backend: str,
    autofocus: str,
    lens_position: float,
    exposure_ev: float,
    exposure_time_us=None,
    analogue_gain=None,
    awb_mode: str = "auto",
    colour_gains=None,
):
    backend = backend.lower()
    if backend == "picamera3":
        backend = "picamera2"
    last_error = None

    if backend in {"auto", "picamera2"}:
        if Picamera2 is None:
            last_error = "Picamera2 is not installed"
        else:
            try:
                picam = Picamera2(camera_index)
                config = picam.create_preview_configuration(
                    main={"size": (width, height), "format": "RGB888"}
                )
                picam.configure(config)
                picam.start()
                controls_payload = {}
                if autofocus == "continuous" and libcamera_controls is not None:
                    controls_payload["AfMode"] = libcamera_controls.AfModeEnum.Continuous
                elif autofocus == "manual":
                    controls_payload["LensPosition"] = float(lens_position)
                    if libcamera_controls is not None:
                        controls_payload["AfMode"] = libcamera_controls.AfModeEnum.Manual
                elif autofocus == "off" and libcamera_controls is not None:
                    controls_payload["AfMode"] = libcamera_controls.AfModeEnum.Manual

                if exposure_time_us is not None or analogue_gain is not None:
                    controls_payload["AeEnable"] = False
                    if exposure_time_us is not None:
                        controls_payload["ExposureTime"] = int(exposure_time_us)
                    if analogue_gain is not None:
                        controls_payload["AnalogueGain"] = float(analogue_gain)
                else:
                    controls_payload["ExposureValue"] = float(exposure_ev)

                awb_mode = str(awb_mode or "auto").lower()
                if awb_mode == "manual":
                    controls_payload["AwbEnable"] = False
                    if colour_gains is not None:
                        controls_payload["ColourGains"] = tuple(float(value) for value in colour_gains)
                elif awb_mode == "auto":
                    controls_payload["AwbEnable"] = True

                scaler_crop = picam.camera_properties.get("ScalerCropMaximum")
                if scaler_crop:
                    controls_payload["ScalerCrop"] = tuple(scaler_crop)
                available_controls = getattr(picam, "camera_controls", {}) or {}
                if available_controls:
                    controls_payload = {
                        name: value
                        for name, value in controls_payload.items()
                        if name in available_controls
                    }
                try:
                    picam.set_controls(controls_payload)
                except Exception:
                    if "ScalerCrop" not in controls_payload:
                        raise
                    controls_payload.pop("ScalerCrop", None)
                    picam.set_controls(controls_payload)
                time.sleep(0.35 if exposure_time_us is not None or awb_mode == "manual" else 0.2)
                return {"backend": "picamera2", "device": picam, "controls": controls_payload}
            except Exception as exc:
                last_error = str(exc)
                if backend == "picamera2":
                    raise RuntimeError(f"Could not open Picamera2 camera {camera_index}: {exc}") from exc

    if backend in {"auto", "opencv"}:
        cap = cv2.VideoCapture(camera_index)
        if cap.isOpened():
            if width > 0:
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            if height > 0:
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            return {"backend": "opencv", "device": cap}
        cap.release()
        if backend == "opencv":
            raise RuntimeError(f"Could not open OpenCV camera index {camera_index}")

    raise RuntimeError(
        "Could not open camera with available backends. "
        f"Last Picamera2 error: {last_error}. "
        "Install Picamera2 (python3-picamera2) for Raspberry Pi Camera Module 3 Wide."
    )


def read_camera_frame(stream):
    if stream["backend"] == "picamera2":
        frame = stream["device"].capture_array()
        if frame is None:
            return False, None
        if len(frame.shape) == 3 and frame.shape[2] == 3:
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        return True, frame

    return stream["device"].read()


def close_camera_stream(stream):
    if stream["backend"] == "picamera2":
        try:
            stream["device"].stop()
        except Exception:
            pass
    else:
        stream["device"].release()


def can_open_display() -> bool:
    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


def main():
    parser = argparse.ArgumentParser(description="Dot detection pipeline")
    parser.add_argument(
        "--input",
        type=str,
        default="camera",
        help="Input image, folder of images, or 'camera'",
    )
    parser.add_argument("--camera-index", type=int, default=0, help="Camera index")
    parser.add_argument(
        "--camera-backend",
        type=str,
        default="auto",
        choices=["auto", "opencv", "picamera2", "picamera3"],
        help="Camera backend selection (picamera3 is an alias of picamera2)",
    )
    parser.add_argument("--cam-width", type=int, default=2304, help="Camera frame width")
    parser.add_argument("--cam-height", type=int, default=1296, help="Camera frame height")
    parser.add_argument(
        "--cam-autofocus",
        type=str,
        default="continuous",
        choices=["continuous", "manual", "off"],
        help="Autofocus mode for Picamera backends",
    )
    parser.add_argument(
        "--lens-position",
        type=float,
        default=1.0,
        help="Lens position when --cam-autofocus manual is used",
    )
    parser.add_argument(
        "--cam-exposure-ev",
        type=float,
        default=0.8,
        help="Exposure compensation (EV) for Picamera backends",
    )
    parser.add_argument(
        "--metadata",
        type=str,
        default="",
        help="Optional JSON metadata with ground-truth centroids",
    )
    parser.add_argument("--display", action="store_true", help="Display output windows")
    parser.add_argument("--delay", type=int, default=0, help="Delay (ms) between frames")
    parser.add_argument("--match-dist", type=float, default=8.0, help="Max match distance")
    parser.add_argument("--scale", type=float, default=0.6, help="Mosaic scale")
    parser.add_argument("--proc-scale", type=float, default=1.0, help="Processing scale (e.g., 0.5)")
    parser.add_argument(
        "--dot-polarity",
        type=str,
        default="auto",
        choices=["auto", "dark", "bright"],
        help="Dot polarity against background",
    )
    parser.add_argument("--log", type=str, default="", help="CSV log path for time-series")
    parser.add_argument("--robust-test", action="store_true", help="Run robustness test on each image")
    parser.add_argument("--pointcloud", action="store_true", default=True, help="Show point cloud of dot displacement")
    parser.add_argument("--no-pointcloud", action="store_false", dest="pointcloud", help="Disable point cloud view")
    args = parser.parse_args()

    workspace = Path(__file__).resolve().parent
    use_camera = args.input.lower() in {"camera", "0"}
    input_path = (workspace / args.input).resolve()
    if not use_camera and not input_path.exists():
        raise FileNotFoundError(f"Input not found: {input_path}")

    metadata_path = (workspace / args.metadata).resolve() if args.metadata else None
    gt_map = load_metadata(metadata_path)

    log_path = (workspace / args.log).resolve() if args.log else None
    log_handle = init_timeseries_log(log_path)

    tracks: Dict[int, Tuple[int, int]] = {}
    next_id = 0
    reference_centroids = None
    frame_idx = 0
    gui_available = can_open_display()
    should_display = args.display and gui_available
    if args.display and not gui_available:
        print("Display requested but no GUI session detected; running headless.")

    if use_camera:
        stream = open_camera_stream(
            args.camera_index,
            args.cam_width,
            args.cam_height,
            args.camera_backend,
            args.cam_autofocus,
            args.lens_position,
            args.cam_exposure_ev,
        )

        while True:
            ok, frame = read_camera_frame(stream)
            if not ok:
                break

            (
                pre,
                thresh,
                    blob_vis,
                heatmap_vis,
                centroids,
                reference_centroids,
                unmatched_ref,
                displacements,
            ) = process_frame(
                frame,
                args.match_dist,
                reference_centroids,
                args.proc_scale,
                args.dot_polarity,
            )

            tracks, _assigned, new_count, lost_count, next_id = update_tracks(
                tracks, centroids, args.match_dist, next_id
            )

            disp_vals = [np.hypot(d[0], d[1]) for d in displacements if d is not None]
            mean_disp = float(np.mean(disp_vals)) if disp_vals else 0.0
            max_disp = float(np.max(disp_vals)) if disp_vals else 0.0
            missing_ratio = (
                len(unmatched_ref) / len(reference_centroids) if reference_centroids else 0.0
            )

            if log_handle:
                f, writer = log_handle
                writer.writerow([
                    frame_idx,
                    time.time(),
                    len(centroids),
                    f"{mean_disp:.4f}",
                    f"{max_disp:.4f}",
                    f"{missing_ratio:.4f}",
                    new_count,
                    lost_count,
                ])

            if should_display:
                pointcloud_vis = None
                if args.pointcloud:
                    pointcloud_points = reference_centroids if reference_centroids else centroids
                    if pointcloud_points is None:
                        pointcloud_points = []
                    pointcloud_displacements = displacements
                    if len(pointcloud_displacements) != len(pointcloud_points):
                        pointcloud_displacements = [(0.0, 0.0)] * len(pointcloud_points)
                    pointcloud_vis = build_pointcloud_view(
                        frame.shape,
                        pointcloud_points,
                        pointcloud_displacements,
                    )
                mosaic = build_mosaic(
                    pre,
                    thresh,
                    blob_vis,
                    heatmap_vis,
                    pointcloud_vis=pointcloud_vis,
                    scale=args.scale,
                )
                cv2.imshow("Dot Pipeline", mosaic)
                key = cv2.waitKey(1)
                if key != -1:
                    if key == 27 or key == ord("q"):
                        break
                    if key == ord("r"):
                        if centroids:
                            reference_centroids = centroids

            frame_idx += 1

        close_camera_stream(stream)
    else:
        images = iter_images(input_path)
        if not images:
            raise FileNotFoundError(f"No images found in: {input_path}")

        aggregate = []
        for image_path in images:
            (
                image,
                pre,
                thresh,
                blob_vis,
                heatmap_vis,
                centroids,
                metrics,
                reference_centroids,
                unmatched_ref,
                displacements,
            ) = process_image(
                image_path,
                gt_map,
                args.match_dist,
                reference_centroids,
                args.proc_scale,
                args.dot_polarity,
            )

            tracks, _assigned, new_count, lost_count, next_id = update_tracks(
                tracks, centroids, args.match_dist, next_id
            )

            disp_vals = [np.hypot(d[0], d[1]) for d in displacements if d is not None]
            mean_disp = float(np.mean(disp_vals)) if disp_vals else 0.0
            max_disp = float(np.max(disp_vals)) if disp_vals else 0.0
            missing_ratio = (
                len(unmatched_ref) / len(reference_centroids) if reference_centroids else 0.0
            )

            print(f"{image_path.name}: detected {len(centroids)} dots")
            print(
                f"  MeanDisp: {mean_disp:.2f}  MaxDisp: {max_disp:.2f}  Missing: {missing_ratio:.2f}  "
                f"New: {new_count}  Lost: {lost_count}"
            )

            if metrics is not None:
                precision, recall, loc_err, tp, fp, fn = metrics
                aggregate.append(metrics)
                print(
                    f"  Precision: {precision:.3f}  Recall: {recall:.3f}  "
                    f"LocErr(px): {loc_err:.2f}  TP/FP/FN: {tp}/{fp}/{fn}"
                )

            if args.robust_test:
                results = robustness_test(image, args.match_dist)
                summary = ", ".join([f"{name}:{count}" for name, count in results])
                print(f"  Robustness: {summary}")

            if log_handle:
                f, writer = log_handle
                writer.writerow([
                    frame_idx,
                    time.time(),
                    len(centroids),
                    f"{mean_disp:.4f}",
                    f"{max_disp:.4f}",
                    f"{missing_ratio:.4f}",
                    new_count,
                    lost_count,
                ])

            if should_display:
                pointcloud_vis = None
                if args.pointcloud:
                    pointcloud_points = reference_centroids if reference_centroids else centroids
                    if pointcloud_points is None:
                        pointcloud_points = []
                    pointcloud_displacements = displacements
                    if len(pointcloud_displacements) != len(pointcloud_points):
                        pointcloud_displacements = [(0.0, 0.0)] * len(pointcloud_points)
                    pointcloud_vis = build_pointcloud_view(
                        image.shape,
                        pointcloud_points,
                        pointcloud_displacements,
                    )
                mosaic = build_mosaic(
                    pre,
                    thresh,
                    blob_vis,
                    heatmap_vis,
                    pointcloud_vis=pointcloud_vis,
                    scale=args.scale,
                )
                cv2.imshow("Dot Pipeline", mosaic)
                key = cv2.waitKey(args.delay if args.delay > 0 else 0)
                if key != -1 and key == 27:
                    break

            frame_idx += 1

        if aggregate:
            avg_precision = float(np.mean([m[0] for m in aggregate]))
            avg_recall = float(np.mean([m[1] for m in aggregate]))
            avg_loc = float(np.mean([m[2] for m in aggregate]))
            print(
                f"Average Precision: {avg_precision:.3f}  "
                f"Average Recall: {avg_recall:.3f}  Average LocErr(px): {avg_loc:.2f}"
            )

    if log_handle:
        f, _writer = log_handle
        f.close()

    if should_display:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
