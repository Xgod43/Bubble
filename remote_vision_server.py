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


SERVICE_BACKEND = "remote-cuda-opencv"
CUDA_PREPROCESS_ENABLED = os.environ.get(
    "BUBBLE_REMOTE_VISION_CUDA_PREPROCESS",
    "1",
).strip().lower() in {"1", "true", "yes", "on"}


class RemoteVisionState:
    def __init__(self):
        self.reference_centroids = None
        self.last_frame_shape = None
        self.last_frame_at = None
        self.last_process_ms = None
        self.last_process_backend = "not-started"
        self.cuda_error = None
        self.frame_count = 0
        self.started_at = time.perf_counter()

    def reset(self):
        self.reference_centroids = None
        self.last_frame_shape = None
        self.last_frame_at = None
        self.last_process_ms = None
        self.last_process_backend = "not-started"
        self.cuda_error = None
        self.frame_count = 0
        self.started_at = time.perf_counter()


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

    started_at = time.perf_counter()
    binary, centroids, match_dist, process_backend = _detect_centroids(frame_bgr, query)

    if reset_reference or not STATE.reference_centroids:
        STATE.reference_centroids = list(centroids)
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
        blob_vis = pipe.build_blob_view(binary, frame_bgr)
        vector_vis = pipe.draw_displacement_vectors(blob_vis, reference, displacements)
        ok, encoded = cv2.imencode(".jpg", vector_vis, [int(cv2.IMWRITE_JPEG_QUALITY), 72])
        if ok:
            payload["preview_jpeg_b64"] = base64.b64encode(encoded.tobytes()).decode("ascii")

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
