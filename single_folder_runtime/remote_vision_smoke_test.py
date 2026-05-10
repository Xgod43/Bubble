from __future__ import annotations

import argparse
import json
import time
from urllib.parse import urlencode
from urllib.request import Request, urlopen

import cv2
import numpy as np


def _synthetic_dot_frame(shift_x=0, shift_y=0, width=640, height=480):
    frame = np.full((height, width, 3), 235, dtype=np.uint8)
    rng = np.random.default_rng(42)
    xs = np.linspace(160, width - 160, 7, dtype=int)
    ys = np.linspace(120, height - 120, 5, dtype=int)
    for y in ys:
        for x in xs:
            jitter_x = int(rng.integers(-3, 4))
            jitter_y = int(rng.integers(-3, 4))
            center = (int(x + jitter_x + shift_x), int(y + jitter_y + shift_y))
            cv2.circle(frame, center, 7, (20, 20, 20), -1)
    return frame


def _encode_raw_frame(frame, raw_format):
    raw_format = str(raw_format).strip().lower()
    if raw_format in {"bgr24", "bgr", "bgr888"}:
        return np.ascontiguousarray(frame).tobytes(), raw_format
    if raw_format in {"rgb24", "rgb", "rgb888"}:
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        return np.ascontiguousarray(rgb).tobytes(), raw_format
    if raw_format in {"gray8", "grey8", "mono8"}:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return np.ascontiguousarray(gray).tobytes(), raw_format
    if raw_format in {"i420", "yuv420", "yuv420p"}:
        yuv = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV_I420)
        return np.ascontiguousarray(yuv).tobytes(), raw_format
    raise RuntimeError(f"Unsupported raw format: {raw_format}")


def _post_frame(url, frame, reset=False, preview=False, transport="raw", raw_format="bgr24"):
    query_params = {
        "reset": "1" if reset else "0",
        "preview": "1" if preview else "0",
        "points": "1",
        "mode": "dark",
        "use_roi": "1",
        "roi_scale": "0.9",
        "min_area": "20",
        "max_area": "800",
        "min_circularity": "0.35",
        "match_dist": "18",
    }
    endpoint = "process"
    content_type = "image/jpeg"
    if transport == "raw":
        height, width = frame.shape[:2]
        body, raw_format = _encode_raw_frame(frame, raw_format)
        query_params.update(
            {
                "width": str(width),
                "height": str(height),
                "format": raw_format,
            }
        )
        endpoint = "process_raw"
        content_type = "application/octet-stream"
    else:
        ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 82])
        if not ok:
            raise RuntimeError("Could not encode synthetic frame.")
        body = encoded.tobytes()
    query = urlencode(query_params)
    request = Request(
        f"{url}/{endpoint}?{query}",
        data=body,
        headers={"Content-Type": content_type},
        method="POST",
    )
    start = time.perf_counter()
    with urlopen(request, timeout=5) as response:
        payload = json.loads(response.read().decode("utf-8"))
    payload["roundtrip_ms"] = (time.perf_counter() - start) * 1000.0
    return payload


def main():
    parser = argparse.ArgumentParser(description="Smoke-test the temporary CUDA vision server.")
    parser.add_argument("--host", default="192.168.4.2")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--transport", choices=("jpeg", "raw"), default="raw")
    parser.add_argument("--raw-format", default="bgr24")
    args = parser.parse_args()

    url = f"http://{args.host}:{args.port}"
    reference = _synthetic_dot_frame(0, 0)
    shifted = _synthetic_dot_frame(5, 3)

    reset_payload = _post_frame(
        url,
        reference,
        reset=True,
        transport=args.transport,
        raw_format=args.raw_format,
    )
    shifted_payload = _post_frame(
        url,
        shifted,
        reset=False,
        transport=args.transport,
        raw_format=args.raw_format,
    )

    print("Reset response:")
    print(json.dumps(reset_payload, indent=2, sort_keys=True))
    print("\nShifted response:")
    print(json.dumps(shifted_payload, indent=2, sort_keys=True))

    mean_disp = float(shifted_payload.get("mean_disp", 0.0))
    if mean_disp <= 1.0:
        raise SystemExit("Remote vision smoke test failed: displacement was too small.")
    print(f"\nOK: remote mean displacement = {mean_disp:.3f} px")


if __name__ == "__main__":
    main()
