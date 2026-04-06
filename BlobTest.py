import cv2
import numpy as np
from picamera2 import Picamera2

try:
    from libcamera import controls as libcamera_controls
except Exception:
    libcamera_controls = None


CONTROL_WIN = "Controls"
VIEW_WIN = "Dot Pattern Detection"
BINARY_WIN = "Binary"
GRAY_WIN = "Grayscale"


DEFAULTS = {
    "cam_width": 1920,
    "cam_height": 1080,
    "mode": "dark",  # auto | dark | bright
    "min_area": 220.0,
    "max_area": 3600.0,
    "min_circularity": 0.32,
    "max_aspect_ratio": 2.0,
    "min_solidity": 0.85,
    "open_iters": 4,
    "close_iters": 3,
    "use_roi": True,
    "roi_scale": 0.68,
    "use_hsv": False,
    "h_min": 0,
    "h_max": 179,
    "s_min": 0,
    "s_max": 255,
    "v_min": 0,
    "v_max": 255,
}


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def noop(_value):
    pass


def preprocess(frame_rgb: np.ndarray) -> np.ndarray:
    gray = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY)
    gray = cv2.medianBlur(gray, 5)
    clahe = cv2.createCLAHE(clipLimit=2.2, tileGridSize=(8, 8))
    gray = clahe.apply(gray)
    blur = cv2.GaussianBlur(gray, (3, 3), 0)
    return cv2.addWeighted(gray, 1.6, blur, -0.6, 0)


def build_binary(pre: np.ndarray, invert: bool, open_iters: int, close_iters: int) -> np.ndarray:
    threshold_type = cv2.THRESH_BINARY_INV if invert else cv2.THRESH_BINARY
    adaptive = cv2.adaptiveThreshold(
        pre,
        255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        threshold_type,
        21,
        3,
    )
    _ret, otsu = cv2.threshold(pre, 0, 255, threshold_type + cv2.THRESH_OTSU)
    out = cv2.bitwise_or(adaptive, otsu)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    out = cv2.morphologyEx(out, cv2.MORPH_OPEN, kernel, iterations=max(0, int(open_iters)))
    out = cv2.morphologyEx(out, cv2.MORPH_CLOSE, kernel, iterations=max(0, int(close_iters)))
    return out


def build_roi_mask(shape, roi_scale: float) -> np.ndarray:
    h, w = shape[:2]
    mask = np.zeros((h, w), dtype=np.uint8)
    cx, cy = w // 2, h // 2
    ax = max(10, int(w * clamp(roi_scale, 0.1, 1.0) * 0.5))
    ay = max(10, int(h * clamp(roi_scale, 0.1, 1.0) * 0.5))
    cv2.ellipse(mask, (cx, cy), (ax, ay), 0, 0, 360, 255, -1)
    return mask


def build_hsv_mask(frame_rgb: np.ndarray, p: dict):
    if not p["use_hsv"]:
        return None
    hsv = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2HSV)
    lower = np.array([p["h_min"], p["s_min"], p["v_min"]], dtype=np.uint8)
    upper = np.array([p["h_max"], p["s_max"], p["v_max"]], dtype=np.uint8)
    return cv2.inRange(hsv, lower, upper)


def build_combined_mask(roi_mask, hsv_mask):
    if roi_mask is None:
        return hsv_mask
    if hsv_mask is None:
        return roi_mask
    return cv2.bitwise_and(roi_mask, hsv_mask)


def detect_shapes(binary: np.ndarray, p: dict):
    contours, _hier = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    items = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < p["min_area"] or area > p["max_area"]:
            continue

        hull = cv2.convexHull(cnt)
        hull_area = cv2.contourArea(hull)
        if hull_area <= 0:
            continue
        if area / hull_area < p["min_solidity"]:
            continue

        perim = cv2.arcLength(cnt, True)
        if perim <= 0:
            continue
        circularity = 4 * np.pi * area / (perim * perim)
        if circularity < p["min_circularity"]:
            continue

        if len(cnt) >= 5:
            (x, y), (a, b), angle = cv2.fitEllipse(cnt)
            major = float(max(a, b))
            minor = float(min(a, b))
            if minor <= 1e-6:
                continue
            if major / minor > p["max_aspect_ratio"]:
                continue
            items.append(("ellipse", int(x), int(y), int(major / 2), int(minor / 2), float(angle)))
        else:
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            items.append(("circle", int(x), int(y), int(radius), int(radius), 0.0))
    return items


def choose_binary(pre: np.ndarray, p: dict, combined_mask):
    mode = p["mode"]

    def run(invert: bool):
        b = build_binary(pre, invert=invert, open_iters=p["open_iters"], close_iters=p["close_iters"])
        if combined_mask is not None:
            b = cv2.bitwise_and(b, combined_mask)
        shapes = detect_shapes(b, p)
        return b, shapes

    if mode == "dark":
        b, s = run(True)
        return b, s, "dark"
    if mode == "bright":
        b, s = run(False)
        return b, s, "bright"

    dark_b, dark_s = run(True)
    bright_b, bright_s = run(False)
    if len(dark_s) >= len(bright_s):
        return dark_b, dark_s, "dark"
    return bright_b, bright_s, "bright"


def init_windows():
    cv2.namedWindow(CONTROL_WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(CONTROL_WIN, 560, 520)
    cv2.namedWindow(VIEW_WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(VIEW_WIN, 1200, 700)
    cv2.namedWindow(BINARY_WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(BINARY_WIN, 600, 350)
    cv2.namedWindow(GRAY_WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(GRAY_WIN, 600, 350)


def init_trackbars(p: dict):
    cv2.createTrackbar("Mode 0A 1D 2B", CONTROL_WIN, {"auto": 0, "dark": 1, "bright": 2}[p["mode"]], 2, noop)
    cv2.createTrackbar("ROI 0/1", CONTROL_WIN, int(p["use_roi"]), 1, noop)
    cv2.createTrackbar("ROI %", CONTROL_WIN, int(p["roi_scale"] * 100), 100, noop)

    cv2.createTrackbar("MinArea", CONTROL_WIN, int(p["min_area"]), 20000, noop)
    cv2.createTrackbar("MaxArea", CONTROL_WIN, int(p["max_area"]), 20000, noop)
    cv2.createTrackbar("Circ x100", CONTROL_WIN, int(p["min_circularity"] * 100), 95, noop)
    cv2.createTrackbar("Solid x100", CONTROL_WIN, int(p["min_solidity"] * 100), 99, noop)
    cv2.createTrackbar("Aspect x10", CONTROL_WIN, int(p["max_aspect_ratio"] * 10), 50, noop)
    cv2.createTrackbar("Open", CONTROL_WIN, int(p["open_iters"]), 6, noop)
    cv2.createTrackbar("Close", CONTROL_WIN, int(p["close_iters"]), 6, noop)

    cv2.createTrackbar("HSV 0/1", CONTROL_WIN, int(p["use_hsv"]), 1, noop)
    cv2.createTrackbar("H min", CONTROL_WIN, int(p["h_min"]), 179, noop)
    cv2.createTrackbar("H max", CONTROL_WIN, int(p["h_max"]), 179, noop)
    cv2.createTrackbar("S min", CONTROL_WIN, int(p["s_min"]), 255, noop)
    cv2.createTrackbar("S max", CONTROL_WIN, int(p["s_max"]), 255, noop)
    cv2.createTrackbar("V min", CONTROL_WIN, int(p["v_min"]), 255, noop)
    cv2.createTrackbar("V max", CONTROL_WIN, int(p["v_max"]), 255, noop)


def read_trackbars(p: dict):
    p["mode"] = ["auto", "dark", "bright"][clamp(cv2.getTrackbarPos("Mode 0A 1D 2B", CONTROL_WIN), 0, 2)]
    p["use_roi"] = cv2.getTrackbarPos("ROI 0/1", CONTROL_WIN) == 1
    p["roi_scale"] = clamp(cv2.getTrackbarPos("ROI %", CONTROL_WIN) / 100.0, 0.1, 1.0)

    min_area = clamp(float(cv2.getTrackbarPos("MinArea", CONTROL_WIN)), 10.0, 19990.0)
    max_area = clamp(float(cv2.getTrackbarPos("MaxArea", CONTROL_WIN)), min_area + 10.0, 20000.0)
    p["min_area"] = min_area
    p["max_area"] = max_area

    p["min_circularity"] = clamp(cv2.getTrackbarPos("Circ x100", CONTROL_WIN) / 100.0, 0.01, 0.95)
    p["min_solidity"] = clamp(cv2.getTrackbarPos("Solid x100", CONTROL_WIN) / 100.0, 0.20, 0.99)
    p["max_aspect_ratio"] = clamp(cv2.getTrackbarPos("Aspect x10", CONTROL_WIN) / 10.0, 1.2, 5.0)
    p["open_iters"] = int(clamp(cv2.getTrackbarPos("Open", CONTROL_WIN), 0, 6))
    p["close_iters"] = int(clamp(cv2.getTrackbarPos("Close", CONTROL_WIN), 0, 6))

    p["use_hsv"] = cv2.getTrackbarPos("HSV 0/1", CONTROL_WIN) == 1
    p["h_min"] = int(clamp(cv2.getTrackbarPos("H min", CONTROL_WIN), 0, 179))
    p["h_max"] = int(clamp(cv2.getTrackbarPos("H max", CONTROL_WIN), p["h_min"], 179))
    p["s_min"] = int(clamp(cv2.getTrackbarPos("S min", CONTROL_WIN), 0, 255))
    p["s_max"] = int(clamp(cv2.getTrackbarPos("S max", CONTROL_WIN), p["s_min"], 255))
    p["v_min"] = int(clamp(cv2.getTrackbarPos("V min", CONTROL_WIN), 0, 255))
    p["v_max"] = int(clamp(cv2.getTrackbarPos("V max", CONTROL_WIN), p["v_min"], 255))

    cv2.setTrackbarPos("MinArea", CONTROL_WIN, int(p["min_area"]))
    cv2.setTrackbarPos("MaxArea", CONTROL_WIN, int(p["max_area"]))
    cv2.setTrackbarPos("H max", CONTROL_WIN, int(p["h_max"]))
    cv2.setTrackbarPos("S max", CONTROL_WIN, int(p["s_max"]))
    cv2.setTrackbarPos("V max", CONTROL_WIN, int(p["v_max"]))


def draw_overlays(frame_bgr: np.ndarray, shapes, p: dict, picked_mode: str, roi_mask, hsv_mask):
    for shape_type, x, y, rx, ry, angle in shapes:
        if shape_type == "ellipse":
            cv2.ellipse(frame_bgr, (x, y), (rx, ry), angle, 0, 360, (0, 0, 255), 2)
        else:
            cv2.circle(frame_bgr, (x, y), rx, (0, 0, 255), 2)
        cv2.circle(frame_bgr, (x, y), 2, (0, 255, 0), -1)

    cv2.putText(
        frame_bgr,
        f"Dots:{len(shapes)} mode:{p['mode']} pick:{picked_mode}",
        (10, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 0),
        2,
    )
    cv2.putText(
        frame_bgr,
        (
            f"A>={int(p['min_area'])} C>={p['min_circularity']:.2f} S>={p['min_solidity']:.2f} "
            f"AR<={p['max_aspect_ratio']:.2f} O/C={p['open_iters']}/{p['close_iters']} "
            f"ROI={'on' if p['use_roi'] else 'off'} HSV={'on' if p['use_hsv'] else 'off'}"
        ),
        (10, 52),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 255, 255),
        1,
    )

    if roi_mask is not None:
        roi_vis = cv2.cvtColor(roi_mask, cv2.COLOR_GRAY2BGR)
        roi_vis[:, :, 1] = 0
        roi_vis[:, :, 2] = 0
        frame_bgr = cv2.addWeighted(frame_bgr, 1.0, roi_vis, 0.15, 0)

    if hsv_mask is not None:
        hsv_vis = cv2.cvtColor(hsv_mask, cv2.COLOR_GRAY2BGR)
        hsv_vis[:, :, 0] = 0
        hsv_vis[:, :, 2] = 0
        frame_bgr = cv2.addWeighted(frame_bgr, 1.0, hsv_vis, 0.10, 0)

    return frame_bgr


def open_camera(width: int, height: int):
    cam = Picamera2()
    config = cam.create_preview_configuration(main={"size": (width, height), "format": "RGB888"})
    cam.configure(config)
    cam.start()

    controls_payload = {"ExposureValue": 0.7}
    if libcamera_controls is not None:
        controls_payload["AfMode"] = libcamera_controls.AfModeEnum.Continuous

    full_crop = cam.camera_properties.get("ScalerCropMaximum")
    if full_crop is not None:
        controls_payload["ScalerCrop"] = full_crop

    cam.set_controls(controls_payload)
    return cam


def main():
    params = dict(DEFAULTS)
    params["roi_mask"] = None
    params["hsv_mask"] = None

    cam = open_camera(params["cam_width"], params["cam_height"])
    init_windows()
    init_trackbars(params)

    print("Running detector. Keys: q=quit, s=save debug frames")

    save_idx = 0
    try:
        while True:
            frame_rgb = cam.capture_array()
            read_trackbars(params)

            pre = preprocess(frame_rgb)
            roi_mask = build_roi_mask(pre.shape, params["roi_scale"]) if params["use_roi"] else None
            hsv_mask = build_hsv_mask(frame_rgb, params)
            combined_mask = build_combined_mask(roi_mask, hsv_mask)

            binary, shapes, picked_mode = choose_binary(pre, params, combined_mask)
            frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
            frame_bgr = draw_overlays(frame_bgr, shapes, params, picked_mode, roi_mask, hsv_mask)

            cv2.imshow(VIEW_WIN, frame_bgr)
            cv2.imshow(BINARY_WIN, binary)
            cv2.imshow(GRAY_WIN, pre)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("s"):
                raw_name = f"blob_debug_raw_{save_idx:03d}.png"
                bin_name = f"blob_debug_bin_{save_idx:03d}.png"
                cv2.imwrite(raw_name, frame_bgr)
                cv2.imwrite(bin_name, binary)
                print(f"Saved {raw_name} and {bin_name}")
                save_idx += 1
    finally:
        cam.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()