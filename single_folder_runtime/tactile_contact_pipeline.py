from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np


@dataclass(frozen=True)
class TactileContactConfig:
    bubble_width_mm: float = 120.0
    bubble_height_mm: float = 80.0
    bubble_max_height_mm: float = 25.0
    grid_width: int = 52
    roi_scale: float = 1.0
    use_roi: bool = True
    gain: float = 1.0
    smooth: float = 1.4
    min_contact_scale_px: float = 0.28
    residual_floor_mad: float = 1.0
    contact_area_threshold: float = 0.06
    spatial_sigma_fraction: float = 0.10
    spatial_sigma_min_px: float = 5.0
    signal_gamma: float = 0.72
    strain_blend: float = 0.68
    interior_fill_blend: float = 0.68


@dataclass
class TactileContactFrame:
    height_map: np.ndarray
    base_map: np.ndarray
    contact_map: np.ndarray
    ellipse_mask: np.ndarray
    scale_px: float
    residual_floor_px: float
    residual_peak_px: float
    residual_mean_px: float
    contact_peak: float
    contact_top_mean: float
    contact_area_ratio: float
    contact_center: Optional[Tuple[float, float]]


def _valid_pairs(reference_centroids, displacements):
    if reference_centroids is None or displacements is None:
        return None, None
    points = []
    vectors = []
    for ref, disp in zip(reference_centroids, displacements):
        if ref is None or disp is None:
            continue
        points.append(ref)
        vectors.append(disp)
    if len(points) < 6:
        return None, None
    return np.array(points, dtype=np.float32), np.array(vectors, dtype=np.float32)


def _affine_residuals(points_arr, vectors_arr, cv2):
    current_arr = points_arr + vectors_arr
    affine = None
    if cv2 is not None:
        try:
            affine, _inliers = cv2.estimateAffinePartial2D(
                points_arr,
                current_arr,
                method=cv2.RANSAC,
                ransacReprojThreshold=3.0,
                maxIters=120,
                confidence=0.96,
            )
        except Exception:
            affine = None

    if affine is not None:
        predicted_arr = cv2.transform(points_arr.reshape(-1, 1, 2), affine).reshape(-1, 2)
        return current_arr - predicted_arr

    global_shift = np.median(vectors_arr, axis=0)
    return vectors_arr - global_shift


def _surface_geometry(points_arr, frame_shape, config: TactileContactConfig):
    height_px, width_px = frame_shape[:2]
    ratio = config.bubble_width_mm / config.bubble_height_mm if config.bubble_height_mm > 0 else 1.0
    roi_scale = float(config.roi_scale) if config.use_roi else 1.0

    if config.use_roi:
        cx = width_px * 0.5
        cy = height_px * 0.5
        max_rx = max(10.0, width_px * roi_scale * 0.5)
        max_ry = max(10.0, height_px * roi_scale * 0.5)
    else:
        lo = np.percentile(points_arr, 3.0, axis=0)
        hi = np.percentile(points_arr, 97.0, axis=0)
        cx = float((lo[0] + hi[0]) * 0.5)
        cy = float((lo[1] + hi[1]) * 0.5)
        max_rx = max(10.0, float((hi[0] - lo[0]) * 0.64))
        max_ry = max(10.0, float((hi[1] - lo[1]) * 0.64))

    if max_rx / max_ry > ratio:
        ry = max_ry
        rx = ry * ratio
    else:
        rx = max_rx
        ry = rx / ratio

    rx = min(rx, max(10.0, width_px * 0.49))
    ry = min(ry, max(10.0, height_px * 0.49))
    cx = float(np.clip(cx, rx, max(rx, width_px - 1 - rx)))
    cy = float(np.clip(cy, ry, max(ry, height_px - 1 - ry)))
    return cx, cy, rx, ry, ratio


def _ellipse_mask(frame_shape, cx, cy, rx, ry):
    height_px, width_px = frame_shape[:2]
    yy, xx = np.ogrid[:height_px, :width_px]
    nx = (xx.astype(np.float32) - cx) / max(rx, 1e-6)
    ny = (yy.astype(np.float32) - cy) / max(ry, 1e-6)
    return (nx * nx + ny * ny) <= 1.0


def _normalize_field(values, mask):
    if values is None:
        return None
    norm = np.zeros_like(values, dtype=np.float32)
    active = np.asarray(values, dtype=np.float32)[mask]
    active = active[np.isfinite(active)]
    if active.size == 0:
        return norm
    low = float(np.percentile(active, 35.0))
    high = float(np.percentile(active, 97.0))
    if high <= low + 1e-6:
        high = low + 1e-6
    norm = np.clip((np.asarray(values, dtype=np.float32) - low) / (high - low), 0.0, 1.0)
    return np.where(mask, norm, 0.0).astype(np.float32)


def _interior_fill_map(points_arr, signal_arr, cx, cy, rx, ry, grid_mask, grid_w, grid_h, cv2):
    if cv2 is None:
        return np.zeros((grid_h, grid_w), dtype=np.float32)
    if points_arr is None or signal_arr is None or len(points_arr) < 3:
        return np.zeros((grid_h, grid_w), dtype=np.float32)

    active_threshold = max(0.16, float(np.percentile(signal_arr, 72.0)))
    active_points = points_arr[signal_arr >= active_threshold]
    active_signal = signal_arr[signal_arr >= active_threshold]
    if len(active_points) < 3 or active_signal.size == 0:
        return np.zeros((grid_h, grid_w), dtype=np.float32)

    px = ((active_points[:, 0] - (cx - rx)) / max(2.0 * rx, 1e-6)) * max(1, grid_w - 1)
    py = ((active_points[:, 1] - (cy - ry)) / max(2.0 * ry, 1e-6)) * max(1, grid_h - 1)
    low_points = np.stack(
        [
            np.clip(px, 0.0, float(max(0, grid_w - 1))),
            np.clip(py, 0.0, float(max(0, grid_h - 1))),
        ],
        axis=1,
    ).astype(np.float32)
    hull = cv2.convexHull(low_points.reshape(-1, 1, 2))
    if hull is None or len(hull) < 3:
        return np.zeros((grid_h, grid_w), dtype=np.float32)

    hull_mask = np.zeros((grid_h, grid_w), dtype=np.uint8)
    cv2.fillConvexPoly(hull_mask, hull.astype(np.int32), 255, lineType=cv2.LINE_AA)
    hull_mask = np.where(grid_mask, hull_mask, 0).astype(np.uint8)
    if int(np.count_nonzero(hull_mask)) < 12:
        return np.zeros((grid_h, grid_w), dtype=np.float32)

    dist = cv2.distanceTransform((hull_mask > 0).astype(np.uint8), cv2.DIST_L2, 3)
    max_dist = float(np.max(dist))
    if max_dist <= 1e-6:
        return np.zeros((grid_h, grid_w), dtype=np.float32)

    weights = np.clip(active_signal.astype(np.float32), 1e-6, None)
    center_x = float(np.average(low_points[:, 0], weights=weights))
    center_y = float(np.average(low_points[:, 1], weights=weights))
    sigma_x = max(2.0, float(np.sqrt(np.average((low_points[:, 0] - center_x) ** 2, weights=weights))) * 0.95)
    sigma_y = max(2.0, float(np.sqrt(np.average((low_points[:, 1] - center_y) ** 2, weights=weights))) * 0.95)
    yy, xx = np.indices((grid_h, grid_w), dtype=np.float32)
    gaussian = np.exp(
        -0.5
        * (
            ((xx - center_x) / max(sigma_x, 1e-6)) ** 2
            + ((yy - center_y) / max(sigma_y, 1e-6)) ** 2
        )
    ).astype(np.float32)
    fill = (gaussian * np.power(np.clip(dist / max_dist, 0.0, 1.0), 0.55)).astype(np.float32)
    strength = float(np.clip(np.percentile(active_signal, 88.0), 0.0, 1.0))
    fill = np.where(grid_mask, fill * strength, 0.0).astype(np.float32)
    fill = cv2.GaussianBlur(fill, (0, 0), sigmaX=1.0, sigmaY=1.0)
    return np.where(grid_mask, np.clip(fill, 0.0, 1.0), 0.0).astype(np.float32)


def _low_res_contact_map(points_arr, vectors_arr, signal_arr, frame_shape, config, cv2):
    height_px, width_px = frame_shape[:2]
    cx, cy, rx, ry, ratio = _surface_geometry(points_arr, frame_shape, config)

    grid_w = int(np.clip(config.grid_width, 24, 140))
    grid_h = max(18, min(int(grid_w / ratio), 120))
    grid_x = np.linspace(cx - rx, cx + rx, grid_w)
    grid_y = np.linspace(cy - ry, cy + ry, grid_h)
    gx, gy = np.meshgrid(grid_x, grid_y)

    nx = (gx - cx) / max(rx, 1e-6)
    ny = (gy - cy) / max(ry, 1e-6)
    r2 = nx * nx + ny * ny
    grid_mask = r2 <= 1.0

    base_low = np.zeros_like(r2, dtype=np.float32)
    base_low[grid_mask] = np.sqrt(np.clip(1.0 - r2[grid_mask], 0.0, 1.0))

    dx = gx[..., None] - points_arr[:, 0]
    dy = gy[..., None] - points_arr[:, 1]
    dist2 = dx * dx + dy * dy

    sigma_fraction = float(np.clip(getattr(config, "spatial_sigma_fraction", 0.10), 0.04, 0.35))
    sigma_min_px = max(1.0, float(getattr(config, "spatial_sigma_min_px", 5.0)))
    sigma = max(sigma_min_px, min(rx, ry) * sigma_fraction)
    weights = np.exp(-dist2 / (2.0 * sigma * sigma)).astype(np.float32)
    weight_sum = np.sum(weights, axis=-1)
    contact_low = np.sum(weights * signal_arr, axis=-1)
    contact_low = np.divide(
        contact_low,
        weight_sum,
        out=np.zeros_like(contact_low),
        where=weight_sum > 1e-6,
    )

    signal_weights = weights * signal_arr.reshape(1, 1, -1)
    signal_weight_sum = np.sum(signal_weights, axis=-1)
    vec_x_low = np.divide(
        np.sum(signal_weights * vectors_arr[:, 0], axis=-1),
        signal_weight_sum,
        out=np.zeros_like(contact_low),
        where=signal_weight_sum > 1e-6,
    )
    vec_y_low = np.divide(
        np.sum(signal_weights * vectors_arr[:, 1], axis=-1),
        signal_weight_sum,
        out=np.zeros_like(contact_low),
        where=signal_weight_sum > 1e-6,
    )

    if np.any(grid_mask):
        confidence = np.clip(
            weight_sum / (np.percentile(weight_sum[grid_mask], 75.0) + 1e-6),
            0.0,
            1.0,
        )
    else:
        confidence = 0.0
    contact_low = np.where(grid_mask, contact_low * confidence, 0.0).astype(np.float32)
    if cv2 is not None:
        contact_low = cv2.GaussianBlur(contact_low, (0, 0), sigmaX=1.0, sigmaY=1.0)
        contact_low = np.where(grid_mask, contact_low, 0.0).astype(np.float32)

    vec_x_low = np.where(grid_mask, vec_x_low * confidence, 0.0).astype(np.float32)
    vec_y_low = np.where(grid_mask, vec_y_low * confidence, 0.0).astype(np.float32)
    if cv2 is not None:
        vec_x_low = cv2.GaussianBlur(vec_x_low, (0, 0), sigmaX=1.0, sigmaY=1.0)
        vec_y_low = cv2.GaussianBlur(vec_y_low, (0, 0), sigmaX=1.0, sigmaY=1.0)
        vec_x_low = np.where(grid_mask, vec_x_low, 0.0).astype(np.float32)
        vec_y_low = np.where(grid_mask, vec_y_low, 0.0).astype(np.float32)

    grid_dx = float(abs(grid_x[1] - grid_x[0])) if grid_w > 1 else 1.0
    grid_dy = float(abs(grid_y[1] - grid_y[0])) if grid_h > 1 else 1.0
    dux_dx = np.gradient(vec_x_low, grid_dx, axis=1)
    duy_dy = np.gradient(vec_y_low, grid_dy, axis=0)
    dux_dy = np.gradient(vec_x_low, grid_dy, axis=0)
    duy_dx = np.gradient(vec_y_low, grid_dx, axis=1)
    strain_low = np.sqrt(
        np.maximum(
            0.0,
            (dux_dx + duy_dy) ** 2 + 0.35 * ((dux_dy + duy_dx) ** 2),
        )
    ).astype(np.float32)
    strain_low = _normalize_field(strain_low, grid_mask)
    if cv2 is not None:
        strain_low = cv2.GaussianBlur(strain_low, (0, 0), sigmaX=1.2, sigmaY=1.2)
        strain_low = np.where(grid_mask, strain_low, 0.0).astype(np.float32)

    strain_blend = float(np.clip(getattr(config, "strain_blend", 0.68), 0.0, 1.0))
    if strain_blend > 0.0:
        contact_low = np.where(
            grid_mask,
            ((1.0 - strain_blend) * contact_low) + (strain_blend * strain_low),
            0.0,
        ).astype(np.float32)

    fill_low = _interior_fill_map(points_arr, signal_arr, cx, cy, rx, ry, grid_mask, grid_w, grid_h, cv2)
    fill_blend = float(np.clip(getattr(config, "interior_fill_blend", 0.68), 0.0, 1.0))
    if fill_blend > 0.0:
        contact_low = np.where(
            grid_mask,
            ((1.0 - fill_blend) * contact_low) + (fill_blend * fill_low),
            0.0,
        ).astype(np.float32)

    ellipse_mask = _ellipse_mask((height_px, width_px), cx, cy, rx, ry)
    contact_full = _resize_to_frame(contact_low, (width_px, height_px), cv2)
    base_full = _resize_to_frame(base_low, (width_px, height_px), cv2)
    contact_full = np.where(ellipse_mask, np.clip(contact_full, 0.0, 1.0), 0.0)
    base_full = np.where(ellipse_mask, np.clip(base_full, 0.0, 1.0), 0.0)
    return contact_full.astype(np.float32), base_full.astype(np.float32), ellipse_mask


def _resize_to_frame(image, size, cv2):
    if cv2 is not None:
        return cv2.resize(image, size, interpolation=cv2.INTER_CUBIC)
    target_w, target_h = size
    y_idx = np.linspace(0, image.shape[0] - 1, target_h).astype(np.int32)
    x_idx = np.linspace(0, image.shape[1] - 1, target_w).astype(np.int32)
    return image[y_idx][:, x_idx]


def surface_from_contact_map(contact_map, base_map, ellipse_mask, config, cv2=None):
    contact = np.clip(contact_map.astype(np.float32), 0.0, 1.0)
    base = np.clip(base_map.astype(np.float32), 0.0, 1.0)
    mask = ellipse_mask.astype(bool)
    edge_lock = np.power(np.clip(base, 0.0, 1.0), 0.72)
    depression = np.clip(config.gain, 0.2, 3.0) * contact * 0.72 * edge_lock
    height = np.clip(base - depression, 0.0, 1.0)
    if cv2 is not None and config.smooth > 0.01:
        height = cv2.GaussianBlur(
            height,
            (0, 0),
            sigmaX=float(config.smooth),
            sigmaY=float(config.smooth),
        )
    return np.where(mask, np.clip(height, 0.0, 1.0), 0.0).astype(np.float32)


def summarize_contact_map(contact_map, ellipse_mask, area_threshold=0.16):
    if contact_map is None or contact_map.size == 0:
        return None
    mask = ellipse_mask.astype(bool) if ellipse_mask is not None else np.isfinite(contact_map)
    values = np.clip(contact_map.astype(np.float32), 0.0, 1.0)[mask]
    if values.size == 0:
        return None
    peak = float(np.percentile(values, 98.0))
    active = values[values > max(0.02, area_threshold * 0.35)]
    mean = float(np.mean(active)) if active.size else 0.0
    top_count = max(1, int(values.size * 0.05))
    top_mean = float(np.mean(np.partition(values, -top_count)[-top_count:]))
    area_ratio = float((np.count_nonzero(values > area_threshold) / values.size) * 100.0)

    center = None
    weighted = np.clip(contact_map.astype(np.float32), 0.0, 1.0) * mask.astype(np.float32)
    total = float(np.sum(weighted))
    if total > 1e-6:
        ys, xs = np.indices(contact_map.shape, dtype=np.float32)
        center = (float(np.sum(xs * weighted) / total), float(np.sum(ys * weighted) / total))

    return {
        "ready": True,
        "mean": mean,
        "top_mean": top_mean,
        "peak": peak,
        "area_ratio": area_ratio,
        "center": center,
    }


def build_tactile_contact_frame(
    reference_centroids: Sequence,
    displacements: Sequence,
    frame_shape,
    config: TactileContactConfig,
    cv2=None,
    previous_scale_px: Optional[float] = None,
    compute_surface: bool = True,
) -> Optional[TactileContactFrame]:
    points_arr, vectors_arr = _valid_pairs(reference_centroids, displacements)
    if points_arr is None:
        return None

    residual_vectors = _affine_residuals(points_arr, vectors_arr, cv2)
    residual_mag = np.linalg.norm(residual_vectors, axis=1).astype(np.float32)
    low_cutoff = float(np.percentile(residual_mag, 60.0))
    low_values = residual_mag[residual_mag <= low_cutoff]
    if low_values.size == 0:
        low_values = residual_mag
    median = float(np.median(low_values))
    mad = float(np.median(np.abs(low_values - median)))
    floor = median + (config.residual_floor_mad * 1.4826 * mad)
    raw_signal = np.maximum(residual_mag - floor, 0.0).astype(np.float32)

    nonzero_signal = raw_signal[raw_signal > 1e-4]
    observed_scale = (
        float(np.percentile(nonzero_signal, 90.0))
        if nonzero_signal.size
        else float(np.percentile(residual_mag, 95.0))
    )
    scale = max(float(config.min_contact_scale_px), observed_scale)
    if previous_scale_px is not None and np.isfinite(previous_scale_px):
        scale = max(float(config.min_contact_scale_px), (0.65 * previous_scale_px) + (0.35 * scale))

    signal_norm = np.clip(raw_signal / max(scale, 1e-6), 0.0, 1.0)
    signal_gamma = float(np.clip(getattr(config, "signal_gamma", 0.72), 0.45, 1.5))
    signal_norm = np.power(signal_norm, signal_gamma).astype(np.float32)
    contact_map, base_map, ellipse_mask = _low_res_contact_map(
        points_arr,
        residual_vectors,
        signal_norm,
        frame_shape,
        config,
        cv2,
    )
    if compute_surface:
        height_map = surface_from_contact_map(contact_map, base_map, ellipse_mask, config, cv2=cv2)
        stats = summarize_contact_map(contact_map, ellipse_mask, config.contact_area_threshold) or {}
    else:
        height_map = base_map
        stats = {}
    return TactileContactFrame(
        height_map=height_map,
        base_map=base_map,
        contact_map=contact_map,
        ellipse_mask=ellipse_mask,
        scale_px=float(scale),
        residual_floor_px=float(floor),
        residual_peak_px=float(np.percentile(residual_mag, 98.0)),
        residual_mean_px=float(np.mean(residual_mag)),
        contact_peak=float(stats.get("peak", 0.0)),
        contact_top_mean=float(stats.get("top_mean", 0.0)),
        contact_area_ratio=float(stats.get("area_ratio", 0.0)),
        contact_center=stats.get("center"),
    )
