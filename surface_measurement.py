from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


@dataclass(frozen=True)
class SurfaceGeometry:
    bubble_max_height_mm: float = 25.0
    camera_to_acrylic_mm: float = 62.4
    camera_distance_sign: float = 1.0


@dataclass(frozen=True)
class ContactMeasurementConfig:
    deadband_mm: float = 0.35
    noise_cap_mm: float = 0.95
    baseline_alpha: float = 0.025
    contact_hold_mm: float = 0.75
    global_drift_area_ratio: float = 101.0


def _valid_height_mask(height_norm, ellipse_mask=None):
    valid_mask = np.isfinite(height_norm)
    if ellipse_mask is not None:
        valid_mask &= ellipse_mask.astype(bool)
    else:
        valid_mask &= height_norm > 0.0
    return valid_mask


def measure_camera_to_surface_mm(
    height_map,
    ellipse_mask=None,
    geometry: SurfaceGeometry = SurfaceGeometry(),
):
    if height_map is None or height_map.size == 0:
        return None

    height_norm = np.clip(height_map.astype(np.float32), 0.0, 1.0)
    valid_mask = _valid_height_mask(height_norm, ellipse_mask)
    if not np.any(valid_mask):
        return None

    height_mm = height_norm * geometry.bubble_max_height_mm
    distance_mm = geometry.camera_to_acrylic_mm + (
        geometry.camera_distance_sign * height_mm
    )
    valid_distance = distance_mm[valid_mask]
    valid_height = height_mm[valid_mask]

    ys, xs = np.nonzero(valid_mask)
    center_y = int(np.clip(round(float(np.mean(ys))), 0, distance_mm.shape[0] - 1))
    center_x = int(np.clip(round(float(np.mean(xs))), 0, distance_mm.shape[1] - 1))

    return {
        "min": float(np.min(valid_distance)),
        "mean": float(np.mean(valid_distance)),
        "max": float(np.max(valid_distance)),
        "center": float(distance_mm[center_y, center_x]),
        "height_peak": float(np.max(valid_height)),
    }


def measure_contact_deformation_mm(
    height_map,
    reference_height_map=None,
    contact_ema=None,
    ellipse_mask=None,
    cv2=None,
    geometry: SurfaceGeometry = SurfaceGeometry(),
    config: ContactMeasurementConfig = ContactMeasurementConfig(),
) -> Tuple[Optional[dict], object, object]:
    if height_map is None or height_map.size == 0:
        return None, reference_height_map, contact_ema

    height_norm = np.clip(height_map.astype(np.float32), 0.0, 1.0)
    valid_mask = _valid_height_mask(height_norm, ellipse_mask)
    if not np.any(valid_mask):
        return None, reference_height_map, contact_ema

    if reference_height_map is None or reference_height_map.shape != height_norm.shape:
        next_reference = height_norm.copy()
        next_contact_ema = np.zeros_like(height_norm, dtype=np.float32)
        return (
            {
                "ready": True,
                "mean": 0.0,
                "top_mean": 0.0,
                "peak": 0.0,
                "area_ratio": 0.0,
                "noise_floor": config.deadband_mm,
            },
            next_reference,
            next_contact_ema,
        )

    baseline = np.clip(reference_height_map.astype(np.float32), 0.0, 1.0)
    signed_delta_mm = (baseline - height_norm) * geometry.bubble_max_height_mm
    ys, xs = np.nonzero(valid_mask)
    if xs.size >= 12:
        sample_step = max(1, xs.size // 5000)
        sample_x = xs[::sample_step].astype(np.float32)
        sample_y = ys[::sample_step].astype(np.float32)
        sample_z = signed_delta_mm[ys[::sample_step], xs[::sample_step]].astype(np.float32)
        x_center = float(np.mean(sample_x))
        y_center = float(np.mean(sample_y))
        xy_scale = float(max(height_norm.shape[0], height_norm.shape[1], 1))
        design = np.column_stack(
            [
                np.ones_like(sample_z, dtype=np.float32),
                (sample_x - x_center) / xy_scale,
                (sample_y - y_center) / xy_scale,
            ]
        )
        try:
            coeffs, *_ = np.linalg.lstsq(design, sample_z, rcond=None)
            grid_y, grid_x = np.indices(height_norm.shape, dtype=np.float32)
            global_trend_mm = (
                coeffs[0]
                + coeffs[1] * ((grid_x - x_center) / xy_scale)
                + coeffs[2] * ((grid_y - y_center) / xy_scale)
            )
            local_delta_mm = signed_delta_mm - global_trend_mm
        except np.linalg.LinAlgError:
            valid_signed_delta = signed_delta_mm[valid_mask]
            global_offset_mm = float(np.median(valid_signed_delta)) if valid_signed_delta.size else 0.0
            local_delta_mm = signed_delta_mm - global_offset_mm
    else:
        valid_signed_delta = signed_delta_mm[valid_mask]
        global_offset_mm = float(np.median(valid_signed_delta)) if valid_signed_delta.size else 0.0
        local_delta_mm = signed_delta_mm - global_offset_mm
    raw_delta_mm = np.maximum(local_delta_mm, 0.0)
    valid_delta = raw_delta_mm[valid_mask]
    median_delta = float(np.median(valid_delta))
    mad_delta = float(np.median(np.abs(valid_delta - median_delta)))
    adaptive_floor = median_delta + (2.2 * 1.4826 * mad_delta)
    noise_floor = max(
        config.deadband_mm,
        min(adaptive_floor, config.noise_cap_mm),
    )

    contact_map = np.maximum(raw_delta_mm - noise_floor, 0.0)
    contact_map = np.where(valid_mask, contact_map, 0.0).astype(np.float32)
    if cv2 is not None:
        contact_map = cv2.GaussianBlur(contact_map, (0, 0), sigmaX=1.1, sigmaY=1.1)
        contact_map = np.where(valid_mask, contact_map, 0.0).astype(np.float32)

    if contact_ema is None or contact_ema.shape != contact_map.shape:
        next_contact_ema = contact_map
    else:
        next_contact_ema = (
            (0.70 * contact_ema.astype(np.float32)) + (0.30 * contact_map)
        )
    contact_filtered = np.where(valid_mask, next_contact_ema, 0.0)
    valid_contact = contact_filtered[valid_mask]

    peak = float(np.percentile(valid_contact, 98.0)) if valid_contact.size else 0.0
    active = valid_contact[valid_contact > 0.05]
    mean = float(np.mean(active)) if active.size else 0.0
    top_count = max(1, int(valid_contact.size * 0.05))
    top_mean = float(np.mean(np.partition(valid_contact, -top_count)[-top_count:]))
    area_ratio = float((np.count_nonzero(valid_contact > 0.10) / valid_contact.size) * 100.0)

    if area_ratio >= config.global_drift_area_ratio:
        return (
            {
                "ready": True,
                "mean": 0.0,
                "top_mean": 0.0,
                "peak": 0.0,
                "area_ratio": 0.0,
                "noise_floor": noise_floor,
            },
            height_norm.copy(),
            np.zeros_like(height_norm, dtype=np.float32),
        )

    next_reference = baseline
    if peak < config.contact_hold_mm:
        if cv2 is not None:
            next_reference = cv2.addWeighted(
                baseline,
                1.0 - config.baseline_alpha,
                height_norm,
                config.baseline_alpha,
                0.0,
            )
        else:
            next_reference = (
                ((1.0 - config.baseline_alpha) * baseline)
                + (config.baseline_alpha * height_norm)
            )

    return (
        {
            "ready": True,
            "mean": mean,
            "top_mean": top_mean,
            "peak": peak,
            "area_ratio": area_ratio,
            "noise_floor": noise_floor,
        },
        next_reference,
        next_contact_ema,
    )
