from __future__ import annotations

import ctypes
import os
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

try:
    import numpy as np
except Exception:  # pragma: no cover - optional runtime dependency
    np = None


@dataclass(slots=True)
class BlobResult:
    x: int
    y: int
    area: int
    perimeter: int
    circularity: float
    left: int
    top: int
    right: int
    bottom: int


class _CBlob(ctypes.Structure):
    _fields_ = [
        ("x", ctypes.c_int),
        ("y", ctypes.c_int),
        ("area", ctypes.c_int),
        ("perimeter", ctypes.c_int),
        ("circularity", ctypes.c_float),
        ("left", ctypes.c_int),
        ("top", ctypes.c_int),
        ("right", ctypes.c_int),
        ("bottom", ctypes.c_int),
    ]


class NativeBlobDetector:
    def __init__(self, library_path: Optional[Path] = None):
        self.library_path = self._resolve_library_path(library_path)
        self._lib = self._load_library(self.library_path) if self.library_path else None
        if self._lib is not None:
            self._lib.bubble_detect_from_binary.argtypes = [
                ctypes.POINTER(ctypes.c_uint8),
                ctypes.c_int,
                ctypes.c_int,
                ctypes.c_int,
                ctypes.c_int,
                ctypes.c_float,
                ctypes.POINTER(_CBlob),
                ctypes.c_int,
            ]
            self._lib.bubble_detect_from_binary.restype = ctypes.c_int

    @property
    def available(self) -> bool:
        return self._lib is not None

    def detect(
        self,
        binary_image,
        min_area: int,
        max_area: int,
        min_circularity: float,
        max_blobs: int = 512,
    ) -> List[BlobResult]:
        if self._lib is None:
            return []
        if np is None:
            raise RuntimeError("numpy is required for the native detector wrapper.")

        array = binary_image
        if array.ndim != 2:
            raise ValueError("Native detector expects a single-channel uint8 image.")
        if array.dtype != np.uint8:
            array = array.astype(np.uint8, copy=False)
        if not array.flags["C_CONTIGUOUS"]:
            array = np.ascontiguousarray(array)

        height, width = array.shape
        buffer_type = ctypes.POINTER(ctypes.c_uint8)
        out = (_CBlob * max_blobs)()
        count = self._lib.bubble_detect_from_binary(
            array.ctypes.data_as(buffer_type),
            width,
            height,
            int(min_area),
            int(max_area),
            float(min_circularity),
            out,
            int(max_blobs),
        )
        if count < 0:
            raise RuntimeError("Native blob detector returned an error status.")

        return [
            BlobResult(
                x=out[index].x,
                y=out[index].y,
                area=out[index].area,
                perimeter=out[index].perimeter,
                circularity=out[index].circularity,
                left=out[index].left,
                top=out[index].top,
                right=out[index].right,
                bottom=out[index].bottom,
            )
            for index in range(min(count, max_blobs))
        ]

    def _resolve_library_path(self, explicit_path: Optional[Path]) -> Optional[Path]:
        if explicit_path is not None and explicit_path.exists():
            return explicit_path

        env_override = os.environ.get("PI_BUBBLE_NATIVE_LIB")
        if env_override:
            candidate = Path(env_override)
            if candidate.exists():
                return candidate

        native_root = Path(__file__).resolve().parents[1] / "native" / "build"
        candidates = [
            native_root / "libbubble_detect.so",
            native_root / "bubble_detect.dll",
            native_root / "libbubble_detect.dylib",
        ]
        for candidate in candidates:
            if candidate.exists():
                return candidate
        return None

    def _load_library(self, library_path: Path):
        try:
            return ctypes.CDLL(str(library_path))
        except OSError:
            return None
