import argparse
import json
from pathlib import Path
from typing import Dict, List, Tuple

import cv2
import numpy as np


def generate_grid_centroids(
    width: int,
    height: int,
    rows: int,
    cols: int,
    spacing: int,
    jitter: float,
    offset: Tuple[int, int],
) -> List[Tuple[int, int]]:
    cx = (width - (cols - 1) * spacing) // 2 + offset[0]
    cy = (height - (rows - 1) * spacing) // 2 + offset[1]
    centroids = []
    for r in range(rows):
        for c in range(cols):
            x = cx + c * spacing + int(np.random.uniform(-jitter, jitter))
            y = cy + r * spacing + int(np.random.uniform(-jitter, jitter))
            if 0 <= x < width and 0 <= y < height:
                centroids.append((x, y))
    return centroids


def draw_dots(
    width: int,
    height: int,
    centroids: List[Tuple[int, int]],
    radius: int,
    noise_sigma: float,
    blur_ksize: int,
    gradient: bool,
) -> np.ndarray:
    image = np.zeros((height, width), dtype=np.uint8)

    for (x, y) in centroids:
        cv2.circle(image, (x, y), radius, 255, -1)

    if gradient:
        x = np.linspace(0, 1, width, dtype=np.float32)
        y = np.linspace(0, 1, height, dtype=np.float32)
        xv, yv = np.meshgrid(x, y)
        grad = (0.5 + 0.5 * xv) * (0.5 + 0.5 * yv)
        image = (image.astype(np.float32) * grad).astype(np.uint8)

    if noise_sigma > 0:
        noise = np.random.normal(0, noise_sigma, image.shape).astype(np.float32)
        image = np.clip(image.astype(np.float32) + noise, 0, 255).astype(np.uint8)

    if blur_ksize > 0 and blur_ksize % 2 == 1:
        image = cv2.GaussianBlur(image, (blur_ksize, blur_ksize), 0)

    return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)


def main():
    parser = argparse.ArgumentParser(description="Generate synthetic dot images with metadata")
    parser.add_argument("--out", type=str, default="synthetic", help="Output folder")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--rows", type=int, default=8)
    parser.add_argument("--cols", type=int, default=10)
    parser.add_argument("--spacing", type=int, default=40)
    parser.add_argument("--radius", type=int, default=6)
    parser.add_argument("--jitter", type=float, default=1.5)
    parser.add_argument("--noise", type=float, default=6.0)
    parser.add_argument("--blur", type=int, default=3)
    parser.add_argument("--gradient", action="store_true")
    parser.add_argument("--frames", type=int, default=1)
    parser.add_argument("--shift", type=int, default=0, help="Per-frame shift in pixels")
    args = parser.parse_args()

    workspace = Path(__file__).resolve().parent
    out_dir = (workspace / args.out).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    metadata: Dict[str, List[Dict[str, List[int]]]] = {"images": []}

    for i in range(args.frames):
        offset = (i * args.shift, i * args.shift)
        centroids = generate_grid_centroids(
            args.width,
            args.height,
            args.rows,
            args.cols,
            args.spacing,
            args.jitter,
            offset,
        )
        image = draw_dots(
            args.width,
            args.height,
            centroids,
            args.radius,
            args.noise,
            args.blur,
            args.gradient,
        )
        name = f"dots_{i:03d}.png"
        cv2.imwrite(str(out_dir / name), image)
        metadata["images"].append(
            {
                "file": name,
                "centroids": [[int(x), int(y)] for (x, y) in centroids],
            }
        )

    (out_dir / "metadata.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    print(f"Saved {args.frames} images to {out_dir}")
    print(f"Metadata: {out_dir / 'metadata.json'}")


if __name__ == "__main__":
    main()