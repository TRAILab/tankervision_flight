from __future__ import annotations

import json
from pathlib import Path
from typing import List, Tuple

import cv2
import numpy as np


# =========================
# User settings
# =========================
IMAGE_DIR = Path("/home/trailab/Documents/repos/tankervision_calib/captures")
IMAGE_TYPE = "thermal"          # "rgb" or "thermal"
PATTERN_SIZE = (4,11)
DOT_SPACING = 0.065       # meters between neighboring points in the grid pattern basis
OUTPUT_DIR = IMAGE_DIR / f"calibration_{IMAGE_TYPE}"
SAVE_DEBUG_IMAGES = True

# Optional filename filters.
# Adjust if your naming differs.
RGB_KEYWORDS = ["rgb"]
THERMAL_KEYWORDS = ["thermal"]

# Image extensions to consider
VALID_EXTENSIONS = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"}


def get_image_paths(image_dir: Path, image_type: str) -> List[Path]:
    if image_type not in {"rgb", "thermal"}:
        raise ValueError("IMAGE_TYPE must be 'rgb' or 'thermal'")

    keywords = RGB_KEYWORDS if image_type == "rgb" else THERMAL_KEYWORDS

    paths = []
    for p in sorted(image_dir.iterdir()):
        if p.is_file() and p.suffix.lower() in VALID_EXTENSIONS:
            name = p.name.lower()
            if any(k in name for k in keywords):
                paths.append(p)

    return paths


def create_asymmetric_grid_object_points(
    pattern_size: Tuple[int, int],
    dot_spacing: float,
) -> np.ndarray:
    """
    Create 3D object points for an asymmetric circles grid.

    OpenCV asymmetric circles grid convention:
    x = (2*j + i%2) * spacing
    y = i * spacing
    z = 0

    pattern_size = (cols, rows)
    """
    cols, rows = pattern_size
    objp = []

    for i in range(rows):
        for j in range(cols):
            x = (2 * j + (i % 2)) * dot_spacing
            y = i * dot_spacing
            z = 0.0
            objp.append([x, y, z])

    return np.array(objp, dtype=np.float32)


def build_blob_detector() -> cv2.SimpleBlobDetector:
    params = cv2.SimpleBlobDetector_Params()

    params.minThreshold = 0
    params.maxThreshold = 255
    params.thresholdStep = 5

    params.filterByArea = True
    params.minArea = 10
    params.maxArea = 400

    params.filterByCircularity = True
    params.minCircularity = 0.7

    params.filterByColor = True
    params.blobColor = 0  # 0 for dark blobs, 255 for bright blobs

    params.filterByConvexity = False
    params.filterByInertia = False
    return cv2.SimpleBlobDetector_create(params)


def maybe_preprocess(gray: np.ndarray, image_type: str) -> np.ndarray:
    """
    Mild preprocessing. Thermal images often benefit from normalization.
    """
    proc = gray.copy()

    if image_type == "thermal":
        norm = cv2.normalize(gray, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        proc = 255 - norm

    #proc = cv2.GaussianBlur(proc, (5, 5), 0)
    return proc


def detect_asymmetric_grid(
    image: np.ndarray,
    pattern_size: Tuple[int, int],
    blob_detector: cv2.SimpleBlobDetector,
    image_type: str,
) -> Tuple[bool, np.ndarray | None, np.ndarray]:
    """
    Returns:
        found, centers, visualization_image
    """
    if image.ndim == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        vis = image.copy()
    else:
        gray = image.copy()
        vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    proc = maybe_preprocess(gray, image_type=image_type)

    flags = cv2.CALIB_CB_ASYMMETRIC_GRID

    found, centers = cv2.findCirclesGrid(
        proc,
        pattern_size,
        flags=flags,
        blobDetector=blob_detector,
    )

    if found and centers is not None:
        cv2.drawChessboardCorners(vis, pattern_size, centers, found)

    return found, centers, vis


def calibrate_from_images(
    image_paths: List[Path],
    pattern_size: Tuple[int, int],
    dot_spacing: float,
    image_type: str,
    output_dir: Path,
    save_debug_images: bool = True,
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    debug_dir = output_dir / "debug_detections"
    if save_debug_images:
        debug_dir.mkdir(parents=True, exist_ok=True)

    blob_detector = build_blob_detector()
    objp = create_asymmetric_grid_object_points(pattern_size, dot_spacing)

    object_points: List[np.ndarray] = []
    image_points: List[np.ndarray] = []

    image_size = None
    used_images = []
    failed_images = []

    for idx, path in enumerate(image_paths):
        image = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
        if image is None:
            print(f"[WARN] Could not read image: {path}")
            failed_images.append(str(path))
            continue

        found, centers, vis = detect_asymmetric_grid(
            image=image,
            pattern_size=pattern_size,
            blob_detector=blob_detector,
            image_type=image_type,
        )

        if image.ndim == 3:
            h, w = image.shape[:2]
        else:
            h, w = image.shape

        image_size = (w, h)

        status_text = "FOUND" if found else "NOT_FOUND"
        print(f"[{idx + 1:02d}/{len(image_paths):02d}] {path.name}: {status_text}")

        if found and centers is not None:
            object_points.append(objp.copy())
            image_points.append(centers)
            used_images.append(str(path))
        else:
            failed_images.append(str(path))

        if save_debug_images:
            out_path = debug_dir / f"{path.stem}_detected{path.suffix}"
            cv2.imwrite(str(out_path), vis)

    if not object_points or image_size is None:
        raise RuntimeError("No valid calibration detections found.")

    print(f"\nUsing {len(object_points)} / {len(image_paths)} images for calibration.")

    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        object_points,
        image_points,
        image_size,
        None,
        None,
    )

    reprojection_errors = []
    total_error = 0.0

    for i in range(len(object_points)):
        projected, _ = cv2.projectPoints(
            object_points[i],
            rvecs[i],
            tvecs[i],
            camera_matrix,
            dist_coeffs,
        )
        err = cv2.norm(image_points[i], projected, cv2.NORM_L2) / len(projected)
        reprojection_errors.append(float(err))
        total_error += err

    mean_reprojection_error = float(total_error / len(object_points))

    results = {
        "image_type": image_type,
        "pattern_size_cols_rows": list(pattern_size),
        "dot_spacing_m": dot_spacing,
        "num_input_images": len(image_paths),
        "num_used_images": len(object_points),
        "rms": float(rms),
        "mean_reprojection_error": mean_reprojection_error,
        "camera_matrix": camera_matrix.tolist(),
        "dist_coeffs": dist_coeffs.tolist(),
        "used_images": used_images,
        "failed_images": failed_images,
        "per_image_reprojection_error": reprojection_errors,
        "image_size_wh": list(image_size),
    }

    np.save(output_dir / f"{image_type}_camera_matrix.npy", camera_matrix)
    np.save(output_dir / f"{image_type}_dist_coeffs.npy", dist_coeffs)

    with open(output_dir / f"{image_type}_calibration.json", "w", encoding="utf-8") as f:
        json.dump(results, f, indent=2)

    print("\n=== Calibration complete ===")
    print(f"RMS: {rms:.6f}")
    print(f"Mean reprojection error: {mean_reprojection_error:.6f}")
    print("Calibration Matrix:", camera_matrix)
    print(f"Saved results to: {output_dir}")


def main() -> None:
    image_paths = get_image_paths(IMAGE_DIR, IMAGE_TYPE)

    if len(image_paths) == 0:
        raise FileNotFoundError(
            f"No {IMAGE_TYPE} images found in {IMAGE_DIR}. "
            f"Check your IMAGE_DIR and filename keywords."
        )

    print(f"Found {len(image_paths)} '{IMAGE_TYPE}' images.")
    for p in image_paths[:5]:
        print(f"  - {p.name}")
    if len(image_paths) > 5:
        print("  ...")

    calibrate_from_images(
        image_paths=image_paths,
        pattern_size=PATTERN_SIZE,
        dot_spacing=DOT_SPACING,
        image_type=IMAGE_TYPE,
        output_dir=OUTPUT_DIR,
        save_debug_images=SAVE_DEBUG_IMAGES,
    )


if __name__ == "__main__":
    main()