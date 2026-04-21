#!/usr/bin/env python3
#!/usr/bin/env python3
"""
calibrate_intrinsics.py
-----------------------
Loads .raw images via arena_raw_loader, detects AprilTag corners on a
calib.io / kalibr-style aprilgrid board, and runs OpenCV intrinsic
calibration.

Board: calib.io kalibr aprilgrid
    Columns (tags):  8
    Rows    (tags):  11
    Tag size:        35 mm  (0.035 m)
    Tag spacing:     11 mm  (0.011 m)
    Tag family:      t36h11  ->  cv2.aruco.DICT_APRILTAG_36h11

Usage
-----
    python calibrate_intrinsics.py --loc /path/to/raw/files [--debug]
"""

import argparse
import re
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))

# -- Default location ---------------------------------------------------------
RAW_DIR: str = "/home/atlas/tankervision_flight/home/atlas/captures/arena_raw"

# -- Board configuration ------------------------------------------------------
TAG_COLS: int      = 11
TAG_ROWS: int      = 8
TAG_SIZE: float    = 0.035   # metres
TAG_SPACING: float = 0.011   # metres
TAG_STRIDE: float  = TAG_SIZE + TAG_SPACING  # 0.046 m

N_TAGS_TOTAL    = TAG_COLS * TAG_ROWS   # 88
N_CORNERS_TOTAL = N_TAGS_TOTAL * 4     # 352

# -- Detector parameters (validated by sweep_detector_params.py) --------------
THRESH_WIN_MIN            = 3
THRESH_WIN_MAX            = 101
THRESH_WIN_STEP           = 4
MIN_MARKER_PERIMETER_RATE = 0.05

# Minimum tags required to accept an image for calibration
MIN_TAGS_PER_IMAGE = 6

# -- Raw file loading ---------------------------------------------------------
_FNAME_RE = re.compile(
    r"frame_(?P<id>\d+)_\d{8}_\d{6}_\d{6}_"
    r"(?P<width>\d+)x(?P<height>\d+)_(?P<bpp>\d+)bpp\.raw$"
)
# BayerRG8 -> OpenCV stores as BayerBG for demosaic
_BAYER_TO_GRAY = cv2.COLOR_BayerBG2GRAY


def load_images(location: str):
    """
    Load all .raw files without CLAHE -- sweep confirmed CLAHE reduces
    tag detection from ~83 to ~9 per image on this camera.
    """
    loc = Path(location).resolve()
    if not loc.is_dir():
        raise FileNotFoundError(f"Directory not found: {loc}")

    raw_files = sorted(loc.glob("*.raw"))
    if not raw_files:
        print(f"[WARN] No .raw files found in {loc}", file=sys.stderr)
        return [], None

    def sort_key(p):
        m = _FNAME_RE.match(p.name)
        return int(m.group("id")) if m else 999999999

    raw_files.sort(key=sort_key)

    images = []
    image_size = None

    for raw_path in raw_files:
        m = _FNAME_RE.match(raw_path.name)
        if not m:
            print(f"[SKIP] Unrecognised filename: {raw_path.name}", file=sys.stderr)
            continue

        w   = int(m.group("width"))
        h   = int(m.group("height"))
        bpp = int(m.group("bpp"))

        dtype    = np.uint16 if bpp > 8 else np.uint8
        expected = w * h * (2 if dtype == np.uint16 else 1)
        raw      = np.frombuffer(raw_path.read_bytes()[:expected], dtype=dtype).reshape(h, w)

        # Normalise to 8-bit using actual data range
        amax = int(raw.max())
        if amax > 255:
            ebits = int(np.ceil(np.log2(amax + 1)))
            raw8  = (raw >> max(ebits - 8, 0)).astype(np.uint8)
        else:
            raw8 = raw.astype(np.uint8)

        gray = cv2.cvtColor(raw8, _BAYER_TO_GRAY)
        images.append(gray)

        if image_size is None:
            image_size = (w, h)

        print(f"[OK] {raw_path.name}  ->  {gray.shape}  uint8")

    print(f"\nLoaded {len(images)} image(s) from {loc}")
    return images, image_size


def build_object_points() -> np.ndarray:
    """
    World coords for every corner of every tag in row-major order.
    Corner order per tag (matches OpenCV ArUco detector output):
        0: top-left   1: top-right   2: bottom-right   3: bottom-left
    Origin = top-left corner of tag ID 0.  Z = 0 (planar board).
    """
    pts = []
    for row in range(TAG_ROWS):
        for col in range(TAG_COLS):
            x0 = col * TAG_STRIDE
            y0 = row * TAG_STRIDE
            x1 = x0 + TAG_SIZE
            y1 = y0 + TAG_SIZE
            pts.extend([
                [x0, y0, 0.0],
                [x1, y0, 0.0],
                [x1, y1, 0.0],
                [x0, y1, 0.0],
            ])
    return np.array(pts, dtype=np.float32)


ALL_OBJECT_POINTS = build_object_points()


def make_detector() -> cv2.aruco.ArucoDetector:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    params = cv2.aruco.DetectorParameters()

    params.adaptiveThreshWinSizeMin    = THRESH_WIN_MIN
    params.adaptiveThreshWinSizeMax    = THRESH_WIN_MAX
    params.adaptiveThreshWinSizeStep   = THRESH_WIN_STEP
    params.adaptiveThreshConstant      = 7

    params.minMarkerPerimeterRate      = MIN_MARKER_PERIMETER_RATE
    params.maxMarkerPerimeterRate      = 4.0
    params.polygonalApproxAccuracyRate = 0.05
    params.minCornerDistanceRate       = 0.05
    params.minDistanceToBorder         = 3

    params.cornerRefinementMethod        = cv2.aruco.CORNER_REFINE_SUBPIX
    params.cornerRefinementWinSize       = 5
    params.cornerRefinementMaxIterations = 50
    params.cornerRefinementMinAccuracy   = 0.01

    return cv2.aruco.ArucoDetector(dictionary, params)


DETECTOR = make_detector()


def detect_apriltags(gray: np.ndarray):
    """
    Returns (image_points, object_points) or (None, None) if fewer than
    MIN_TAGS_PER_IMAGE tags are found.
    """
    corners_list, ids, _ = DETECTOR.detectMarkers(gray)

    if ids is None or len(ids) < MIN_TAGS_PER_IMAGE:
        return None, None

    img_pts = []
    obj_pts = []

    for tag_corners, tag_id in zip(corners_list, ids.flatten()):
        if tag_id >= N_TAGS_TOTAL:
            continue
        detected = tag_corners.reshape(4, 2)
        base_idx = int(tag_id) * 4
        world    = ALL_OBJECT_POINTS[base_idx : base_idx + 4]
        img_pts.append(detected)
        obj_pts.append(world)

    if len(img_pts) < MIN_TAGS_PER_IMAGE:
        return None, None

    img_arr = np.concatenate(img_pts).reshape(-1, 1, 2).astype(np.float32)
    obj_arr = np.concatenate(obj_pts).reshape(-1, 1, 3).astype(np.float32)
    return img_arr, obj_arr


def compute_mean_reprojection_error(
    obj_pts_list, img_pts_list, rvecs, tvecs, K, dist
) -> float:
    total_err = 0.0
    total_pts = 0
    for obj_pts, img_pts, rvec, tvec in zip(
        obj_pts_list, img_pts_list, rvecs, tvecs
    ):
        projected, _ = cv2.projectPoints(obj_pts, rvec, tvec, K, dist)
        err = cv2.norm(img_pts, projected, cv2.NORM_L2)
        total_err += err * err
        total_pts += len(obj_pts)
    return float(np.sqrt(total_err / total_pts))


def calibrate(location: str, debug: bool = False):
    loc = Path(location).resolve()
    print(f"\nLoading .raw images from: {loc}")

    gray_images, image_size = load_images(str(loc))

    if not gray_images:
        print("[ERROR] No images loaded.", file=sys.stderr)
        sys.exit(1)

    debug_dir = loc / "debug"
    if debug:
        debug_dir.mkdir(exist_ok=True)

    all_obj_pts = []
    all_img_pts = []
    n_used      = 0

    print(f"\nDetecting AprilTags ({TAG_COLS}x{TAG_ROWS} grid (landscape), "
          f"tag={TAG_SIZE*1000:.0f}mm, spacing={TAG_SPACING*1000:.0f}mm)...\n")

    for idx, gray in enumerate(gray_images):
        img_pts, obj_pts = detect_apriltags(gray)
        n_tags    = 0 if img_pts is None else len(img_pts) // 4
        n_corners = 0 if img_pts is None else len(img_pts)
        status    = "OK" if img_pts is not None else "SKIP"

        print(
            f"  [{idx+1:>2}/{len(gray_images)}]  "
            f"tags: {n_tags:>3} / {N_TAGS_TOTAL}  "
            f"corners: {n_corners:>4} / {N_CORNERS_TOTAL}  [{status}]"
        )

        if img_pts is None:
            if debug:
                _save_debug(gray, [], debug_dir, idx, n_tags=0)
            continue

        all_obj_pts.append(obj_pts)
        all_img_pts.append(img_pts)
        n_used += 1

        if debug:
            _save_debug(gray, img_pts, debug_dir, idx, n_tags=n_tags)

    print(f"\nUsing {n_used} / {len(gray_images)} images for calibration.")

    if n_used < 4:
        print(
            "[ERROR] Need at least 4 usable images. "
            "Run with --debug to inspect detections.",
            file=sys.stderr,
        )
        sys.exit(1)

    print("Running cv2.calibrateCamera...")

    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        all_obj_pts,
        all_img_pts,
        image_size,
        None,
        None,
        flags=cv2.CALIB_RATIONAL_MODEL,
    )

    mean_err = compute_mean_reprojection_error(
        all_obj_pts, all_img_pts, rvecs, tvecs, K, dist
    )

    print()
    print("=" * 35)
    print("=== Calibration complete ===")
    print(f"RMS: {rms:.6f}")
    print(f"Mean reprojection error: {mean_err:.6f}")
    print("Calibration Matrix:")
    _print_matrix(K)
    print("\nDistortion Coefficients:")
    print(np.array2string(
        dist.flatten(),
        formatter={"float_kind": lambda x: f"{x: .8f}"}
    ))
    print("=" * 35)

    out_path = loc / "intrinsics.npz"
    np.savez(
        str(out_path),
        K=K,
        dist=dist,
        rms=np.array([rms]),
        mean_reprojection_error=np.array([mean_err]),
        image_size=np.array(image_size),
    )
    print(f"\nCalibration saved to: {out_path}")
    print("  Load with:  data = np.load('intrinsics.npz')")
    print("              K    = data['K']")
    print("              dist = data['dist']")

    return K, dist, rms, mean_err


def _print_matrix(M: np.ndarray):
    rows = ["[" + "  ".join(f"{v:>14.8f}" for v in row) + "]" for row in M]
    print("[" + rows[0])
    for r in rows[1:-1]:
        print(" " + r)
    print(" " + rows[-1] + "]")


def _save_debug(gray, img_pts, debug_dir, idx, n_tags):
    vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    accepted = n_tags > 0
    if accepted and len(img_pts) > 0:
        for pt in np.array(img_pts).reshape(-1, 2):
            cv2.circle(vis, (int(pt[0]), int(pt[1])), 10, (0, 255, 0), -1)
    label = f"OK_{n_tags}tags" if accepted else "NO_DETECT"
    cv2.putText(vis, label, (30, 100), cv2.FONT_HERSHEY_SIMPLEX, 3,
                (0, 255, 0) if accepted else (0, 0, 255), 5)
    scale = 0.15
    small = cv2.resize(vis, (int(vis.shape[1]*scale), int(vis.shape[0]*scale)))
    cv2.imwrite(str(debug_dir / f"debug_{idx:03d}_{label}.jpg"), small)


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Intrinsic calibration from arena_camera_node .raw images.",
    )
    p.add_argument("--loc", default=RAW_DIR,
                   help=f"Directory containing .raw files (default: {RAW_DIR})")
    p.add_argument("--debug", action="store_true",
                   help="Save annotated detection images into <loc>/debug/")
    return p


if __name__ == "__main__":
    args = _build_parser().parse_args()
    calibrate(location=args.loc, debug=args.debug)