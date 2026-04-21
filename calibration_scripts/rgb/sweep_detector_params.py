#!/usr/bin/env python3
"""
sweep_detector_params.py
------------------------
Sweeps adaptive threshold window sizes and CLAHE settings to find the
combination that detects the most AprilTags on your images.

Usage:
    python sweep_detector_params.py --loc /path/to/raw/files
"""

import argparse
import sys
from pathlib import Path
import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))

RAW_DIR = "/home/atlas/tankervision_flight/home/atlas/captures/arena_raw/"

# ---- raw decode WITHOUT CLAHE so we can test it separately ------------------
import re
_FNAME_RE = re.compile(
    r"frame_(?P<id>\d+)_\d{8}_\d{6}_\d{6}_"
    r"(?P<width>\d+)x(?P<height>\d+)_(?P<bpp>\d+)bpp\.raw$"
)
_BAYER_GRAY = {
    "bayerrg8": cv2.COLOR_BayerBG2GRAY,
    "bayerrg16": cv2.COLOR_BayerBG2GRAY,
}

def load_one_raw(path: Path, use_clahe: bool = False) -> np.ndarray:
    m = _FNAME_RE.match(path.name)
    w, h, bpp = int(m.group("width")), int(m.group("height")), int(m.group("bpp"))
    dtype = np.uint16 if bpp > 8 else np.uint8
    raw = np.frombuffer(path.read_bytes()[:w*h*2], dtype=dtype).reshape(h, w)
    # shift to 8-bit using actual data range
    amax = int(raw.max())
    if amax > 255:
        ebits = int(np.ceil(np.log2(amax + 1)))
        raw = (raw >> max(ebits - 8, 0)).astype(np.uint8)
    else:
        raw = raw.astype(np.uint8)
    if use_clahe:
        raw = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8)).apply(raw)
    gray = cv2.cvtColor(raw, cv2.COLOR_BayerBG2GRAY)
    return gray


def try_detect(gray, win_min, win_max, win_step, min_perim):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    params = cv2.aruco.DetectorParameters()
    params.adaptiveThreshWinSizeMin    = win_min
    params.adaptiveThreshWinSizeMax    = win_max
    params.adaptiveThreshWinSizeStep   = win_step
    params.adaptiveThreshConstant      = 7
    params.minMarkerPerimeterRate      = min_perim
    params.maxMarkerPerimeterRate      = 4.0
    params.polygonalApproxAccuracyRate = 0.05
    params.cornerRefinementMethod      = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.ArucoDetector(dictionary, params)
    corners, ids, _ = detector.detectMarkers(gray)
    return 0 if ids is None else len(ids)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--loc", default=RAW_DIR)
    parser.add_argument("--n_images", type=int, default=3,
                        help="Number of images to test on (default 3)")
    args = parser.parse_args()

    loc = Path(args.loc).resolve()
    raw_files = sorted(loc.glob("*.raw"))[:args.n_images]
    if not raw_files:
        print(f"No .raw files in {loc}")
        sys.exit(1)

    print(f"Testing on {len(raw_files)} image(s) from {loc}\n")

    # First: measure actual tag pixel size from the image
    gray_no_clahe = load_one_raw(raw_files[0], use_clahe=False)
    gray_clahe    = load_one_raw(raw_files[0], use_clahe=True)
    h, w = gray_no_clahe.shape
    print(f"Image size: {w}x{h}")
    print(f"Pixel stats (no CLAHE): min={gray_no_clahe.min()} max={gray_no_clahe.max()} mean={gray_no_clahe.mean():.1f}")
    print(f"Pixel stats (CLAHE):    min={gray_clahe.min()}    max={gray_clahe.max()}    mean={gray_clahe.mean():.1f}\n")

    # Save a heavily downscaled version for inspection
    scale = 0.1
    small = cv2.resize(gray_no_clahe, (int(w*scale), int(h*scale)))
    cv2.imwrite("sweep_sample_noCLAHE.png", small)
    small_c = cv2.resize(gray_clahe, (int(w*scale), int(h*scale)))
    cv2.imwrite("sweep_sample_CLAHE.png", small_c)
    print("Saved sweep_sample_noCLAHE.png and sweep_sample_CLAHE.png\n")

    # --- Sweep parameters ----------------------------------------------------
    # Window sizes to try (must be odd)
    win_configs = [
        (3,  23,  2),   # very small - original
        (3,  53,  10),  # small-medium
        (5,  51,  4),   # fine small
        (7,  49,  6),   # medium small
        (11, 61,  10),  # medium
        (15, 75,  10),  # medium-large
        (21, 101, 20),  # large
        (31, 151, 20),  # larger
        (41, 201, 20),  # very large (previous attempt)
        (3,  201, 10),  # full range coarse
        (3,  101, 4),   # full small-medium fine
    ]
    min_perims = [0.003, 0.01, 0.05]
    clahe_opts = [False, True]

    print(f"{'CLAHE':<6} {'win_min':>7} {'win_max':>7} {'step':>5} {'min_per':>8}  "
          + "  ".join([f"img{i+1:02d}" for i in range(len(raw_files))]) + "  total")
    print("-" * 80)

    best = {"total": -1, "config": None}

    for use_clahe in clahe_opts:
        grays = [load_one_raw(f, use_clahe=use_clahe) for f in raw_files]
        for (wmin, wmax, wstep) in win_configs:
            for min_p in min_perims:
                counts = [try_detect(g, wmin, wmax, wstep, min_p) for g in grays]
                total = sum(counts)
                counts_str = "  ".join(f"{c:>5}" for c in counts)
                print(f"{'yes':<6} {wmin:>7} {wmax:>7} {wstep:>5} {min_p:>8.3f}  {counts_str}  {total:>5}"
                      if use_clahe else
                      f"{'no':<6} {wmin:>7} {wmax:>7} {wstep:>5} {min_p:>8.3f}  {counts_str}  {total:>5}")
                if total > best["total"]:
                    best = {
                        "total": total,
                        "config": dict(use_clahe=use_clahe, wmin=wmin,
                                       wmax=wmax, wstep=wstep, min_p=min_p),
                        "counts": counts,
                    }

    print("\n" + "=" * 80)
    print(f"BEST: {best['config']}  ->  detections per image: {best['counts']}  total: {best['total']}")

    # Save annotated image using best params
    cfg = best["config"]
    if cfg:
        gray_best = load_one_raw(raw_files[0], use_clahe=cfg["use_clahe"])
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        params = cv2.aruco.DetectorParameters()
        params.adaptiveThreshWinSizeMin    = cfg["wmin"]
        params.adaptiveThreshWinSizeMax    = cfg["wmax"]
        params.adaptiveThreshWinSizeStep   = cfg["wstep"]
        params.minMarkerPerimeterRate      = cfg["min_p"]
        params.maxMarkerPerimeterRate      = 4.0
        params.polygonalApproxAccuracyRate = 0.05
        params.cornerRefinementMethod      = cv2.aruco.CORNER_REFINE_SUBPIX
        detector = cv2.aruco.ArucoDetector(dictionary, params)
        corners, ids, _ = detector.detectMarkers(gray_best)
        vis = cv2.cvtColor(gray_best, cv2.COLOR_GRAY2BGR)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(vis, corners, ids)
        small_vis = cv2.resize(vis, (int(vis.shape[1]*0.15), int(vis.shape[0]*0.15)))
        cv2.imwrite("sweep_best_result.jpg", small_vis)
        print("Saved sweep_best_result.jpg -- check that tags are correctly detected")

        print("\n--- Copy these values into calibrate_intrinsics.py ---")
        print(f"THRESH_WIN_MIN  = {cfg['wmin']}")
        print(f"THRESH_WIN_MAX  = {cfg['wmax']}")
        print(f"THRESH_WIN_STEP = {cfg['wstep']}")
        print(f"MIN_MARKER_PERIMETER_RATE = {cfg['min_p']}")
        print(f"use_clahe in arena_raw_loader = {cfg['use_clahe']}")

if __name__ == "__main__":
    main()