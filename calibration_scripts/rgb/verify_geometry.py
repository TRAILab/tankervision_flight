#!/usr/bin/env python3
"""
verify_geometry.py
------------------
Cross-checks detected tag positions against expected world geometry.
Measures actual pixel distances between adjacent tags and compares
to expected pixel distances implied by our object points + initial K estimate.

Run this BEFORE calibration to verify the world point layout is correct.

Usage:
    python verify_geometry.py --loc /path/to/raw/files
"""
import re
import sys
from pathlib import Path
import cv2
import numpy as np

RAW_DIR = "/home/atlas/tankervision_flight/home/atlas/captures/arena_raw"

# -- Board config (must match calibrate_intrinsics.py) -----------------------
TAG_COLS   = 11
TAG_ROWS   = 8
TAG_SIZE   = 0.035
TAG_SPACING = 0.011
TAG_STRIDE = TAG_SIZE + TAG_SPACING  # 0.046 m

_FNAME_RE = re.compile(
    r"frame_(?P<id>\d+)_\d{8}_\d{6}_\d{6}_"
    r"(?P<width>\d+)x(?P<height>\d+)_(?P<bpp>\d+)bpp\.raw$"
)

def load_first(loc):
    files = sorted(Path(loc).resolve().glob("*.raw"))
    for f in files:
        m = _FNAME_RE.match(f.name)
        if not m: continue
        w,h,bpp = int(m.group("width")), int(m.group("height")), int(m.group("bpp"))
        dtype = np.uint16 if bpp > 8 else np.uint8
        raw = np.frombuffer(f.read_bytes()[:w*h*2], dtype=dtype).reshape(h,w)
        amax = int(raw.max())
        raw8 = (raw >> max(int(np.ceil(np.log2(amax+1)))-8, 0)).astype(np.uint8) if amax > 255 else raw.astype(np.uint8)
        return cv2.cvtColor(raw8, cv2.COLOR_BayerBG2GRAY), w, h, f.name
    return None, 0, 0, ""

def detect(gray):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    params = cv2.aruco.DetectorParameters()
    params.adaptiveThreshWinSizeMin  = 3
    params.adaptiveThreshWinSizeMax  = 101
    params.adaptiveThreshWinSizeStep = 4
    params.minMarkerPerimeterRate    = 0.05
    params.cornerRefinementMethod    = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.ArucoDetector(dictionary, params)
    corners, ids, _ = detector.detectMarkers(gray)
    return corners, ids

def tag_center(corners_list, ids, tid):
    for c, i in zip(corners_list, ids.flatten()):
        if i == tid:
            return c.reshape(4,2).mean(axis=0)
    return None

import argparse
p = argparse.ArgumentParser()
p.add_argument("--loc", default=RAW_DIR)
args = p.parse_args()

gray, img_w, img_h, fname = load_first(args.loc)
print(f"Image: {fname}  ({img_w}x{img_h})")

corners_list, ids = detect(gray)
n = 0 if ids is None else len(ids)
print(f"Tags detected: {n}")
print(f"Tag IDs (sorted): {sorted(ids.flatten().tolist()) if ids is not None else []}\n")

if ids is None or n < 10:
    print("Not enough tags to verify geometry.")
    sys.exit(1)

ids_flat = ids.flatten().tolist()

# Build map: tag_id -> center pixel
centers = {}
for c, tid in zip(corners_list, ids.flatten()):
    centers[int(tid)] = c.reshape(4,2).mean(axis=0)

# -- Measure pixel distances between horizontally adjacent tags ---------------
print("=== Horizontal adjacent tag spacing (should be constant) ===")
print(f"{'Tag A':>6}  {'Tag B':>6}  {'px dist':>10}  {'world dist (m)':>16}  {'px/m':>10}")
h_dists = []
for tid_a in sorted(ids_flat):
    row_a = tid_a // TAG_COLS
    col_a = tid_a % TAG_COLS
    tid_b = tid_a + 1  # next tag in same row
    row_b = tid_b // TAG_COLS
    if row_b != row_a: continue  # skip row boundary
    if tid_b not in centers: continue
    dx = centers[tid_b] - centers[tid_a]
    px_dist = float(np.linalg.norm(dx))
    world_dist = TAG_STRIDE
    h_dists.append(px_dist)
    print(f"{tid_a:>6}  {tid_b:>6}  {px_dist:>10.2f}  {world_dist:>16.4f}  {px_dist/world_dist:>10.1f}")

print()
print("=== Vertical adjacent tag spacing (should be constant) ===")
print(f"{'Tag A':>6}  {'Tag B':>6}  {'px dist':>10}  {'world dist (m)':>16}  {'px/m':>10}")
v_dists = []
for tid_a in sorted(ids_flat):
    tid_b = tid_a + TAG_COLS  # tag directly below
    if tid_b not in centers: continue
    dy = centers[tid_b] - centers[tid_a]
    px_dist = float(np.linalg.norm(dy))
    world_dist = TAG_STRIDE
    v_dists.append(px_dist)
    print(f"{tid_a:>6}  {tid_b:>6}  {px_dist:>10.2f}  {world_dist:>16.4f}  {px_dist/world_dist:>10.1f}")

print()
if h_dists and v_dists:
    h_mean = np.mean(h_dists)
    v_mean = np.mean(v_dists)
    print(f"Mean horizontal spacing: {h_mean:.2f} px  (px/m = {h_mean/TAG_STRIDE:.1f})")
    print(f"Mean vertical spacing:   {v_mean:.2f} px  (px/m = {v_mean/TAG_STRIDE:.1f})")
    print(f"Ratio h/v: {h_mean/v_mean:.4f}  (should be ~1.0 for square grid)")
    print()

    # Naive focal length estimate (assuming principal point at image center)
    cx, cy = img_w/2, img_h/2
    # f ~ px_dist / atan(world_stride / dist_to_board)
    # We can't know dist but we can check consistency:
    # If h and v spacings are similar in px and world, focal lengths should match
    print(f"Implied fx (from horizontal): {h_mean/TAG_STRIDE:.1f} px/m  "
          f"-> if board ~2m away, f ~ {h_mean/TAG_STRIDE * 2:.0f} px")
    print(f"Implied fy (from vertical):   {v_mean/TAG_STRIDE:.1f} px/m  "
          f"-> if board ~2m away, f ~ {v_mean/TAG_STRIDE * 2:.0f} px")

    print()
    print("=== TAG ID LAYOUT CHECK ===")
    print("Verifying tag 0 is top-left, IDs increase left->right then top->bottom")
    for check_row in range(min(3, TAG_ROWS)):
        row_ids = []
        for check_col in range(TAG_COLS):
            tid = check_row * TAG_COLS + check_col
            if tid in centers:
                row_ids.append(f"{tid:2d}@({centers[tid][0]:.0f},{centers[tid][1]:.0f})")
            else:
                row_ids.append(f"{tid:2d}@MISSING")
        print(f"  Row {check_row}: {' | '.join(row_ids[:5])} ...")

    print()
    print("Check: do X coords INCREASE as col increases? Do Y coords INCREASE as row increases?")
    print("If X coords DECREASE or Y is wrong direction, the board ID layout is mirrored/rotated.")