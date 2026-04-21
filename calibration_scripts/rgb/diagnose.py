#!/usr/bin/env python3
# diag2.py
import sys, cv2, numpy as np
sys.path.insert(0, ".")
from arena_raw_loader import load_raw_images

LOC = "/home/atlas/tankervision_flight/home/atlas/captures/arena_raw/"  # <-- hardcoded absolute path

imgs = load_raw_images(LOC, as_rgb=False)
gray = imgs[0]

print(f"Image shape: {gray.shape}, dtype: {gray.dtype}")
print(f"Pixel min: {gray.min()}, max: {gray.max()}, mean: {gray.mean():.1f}")

# Save the raw grayscale as PNG so you can visually inspect it
small = cv2.resize(gray, (gray.shape[1]//8, gray.shape[0]//8))
cv2.imwrite("diag_gray.png", small)
print("Saved diag_gray.png -- open this and confirm the image looks correct")

# Run detector
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
params = cv2.aruco.DetectorParameters()
params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
params.minMarkerPerimeterRate = 0.003
detector = cv2.aruco.ArucoDetector(dictionary, params)
corners_list, ids, _ = detector.detectMarkers(gray)

n = 0 if ids is None else len(ids)
print(f"Tags detected: {n}")

if ids is not None:
    id_list = sorted(ids.flatten().tolist())
    print(f"Tag IDs found: {id_list}")
    
    # Print corners for tag ID 0 specifically (should be top-left of board)
    for c, tid in zip(corners_list, ids.flatten()):
        if tid == 0:
            pts = c.reshape(4,2)
            print(f"\nTag ID 0 corners (should be top-left tag on board):")
            for j, pt in enumerate(pts):
                print(f"  [{j}] ({pt[0]:.1f}, {pt[1]:.1f})")
            break

# Save annotated image
vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
if ids is not None:
    cv2.aruco.drawDetectedMarkers(vis, corners_list, ids)
small_vis = cv2.resize(vis, (vis.shape[1]//8, vis.shape[0]//8))
cv2.imwrite("diag_detected.png", small_vis)
print("\nSaved diag_detected.png -- confirm tags are detected correctly")