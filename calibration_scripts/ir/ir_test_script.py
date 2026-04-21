from pathlib import Path

import cv2
import numpy as np

IMAGE_PATH = Path("captures/calibration_thermal/debug_detections/20260414_231609_334509_thermal_detected.png")
PATTERN_SIZE = (4, 11)


def order_quad_points(pts: np.ndarray) -> np.ndarray:
    pts = pts.astype(np.float32)
    s = pts.sum(axis=1)
    d = np.diff(pts, axis=1).reshape(-1)

    ordered = np.zeros((4, 2), dtype=np.float32)
    ordered[0] = pts[np.argmin(s)]   # top-left
    ordered[2] = pts[np.argmax(s)]   # bottom-right
    ordered[1] = pts[np.argmin(d)]   # top-right
    ordered[3] = pts[np.argmax(d)]   # bottom-left
    return ordered


def find_board_mask(gray: np.ndarray) -> tuple[np.ndarray, np.ndarray | None]:
    """
    Find the calibration board as a large warm-ish rectangle.
    Returns:
        mask: uint8 mask, 255 inside board, 0 outside
        quad: 4x2 float32 ordered corners if found, else None
    """
    norm = cv2.normalize(gray, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

    # Smooth a bit to stabilize board contour
    blur = cv2.GaussianBlur(norm, (7, 7), 0)

    # Otsu threshold on the non-inverted image to pick out the warm board
    _, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # Close holes / clean boundary
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=1)

    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    h, w = gray.shape
    image_area = h * w

    best_quad = None
    best_area = 0

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 0.08 * image_area:
            continue

        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.03 * peri, True)

        if len(approx) == 4:
            quad = approx.reshape(4, 2)
            if area > best_area:
                best_area = area
                best_quad = order_quad_points(quad)

    mask = np.zeros_like(gray, dtype=np.uint8)

    if best_quad is not None:
        cv2.fillConvexPoly(mask, best_quad.astype(np.int32), 255)
        # Shrink slightly to exclude bright board border
        erode_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11))
        mask = cv2.erode(mask, erode_kernel, iterations=1)

    return mask, best_quad


def build_blob_detector(blob_color: int) -> cv2.SimpleBlobDetector:
    params = cv2.SimpleBlobDetector_Params()

    params.minThreshold = 0
    params.maxThreshold = 255
    params.thresholdStep = 5

    params.filterByArea = True
    params.minArea = 5
    params.maxArea = 400

    params.filterByCircularity = True
    params.minCircularity = 0.7

    params.filterByColor = True
    params.blobColor = 0  # 0 for dark blobs, 255 for bright blobs

    params.filterByConvexity = False
    params.filterByInertia = False

    return cv2.SimpleBlobDetector_create(params)


def preprocess_for_dots(gray: np.ndarray) -> np.ndarray:
    norm = cv2.normalize(gray, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    proc = 255 - norm
    #proc = cv2.GaussianBlur(proc, (5, 5), 0)
    return proc


def detect_grid(
    proc: np.ndarray,
    mask: np.ndarray,
    pattern_size: tuple[int, int],
) -> tuple[bool, np.ndarray | None, list[cv2.KeyPoint], np.ndarray, np.ndarray]:
    masked = cv2.bitwise_and(proc, proc, mask=mask)

    detector = build_blob_detector(blob_color=255)
    keypoints = detector.detect(masked)

    kp_vis = cv2.drawKeypoints(
        masked,
        keypoints,
        None,
        (0, 0, 255),
        cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS,
    )

    flags = cv2.CALIB_CB_ASYMMETRIC_GRID | cv2.CALIB_CB_CLUSTERING

    found, centers = cv2.findCirclesGrid(
        masked,
        pattern_size,
        flags=flags,
        blobDetector=detector,
    )

    grid_vis = cv2.cvtColor(masked, cv2.COLOR_GRAY2BGR)
    if found and centers is not None:
        cv2.drawChessboardCorners(grid_vis, pattern_size, centers, found)

    return found, centers, keypoints, kp_vis, grid_vis


def main() -> None:
    if not IMAGE_PATH.exists():
        raise FileNotFoundError(f"Image not found: {IMAGE_PATH}")

    img = cv2.imread(str(IMAGE_PATH), cv2.IMREAD_UNCHANGED)
    if img is None:
        raise RuntimeError(f"Could not read image: {IMAGE_PATH}")

    if img.ndim == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img.copy()

    proc = preprocess_for_dots(gray)
    mask, quad = find_board_mask(gray)

    board_vis = cv2.cvtColor(cv2.normalize(gray, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8), cv2.COLOR_GRAY2BGR)
    if quad is not None:
        cv2.polylines(board_vis, [quad.astype(np.int32)], True, (0, 255, 0), 2)
    else:
        print("Board rectangle not found.")
        cv2.imshow("board_vis", board_vis)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        return

    masked_vis = cv2.bitwise_and(proc, proc, mask=mask)

    print(f"Testing image: {IMAGE_PATH}")
    print("Board found.")
    print()

    pattern_sizes = [PATTERN_SIZE, (PATTERN_SIZE[1], PATTERN_SIZE[0])]
    any_success = False

    cv2.imshow("board_vis", board_vis)
    cv2.imshow("masked_proc", masked_vis)
    cv2.waitKey(1)

    for pat in pattern_sizes:
        found, centers, keypoints, kp_vis, grid_vis = detect_grid(proc, mask, pat)

        print(f"pattern={pat}, keypoints={len(keypoints)}, found={found}")

        cv2.imshow(f"keypoints_{pat}", kp_vis)
        cv2.imshow(f"grid_{pat}", grid_vis)

        key = cv2.waitKey(0) & 0xFF
        if key == ord("q"):
            cv2.destroyAllWindows()
            return

        if found:
            any_success = True
            print(f"\nSUCCESS with pattern {pat}\n")

    cv2.destroyAllWindows()

    if not any_success:
        print("\nNo detections found with rectangle filtering.")


if __name__ == "__main__":
    main()