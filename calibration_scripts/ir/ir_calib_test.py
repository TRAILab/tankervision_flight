from pathlib import Path
import cv2
import numpy as np

LOC_PATH = Path("captures/calibration_thermal/debug_detections/")
IMAGE_PATH = Path("captures/calibration_thermal/debug_detections/20260414_231609_334509_thermal_detected.png")
PATTERN_SIZE = (4, 11)   # also tries flipped automatically
# Optional filename filters.
# Adjust if your naming differs.
RGB_KEYWORDS = ["rgb"]
THERMAL_KEYWORDS = ["thermal"]

# Image extensions to consider
VALID_EXTENSIONS = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"}
def build_blob_detector(blob_color: int) -> cv2.SimpleBlobDetector:
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
    params.blobColor = blob_color  # 0 for dark blobs, 255 for bright blobs

    params.filterByConvexity = False
    params.filterByInertia = False

    return cv2.SimpleBlobDetector_create(params)


def preprocess(gray: np.ndarray) -> dict[str, np.ndarray]:
    # Normalize to 8-bit
    norm = cv2.normalize(gray, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

    #Inv Norm
    inv_norm = 255 - norm

    #Thresholded Image
    _, thresh = cv2.threshold(inv_norm, 15, 255, cv2.THRESH_BINARY)

    # CLAHE usually helps on thermal images
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    clahe_img = clahe.apply(norm)

    # Slight blur can stabilize blob detection
    blur = cv2.GaussianBlur(clahe_img, (5, 5), 0)

    inv = 255 - blur

    return {
        #"norm": norm,
        "inv_norm": inv_norm, 
        "inv_thresh": thresh,
        #"clahe_blur": blur,
        #"clahe_blur_inv": inv,
    }


def try_detect(
    img: np.ndarray,
    pattern_size: tuple[int, int],
    blob_color: int,
) -> tuple[bool, np.ndarray | None, list[cv2.KeyPoint], np.ndarray, np.ndarray]:
    detector = build_blob_detector(blob_color)

    keypoints = detector.detect(img)
    kp_vis = cv2.drawKeypoints(
        img,
        keypoints,
        None,
        (0, 0, 255),
        cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS,
    )

    flags = cv2.CALIB_CB_ASYMMETRIC_GRID | cv2.CALIB_CB_CLUSTERING

    found, centers = cv2.findCirclesGrid(
        img,
        pattern_size,
        flags=flags,
        blobDetector=detector,
    )

    grid_vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    if found and centers is not None:
        cv2.drawChessboardCorners(grid_vis, pattern_size, centers, found)

    return found, centers, keypoints, kp_vis, grid_vis

def get_image_paths(image_dir: Path, image_type: str):
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

def main() -> None:
    #if not IMAGE_PATH.exists():
        #raise FileNotFoundError(f"Image not found: {IMAGE_PATH}")
    
    LOC = get_image_paths(LOC_PATH, "thermal")

    for IMAGE_PATH in LOC: 
        img = cv2.imread(str(IMAGE_PATH), cv2.IMREAD_UNCHANGED)
        if img is None:
            raise RuntimeError(f"Could not read image: {IMAGE_PATH}")

        if img.ndim == 3:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            gray = img.copy()

        processed_versions = preprocess(gray)
        pattern_sizes = [PATTERN_SIZE]#, (PATTERN_SIZE[1], PATTERN_SIZE[0])]

        print(f"Testing image: {IMAGE_PATH}")
        print(f"Original PATTERN_SIZE: {PATTERN_SIZE}")
        print()

        any_success = False

        for proc_name, proc_img in processed_versions.items():
            for pat in pattern_sizes:
                for blob_color, blob_label in [(0, "dark")]: # (255, "bright")
                    found, centers, keypoints, kp_vis, grid_vis = try_detect(
                        proc_img, pat, blob_color
                    )

                    print(
                        f"[{proc_name:14s}] pattern={pat}, blobs={blob_label:6s}, "
                        f"keypoints={len(keypoints):3d}, found={found}"
                    )

                    cv2.imshow(f"{proc_name} | pattern={pat} | blobs={blob_label} | keypoints", kp_vis)
                    cv2.imshow(f"{proc_name} | pattern={pat} | blobs={blob_label} | grid", grid_vis)

                    key = cv2.waitKey(0) & 0xFF
                    if key == ord("q"):
                        cv2.destroyAllWindows()
                        return

                    if found:
                        any_success = True
                        print("\nSUCCESS:")
                        print(f"  processed image : {proc_name}")
                        print(f"  pattern size    : {pat}")
                        print(f"  blob polarity   : {blob_label}")
                        print("\nPress any key in the image window to continue, or q to quit.\n")

        cv2.destroyAllWindows()

        if not any_success:
            print("\nNo detections found with the tested settings.")


if __name__ == "__main__":
    main()