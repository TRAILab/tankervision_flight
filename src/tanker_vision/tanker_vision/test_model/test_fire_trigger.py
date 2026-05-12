#!/usr/bin/env python3
import time
from pathlib import Path

import cv2
from ultralytics import YOLO


REPO_ROOT = Path(__file__).resolve().parents[4]
MODEL_PATH = REPO_ROOT / "src/tanker_vision/tanker_vision/fire_detection.pt"
IMAGE_PATH = REPO_ROOT / "src/tanker_vision/tanker_vision/test_model/images/smoke.png"


def will_trigger_recording() -> bool:
    # Load model
    model = YOLO(str(MODEL_PATH), verbose=False)
    # Read image (BGR)
    print(model.names)
    print(f"Model input size: {getattr(model, 'overrides', {}).get('imgsz', 'default/auto')}")

    img = cv2.imread(str(IMAGE_PATH))
    if img is None:
        raise FileNotFoundError(f"Could not load image: {IMAGE_PATH}")
    print(f"Source image size: {img.shape[1]}x{img.shape[0]} pixels")

    # Run inference
    start = time.perf_counter()
    results = model(img, conf=0.3, verbose=False)
    elapsed_ms = (time.perf_counter() - start) * 1000.0
    print(f"YOLO inference time: {elapsed_ms:.2f} ms")
    if results:
        print(f"YOLO result original shape: {results[0].orig_shape}")

    # Check for any detection of the target class
    if results and hasattr(results[0], "boxes"):
        if 2 in results[0].boxes.cls.int().tolist() or 0 in results[0].boxes.cls.int().tolist():
            return True
    return False

def main():
    try:
        trigger = will_trigger_recording()
    except Exception as e:
        print(f"ERROR: {e}")
        exit(2)

    if trigger:
        print("✅ Fire detected! Would trigger recording.")
        exit(0)
    else:
        print("❌ No fire detected. No recording.")
        exit(1)

if __name__ == "__main__":
    main()
