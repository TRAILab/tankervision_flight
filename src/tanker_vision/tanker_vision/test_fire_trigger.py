#!/usr/bin/env python3
import time
from pathlib import Path

import cv2
from ultralytics import YOLO


REPO_ROOT = Path(__file__).resolve().parents[3]
MODEL_PATH = REPO_ROOT / "src/tanker_vision/tanker_vision/fire_detection.pt"
IMAGE_PATH = REPO_ROOT / "src/tanker_vision/tanker_vision/test_model/images/smoke.png"


def will_trigger_recording() -> bool:
    import torch

    # Load model
    model = YOLO(str(MODEL_PATH), verbose=False)
    model.to("cuda")

    print(model.names)
    print(f"Model input size: {getattr(model, 'overrides', {}).get('imgsz', 'default/auto')}")
    print("Model device:", next(model.model.parameters()).device)

    # Read image (BGR)
    img = cv2.imread(str(IMAGE_PATH))
    if img is None:
        raise FileNotFoundError(f"Could not load image: {IMAGE_PATH}")

    print(f"Source image size: {img.shape[1]}x{img.shape[0]} pixels")

    # Warmup: do not time this
    _ = model.predict(
        img,
        imgsz=640,
        conf=0.3,
        device=0,
        verbose=False,
    )

    torch.cuda.synchronize()

    # Real timed inference
    start = time.perf_counter()

    results = model.predict(
        img,
        imgsz=640,
        conf=0.3,
        device=0,
        verbose=False,
    )

    torch.cuda.synchronize()
    elapsed_ms = (time.perf_counter() - start) * 1000.0

    print(f"YOLO inference time: {elapsed_ms:.2f} ms")

    if results:
        print(f"YOLO result original shape: {results[0].orig_shape}")

    # Debug: print all detections
    detected_classes = []

    if results and hasattr(results[0], "boxes") and results[0].boxes is not None:
        boxes = results[0].boxes

        if len(boxes) == 0:
            print("No boxes detected")
        else:
            for box in boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                name = model.names[cls_id]
                xyxy = box.xyxy[0].tolist()

                detected_classes.append(cls_id)
                print(f"Detected {name}: conf={conf:.3f}, box={xyxy}")

    # Trigger on fire or smoke
    return 0 in detected_classes or 2 in detected_classes

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
