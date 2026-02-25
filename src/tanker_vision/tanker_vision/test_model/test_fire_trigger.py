#!/usr/bin/env python3
import argparse
import cv2
from ultralytics import YOLO

def will_trigger_recording() -> bool:
    # Load model
    model = YOLO('/home/trail/Desktop/TANKER_VISION/ros2_ws/src/tanker_vision/tanker_vision/fire_detection.pt', verbose=False)
    # Read image (BGR)
    print(model.names)
    img = cv2.imread("/home/trail/Desktop/TANKER_VISION/ros2_ws/src/tanker_vision/tanker_vision/test_model/images/smoke.png")
    if img is None:
        raise FileNotFoundError("Could not load image")
    # Run inference
    results = model(img, conf=0.3, verbose=False)
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
