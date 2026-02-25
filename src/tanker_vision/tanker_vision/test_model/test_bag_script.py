#!/usr/bin/env python3
import argparse
import os
import subprocess
import time
from datetime import datetime

def start_recording(path_root: str, record_secs: int, ros2_bin: str):
    # 1) Build timestamped path
    now = datetime.now()
    ts = now.strftime("%Y-%m-%d_%H-%M-%S")
    bag_path = os.path.join(path_root, ts)

    # 2) Launch ros2 bag record
    cmd = [
        ros2_bin, "bag", "record",
        "-o", bag_path,
        "-a",
        "--compression-mode", "message",
        "--compression-format", "zstd",
        "-b", "100000000"
    ]
    print(f"[TEST] Starting recording: {' '.join(cmd)}")
    proc = subprocess.Popen(cmd)

    # 3) Let it run for record_secs seconds
    time.sleep(20)

    # 4) Terminate
    print(f"[TEST] Stopping recording after {record_secs}s")
    proc.terminate()
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()

    # 5) Check output folder contents
    files = os.listdir(bag_path)
    if files:
        print(f"[PASS] Bag directory '{bag_path}' contains {len(files)} file(s):")
        for f in files:
            print("   ", f)
        return True
    else:
        print(f"[FAIL] No files found in '{bag_path}'")
        return False

def main():
    parser = argparse.ArgumentParser(
        description="Test whether ros2 bag record works as expected."
    )
    parser.add_argument(
        "--path-root", "-p",
        required=True,
        help="Root folder under which to create timestamped bag dirs"
    )
    parser.add_argument(
        "--duration", "-d",
        type=int,
        default=5,
        help="How many seconds to record before stopping (default: 5)"
    )
    parser.add_argument(
        "--ros2-bin", "-r",
        default="/opt/ros/humble/bin/ros2",
        help="Path to your ros2 executable (default: /opt/ros/humble/bin/ros2)"
    )
    args = parser.parse_args()

    success = start_recording(args.path_root, args.duration, args.ros2_bin)
    exit(0 if success else 1)

if __name__ == "__main__":
    main()
