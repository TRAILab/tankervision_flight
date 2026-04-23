#!/bin/bash
# Usage: bash save_imu.sh --output /mnt/storage/{SEQ_NAME}
#
# Requires: sudo apt install ros-humble-rosbag2-storage-mcap

# ── Parse arguments ──────────────────────────────────────────────
OUTPUT_DIR=""
IMU_PORT=/dev/im19_mems
IMU_BAUD=115200

while [[ $# -gt 0 ]]; do
  case $1 in
    --output)   OUTPUT_DIR="$2";   shift 2 ;;
    --port)     IMU_PORT="$2";     shift 2 ;;
    *) echo "[ERROR] Unknown argument: $1"; exit 1 ;;
  esac
done

if [[ -z "$OUTPUT_DIR" ]]; then
  echo "[ERROR] --output is required. e.g. bash save_imu.sh --output /mnt/storage/seq_01"
  exit 1
fi

# ── Setup ────────────────────────────────────────────────────────
source /opt/ros/humble/setup.bash

BAG_PATH="$OUTPUT_DIR/imu_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUTPUT_DIR"

# ── Configure IM19 via AT commands ───────────────────────────────
echo "[INFO] Configuring IM19 on $IMU_PORT ..."

python3 - <<PY
import time, serial, sys

try:
    ser = serial.Serial('$IMU_PORT', $IMU_BAUD, timeout=0.5)
except Exception as e:
    print(f"[ERROR] Cannot open $IMU_PORT: {e}")
    sys.exit(1)

time.sleep(1)

cmds = [
    b'AT+MEMS_OUTPUT=UART1,ON\r\n',
    b'AT+SAVE_ALL\r\n',
]

for cmd in cmds:
    ser.reset_input_buffer()
    ser.write(cmd)
    ser.flush()
    time.sleep(0.8)
    resp = ser.read(8192).decode(errors='replace')
    if 'OK' not in resp:
        print(f"[WARN] Unexpected response: {resp!r}")

ser.close()
print("[INFO] IM19 configuration done.")
PY

if [[ $? -ne 0 ]]; then
  echo "[ERROR] IM19 configuration failed. Aborting."
  exit 1
fi

# ── Record ROS2 bag (MCAP) ───────────────────────────────────────
echo "[INFO] Starting ros2 bag record (MCAP) -> $BAG_PATH"

ros2 bag record \
  --storage mcap \
  --output "$BAG_PATH" \
  /im19/imu \
  /im19/raw_data

echo "[INFO] Recording stopped. Bag saved to: $BAG_PATH"