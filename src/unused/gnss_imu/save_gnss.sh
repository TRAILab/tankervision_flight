#!/bin/bash
# Usage: bash save_gnss.sh --output /mnt/storage/{SEQ_NAME}

# ── Parse arguments ──────────────────────────────────────────────
OUTPUT_DIR=""
GNSS_PORT=/dev/gnss_raw
GNSS_BAUD=921600
CHUNK_DURATION=1800  # seconds per file (default 30 min)

while [[ $# -gt 0 ]]; do
  case $1 in
    --output)   OUTPUT_DIR="$2";       shift 2 ;;
    --port)     GNSS_PORT="$2";        shift 2 ;;
    --chunk)    CHUNK_DURATION="$2";   shift 2 ;;
    *) echo "[ERROR] Unknown argument: $1"; exit 1 ;;
  esac
done

if [[ -z "$OUTPUT_DIR" ]]; then
  echo "[ERROR] --output is required. e.g. bash save_gnss.sh --output /mnt/storage/seq_01"
  exit 1
fi

# ── Setup ────────────────────────────────────────────────────────
mkdir -p "$OUTPUT_DIR"
echo "[INFO] Saving GNSS data to: $OUTPUT_DIR"
echo "[INFO] Chunk duration: ${CHUNK_DURATION}s"

# ── Wait for device ──────────────────────────────────────────────
while [[ ! -e "$GNSS_PORT" ]]; do
  echo "[INFO] Waiting for $GNSS_PORT ..."
  sleep 1
done

# ── Configure serial port ────────────────────────────────────────
stty -F "$GNSS_PORT" "$GNSS_BAUD" raw -echo -ixon -ixoff -icrnl -inlcr -opost
if [[ $? -ne 0 ]]; then
  echo "[ERROR] Failed to configure $GNSS_PORT. Aborting."
  exit 1
fi
echo "[INFO] $GNSS_PORT configured at $GNSS_BAUD baud."

# ── Record loop ──────────────────────────────────────────────────
while true; do
  # Re-check device still exists (e.g. USB unplugged mid-session)
  if [[ ! -e "$GNSS_PORT" ]]; then
    echo "[WARN] $GNSS_PORT disappeared. Waiting for reconnect ..."
    while [[ ! -e "$GNSS_PORT" ]]; do sleep 1; done
    stty -F "$GNSS_PORT" "$GNSS_BAUD" raw -echo -ixon -ixoff -icrnl -inlcr -opost
  fi

  OUTFILE="$OUTPUT_DIR/gnss_$(date +%Y%m%d_%H%M%S).ubx"
  echo "[INFO] Recording -> $OUTFILE"
  timeout "$CHUNK_DURATION" cat "$GNSS_PORT" > "$OUTFILE"
  echo "[INFO] Chunk saved: $OUTFILE"
done
