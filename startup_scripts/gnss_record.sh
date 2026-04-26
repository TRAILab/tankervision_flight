#!/bin/bash
set -euo pipefail

while [[ ! -e /dev/gnss_raw ]]; do
    sleep 1
done

while [[ $(date +%Y) -lt 2020 ]]; do
    sleep 1
done

OUTPUT_DIR=/mnt/storage/$(date +%Y-%m-%d)/gnss_$(date +%H-%M-%S)
mkdir -p "$OUTPUT_DIR"

stty -F /dev/gnss_raw 921600 raw -echo -ixon -ixoff -icrnl -inlcr -opost

while true; do
    if [[ ! -e /dev/gnss_raw ]]; then
        while [[ ! -e /dev/gnss_raw ]]; do sleep 1; done
        stty -F /dev/gnss_raw 921600 raw -echo -ixon -ixoff -icrnl -inlcr -opost
    fi
    OUTFILE="$OUTPUT_DIR/gnss_$(date +%Y%m%d_%H%M%S).ubx"
    timeout 1800 cat /dev/gnss_raw > "$OUTFILE" || true
done
ß