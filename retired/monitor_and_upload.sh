#!/usr/bin/env bash
set -euo pipefail

### CONFIGURATION ###

# Local folder to monitor (trailing slash is important)
LOCAL_DIR="/mnt/wildfire/upload"

# Remote path on the server (will be expanded via your ~/.ssh/config alias)
REMOTE_PATH="/home/rob501/wildfire"

# SSH alias from ~/.ssh/config
#SSH_ALIAS="wildfire-server"
REMOTE_HOST="rob501@trail.utias.utoronto.ca"

# Where to write our log
LOGFILE="/var/log/monitor_and_upload.log"

# How long to wait between sync passes (seconds)
SLEEP_INTERVAL=30

### MAIN LOOP ###

while true; do
  echo "[$(date '+%F %T')] Starting rsync pass" >> "$LOGFILE"

  rsync -az \
        --partial \
        --remove-source-files \
        --stats \
        -e "ssh -i /home/trail/.ssh/upload_key" \
      "${LOCAL_DIR}"  "${REMOTE_HOST}:/home/rob501/wildfire"
    >> "$LOGFILE" 2>&1 \
    || echo "[$(date '+%F %T')] rsync failed, will retry" >> "$LOGFILE"

  # Remove any now-empty directories under LOCAL_DIR
  find "${LOCAL_DIR%/}" -mindepth 1 -type d -empty -delete

  echo "[$(date '+%F %T')] Sleeping for ${SLEEP_INTERVAL}s" >> "$LOGFILE"
  sleep "$SLEEP_INTERVAL"
done
