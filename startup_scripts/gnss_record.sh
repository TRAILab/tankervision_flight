#!/bin/bash

GNSS_DEVICE="/dev/gnss_raw"
SESSION_FILE="/tmp/tankervision_session_path"
CURRENT_SESSION=""
CAT_PID=""
RESTART_DELAY_SEC=5

log() {
    echo "[gnss_record] $(date --iso-8601=seconds) $*"
}

configure_device() {
    log "Configuring $GNSS_DEVICE at 921600 baud"
    stty -F "$GNSS_DEVICE" 921600 raw -echo -ixon -ixoff -icrnl -inlcr -opost
}

while [[ ! -e "$GNSS_DEVICE" ]]; do
    log "Waiting for $GNSS_DEVICE"
    sleep 1
done

configure_device

start_cat() {
    if [[ -z "$CURRENT_SESSION" ]]; then
        log "start_cat called without CURRENT_SESSION; skipping"
        return
    fi
    local output_file="$CURRENT_SESSION/gnss_$(date +%Y%m%d_%H%M%S).ubx"
    log "Starting GNSS capture from $GNSS_DEVICE to $output_file"
    cat "$GNSS_DEVICE" >> "$output_file" &
    CAT_PID=$!
    log "GNSS cat pid=$CAT_PID"
}

stop_cat() {
    if [[ -n "$CAT_PID" ]]; then
        log "Stopping GNSS cat pid=$CAT_PID"
        kill "$CAT_PID" 2>/dev/null || true
        wait "$CAT_PID" 2>/dev/null || true
        CAT_PID=""
    fi
}

while true; do
    if [[ ! -e "$GNSS_DEVICE" ]]; then
        log "$GNSS_DEVICE disappeared"
        stop_cat
        while [[ ! -e "$GNSS_DEVICE" ]]; do
            sleep 1
        done
        log "$GNSS_DEVICE reappeared"
        configure_device
        [[ -n "$CURRENT_SESSION" ]] && start_cat
    fi

    SESSION=$(tr -d '\r\n' < "$SESSION_FILE" 2>/dev/null || echo "")
    if [[ -n "$SESSION" ]] && [[ "$SESSION" != "$CURRENT_SESSION" ]]; then
        log "Session changed: '${CURRENT_SESSION:-none}' -> '$SESSION'"
        stop_cat
        CURRENT_SESSION="$SESSION"
        mkdir -p "$CURRENT_SESSION"
        start_cat
    fi

    if [[ -n "$CAT_PID" ]] && ! kill -0 "$CAT_PID" 2>/dev/null; then
        wait "$CAT_PID"
        CAT_STATUS=$?
        log "GNSS cat pid=$CAT_PID exited with status $CAT_STATUS; restarting in ${RESTART_DELAY_SEC}s"
        CAT_PID=""
        sleep "$RESTART_DELAY_SEC"
        if [[ -e "$GNSS_DEVICE" ]] && [[ -n "$CURRENT_SESSION" ]]; then
            start_cat
        fi
    fi

    sleep 2
done
