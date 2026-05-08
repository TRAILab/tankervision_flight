#!/bin/bash

while [[ ! -e /dev/gnss_raw ]]; do
    sleep 1
done

stty -F /dev/gnss_raw 921600 raw -echo -ixon -ixoff -icrnl -inlcr -opost

CURRENT_SESSION=""
CAT_PID=""

start_cat() {
    cat /dev/gnss_raw >> "$CURRENT_SESSION/gnss_$(date +%Y%m%d_%H%M%S).ubx" &
    CAT_PID=$!
}

while true; do
    if [[ ! -e /dev/gnss_raw ]]; then
        [[ -n "$CAT_PID" ]] && kill "$CAT_PID" 2>/dev/null; CAT_PID=""
        while [[ ! -e /dev/gnss_raw ]]; do sleep 1; done
        stty -F /dev/gnss_raw 921600 raw -echo -ixon -ixoff -icrnl -inlcr -opost
        [[ -n "$CURRENT_SESSION" ]] && start_cat
    fi

    SESSION=$(cat /tmp/tankervision_session_path 2>/dev/null || echo "")
    if [[ -n "$SESSION" ]] && [[ "$SESSION" != "$CURRENT_SESSION" ]]; then
        [[ -n "$CAT_PID" ]] && kill "$CAT_PID" 2>/dev/null; CAT_PID=""
        CURRENT_SESSION="$SESSION"
        mkdir -p "$CURRENT_SESSION"
        start_cat
    fi

    if [[ -n "$CAT_PID" ]] && ! kill -0 "$CAT_PID" 2>/dev/null; then
        CAT_PID=""
        [[ -n "$CURRENT_SESSION" ]] && start_cat
    fi

    sleep 2
done
