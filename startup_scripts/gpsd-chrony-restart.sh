#!/bin/bash
set -e  # Exit immediately if a command exits with a non-zero status

# Kill any running gpsd and chronyd processes
killall -9 gpsd chronyd || true

# Restart chronyd with the specified configuration file
if ! chronyd -f /etc/chrony/chrony.conf; then
    echo "chronyd failed to start."
    exit 1
fi
sleep 2

# Start gpsd with the given device and PPS source
if ! gpsd -n /dev/ttyV1 /dev/pps1; then
    echo "gpsd failed to start."
    exit 1
fi
sleep 10

# Synchronize the system clock with the PTP clock.
sudo phc2sys -s CLOCK_REALTIME -c /dev/ptp0 -w -O 0 -m &
phc2sys_pid=$!
sleep 1  # Give it a moment to initialize
if ! kill -0 "$phc2sys_pid" 2>/dev/null; then
    echo "phc2sys synchronization failed."
    exit 1
fi

echo "All services started successfully."
