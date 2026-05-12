#!/bin/bash
set -e  # Exit immediately if a command exits with a non-zero status

# Kill any running gpsd and chronyd processes
killall -9 gpsd chronyd || true

# Restart chronyd with the specified configuration file
if ! chronyd -f /etc/chrony/chrony.conf; then
    echo "CHRONY failed to start."
    exit 1
fi
sleep 2

# Start gpsd with the given device and PPS source
if ! gpsd -n /dev/ttyACM0 /dev/pps1; then
    echo "GPSD failed to start."
    exit 1
fi

echo "GPSD and Chrony restarted successfully."
