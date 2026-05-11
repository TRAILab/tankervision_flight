#!/bin/bash
# configure-dfg-camera.sh — Apply V4L2 settings to the DFG2USB Pro capture card.
#
# Reads analog camera config from tankervision.yaml.
# Called once at startup before the v4l2_camera ROS node launches.
#
# Usage: bash configure-dfg-camera.sh [path/to/tankervision.yaml]

set -e

GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'
info()  { echo -e "${GREEN}[dfg-camera]${NC} $*"; }
warn()  { echo -e "${YELLOW}[dfg-camera]${NC} $*"; }
error() { echo -e "${RED}[dfg-camera]${NC} $*"; exit 1; }

# ── Locate config YAML ────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
YAML="${1:-$SCRIPT_DIR/../config/tankervision.yaml}"
[[ -f "$YAML" ]] || error "Config not found: $YAML"
info "Reading config from: $YAML"

# ── Parse YAML with grep/sed (no dependencies required) ──────────
_yaml_val() {
    grep -A1 "analog_camera:" "$YAML" | grep -v "analog_camera:" | \
    grep "$1:" | sed 's/.*: *//' | tr -d ' "' | head -1
    # Fall back to searching whole file if not found under analog_camera block
}
yaml_val() {
    # Search for key under analog_camera section
    python3 -c "
import sys
import re

with open('$YAML') as f:
    content = f.read()

# Find analog_camera section and extract key
in_section = False
for line in content.splitlines():
    if re.match(r'^analog_camera:', line):
        in_section = True
        continue
    if in_section:
        if re.match(r'^[a-z]', line) and not line.startswith(' '):
            break
        m = re.match(r'\s+$1:\s*(.*)', line)
        if m:
            print(m.group(1).strip())
            sys.exit(0)
sys.exit(1)
" 2>/dev/null || echo ""
}

DEVICE=$(python3 -c "
import re
with open('$YAML') as f:
    content = f.read()
in_section = False
for line in content.splitlines():
    if re.match(r'^analog_camera:', line):
        in_section = True
        continue
    if in_section:
        if line and not line.startswith(' '):
            break
        m = re.match(r'\s+device:\s*(.*)', line)
        if m:
            print(m.group(1).strip())
            break
" 2>/dev/null || echo "/dev/video0")

STANDARD=$(python3 -c "
import re
with open('$YAML') as f:
    content = f.read()
in_section = False
for line in content.splitlines():
    if re.match(r'^analog_camera:', line):
        in_section = True
        continue
    if in_section:
        if line and not line.startswith(' '):
            break
        m = re.match(r'\s+standard:\s*(.*)', line)
        if m:
            print(m.group(1).strip().upper())
            break
" 2>/dev/null || echo "NTSC")

# ── Validate ──────────────────────────────────────────────────────
[[ "$STANDARD" == "NTSC" || "$STANDARD" == "PAL" ]] || \
    error "Invalid standard '$STANDARD' in config. Must be NTSC or PAL."

# ── Check device exists ───────────────────────────────────────────
[[ -e "$DEVICE" ]] || error "Device $DEVICE not found. Is the DFG2USB Pro plugged in?"

info "Configuring $DEVICE as $STANDARD..."

# ── Apply settings ────────────────────────────────────────────────
sleep 1   # Give device a moment to settle after detection

v4l2-ctl --device="$DEVICE" --set-input=0 \
    && info "  Input: 0" \
    || warn "  --set-input failed (device may not support it)"

v4l2-ctl --device="$DEVICE" --set-standard="$STANDARD" \
    && info "  Standard: $STANDARD" \
    || warn "  --set-standard failed (device may auto-detect standard)"

if [[ "$STANDARD" == "NTSC" ]]; then
    v4l2-ctl --device="$DEVICE" --set-fmt-video=width=720,height=480,pixelformat=YUYV \
        && info "  Format: YUYV 720x480 (NTSC)" \
        || warn "  --set-fmt-video failed"
else
    v4l2-ctl --device="$DEVICE" --set-fmt-video=width=720,height=576,pixelformat=YUYV \
        && info "  Format: YUYV 720x576 (PAL)" \
        || warn "  --set-fmt-video failed"
fi

info "DFG2USB Pro configuration complete."