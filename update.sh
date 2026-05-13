#!/bin/bash
# update.sh — Deploy config and code changes to this flight unit.
#
# Run after every `git pull` to propagate repo changes to the system.
# Safe to run repeatedly. Does NOT install packages or modify nmcli profiles.
# For first-time machine setup, run install.sh instead.
#
# Usage: sudo ./update.sh

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Disable exit-on-error for the service restart section only
# (individual restarts handle their own errors)

# ── Colour helpers ────────────────────────────────────────────────
GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'
info()  { echo -e "${GREEN}[update]${NC} $*"; }
warn()  { echo -e "${YELLOW}[update]${NC} $*"; }
error() { echo -e "${RED}[update]${NC} $*"; exit 1; }

# ── Root check ────────────────────────────────────────────────────
[[ $EUID -ne 0 ]] && error "Run as root: sudo ./update.sh"

# ── Detect camera interface (same logic as install.sh) ────────────
CAMERA_IFACE=$(ip link show | grep -oP '(?<=\d: )(en[^x]\S+)(?=:)' | head -1)
[[ -z "$CAMERA_IFACE" ]] && error "Could not detect PCI ethernet interface."
info "Camera interface: $CAMERA_IFACE"

# ── 1. Deploy configs ─────────────────────────────────────────────
info "Deploying configs..."

cp_config() {
    local SRC="$1" DST="$2"
    if [[ -f "$SRC" ]]; then
        cp "$SRC" "$DST"
    else
        warn "  Skipping $(basename $SRC) — not found in repo (add it to deploy)"
    fi
}

# chrony
cp_config "$REPO_DIR/config/chrony/chrony.conf" /etc/chrony/chrony.conf
info "  chrony.conf"

# gpsd
mkdir -p /etc/systemd/system/gpsd.service.d/
cp_config "$REPO_DIR/config/gpsd/gpsd-service-override.conf" /etc/systemd/system/gpsd.service.d/override.conf
cp_config "$REPO_DIR/config/gpsd/gpsd-defaults" /etc/default/gpsd
info "  gpsd config"

# ptp4l — re-substitute interface name each time
mkdir -p /etc/linuxptp
if [[ -f "$REPO_DIR/config/ptp/ptp4l.conf" ]]; then
    sed "s/\[eno1\]/[$CAMERA_IFACE]/" "$REPO_DIR/config/ptp/ptp4l.conf" \
        | tee /etc/linuxptp/ptp4l.conf > /dev/null
    info "  ptp4l.conf (iface: $CAMERA_IFACE)"
else
    warn "  Skipping ptp4l.conf — not found in repo"
fi

# udev rules
cp_config "$REPO_DIR/config/udev/99-dfg-camera.rules" /etc/udev/rules.d/99-dfg-camera.rules
udevadm control --reload-rules
udevadm trigger
info "  udev rules"

# scripts
if [[ -f "$REPO_DIR/scripts/configure-dfg-camera.sh" ]]; then
    cp "$REPO_DIR/scripts/configure-dfg-camera.sh" /usr/local/bin/configure-dfg-camera.sh
    chmod +x /usr/local/bin/configure-dfg-camera.sh
fi
if [[ -f "$REPO_DIR/startup_scripts/gnss_record.sh" ]]; then
    cp "$REPO_DIR/startup_scripts/gnss_record.sh" /usr/local/bin/gnss_record.sh
    chmod +x /usr/local/bin/gnss_record.sh
fi
if [[ -f "$REPO_DIR/startup_scripts/gpsd-chrony-restart.sh" ]]; then
    cp "$REPO_DIR/startup_scripts/gpsd-chrony-restart.sh" /usr/local/bin/gpsd-chrony-restart.sh
    chmod +x /usr/local/bin/gpsd-chrony-restart.sh
fi
info "  helper scripts"

# ── 2. Deploy systemd services ────────────────────────────────────
info "Deploying systemd services..."

# ptp4l — substitute interface name
sed "s/eno1/$CAMERA_IFACE/g" "$REPO_DIR/startup_scripts/ptp4l.service" \
    | tee /etc/systemd/system/ptp4l.service > /dev/null
info "  ptp4l.service"

# tankervision — always write fresh from template then substitute user
CURRENT_USER=$(logname 2>/dev/null || echo "${SUDO_USER:-trail}")
cp "$REPO_DIR/startup_scripts/tankervision.service" /etc/systemd/system/tankervision.service
sed -i "s/FLIGHT_USER/$CURRENT_USER/g" /etc/systemd/system/tankervision.service
info "  tankervision.service (user: $CURRENT_USER)"

# remaining active services — copy as-is
for SVC in \
    gpsd-chrony.service \
    gnss-record.service
do
    if [[ -f "$REPO_DIR/startup_scripts/$SVC" ]]; then
        cp "$REPO_DIR/startup_scripts/$SVC" /etc/systemd/system/
        info "  $SVC"
    fi
done

# ── 3. Reload systemd ─────────────────────────────────────────────
info "Reloading systemd..."
systemctl daemon-reload
systemctl disable --now gpsd.service chrony 2>/dev/null || true
systemctl disable gpsd.socket || true
systemctl mask gpsd.socket || true

# Services that must always be enabled
for SVC in \
    gpsd-chrony.service \
    ptp4l.service \
    tankervision.service \
    gnss-record.service
do
    if ! systemctl is-enabled --quiet "$SVC" 2>/dev/null; then
        systemctl enable "$SVC"
        info "  enabled $SVC"
    fi
done

# ── 4. Restart running services ───────────────────────────────────
info "Restarting affected services..."

restart_svc() {
    local SVC="$1"
    if systemctl is-enabled --quiet "$SVC" 2>/dev/null; then
        if systemctl stop "$SVC" 2>/dev/null; then true; fi
        if systemctl start "$SVC" 2>/dev/null; then
            info "  started $SVC"
        else
            warn "  $SVC failed to start — check: journalctl -u $SVC"
        fi
    else
        warn "  $SVC not enabled — skipping (enable with install.sh if needed)"
    fi
}

restart_svc gpsd-chrony.service
restart_svc ptp4l.service

# ── 5. Rebuild ROS2 workspace ─────────────────────────────────────
info "Building ROS2 workspace..."
if [[ -x /usr/local/cuda/bin/nvcc ]]; then
    export CUDACXX=/usr/local/cuda/bin/nvcc
    export PATH=/usr/local/cuda/bin:$PATH
else
    error "CUDA compiler not found at /usr/local/cuda/bin/nvcc. Install the Jetson CUDA toolkit/compiler before building arena_camera_node."
fi
set +u
source /opt/ros/humble/setup.bash
set -u
cd "$REPO_DIR"
rm -rf \
    "$REPO_DIR/build/arena_camera_node" \
    "$REPO_DIR/install/arena_camera_node" \
    "$REPO_DIR/build/tanker_vision" \
    "$REPO_DIR/install/tanker_vision" \
    "$REPO_DIR/src/tanker_vision/tanker_vision.egg-info"
colcon build --symlink-install --packages-skip xsens_mti_ros2_driver \
    --parallel-workers 2 \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

# ── 5b. Restart tankervision after build ──────────────────────────
restart_svc tankervision.service
restart_svc gnss-record.service

# ── 6. Summary ────────────────────────────────────────────────────
echo ""
info "Done. Active service status:"
for SVC in \
    gpsd-chrony.service \
    ptp4l.service \
    tankervision.service \
    gnss-record.service
do
    STATUS=$(systemctl is-active "$SVC" 2>/dev/null || echo "inactive")
    if [[ "$STATUS" == "active" ]]; then
        echo -e "  ${GREEN}●${NC} $SVC"
    else
        echo -e "  ${YELLOW}○${NC} $SVC ($STATUS)"
    fi
done
echo ""
info "ROS nodes still starting — check with: ros2 node list"
echo ""
