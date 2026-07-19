#!/bin/bash
# update.sh — Deploy config and code changes to this flight unit.
#
# Run after every `git pull` to propagate repo changes to the system.
# Safe to run repeatedly. Does NOT install packages.
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

info "Disabling fstab-managed storage pool mounts..."
cp /etc/fstab /etc/fstab.tankervision-storage.bak
sed -i -E \
    -e '/^[[:space:]]*[^#][^[:space:]]+[[:space:]]+\/mnt\/hdd[0-2][[:space:]]/ s/^/# tankervision storage-pool manages this: /' \
    -e '/^[[:space:]]*\/mnt\/hdd0:\/mnt\/hdd1:\/mnt\/hdd2[[:space:]]+\/mnt\/storage[[:space:]]+mergerfs[[:space:]]/ s/^/# tankervision storage-pool manages this: /' \
    /etc/fstab
command -v mergerfs >/dev/null || warn "  mergerfs is not installed — run install.sh or install mergerfs"

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

# GigE camera socket buffer settings (32MB recv buffers)
cp_config "$REPO_DIR/startup_scripts/99-custom.conf" /etc/sysctl.d/99-custom.conf
sysctl -p /etc/sysctl.d/99-custom.conf > /dev/null
info "  99-custom.conf (socket buffers)"

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
if [[ -f "$REPO_DIR/startup_scripts/continuous_offload.sh" ]]; then
    cp "$REPO_DIR/startup_scripts/continuous_offload.sh" /usr/local/sbin/continuous_offload.sh
    chmod +x /usr/local/sbin/continuous_offload.sh
fi
info "  helper scripts"

# ── 2. Deploy systemd services ────────────────────────────────────
info "Deploying systemd services..."

# ptp4l — substitute interface name
sed "s/eno1/$CAMERA_IFACE/g" "$REPO_DIR/startup_scripts/ptp4l.service" \
    | tee /etc/systemd/system/ptp4l.service > /dev/null
info "  ptp4l.service"

CURRENT_USER=$(logname 2>/dev/null || echo "${SUDO_USER:-trail}")
cp "$REPO_DIR/startup_scripts/continuous-offload.service" /etc/systemd/system/continuous-offload.service
sed -i "s/FLIGHT_USER/$CURRENT_USER/g" /etc/systemd/system/continuous-offload.service
info "  continuous-offload.service (user: $CURRENT_USER)"
cp "$REPO_DIR/startup_scripts/tankervision.service" /etc/systemd/system/tankervision.service
sed -i "s/FLIGHT_USER/$CURRENT_USER/g" /etc/systemd/system/tankervision.service
info "  tankervision.service (user: $CURRENT_USER)"
cp "$REPO_DIR/startup_scripts/gnss-record.service" /etc/systemd/system/gnss-record.service
sed -i "s/FLIGHT_USER/$CURRENT_USER/g" /etc/systemd/system/gnss-record.service
info "  gnss-record.service (user: $CURRENT_USER)"

sudo -u "$CURRENT_USER" DBUS_SESSION_BUS_ADDRESS="unix:path=/run/user/$(id -u "$CURRENT_USER")/bus" \
    gsettings set org.gnome.desktop.media-handling automount false 2>/dev/null || \
    warn "  desktop automount disable skipped"
sudo -u "$CURRENT_USER" DBUS_SESSION_BUS_ADDRESS="unix:path=/run/user/$(id -u "$CURRENT_USER")/bus" \
    gsettings set org.gnome.desktop.media-handling automount-open false 2>/dev/null || true

# remaining active services — copy as-is
for SVC in \
    gpsd-chrony.service
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

# Remove the old storage-pool service if it exists
systemctl disable --now storage-pool.service 2>/dev/null || true
systemctl mask storage-pool.service 2>/dev/null || true
rm -f /etc/systemd/system/storage-pool.service

# Services that must always be enabled
for SVC in \
    continuous-offload.service \
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
systemctl stop tankervision.service gnss-record.service 2>/dev/null || true
restart_svc continuous-offload.service

# ── 5. Rebuild ROS2 workspace as the flight user ─────────────────
info "Building ROS2 workspace as $CURRENT_USER..."

if [[ ! -x /usr/local/cuda/bin/nvcc ]]; then
    error "CUDA compiler not found at /usr/local/cuda/bin/nvcc. Install the Jetson CUDA toolkit/compiler before building arena_camera_node."
fi

USER_HOME="$(getent passwd "$CURRENT_USER" | cut -d: -f6)"
USER_UID="$(id -u "$CURRENT_USER")"
USER_GID="$(id -g "$CURRENT_USER")"

[[ -n "$USER_HOME" ]] || error "Could not determine home directory for $CURRENT_USER"

CUDSS_LIB="$USER_HOME/.local/lib/python3.10/site-packages/nvidia/cu12/lib"

if [[ ! -f "$CUDSS_LIB/libcudss.so.0" ]]; then
    error "cuDSS library not found at $CUDSS_LIB/libcudss.so.0"
fi

# Previous sudo builds may have left root-owned workspace artifacts.
chown -R "$USER_UID:$USER_GID" "$REPO_DIR"

# Clean selected build artifacts as the normal user.
sudo -u "$CURRENT_USER" \
    HOME="$USER_HOME" \
    rm -rf \
        "$REPO_DIR/build/arena_camera_node" \
        "$REPO_DIR/install/arena_camera_node" \
        "$REPO_DIR/build/im19_ros2" \
        "$REPO_DIR/install/im19_ros2" \
        "$REPO_DIR/build/tanker_vision" \
        "$REPO_DIR/install/tanker_vision" \
        "$REPO_DIR/src/im19_ros2/im19_ros2.egg-info" \
        "$REPO_DIR/src/tanker_vision/tanker_vision.egg-info"

# Ensure shared colcon directories remain writable by the normal user.
mkdir -p "$REPO_DIR/build" "$REPO_DIR/install" "$REPO_DIR/log"
chown -R "$USER_UID:$USER_GID" \
    "$REPO_DIR/build" \
    "$REPO_DIR/install" \
    "$REPO_DIR/log"

sudo -u "$CURRENT_USER" \
    HOME="$USER_HOME" \
    USER="$CURRENT_USER" \
    PATH="$USER_HOME/.local/bin:/usr/local/cuda/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" \
    CUDACXX="/usr/local/cuda/bin/nvcc" \
    LD_LIBRARY_PATH="/usr/local/cuda/lib64:$CUDSS_LIB:/usr/lib/aarch64-linux-gnu" \
    bash -c "
        set -e
        set +u
        source /opt/ros/humble/setup.bash
        set -u
        cd '$REPO_DIR'

        python3 - <<'PY'
import packaging
import setuptools

print('[update] Python packaging:', packaging.__version__, packaging.__file__)
print('[update] setuptools:', setuptools.__version__, setuptools.__file__)
PY

        colcon build \
            --symlink-install \
            --packages-skip xsens_mti_ros2_driver \
            --parallel-workers 2 \
            --cmake-args -DCMAKE_BUILD_TYPE=Release
    "

# ── 5b. Restart tankervision after build ──────────────────────────
restart_svc tankervision.service
restart_svc gnss-record.service

# ── 6. Summary ────────────────────────────────────────────────────
echo ""
info "Done. Active service status:"
for SVC in \
    continuous-offload.service \
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
