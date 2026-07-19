#!/bin/bash
set -e

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ $EUID -eq 0 ]]; then
    echo "[install] ERROR: Run this installer as the normal user:"
    echo "[install]   ./install.sh"
    echo "[install] Do not run: sudo ./install.sh"
    exit 1
fi

echo "[install] Installing Python dependencies..."

python3 -m pip install --user \
    "numpy<2" \
    gps \
    "setuptools<80" \
    "packaging>=22,<26"

echo "[install] Installing Jetson PyTorch for CUDA 12.6..."

python3 -m pip install --user \
    --index-url https://pypi.jetson-ai-lab.io/jp6/cu126 \
    torch \
    torchvision

# Torch requires cuDSS. Do not allow this package to pull its own
# CUDA 12.9/cuBLAS dependencies.
python3 -m pip install --user --no-deps \
    nvidia-cudss-cu12

echo "[install] Locating cuDSS runtime..."

CUDSS_FILE="$(
    find "$HOME/.local/lib/python3.10/site-packages" \
        -name 'libcudss.so.0' \
        -print \
        -quit
)"

if [[ -z "$CUDSS_FILE" ]]; then
    echo "[install] ERROR: libcudss.so.0 was not found."
    exit 1
fi

CUDSS_LIB="$(dirname "$CUDSS_FILE")"

echo "[install] cuDSS library directory: $CUDSS_LIB"

# Use JetPack's CUDA libraries first, then the pip-installed cuDSS library.
# Do not append an inherited LD_LIBRARY_PATH because it may contain
# incompatible CUDA libraries.
export LD_LIBRARY_PATH="/usr/local/cuda/lib64:$CUDSS_LIB:/usr/lib/aarch64-linux-gnu"

echo "[install] Installing Ultralytics dependencies..."

python3 -m pip install --user \
    matplotlib \
    pillow \
    pyyaml \
    requests \
    scipy \
    psutil \
    polars \
    ultralytics-thop \
    nvidia-ml-py

# Prevent Ultralytics from replacing Jetson's Torch/Torchvision.
python3 -m pip install --user --no-deps \
    "ultralytics==8.4.102"

echo "[install] Validating PyTorch CUDA support..."

python3 - <<'PY'
import sys

import torch
import ultralytics

print("[install] Torch:", torch.__version__)
print("[install] Torch CUDA:", torch.version.cuda)
print("[install] CUDA available:", torch.cuda.is_available())
print("[install] Ultralytics:", ultralytics.__version__)

if not torch.cuda.is_available():
    print("[install] ERROR: PyTorch cannot access the Jetson GPU.")
    sys.exit(1)

print("[install] GPU:", torch.cuda.get_device_name(0))

x = torch.ones((16, 16), device="cuda")
print("[install] CUDA allocation test:", x.sum().item())
print("[install] cuBLAS test:", (x @ x)[0, 0].item())

print("[install] PyTorch CUDA validation passed.")
PY

echo "[install] Detecting camera network interface..."
# Find PCI ethernet - starts with 'en' but not 'enx' (USB) 
CAMERA_IFACE=$(ip link show | grep -oP '(?<=\d: )(en[^x]\S+)(?=:)' | head -1)
if [ -z "$CAMERA_IFACE" ]; then
    echo "[install] ERROR: Could not detect PCI ethernet interface. Please specify manually."
    exit 1
fi
echo "[install] Detected camera interface: $CAMERA_IFACE"

echo "[install] Installing system dependencies..."
sudo apt install -y chrony gpsd gpsd-clients linuxptp arp-scan \
    mergerfs \
    libopencv-dev ros-humble-cv-bridge ros-humble-nmea-msgs \
    ros-humble-v4l2-camera ros-humble-topic-tools

echo "[install] Deploying configs..."

echo "[install] Disabling fstab-managed storage pool mounts..."
sudo cp /etc/fstab /etc/fstab.tankervision-storage.bak
sudo sed -i -E \
    -e '/^[[:space:]]*[^#][^[:space:]]+[[:space:]]+\/mnt\/hdd[0-2][[:space:]]/ s/^/# tankervision storage-pool manages this: /' \
    -e '/^[[:space:]]*\/mnt\/hdd0:\/mnt\/hdd1:\/mnt\/hdd2[[:space:]]+\/mnt\/storage[[:space:]]+mergerfs[[:space:]]/ s/^/# tankervision storage-pool manages this: /' \
    /etc/fstab

# chrony
sudo cp "$REPO_DIR/config/chrony/chrony.conf" /etc/chrony/chrony.conf

# GigE camera socket buffer settings (32MB recv buffers)
sudo cp "$REPO_DIR/startup_scripts/99-custom.conf" /etc/sysctl.d/99-custom.conf
sudo sysctl -p /etc/sysctl.d/99-custom.conf

# gpsd
sudo mkdir -p /etc/systemd/system/gpsd.service.d/
sudo cp "$REPO_DIR/config/gpsd/gpsd-service-override.conf" /etc/systemd/system/gpsd.service.d/override.conf
sudo cp "$REPO_DIR/config/gpsd/gpsd-defaults" /etc/default/gpsd

# ptp4l - substitute detected interface name
sudo mkdir -p /etc/linuxptp
sed "s/\[eno1\]/[$CAMERA_IFACE]/" "$REPO_DIR/config/ptp/ptp4l.conf" | sudo tee /etc/linuxptp/ptp4l.conf > /dev/null

# udev rules
sudo cp "$REPO_DIR/config/udev/99-dfg-camera.rules" /etc/udev/rules.d/99-dfg-camera.rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# scripts
sudo cp "$REPO_DIR/scripts/configure-dfg-camera.sh" /usr/local/bin/configure-dfg-camera.sh
sudo chmod +x /usr/local/bin/configure-dfg-camera.sh
# systemd services - substitute detected interface name
sed "s/eno1/$CAMERA_IFACE/g" "$REPO_DIR/startup_scripts/ptp4l.service" | sudo tee /etc/systemd/system/ptp4l.service > /dev/null
CURRENT_USER=$(logname)
sudo cp "$REPO_DIR/startup_scripts/continuous-offload.service" /etc/systemd/system/
sudo sed -i "s/FLIGHT_USER/$CURRENT_USER/g" /etc/systemd/system/continuous-offload.service
sudo cp "$REPO_DIR/startup_scripts/continuous_offload.sh" /usr/local/sbin/continuous_offload.sh
sudo chmod +x /usr/local/sbin/continuous_offload.sh
sudo cp "$REPO_DIR/startup_scripts/tankervision.service" /etc/systemd/system/
sudo sed -i "s/FLIGHT_USER/$CURRENT_USER/g" /etc/systemd/system/tankervision.service
sudo cp "$REPO_DIR/startup_scripts/gnss-record.service" /etc/systemd/system/
sudo sed -i "s/FLIGHT_USER/$CURRENT_USER/g" /etc/systemd/system/gnss-record.service
sudo cp "$REPO_DIR/startup_scripts/gnss_record.sh" /usr/local/bin/gnss_record.sh
sudo chmod +x /usr/local/bin/gnss_record.sh
sudo cp "$REPO_DIR/startup_scripts/gpsd-chrony.service" /etc/systemd/system/
sudo cp "$REPO_DIR/startup_scripts/gpsd-chrony-restart.sh" /usr/local/bin/gpsd-chrony-restart.sh
sudo chmod +x /usr/local/bin/gpsd-chrony-restart.sh

echo "[install] Configuring Lucid Triton2 GigE camera network interface ($CAMERA_IFACE)..."
if nmcli connection show "$CAMERA_IFACE" &>/dev/null; then
    echo "[install] $CAMERA_IFACE profile exists, updating..."
    sudo nmcli connection modify "$CAMERA_IFACE" ipv4.method manual ipv4.addresses 169.254.0.1/16
    sudo nmcli connection modify "$CAMERA_IFACE" ipv4.gateway ""
    sudo nmcli connection modify "$CAMERA_IFACE" ipv6.method ignore
    sudo nmcli connection modify "$CAMERA_IFACE" connection.autoconnect yes
else
    echo "[install] Creating $CAMERA_IFACE profile..."
    sudo nmcli connection add type ethernet ifname "$CAMERA_IFACE" con-name "$CAMERA_IFACE" \
        ipv4.method manual ipv4.addresses 169.254.0.1/16 \
        ipv4.gateway "" \
        ipv6.method ignore \
        connection.autoconnect yes
fi
sudo nmcli connection up "$CAMERA_IFACE" || true

echo "[install] Enabling services..."
sudo systemctl daemon-reload
sudo systemctl disable --now gpsd.service chrony 2>/dev/null || true
sudo systemctl disable gpsd.socket || true
sudo systemctl mask gpsd.socket || true
for SVC in \
    continuous-offload.service \
    gpsd-chrony.service \
    ptp4l.service \
    tankervision.service \
    gnss-record.service
do
    sudo systemctl enable "$SVC"
done

echo "[install] Building ROS2 workspace..."
if [[ -x /usr/local/cuda/bin/nvcc ]]; then
    export CUDACXX=/usr/local/cuda/bin/nvcc
    export PATH=/usr/local/cuda/bin:$PATH
else
    echo "[install] ERROR: CUDA compiler not found at /usr/local/cuda/bin/nvcc."
    echo "[install] Install the Jetson CUDA toolkit/compiler before building arena_camera_node."
    exit 1
fi
source /opt/ros/humble/setup.bash
cd "$REPO_DIR"
colcon build --symlink-install --packages-skip xsens_mti_ros2_driver

echo "[install] Configuring auto-login and disabling screen lock..."

# Console auto-login via getty override (works headless and with desktop)
sudo mkdir -p /etc/systemd/system/getty@tty1.service.d/
sudo tee /etc/systemd/system/getty@tty1.service.d/autologin.conf > /dev/null << UNIT
[Service]
ExecStart=
ExecStart=-/sbin/agetty --autologin $CURRENT_USER --noclear %I \$TERM
UNIT

# GDM auto-login (display manager, if present)
if [ -f /etc/gdm3/custom.conf ]; then
    sudo sed -i "s/^#\?\s*AutomaticLoginEnable\s*=.*/AutomaticLoginEnable=true/" /etc/gdm3/custom.conf
    sudo sed -i "s/^#\?\s*AutomaticLogin\s*=.*/AutomaticLogin=$CURRENT_USER/" /etc/gdm3/custom.conf
    grep -q "^AutomaticLoginEnable" /etc/gdm3/custom.conf || \
        sudo sed -i "/^\[daemon\]/a AutomaticLoginEnable=true\nAutomaticLogin=$CURRENT_USER" /etc/gdm3/custom.conf
fi

# Disable screen lock and idle blank
# (best-effort — if install runs before first login, re-run manually:
#  gsettings set org.gnome.desktop.screensaver lock-enabled false)
_dbus="unix:path=/run/user/$(id -u "$CURRENT_USER")/bus"
sudo -u "$CURRENT_USER" DBUS_SESSION_BUS_ADDRESS="$_dbus" \
    gsettings set org.gnome.desktop.screensaver lock-enabled false 2>/dev/null || \
    echo "[install] Note: gsettings screen-lock — run manually after first login if this failed"
sudo -u "$CURRENT_USER" DBUS_SESSION_BUS_ADDRESS="$_dbus" \
    gsettings set org.gnome.desktop.session idle-delay 0 2>/dev/null || true
sudo -u "$CURRENT_USER" DBUS_SESSION_BUS_ADDRESS="$_dbus" \
    gsettings set org.gnome.desktop.media-handling automount false 2>/dev/null || \
    echo "[install] Note: gsettings automount — run manually after first login if this failed"
sudo -u "$CURRENT_USER" DBUS_SESSION_BUS_ADDRESS="$_dbus" \
    gsettings set org.gnome.desktop.media-handling automount-open false 2>/dev/null || true

echo "[install] Done. Reboot to verify all services start correctly."
