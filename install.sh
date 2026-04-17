#!/bin/bash
set -e
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "[install] Installing system dependencies..."
sudo apt install -y chrony gpsd gpsd-clients linuxptp arp-scan \
    libopencv-dev ros-humble-cv-bridge ros-humble-nmea-msgs \
    ros-humble-v4l2-camera

echo "[install] Deploying configs..."

# chrony
sudo cp "$REPO_DIR/config/chrony/chrony.conf" /etc/chrony/chrony.conf

# gpsd
sudo mkdir -p /etc/systemd/system/gpsd.service.d/
sudo cp "$REPO_DIR/config/gpsd/gpsd-service-override.conf" /etc/systemd/system/gpsd.service.d/override.conf
sudo cp "$REPO_DIR/config/gpsd/gpsd-defaults" /etc/default/gpsd

# ptp4l
sudo mkdir -p /etc/linuxptp
sudo cp "$REPO_DIR/config/ptp/ptp4l.conf" /etc/linuxptp/ptp4l.conf

# udev rules
sudo cp "$REPO_DIR/config/udev/99-dfg-camera.rules" /etc/udev/rules.d/99-dfg-camera.rules
sudo cp "$REPO_DIR/config/udev/99-gps.rules" /etc/udev/rules.d/99-gps.rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# scripts
sudo cp "$REPO_DIR/scripts/configure-dfg-camera.sh" /usr/local/bin/configure-dfg-camera.sh
sudo chmod +x /usr/local/bin/configure-dfg-camera.sh

# systemd services
sudo cp "$REPO_DIR/startup_scripts/ptp4l.service" /etc/systemd/system/
sudo cp "$REPO_DIR/startup_scripts/tankervision.service" /etc/systemd/system/

echo "[install] Configuring Lucid Triton2 GigE camera network interface (eno1)..."
if nmcli connection show eno1 &>/dev/null; then
    echo "[install] eno1 profile exists, updating..."
    sudo nmcli connection modify eno1 ipv4.method manual ipv4.addresses 169.254.0.1/16
    sudo nmcli connection modify eno1 ipv4.gateway ""
    sudo nmcli connection modify eno1 ipv6.method ignore
    sudo nmcli connection modify eno1 connection.autoconnect yes
else
    echo "[install] Creating eno1 profile..."
    sudo nmcli connection add type ethernet ifname eno1 con-name eno1 \
        ipv4.method manual ipv4.addresses 169.254.0.1/16 \
        ipv4.gateway "" \
        ipv6.method ignore \
        connection.autoconnect yes
fi
sudo nmcli connection up eno1 || true

echo "[install] Enabling services..."
sudo systemctl daemon-reload
sudo systemctl enable gpsd.service
sudo systemctl disable gpsd.socket || true
sudo systemctl mask gpsd.socket || true
sudo systemctl enable chrony
sudo systemctl enable ptp4l.service
sudo systemctl enable tankervision.service

echo "[install] Building ROS2 workspace..."
source /opt/ros/humble/setup.bash
cd "$REPO_DIR"
colcon build --symlink-install --packages-skip xsens_mti_ros2_driver

echo "[install] Done. Reboot to verify all services start correctly."
