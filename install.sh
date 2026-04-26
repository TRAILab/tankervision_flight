#!/bin/bash
set -e
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

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
    libopencv-dev ros-humble-cv-bridge ros-humble-nmea-msgs \
    ros-humble-v4l2-camera

echo "[install] Deploying configs..."

# chrony
sudo cp "$REPO_DIR/config/chrony/chrony.conf" /etc/chrony/chrony.conf

# gpsd
sudo mkdir -p /etc/systemd/system/gpsd.service.d/
sudo cp "$REPO_DIR/config/gpsd/gpsd-service-override.conf" /etc/systemd/system/gpsd.service.d/override.conf
sudo cp "$REPO_DIR/config/gpsd/gpsd-defaults" /etc/default/gpsd

# ptp4l - substitute detected interface name
sudo mkdir -p /etc/linuxptp
sed "s/\[eno1\]/[$CAMERA_IFACE]/" "$REPO_DIR/config/ptp/ptp4l.conf" | sudo tee /etc/linuxptp/ptp4l.conf > /dev/null

# udev rules
sudo cp "$REPO_DIR/config/udev/99-dfg-camera.rules" /etc/udev/rules.d/99-dfg-camera.rules
sudo cp "$REPO_DIR/startup_scripts/identify_and_install_udev.sh" /usr/local/bin/identify_and_install_udev.sh
sudo chmod +x /usr/local/bin/identify_and_install_udev.sh
sudo cp "$REPO_DIR/startup_scripts/identify_and_install_udev.sh" /usr/local/bin/identify_and_install_udev.sh
sudo chmod +x /usr/local/bin/identify_and_install_udev.sh
sudo cp "$REPO_DIR/config/udev/99-gps.rules" /etc/udev/rules.d/99-gps.rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# scripts
sudo cp "$REPO_DIR/scripts/configure-dfg-camera.sh" /usr/local/bin/configure-dfg-camera.sh
sudo chmod +x /usr/local/bin/configure-dfg-camera.sh

# systemd services - substitute detected interface name
sed "s/eno1/$CAMERA_IFACE/g" "$REPO_DIR/startup_scripts/ptp4l.service" | sudo tee /etc/systemd/system/ptp4l.service > /dev/null
sudo cp "$REPO_DIR/startup_scripts/tankervision.service" /etc/systemd/system/
CURRENT_USER=$(logname)
sudo sed -i "s/FLIGHT_USER/$CURRENT_USER/g" /etc/systemd/system/tankervision.service
sudo cp "$REPO_DIR/startup_scripts/gnss-record.service" /etc/systemd/system/
sudo cp "$REPO_DIR/startup_scripts/gnss_record.sh" /usr/local/bin/gnss_record.sh
sudo chmod +x /usr/local/bin/gnss_record.sh

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
