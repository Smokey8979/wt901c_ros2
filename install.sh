#!/usr/bin/env bash

# ============================================================
# WT901C485 ROS 2 Driver - Universal Dependency Installer
# Supported OS : Ubuntu 22.04, Ubuntu 24.04
# Supported ROS: Humble, Jazzy
# ============================================================

set -e

echo "=============================================="
echo " WT901C485 ROS 2 Driver - Dependency Installer "
echo "=============================================="

# ------------------------------
# 1. Detect Ubuntu version
# ------------------------------
source /etc/os-release

if [[ "$VERSION_ID" != "22.04" && "$VERSION_ID" != "24.04" ]]; then
  echo "[ERROR] Unsupported Ubuntu version: $VERSION_ID"
  echo "Supported versions: Ubuntu 22.04, Ubuntu 24.04"
  exit 1
fi

echo "[OK] Ubuntu $VERSION_ID detected"

# ------------------------------
# 2. Detect ROS 2 distribution
# ------------------------------
ROS_DISTRO=""

if [ -d "/opt/ros/humble" ]; then
  ROS_DISTRO="humble"
elif [ -d "/opt/ros/jazzy" ]; then
  ROS_DISTRO="jazzy"
else
  echo "[ERROR] No supported ROS 2 distribution found."
  echo "Install ROS 2 Humble or Jazzy before running this script."
  exit 1
fi

echo "[OK] ROS 2 $ROS_DISTRO detected"

# ------------------------------
# 3. Update system
# ------------------------------
echo "[INFO] Updating system packages..."
sudo apt update

# ------------------------------
# 4. Install base system dependencies
# ------------------------------
echo "[INFO] Installing system dependencies..."

sudo apt install -y \
  python3-pip \
  python3-serial \
  python3-colcon-common-extensions \
  python3-rosdep \
  udev \
  build-essential \
  git

# ------------------------------
# 5. Install kernel modules (USB-RS485 / CH340)
# ------------------------------
echo "[INFO] Installing kernel extra modules (CH340 / RS485 support)..."

sudo apt install -y linux-modules-extra-$(uname -r)

# ------------------------------
# 6. Remove conflicting services (BRLTTY)
# ------------------------------
if dpkg -l | grep -q brltty; then
  echo "[INFO] Removing brltty (conflicts with USB serial devices)..."
  sudo apt purge -y brltty
fi

# ------------------------------
# 7. Python dependencies
# ------------------------------
echo "[INFO] Installing Python dependencies..."

pip3 install --upgrade pip
pip3 install pyserial flask

# ------------------------------
# 8. ROS dependency initialization
# ------------------------------
echo "[INFO] Initializing rosdep..."

if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
  sudo rosdep init
fi
rosdep update

# ------------------------------
# 9. Install IMU tools for RViz2
# ------------------------------
echo "[INFO] Installing IMU tools for RViz2..."

sudo apt install -y ros-${ROS_DISTRO}-imu-tools

# ------------------------------
# 10. udev rule for WT901C (persistent port)
# ------------------------------
UDEV_RULE_FILE="/etc/udev/rules.d/99-wt901c.rules"

if [ ! -f "$UDEV_RULE_FILE" ]; then
  echo "[INFO] Installing udev rule for WT901C IMU..."

  sudo tee "$UDEV_RULE_FILE" > /dev/null <<EOF
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="wt901c"
EOF

  sudo udevadm control --reload-rules
  sudo udevadm trigger
else
  echo "[INFO] udev rule already exists"
fi

# ------------------------------
# 11. Permissions
# ------------------------------
echo "[INFO] Adding user to dialout group..."

sudo usermod -a -G dialout "$USER"

# ------------------------------
# 12. Final message
# ------------------------------
echo ""
echo "=============================================="
echo " Installation complete"
echo "=============================================="
