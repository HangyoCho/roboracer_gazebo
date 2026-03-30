#!/bin/bash
# Unicorn Model - Setup Script
# Creates symlinks and builds thirdparty dependencies

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_SRC="$(dirname "$SCRIPT_DIR")"
THIRDPARTY="$SCRIPT_DIR/thirdparty"

echo "=== Unicorn Model Setup ==="

# 1. Install system dependencies
echo "[1/5] Checking system dependencies..."
DEPS=(libapr1-dev libpcl-dev ros-noetic-pcl-ros ros-noetic-gazebo-ros-control ros-noetic-effort-controllers ros-noetic-velocity-controllers ros-noetic-joint-state-controller)
MISSING=()
for dep in "${DEPS[@]}"; do
    if ! dpkg -s "$dep" &>/dev/null; then
        MISSING+=("$dep")
    fi
done

if [ ${#MISSING[@]} -gt 0 ]; then
    echo "  Installing: ${MISSING[*]}"
    sudo apt-get update -qq
    sudo apt-get install -y -qq "${MISSING[@]}"
else
    echo "  All system dependencies installed."
fi

# 2. Build Livox-SDK
echo "[2/5] Building Livox-SDK..."
if [ ! -f "$THIRDPARTY/Livox-SDK/build/sdk_core/liblivox_sdk_static.a" ]; then
    mkdir -p "$THIRDPARTY/Livox-SDK/build"
    cd "$THIRDPARTY/Livox-SDK/build"
    cmake ..
    make -j$(nproc)
    echo "  Livox-SDK built successfully."
else
    echo "  Livox-SDK already built, skipping."
fi

# 3. Create symlinks for catkin
echo "[3/5] Creating symlinks..."
for pkg in livox_laser_simulation livox_ros_driver; do
    target="$THIRDPARTY/$pkg"
    link="$WS_SRC/$pkg"
    if [ -L "$link" ]; then
        echo "  $pkg symlink already exists."
    elif [ -e "$link" ]; then
        echo "  WARNING: $link exists but is not a symlink. Skipping."
    else
        ln -s "$target" "$link"
        echo "  Created symlink: $pkg"
    fi
done

# 4. Build workspace
# 4. Install Python dependencies
echo "[4/5] Installing Python dependencies..."
pip3 install -r "$SCRIPT_DIR/requirements.txt"

# 5. Build workspace
echo "[5/5] Building workspace..."
cd "$WS_SRC/.."
catkin_make
source devel/setup.bash

echo ""
echo "=== Setup complete! ==="
echo "Run: source $(cd "$WS_SRC/.." && pwd)/devel/setup.bash"
