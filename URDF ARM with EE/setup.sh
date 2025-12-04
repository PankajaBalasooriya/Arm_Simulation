#!/bin/bash

# GP7 Robot ROS2 Setup Script
# This script helps set up the GP7 Robot Description package for ROS2

set -e

echo "==================================="
echo "GP7 Robot ROS2 Setup"
echo "==================================="

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "ERROR: ROS2 is not sourced. Please source your ROS2 installation first:"
    echo "  source /opt/ros/<distro>/setup.bash"
    exit 1
fi

echo "ROS2 Distribution: $ROS_DISTRO"

# Check for required packages
echo ""
echo "Checking for required ROS2 packages..."

REQUIRED_PACKAGES=(
    "robot-state-publisher"
    "joint-state-publisher"
    "joint-state-publisher-gui"
    "rviz2"
    "xacro"
    "gazebo-ros-pkgs"
)

MISSING_PACKAGES=()

for pkg in "${REQUIRED_PACKAGES[@]}"; do
    if ! dpkg -l | grep -q "ros-${ROS_DISTRO}-${pkg}"; then
        MISSING_PACKAGES+=("ros-${ROS_DISTRO}-${pkg}")
    fi
done

if [ ${#MISSING_PACKAGES[@]} -gt 0 ]; then
    echo "WARNING: The following packages are missing:"
    for pkg in "${MISSING_PACKAGES[@]}"; do
        echo "  - $pkg"
    done
    echo ""
    echo "Install them with:"
    echo "  sudo apt install ${MISSING_PACKAGES[@]}"
    read -p "Do you want to continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
else
    echo "All required packages are installed!"
fi

# Find or create workspace
echo ""
if [ -z "$COLCON_PREFIX_PATH" ]; then
    echo "No active colcon workspace found."
    read -p "Enter your ROS2 workspace path (default: ~/ros2_ws): " WS_PATH
    WS_PATH=${WS_PATH:-~/ros2_ws}
else
    WS_PATH=$(echo $COLCON_PREFIX_PATH | cut -d':' -f1 | sed 's/\/install$//')
    echo "Using active workspace: $WS_PATH"
fi

# Create workspace if it doesn't exist
mkdir -p "$WS_PATH/src"

# Copy package
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PACKAGE_NAME="gp7_robot_description"
DEST_DIR="$WS_PATH/src/$PACKAGE_NAME"

echo ""
echo "Copying package to workspace..."
if [ -d "$DEST_DIR" ]; then
    read -p "Package already exists at $DEST_DIR. Overwrite? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Setup cancelled."
        exit 0
    fi
    rm -rf "$DEST_DIR"
fi

cp -r "$SCRIPT_DIR" "$DEST_DIR"
echo "Package copied to: $DEST_DIR"

# Check for meshes
echo ""
if [ ! -d "$DEST_DIR/meshes" ] || [ -z "$(ls -A $DEST_DIR/meshes)" ]; then
    echo "WARNING: meshes/ directory is empty or missing!"
    echo "You need to copy the STL mesh files from your original package:"
    echo "  cp -r /path/to/original/meshes $DEST_DIR/"
    read -p "Press Enter to continue..."
fi

# Build package
echo ""
read -p "Build the package now? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "Building package..."
    cd "$WS_PATH"
    colcon build --packages-select $PACKAGE_NAME
    
    if [ $? -eq 0 ]; then
        echo ""
        echo "==================================="
        echo "Setup Complete!"
        echo "==================================="
        echo ""
        echo "To use the package, source your workspace:"
        echo "  source $WS_PATH/install/setup.bash"
        echo ""
        echo "Then launch with:"
        echo "  ros2 launch $PACKAGE_NAME display.launch.py"
        echo "  ros2 launch $PACKAGE_NAME gazebo.launch.py"
    else
        echo "Build failed. Please check the errors above."
        exit 1
    fi
else
    echo ""
    echo "To build later, run:"
    echo "  cd $WS_PATH"
    echo "  colcon build --packages-select $PACKAGE_NAME"
fi
