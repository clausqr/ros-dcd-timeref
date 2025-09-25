#!/usr/bin/env bash
set -euo pipefail

echo "Setting up ROS Noetic development environment..."

# Install dependencies
apt-get update && apt-get install -y \
    python3-catkin-tools \
    python3-catkin-pkg \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    ros-noetic-catkin

# Initialize rosdep
rosdep init || true
rosdep update

# Create separate build workspace
mkdir -p /workspace/catkin_ws/src

# Link the package (your repo) to the build workspace
# The workspace folder is mounted at /workspaces/ros-noetic-dcd-timeref
WORKSPACE_SOURCE="/workspaces/ros-noetic-dcd-timeref"
if [ ! -e "/workspace/catkin_ws/src/dcd_timeref" ]; then
    ln -s "$WORKSPACE_SOURCE" "/workspace/catkin_ws/src/dcd_timeref"
    echo "Linked package to build workspace"
fi

# Install package dependencies
rosdep install --from-paths /workspace/catkin_ws/src --ignore-src -r -y

# Build the package (using simple CMakeLists)
cd /workspace/catkin_ws
source /opt/ros/noetic/setup.bash
cp src/dcd_timeref/CMakeLists-simple.txt src/dcd_timeref/CMakeLists.txt
catkin_make

# Source the workspace
echo "source /workspace/catkin_ws/devel/setup.bash" >> /root/.bashrc

echo "ROS Noetic development environment setup complete!"
