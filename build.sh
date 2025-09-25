#!/bin/bash

# Build script for dcd_timeref package
# This script compiles the ROS package in the container

set -e

echo "Building dcd_timeref package..."

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Always navigate to the catkin workspace (two levels up from the package)
WORKSPACE_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"
echo "Using workspace directory: $WORKSPACE_DIR"

# Verify this is actually a catkin workspace
if [ ! -d "$WORKSPACE_DIR/src" ]; then
    echo "Error: $WORKSPACE_DIR is not a valid catkin workspace"
    echo "Expected src/ directory not found"
    exit 1
fi

# Check if it's a catkin workspace (either has CMakeLists.txt or .catkin_workspace)
if [ ! -f "$WORKSPACE_DIR/CMakeLists.txt" ] && [ ! -f "$WORKSPACE_DIR/.catkin_workspace" ]; then
    echo "Warning: No CMakeLists.txt or .catkin_workspace found, but proceeding anyway"
fi

# Verify workspace structure
if [ ! -d "$WORKSPACE_DIR/src" ]; then
    echo "Error: Not a valid catkin workspace. Expected 'src' directory not found."
    echo "Current directory: $(pwd)"
    echo "Workspace directory: $WORKSPACE_DIR"
    exit 1
fi

# Source ROS environment
source /opt/ros/noetic/setup.bash

# Navigate to workspace
cd "$WORKSPACE_DIR"

# Install dependencies
echo "Installing dependencies..."
rosdep install --from-paths src --ignore-src -r -y

# Build the package
echo "Building package..."
catkin_make

# Source the workspace
source devel/setup.bash

echo "Build completed successfully!"
echo "To run the node:"
echo "  roslaunch dcd_timeref dcd_timeref.launch"
echo "  rosrun dcd_timeref dcd_timeref"
