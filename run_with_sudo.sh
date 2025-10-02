#!/bin/bash
set -euo pipefail  # Enable strict error handling

# Script to run dcd_timeref with proper permissions and library paths

# Get script directory dynamically
if [ -n "${BASH_SOURCE[0]:-}" ]; then
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
else
    SCRIPT_DIR="$(pwd)"
fi
WORKSPACE_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

# Validate workspace exists
if [ ! -d "$WORKSPACE_DIR/devel" ]; then
    echo "Error: Catkin workspace not found at $WORKSPACE_DIR"
    echo "Expected structure: $WORKSPACE_DIR/devel/"
    exit 1
fi

# Set up environment
cd "$WORKSPACE_DIR"
source devel/setup.bash
export LD_LIBRARY_PATH="/opt/ros/noetic/lib:$LD_LIBRARY_PATH"

# Validate executable exists
EXECUTABLE_PATH="$WORKSPACE_DIR/devel/lib/dcd_timeref/dcd_timeref"
if [ ! -f "$EXECUTABLE_PATH" ]; then
    echo "Error: Executable not found at $EXECUTABLE_PATH"
    echo "Please build the package first: catkin_make"
    exit 1
fi

# Run with sudo, preserving environment
sudo -E env LD_LIBRARY_PATH="$LD_LIBRARY_PATH" ROS_MASTER_URI="$ROS_MASTER_URI" \
    "$EXECUTABLE_PATH" \
    __name:=dcd_timeref \
    _pps_device:=/dev/pps0 \
    _edge:=assert \
    _source:=DCD_PPS \
    _frame_id:=lidar_trigger_time
