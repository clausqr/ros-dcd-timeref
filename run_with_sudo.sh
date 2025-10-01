#!/bin/bash

# Script to run dcd_timeref with proper permissions and library paths

# Set up environment
cd /home/udesa/catkin_ws
source devel/setup.bash
export LD_LIBRARY_PATH=/opt/ros/noetic/lib:$LD_LIBRARY_PATH

# Run with sudo, preserving environment
sudo -E env LD_LIBRARY_PATH=$LD_LIBRARY_PATH ROS_MASTER_URI=$ROS_MASTER_URI \
    /home/udesa/catkin_ws/devel/lib/dcd_timeref/dcd_timeref \
    __name:=dcd_timeref \
    _pps_device:=/dev/pps0 \
    _edge:=assert \
    _source:=DCD_PPS \
    _frame_id:=gps_time
