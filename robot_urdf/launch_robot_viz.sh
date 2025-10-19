#!/bin/bash

# Script to launch CRS A465 robot visualization in RViz2
# This script fixes the snap/library conflict issue

# Navigate to the robot_urdf directory
cd /home/david/School/rob/Semestralka/ROB_semestralka/robot_urdf

# Workaround for snap library conflict
export LD_PRELOAD=/lib/x86_64-linux-gnu/libpthread.so.0

# Clear GTK_PATH to avoid conflicts
unset GTK_PATH

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Source local workspace
source install/setup.bash

# Launch the robot visualization
ros2 launch crs_a465_description display.launch.py
