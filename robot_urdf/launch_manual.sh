#!/bin/bash

# Manual launch script for CRS A465 robot visualization
# Launches each component separately without using launch.py

# Navigate to the robot_urdf directory
cd /home/david/School/rob/Semestralka/ROB_semestralka/robot_urdf

# Workaround for snap library conflict
export LD_PRELOAD=/lib/x86_64-linux-gnu/libpthread.so.0
unset GTK_PATH

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
source install/setup.bash

# Path to URDF file
URDF_FILE="/home/david/School/rob/Semestralka/ROB_semestralka/robot_urdf/urdf/crs_a465.urdf"
RVIZ_CONFIG="/home/david/School/rob/Semestralka/ROB_semestralka/robot_urdf/rviz/crs_a465.rviz"

echo "Starting Robot State Publisher..."
ros2 run robot_state_publisher robot_state_publisher "$URDF_FILE" &
RSP_PID=$!

sleep 1

echo "Starting Joint State Publisher GUI..."
ros2 run joint_state_publisher_gui joint_state_publisher_gui &
JSP_PID=$!

sleep 1

echo "Starting RViz2..."
ros2 run rviz2 rviz2 -d "$RVIZ_CONFIG"
RVIZ_PID=$!

# Wait for user to close
wait $RVIZ_PID

# Clean up
echo "Shutting down..."
kill $RSP_PID 2>/dev/null
kill $JSP_PID 2>/dev/null
