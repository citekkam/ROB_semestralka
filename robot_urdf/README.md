# CRS A465 Robot Visualization

This package provides ROS 2 visualization for the CRS A465 robot.

## Quick Start

### Option 1: Using the Launch Script (Recommended)

Simply run:
```bash
./launch_robot_viz.sh
```

### Option 2: Manual Launch

```bash
cd /home/david/School/rob/Semestralka/ROB_semestralka/robot_urdf

# Workaround for snap library conflict
export LD_PRELOAD=/lib/x86_64-linux-gnu/libpthread.so.0
unset GTK_PATH

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch
ros2 launch crs_a465_description display.launch.py
```

## Building the Package

If you make changes to the URDF, RViz config, or launch files:

```bash
cd /home/david/School/rob/Semestralka/ROB_semestralka/robot_urdf
source /opt/ros/humble/setup.bash
colcon build --packages-select crs_a465_description --symlink-install
```

## What Gets Launched

The launch file starts three nodes:

1. **Joint State Publisher GUI** - Interactive sliders to control robot joints
2. **Robot State Publisher** - Publishes robot transforms based on URDF
3. **RViz2** - 3D visualization with:
   - Grid display
   - Robot model visualization
   - TF (transform) display

## Troubleshooting

### RViz2 crashes with symbol lookup error

This is due to snap packages interfering with system libraries. The fix is already included in the launch script:
```bash
export LD_PRELOAD=/lib/x86_64-linux-gnu/libpthread.so.0
```

### Qt platform plugin warning

The warning `qt.qpa.plugin: Could not find the Qt platform plugin "wayland"` is harmless and can be ignored. The application still works correctly.

### Package not found error

Make sure you've sourced both setup files:
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## Files

- `urdf/crs_a465.urdf` - Robot URDF description
- `rviz/crs_a465.rviz` - RViz2 configuration (ROS 2 format)
- `launch/display.launch.py` - Launch file
- `package.xml` - ROS 2 package manifest
- `CMakeLists.txt` - Build configuration
