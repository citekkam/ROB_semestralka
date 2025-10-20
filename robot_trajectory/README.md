# Robot Trajectory

This module defines and executes trajectories for the CRS97 robot.

## Files

### `trajectory_points.py`
Defines trajectory waypoints as transformation matrices (SE(3)).

**Key components:**
- `ROTATION_MATRIX`: Constant rotation with diagonal [-1, 1, -1]
- `POINT_1` to `POINT_4`: Four waypoints with positions:
  - Point 1: (0.07, 0.0, 0.2)
  - Point 2: (0.07, 0.0, 0.15)
  - Point 3: (0.0, 0.0, 0.08)
  - Point 4: (0.0, 0.0, 0.0)
- `TRAJECTORY_POINTS`: List of all waypoints in sequence

### `interpolation.py`
Provides interpolation functions for smooth trajectory generation.

**Key functions:**
- `interpolate_position_linear(p1, p2, t)`: Linear interpolation for positions
- `interpolate_rotation_slerp(R1, R2, t)`: Spherical linear interpolation for rotations
- `interpolate_transformation(T1, T2, t)`: Interpolate full transformation matrices
- `generate_trajectory_segment(T_start, T_end, num_points)`: Generate points between two poses
- `generate_full_trajectory(trajectory_points, num_points_per_segment)`: Generate complete trajectory

### `execute_trajectory.py`
Executes the trajectory on the CRS97 robot using inverse kinematics.

**Key functions:**
- `execute_trajectory(robot, trajectory_points, num_points_per_segment)`: Execute interpolated trajectory
- `execute_waypoints_only(robot, trajectory_points)`: Move only to defined waypoints

## Usage

### 1. Test trajectory definition
```bash
python robot_trajectory/trajectory_points.py
```

### 2. Test interpolation
```bash
python robot_trajectory/interpolation.py
```

### 3. Execute on robot
```bash
python robot_trajectory/execute_trajectory.py
```

## Example: Using in your own code

```python
from robot_trajectory.trajectory_points import TRAJECTORY_POINTS
from robot_trajectory.interpolation import generate_full_trajectory
from ctu_crs import CRS97

# Initialize robot
robot = CRS97()
robot.initialize()

# Generate interpolated trajectory
trajectory = generate_full_trajectory(TRAJECTORY_POINTS, num_points_per_segment=50)

# Execute trajectory
for target_pose in trajectory:
    ik_sols = robot.ik(target_pose)
    if ik_sols:
        robot.move_to_q(ik_sols[0])

robot.close()
```

## Transformation Matrix Format

Each pose is represented as a 4x4 homogeneous transformation matrix:

```
T = [R | p]
    [0 | 1]

where:
- R is the 3x3 rotation matrix
- p is the 3x1 position vector
```

The rotation matrix used has diagonal elements [-1, 1, -1]:
```
R = [-1  0  0]
    [ 0  1  0]
    [ 0  0 -1]
```

This represents a 180° rotation around the Y-axis.
