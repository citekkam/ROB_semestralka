# IKT6 Python Library

Python implementation of the IKT6 inverse kinematics library for 6-DOF robotic manipulators.

## Overview

This is a complete Python port of the C++ IKT6 library. It provides:
- **Forward kinematics (DKT)**: Calculate end-effector position from joint angles
- **Inverse kinematics (IKT)**: Calculate joint angles from desired end-effector position
- **Robot parameter management**: Define robot geometry and limits

## Structure

```
ikt6_python/
├── __init__.py          # Main package interface
├── robot.py             # Robot class and initialization
├── kinematics.py        # Forward and inverse kinematics
├── util.py              # Utility functions (transformations, etc.)
└── tests/
    ├── test_utils.py    # Testing framework
    └── my_custom_test.py # Example test
```

## Installation

No installation needed - just import the module:

```python
from ikt6_python import ikt6_robot_init, ikt6_dkt, ikt6_ikt
```

## Quick Start

### 1. Initialize Robot

```python
import numpy as np
from ikt6_python import ikt6_robot_init

# Example: CRS93 parameters
lengths = np.array([440, 0, 305, 0, 330, 211])  # mm
offsets = np.array([0, 0, 0, 0, 0, 0])  # radians
directions = np.array([1, -1, -1, 1, -1, 1])
limits_max = np.deg2rad([175, 90, 110, 180, 105, 180])
limits_min = np.deg2rad([-175, -90, -110, -180, -105, -180])

robot = ikt6_robot_init(
    name="CRS93",
    lengths=lengths,
    offsets=offsets,
    directions=directions,
    limits_max=limits_max,
    limits_min=limits_min
)
```

### 2. Forward Kinematics

```python
from ikt6_python import ikt6_dkt, ikt6_dkt_T

# Joint angles in radians
J = np.array([0.1, -1.0, -1.9, 0.0, -0.1, 0.1])

# Get position and orientation [x, y, z, roll, pitch, yaw]
P = ikt6_dkt(robot, J)
print(f"Position: {P[:3]}")
print(f"Orientation: {P[3:]}")

# Or get full 4x4 transformation matrix
T = ikt6_dkt_T(robot, J)
```

### 3. Inverse Kinematics

```python
from ikt6_python import ikt6_ikt

# Target pose [x, y, z, roll, pitch, yaw]
P_target = np.array([500, 200, 600, 0, np.pi/4, 0])

# Get all IK solutions (up to 8)
J_solutions = ikt6_ikt(robot, P=P_target)

# Extract valid solutions (non-NaN columns)
for i in range(J_solutions.shape[1]):
    if not np.any(np.isnan(J_solutions[:, i])):
        print(f"Solution {i+1}: {J_solutions[:, i]}")
```

## Running Tests

```bash
cd ikt6_python/tests
python my_custom_test.py
```

Expected output:
```
=== Test 1: Normal Operation ===
Total tested configurations                    1000
Total solutions                                4545
Correct count                                  4545
Successfulness:                               100.00 %
```

## API Reference

### Main Functions

#### `ikt6_robot_init(name, lengths, offsets, directions, limits_max, limits_min, base=None, tool=None)`
Initialize robot parameters.

**Parameters:**
- `name` (str): Robot identifier
- `lengths` (array): Link lengths [L1, L2, L3, L4, L5, L6]
- `offsets` (array): Joint offsets in radians
- `directions` (array): Joint directions (+1 or -1)
- `limits_max` (array): Maximum joint limits in radians
- `limits_min` (array): Minimum joint limits in radians
- `base` (4x4 array): Base transformation (default: identity)
- `tool` (4x4 array): Tool transformation (default: identity)

**Returns:** Robot object

#### `ikt6_dkt(robot, J)`
Forward kinematics - returns position and orientation.

**Parameters:**
- `robot` (Robot): Robot parameters
- `J` (array): Joint angles [j1, j2, j3, j4, j5, j6] in radians

**Returns:** 6D array [x, y, z, roll, pitch, yaw]

#### `ikt6_dkt_T(robot, J)`
Forward kinematics - returns transformation matrix.

**Parameters:**
- `robot` (Robot): Robot parameters
- `J` (array): Joint angles in radians

**Returns:** 4x4 transformation matrix

#### `ikt6_ikt(robot, P=None, T=None)`
Inverse kinematics.

**Parameters:**
- `robot` (Robot): Robot parameters
- `P` (array): Target pose [x, y, z, roll, pitch, yaw] (optional)
- `T` (4x4 array): Target transformation (optional)

**Returns:** 6x8 matrix with up to 8 solutions (NaN columns are invalid)

## Comparison with C++ Version

| Feature | C++ | Python |
|---------|-----|--------|
| Speed | ⚡ Fast | 🐌 ~10-50x slower |
| Dependencies | Eigen3 | NumPy |
| Installation | Compile needed | Import only |
| Debugging | Harder | Easier |
| Integration | ROS, MoveIt | Easy with Python code |

## Examples

See `tests/my_custom_test.py` for complete examples including:
- Random configuration testing
- Singularity testing
- Manual verification
- Performance benchmarking

## Differences from C++

1. **Arrays instead of Eigen**: Uses NumPy arrays instead of Eigen matrices
2. **No pointers**: Python uses object references
3. **NaN handling**: Invalid solutions use `np.nan` instead of Eigen's NaN
4. **Function signatures**: Combined some overloaded functions

## License

Same as original C++ IKT6 library.
