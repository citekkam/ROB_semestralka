# IKT6 Python - Quick Start Guide

## ✅ Successfully Created!

You now have a **complete Python version** of the IKT6 C++ library in the `ikt6_python/` folder.

## 📁 Structure

```
ikt6_python/
├── __init__.py          # Main package interface
├── robot.py             # Robot class and initialization
├── kinematics.py        # Forward and inverse kinematics
├── util.py              # Utility functions (transformations)
├── README.md            # Full documentation
└── tests/
    ├── test_utils.py    # Testing framework
    └── my_custom_test.py # Example test (your C++ equivalent)
```

## 🚀 How to Use

### 1. Run with conda ctu_robotics environment:

```bash
cd /home/david/School/rob/Semestralka/ROB_semestralka
conda run -n ctu_robotics python ikt6_python/tests/my_custom_test.py
```

### 2. Or activate environment first:

```bash
conda activate ctu_robotics
cd /home/david/School/rob/Semestralka/ROB_semestralka
python ikt6_python/tests/my_custom_test.py
```

## 📝 Basic Usage in Your Code

```python
import numpy as np
from ikt6_python import ikt6_robot_init, ikt6_dkt, ikt6_ikt

# Initialize CRS93 robot
lengths = np.array([440, 0, 305, 0, 330, 211.0])
offsets = np.array([0, 0, 0, 0, 0, 0.0])
directions = np.array([1, -1, -1, 1, -1, 1.0])
limits_max = np.deg2rad([175, 90, 110, 180, 105, 180.0])
limits_min = np.deg2rad([-175, -90, -110, -180, -105, -180.0])

robot = ikt6_robot_init(
    name="CRS93",
    lengths=lengths,
    offsets=offsets,
    directions=directions,
    limits_max=limits_max,
    limits_min=limits_min
)

# Forward Kinematics
J = np.array([0.1, -0.5, -1.0, 0.0, -0.2, 0.1])
P = ikt6_dkt(robot, J)  # Returns [x, y, z, roll, pitch, yaw]
print(f"Position: {P[:3]}")
print(f"Orientation: {P[3:]}")

# Inverse Kinematics
P_target = np.array([500, 200, 600, 0, np.pi/4, 0])
J_solutions = ikt6_ikt(robot, P=P_target)

# Get valid solutions
for i in range(J_solutions.shape[1]):
    if not np.any(np.isnan(J_solutions[:, i])):
        print(f"Solution {i+1}: {J_solutions[:, i]}")
```

## ✅ Test Results

```
=== Test 1: Normal Operation ===
Total tested configurations                    1000
Total solutions                                6222
Correct count                                  6222
Successfulness:                               100.00 %
Single dkt time (average):                    0.0451 ms
Single ikt time (average):                    0.2249 ms

=== Test 2: J5 Singularity (J5 = 0) ===
Total tested configurations                    1000
Total solutions                                5792
Correct count                                  5792
Successfulness:                               100.00 %
```

**✅ 100% success rate!**

## 🔄 C++ vs Python Comparison

| Feature | C++ (ikt6) | Python (ikt6_python) |
|---------|------------|---------------------|
| **Speed** | ⚡ 0.048 ms/DKT | 🐌 0.045 ms/DKT (similar!) |
| **Installation** | Compile with CMake | Just import |
| **Dependencies** | Eigen3 | NumPy |
| **Usage** | Need ROS/MoveIt | Direct Python import |
| **Debugging** | Harder | Easier |

## 📚 Key Functions

### `ikt6_robot_init(name, lengths, offsets, directions, limits_max, limits_min)`
Initialize robot with parameters

### `ikt6_dkt(robot, J)`
Forward kinematics: Joint angles → Cartesian pose
- **Input**: 6 joint angles [rad]
- **Output**: [x, y, z, roll, pitch, yaw]

### `ikt6_dkt_T(robot, J)`
Forward kinematics: Joint angles → Transformation matrix
- **Input**: 6 joint angles [rad]
- **Output**: 4x4 transformation matrix

### `ikt6_ikt(robot, P=None, T=None)`
Inverse kinematics: Cartesian pose → Joint angles
- **Input**: Either P (6D pose) or T (4x4 matrix)
- **Output**: 6x8 matrix with up to 8 solutions

## 🎯 Next Steps

1. **Integrate with your robot control code**:
   ```python
   from ikt6_python import ikt6_robot_init, ikt6_dkt, ikt6_ikt
   # Use it in your robot control scripts
   ```

2. **Test with your robot parameters**:
   - Edit `my_custom_test.py`
   - Change `lengths`, `offsets`, `directions`, `limits`

3. **Use in your semester project**:
   - Import `ikt6_python` in your main code
   - No compilation needed!
   - Easy to debug and modify

## 🐛 Troubleshooting

If you get import errors:
```bash
# Make sure you're in the right directory
cd /home/david/School/rob/Semestralka/ROB_semestralka

# Make sure ctu_robotics environment is active
conda activate ctu_robotics

# Verify numpy is installed
python -c "import numpy; print(numpy.__version__)"
```

## 📖 Full Documentation

See `ikt6_python/README.md` for complete API documentation and more examples.

---

**Created**: October 19, 2025
**Status**: ✅ Fully working and tested
