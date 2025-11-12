#!/usr/bin/env python
"""
Trajectory points defined as transformation matrices (SE(3))

Each transformation matrix represents a pose with:
- Position: translation vector (x, y, z)
- Orientation: rotation matrix R with diagonal [-1, 1, -1]
"""

import numpy as np
import sys
from pathlib import Path

# Add parent directory to path to import SE3 and SO3
sys.path.append(str(Path(__file__).parent.parent))

from se3 import SE3
from so3 import SO3
from utils import print_trajectory_info


# Rotation matrix with diagonal elements [-1, 1, -1]
# # This represents a 180° rotation around Y-axis
# ROTATION_MATRIX = np.array([
#     [-1.0,  0.0,  0.0],
#     [ 0.0,  1.0,  0.0],
#     [ 0.0,  0.0, -1.0]
# ])

# # Create the constant rotation as SO3 object
# ROTATION = SO3(ROTATION_MATRIX)


ROTATION = SO3() # 180° rotation around Y-axis

# ========== PUZZLE A TRAJECTORY ==========
# Define trajectory points for Puzzle A as SE3 objects (global constants)
PUZZLE_A_POINT_1 = SE3(translation=np.array([0.0, 0.0, 0.2]), rotation=ROTATION)
PUZZLE_A_POINT_2 = SE3(translation=np.array([0.0, 0.0, 0.01]), rotation=ROTATION)

# ========== PUZZLE B TRAJECTORY ==========
# Define trajectory points for Puzzle B as SE3 objects (global constants)
PUZZLE_B_POINT_1 = SE3(translation=np.array([0.07, 0.0, 0.2]), rotation=ROTATION)
PUZZLE_B_POINT_2 = SE3(translation=np.array([0.07, 0.0, 0.15]), rotation=ROTATION)
PUZZLE_B_POINT_3 = SE3(translation=np.array([0.0, 0.0, 0.08]), rotation=ROTATION)
PUZZLE_B_POINT_4 = SE3(translation=np.array([0.0, 0.0, 0.01]), rotation=ROTATION)

# ========== PUZZLE C TRAJECTORY ==========
# Define trajectory points for Puzzle C as SE3 objects (global constants)
# TODO: Define your Puzzle C points here
PUZZLE_C_POINT_1 = SE3(translation=np.array([-0.05, -0.05, 0.2]), rotation=ROTATION)
PUZZLE_C_POINT_2 = SE3(translation=np.array([-0.05, -0.05, 0.15]), rotation=ROTATION)
PUZZLE_C_POINT_3 = SE3(translation=np.array([-0.05, 0.0, 0.1]), rotation=ROTATION)
PUZZLE_C_POINT_4 = SE3(translation=np.array([0.0, 0.0, 0.05]), rotation=ROTATION)
PUZZLE_C_POINT_5 = SE3(translation=np.array([0.0, 0.0, 0.01]), rotation=ROTATION)



# ========== PUZZLE D TRAJECTORY ==========
# Define trajectory points for Puzzle D as SE3 objects (global constants)
# TODO: Define your Puzzle D points here
PUZZLE_D_POINT_1 = SE3(translation=np.array([0.085, 0.0, 0.155]), rotation=ROTATION * SO3().ry(np.pi/2))
PUZZLE_D_POINT_2 = SE3(translation=np.array([0.035, 0.0, 0.155]), rotation=ROTATION * SO3().ry(np.pi/2))
PUZZLE_D_POINT_3 = SE3(translation=np.array([-0.015, 0.0, 0.105]), rotation=ROTATION)
PUZZLE_D_POINT_4 = SE3(translation=np.array([-0.015, 0.0, 0.045]), rotation=ROTATION)
PUZZLE_D_POINT_5 = SE3(translation=np.array([0.0, 0.0, 0.03]), rotation=ROTATION)
PUZZLE_D_POINT_6 = SE3(translation=np.array([0.0, 0.0, 0.01]), rotation=ROTATION)

# ========== PUZZLE E TRAJECTORY ==========
# Define trajectory points for Puzzle E as SE3 objects (global constants)
# TODO: Define your Puzzle E points here
PUZZLE_E_POINT_1 = SE3(translation=np.array([0.0327, 0.1, 0.16]), rotation=ROTATION * SO3().rx(-np.pi/2))
PUZZLE_E_POINT_2 = SE3(translation=np.array([0.0327, 0.05, 0.16]), rotation=ROTATION * SO3().rx(-np.pi/2))
PUZZLE_E_POINT_3 = SE3(translation=np.array([-0.0173, 0.0, 0.16]), rotation=ROTATION * SO3().rx(-np.pi/2))
PUZZLE_E_POINT_4 = SE3(translation=np.array([-0.0173, -0.05, 0.11]), rotation=ROTATION)
PUZZLE_E_POINT_5 = SE3(translation=np.array([-0.0173, 0.0, 0.06]), rotation=ROTATION)
PUZZLE_E_POINT_6 = SE3(translation=np.array([0.0, 0.0, 0.03]), rotation=ROTATION)
PUZZLE_E_POINT_7 = SE3(translation=np.array([0.0, 0.0, 0.01]), rotation=ROTATION)

# Trajectory sequence for Puzzle A
TRAJECTORY_POINTS_PUZZLE_A = [PUZZLE_A_POINT_1, PUZZLE_A_POINT_2]

# Trajectory sequence for Puzzle B
TRAJECTORY_POINTS_PUZZLE_B = [PUZZLE_B_POINT_1, PUZZLE_B_POINT_2, PUZZLE_B_POINT_3, PUZZLE_B_POINT_4]

# Trajectory sequence for Puzzle C
TRAJECTORY_POINTS_PUZZLE_C = [PUZZLE_C_POINT_1, PUZZLE_C_POINT_2, PUZZLE_C_POINT_3, PUZZLE_C_POINT_4, PUZZLE_C_POINT_5]

# Trajectory sequence for Puzzle D
TRAJECTORY_POINTS_PUZZLE_D = [PUZZLE_D_POINT_1, PUZZLE_D_POINT_2, PUZZLE_D_POINT_3, PUZZLE_D_POINT_4, PUZZLE_D_POINT_5, PUZZLE_D_POINT_6]

# Trajectory sequence for Puzzle E
TRAJECTORY_POINTS_PUZZLE_E = [PUZZLE_E_POINT_1, PUZZLE_E_POINT_2, PUZZLE_E_POINT_3, PUZZLE_E_POINT_4, PUZZLE_E_POINT_5, PUZZLE_E_POINT_6, PUZZLE_E_POINT_7]



if __name__ == "__main__":
    print_trajectory_info(TRAJECTORY_POINTS_PUZZLE_A, "Puzzle A")
    print("\n")
    print_trajectory_info(TRAJECTORY_POINTS_PUZZLE_B, "Puzzle B")
    print("\n")
    print_trajectory_info(TRAJECTORY_POINTS_PUZZLE_C, "Puzzle C")
    print("\n")
    print_trajectory_info(TRAJECTORY_POINTS_PUZZLE_D, "Puzzle D")
    print("\n")
    print_trajectory_info(TRAJECTORY_POINTS_PUZZLE_E, "Puzzle E")
    print(ROTATION.rot)
