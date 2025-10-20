#!/usr/bin/env python
"""
Trajectory points defined as transformation matrices (SE(3))

Each transformation matrix represents a pose with:
- Position: translation vector (x, y, z)
- Orientation: rotation matrix R with diagonal [-1, 1, -1]
"""

import numpy as np


# Rotation matrix with diagonal elements [-1, 1, -1]
# This represents a 180° rotation around Y-axis
ROTATION_MATRIX = np.array([
    [-1.0,  0.0,  0.0],
    [ 0.0,  1.0,  0.0],
    [ 0.0,  0.0, -1.0]
])


def create_transformation_matrix(position):
    """
    Create a 4x4 transformation matrix from position and the constant rotation.
    
    Args:
        position: array-like of shape (3,) representing [x, y, z]
    
    Returns:
        4x4 numpy array representing the homogeneous transformation matrix
    """
    T = np.eye(4)
    T[:3, :3] = ROTATION_MATRIX
    T[:3, 3] = position
    return T


# Define trajectory points
POINT_1 = create_transformation_matrix(np.array([0.07, 0.0, 0.2]))
POINT_2 = create_transformation_matrix(np.array([0.07, 0.0, 0.15]))
POINT_3 = create_transformation_matrix(np.array([0.0, 0.0, 0.08]))
POINT_4 = create_transformation_matrix(np.array([0.0, 0.0, 0.0]))

# Trajectory sequence
TRAJECTORY_POINTS = [POINT_1, POINT_2, POINT_3, POINT_4]


def print_trajectory_info():
    """Print information about the trajectory points."""
    print("Trajectory Points:")
    print("=" * 60)
    for i, point in enumerate(TRAJECTORY_POINTS, 1):
        print(f"\nPoint {i}:")
        print(f"Position: {point[:3, 3]}")
        print(f"Rotation matrix:\n{point[:3, :3]}")
    print("=" * 60)


if __name__ == "__main__":
    print_trajectory_info()
