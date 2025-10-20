#!/usr/bin/env python
"""
Interpolation functions for trajectory execution

Provides linear and SLERP interpolation for transformation matrices.
"""

import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp


def interpolate_position_linear(p1, p2, t):
    """
    Linear interpolation between two positions.
    
    Args:
        p1: Starting position (3D vector)
        p2: Ending position (3D vector)
        t: Interpolation parameter [0, 1]
    
    Returns:
        Interpolated position
    """
    return (1 - t) * p1 + t * p2


def interpolate_rotation_slerp(R1, R2, t):
    """
    Spherical linear interpolation (SLERP) between two rotation matrices.
    
    Args:
        R1: Starting rotation matrix (3x3)
        R2: Ending rotation matrix (3x3)
        t: Interpolation parameter [0, 1]
    
    Returns:
        Interpolated rotation matrix (3x3)
    """
    rot1 = R.from_matrix(R1)
    rot2 = R.from_matrix(R2)
    
    # Create SLERP interpolator
    key_times = [0, 1]
    key_rots = R.from_quat([rot1.as_quat(), rot2.as_quat()])
    slerp = Slerp(key_times, key_rots)
    
    # Interpolate
    interp_rot = slerp(t)
    return interp_rot.as_matrix()


def interpolate_transformation(T1, T2, t):
    """
    Interpolate between two transformation matrices.
    
    Uses linear interpolation for position and SLERP for rotation.
    
    Args:
        T1: Starting transformation matrix (4x4)
        T2: Ending transformation matrix (4x4)
        t: Interpolation parameter [0, 1]
    
    Returns:
        Interpolated transformation matrix (4x4)
    """
    # Extract positions and rotations
    p1 = T1[:3, 3]
    p2 = T2[:3, 3]
    R1 = T1[:3, :3]
    R2 = T2[:3, :3]
    
    # Interpolate
    p_interp = interpolate_position_linear(p1, p2, t)
    R_interp = interpolate_rotation_slerp(R1, R2, t)
    
    # Create interpolated transformation matrix
    T_interp = np.eye(4)
    T_interp[:3, :3] = R_interp
    T_interp[:3, 3] = p_interp
    
    return T_interp


def generate_trajectory_segment(T_start, T_end, num_points=50):
    """
    Generate a trajectory segment between two poses.
    
    Args:
        T_start: Starting transformation matrix (4x4)
        T_end: Ending transformation matrix (4x4)
        num_points: Number of interpolation points
    
    Returns:
        List of transformation matrices representing the trajectory
    """
    trajectory = []
    for i in range(num_points):
        t = i / (num_points - 1)  # Normalize to [0, 1]
        T_interp = interpolate_transformation(T_start, T_end, t)
        trajectory.append(T_interp)
    
    return trajectory


def generate_full_trajectory(trajectory_points, num_points_per_segment=50):
    """
    Generate a full trajectory through all waypoints.
    
    Args:
        trajectory_points: List of transformation matrices (waypoints)
        num_points_per_segment: Number of interpolation points between each pair
    
    Returns:
        List of transformation matrices representing the full trajectory
    """
    full_trajectory = []
    
    for i in range(len(trajectory_points) - 1):
        segment = generate_trajectory_segment(
            trajectory_points[i], 
            trajectory_points[i + 1], 
            num_points_per_segment
        )
        
        # Avoid duplicating points at segment boundaries
        if i > 0:
            segment = segment[1:]
        
        full_trajectory.extend(segment)
    
    return full_trajectory


def visualize_trajectory(trajectory_points, num_points_per_segment=50):
    """
    Visualize the trajectory by printing positions.
    
    Args:
        trajectory_points: List of transformation matrices (waypoints)
        num_points_per_segment: Number of interpolation points between each pair
    """
    full_traj = generate_full_trajectory(trajectory_points, num_points_per_segment)
    
    print(f"Generated trajectory with {len(full_traj)} points")
    print("\nSample points:")
    print("-" * 60)
    
    # Print every 10th point
    for i in range(0, len(full_traj), len(full_traj) // 10):
        T = full_traj[i]
        print(f"Point {i:3d}: Position = [{T[0,3]:7.4f}, {T[1,3]:7.4f}, {T[2,3]:7.4f}]")


if __name__ == "__main__":
    # Example usage
    from trajectory_points import TRAJECTORY_POINTS
    
    print("Interpolation Demo")
    print("=" * 60)
    visualize_trajectory(TRAJECTORY_POINTS, num_points_per_segment=20)
