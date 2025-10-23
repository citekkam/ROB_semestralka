#!/usr/bin/env python
"""
Interpolation functions for trajectory execution

Provides linear and SLERP interpolation for SE3 transformations using SO3 rotations.
"""

import numpy as np
import sys
from pathlib import Path

# Add parent directory to path to import SE3 and SO3
sys.path.append(str(Path(__file__).parent.parent))

from se3 import SE3
from so3 import SO3


def interpolate_position_linear(p1, p2, t):
    """
    Linear interpolation between two positions.
    
    Linear interpolation formula: p(t) = (1-t)*p1 + t*p2
    
    Args:
        p1: Starting position (3D vector)
        p2: Ending position (3D vector)
        t: Interpolation parameter, normalized time [0, 1]
           - t=0: returns p1 (starting position)
           - t=0.5: returns midpoint between p1 and p2
           - t=1: returns p2 (ending position)
           - 0 < t < 1: returns weighted blend of p1 and p2
    
    Returns:
        Interpolated position (3D numpy array)
    """
    return (1 - t) * p1 + t * p2

def interpolate_rotation_slerp(rot1, rot2, t):
    """
    Spherical linear interpolation (SLERP) between two SO3 rotations.
    
    SLERP provides smooth interpolation along the shortest path on the rotation manifold.
    Uses the exponential map method: R(t) = R1 * exp(t * log(R1^T * R2))
    
    Args:
        rot1: Starting rotation (SO3 object)
        rot2: Ending rotation (SO3 object)
        t: Interpolation parameter, normalized time [0, 1]
           - t=0: returns rot1 (starting rotation)
           - t=0.5: returns rotation halfway between rot1 and rot2
           - t=1: returns rot2 (ending rotation)
           - 0 < t < 1: returns smooth rotation between rot1 and rot2
           The rotation speed is constant along the interpolation path.
    
    Returns:
        Interpolated rotation (SO3 object)
    """
    # Compute the relative rotation: R_rel = R1^T * R2
    R_rel = rot1.inverse() * rot2
    
    # Get the rotation vector (logarithm) of the relative rotation
    omega = R_rel.log()
    
    # Scale by t and convert back to rotation
    R_interp_rel = SO3.exp(t * omega)
    
    # Compose with the starting rotation
    R_interp = rot1 * R_interp_rel
    
    return R_interp

def interpolate_transformation(T1, T2, t):
    """
    Interpolate between two SE3 transformations.
    
    Uses linear interpolation for position and SLERP for rotation.
    
    Args:
        T1: Starting transformation (SE3 object)
        T2: Ending transformation (SE3 object)
        t: Interpolation parameter [0, 1]
    
    Returns:
        Interpolated transformation (SE3 object)
    """
    # Interpolate position
    p_interp = interpolate_position_linear(T1.translation, T2.translation, t)
    
    # Interpolate rotation
    R_interp = interpolate_rotation_slerp(T1.rotation, T2.rotation, t)
    
    # Create interpolated SE3 transformation
    return SE3(translation=p_interp, rotation=R_interp)

def generate_trajectory_segment(T_start, T_end, num_points=50):
    """
    Generate a trajectory segment between two poses with fixed point count.
    
    Args:
        T_start: Starting transformation (SE3 object)
        T_end: Ending transformation (SE3 object)
        num_points: Number of interpolation points (default: 50)
    
    Returns:
        List of SE3 objects representing the trajectory segment
    """
    trajectory = []
    for i in range(num_points):
        t = i / (num_points - 1)  # Normalize to [0, 1]
        T_interp = interpolate_transformation(T_start, T_end, t)
        trajectory.append(T_interp)
    
    return trajectory

def generate_traj_count(trajectory_points, num_points_per_segment=50):
    """
    Generate a full trajectory through all waypoints with fixed point count per segment.
    
    Args:
        trajectory_points: List of SE3 transformations (waypoints)
        num_points_per_segment: Number of interpolation points between each pair (default: 50)
    
    Returns:
        List of SE3 objects representing the full trajectory
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

def generate_traj_len(trajectory_points, segment_length=0.01):
    """
    Generate a full trajectory through all waypoints with fixed spacing.
    
    Args:
        trajectory_points: List of SE3 transformations (waypoints)
        segment_length: Desired spacing between points in meters (default: 0.01m = 1cm)
    
    Returns:
        List of SE3 objects representing the full trajectory
    """
    full_trajectory = []
    
    for i in range(len(trajectory_points) - 1):
        # Calculate Euclidean distance between start and end positions
        distance = np.linalg.norm(trajectory_points[i + 1].translation - trajectory_points[i].translation)

        # Calculate number of points based on segment length
        # Ensure at least 2 points (start and end)
        num_points = max(2, int(np.ceil(distance / segment_length)) + 1)

        segment = generate_trajectory_segment(
            trajectory_points[i], 
            trajectory_points[i + 1], 
            num_points
        )
        
        # Avoid duplicating points at segment boundaries
        if i > 0:
            segment = segment[1:]
        
        full_trajectory.extend(segment)
    
    return full_trajectory

if __name__ == "__main__":
    # Example usage
    from trajectory_points import TRAJECTORY_POINTS_PUZZLE_B
    from utils import visualize_trajectory, plot_trajectory_3d
    
    print("Interpolation Demo")
    print("=" * 60)
    
    # Generate trajectory with fixed spacing (1cm between points)
    print("\nGenerating trajectory with fixed spacing (segment_length=0.01m)...")
    full_traj_len = generate_traj_len(TRAJECTORY_POINTS_PUZZLE_B, segment_length=0.01)
    print(f"Generated {len(full_traj_len)} points with length-based method")
    
    # Generate trajectory with fixed point count
    print("\nGenerating trajectory with fixed point count (num_points=50)...")
    full_traj_count = generate_traj_count(TRAJECTORY_POINTS_PUZZLE_B, num_points_per_segment=5)
    print(f"Generated {len(full_traj_count)} points with count-based method")
    
    # Visualize the length-based trajectory
    print("\nVisualizing length-based trajectory...")
    visualize_trajectory(full_traj_len)
    
    print("\nGenerating 3D plot...")
    plot_trajectory_3d(full_traj_len)
