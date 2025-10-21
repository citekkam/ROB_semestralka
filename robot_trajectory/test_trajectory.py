#!/usr/bin/env python
"""
Execute trajectory on the CRS97 robot

This script demonstrates how to execute the defined trajectory using IK.
"""

import numpy as np
import sys
from pathlib import Path

# Add parent directory to path
sys.path.append(str(Path(__file__).parent.parent))

from trajectory_points import TRAJECTORY_POINTS_PUZZLE_B
from utils import visualize_trajectory, plot_trajectory_3d
from robot_trajectory.interpolation import generate_traj_len, generate_traj_count
from robot_trajectory.utils import print_trajectory_info, visualize_trajectory, plot_trajectory_3d



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



