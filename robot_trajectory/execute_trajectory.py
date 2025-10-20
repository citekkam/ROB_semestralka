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

from ctu_crs import CRS97
from robot_trajectory.trajectory_points import TRAJECTORY_POINTS
from robot_trajectory.interpolation import generate_full_trajectory


def execute_trajectory(robot, trajectory_points, num_points_per_segment=30, wait_time=0.1):
    """
    Execute a trajectory on the robot using inverse kinematics.
    
    Args:
        robot: CRS97 robot instance
        trajectory_points: List of transformation matrices (waypoints)
        num_points_per_segment: Number of interpolation points between waypoints
        wait_time: Time to wait at each waypoint (seconds)
    """
    # Generate full interpolated trajectory
    full_trajectory = generate_full_trajectory(trajectory_points, num_points_per_segment)
    
    print(f"Executing trajectory with {len(full_trajectory)} points...")
    
    current_q = robot.get_q()
    
    for i, target_pose in enumerate(full_trajectory):
        # Compute inverse kinematics
        ik_solutions = robot.ik(target_pose)
        
        if len(ik_solutions) == 0:
            print(f"Warning: No IK solution found for point {i}")
            print(f"Target position: {target_pose[:3, 3]}")
            continue
        
        # Choose solution closest to current configuration
        closest_q = min(ik_solutions, key=lambda q: np.linalg.norm(q - current_q))
        
        # Move to target configuration
        robot.move_to_q(closest_q)
        current_q = closest_q
        
        # Print progress
        if i % 10 == 0:
            print(f"Progress: {i}/{len(full_trajectory)} - Position: {target_pose[:3, 3]}")
    
    # Wait for final motion to complete
    robot.wait_for_motion_stop()
    print("Trajectory execution completed!")


def execute_waypoints_only(robot, trajectory_points, wait_time=2.0):
    """
    Execute trajectory by moving only to the defined waypoints.
    
    Args:
        robot: CRS97 robot instance
        trajectory_points: List of transformation matrices (waypoints)
        wait_time: Time to wait at each waypoint (seconds)
    """
    print(f"Executing {len(trajectory_points)} waypoints...")
    
    current_q = robot.get_q()
    
    for i, target_pose in enumerate(trajectory_points):
        print(f"\nMoving to waypoint {i+1}/{len(trajectory_points)}")
        print(f"Target position: {target_pose[:3, 3]}")
        
        # Compute inverse kinematics
        ik_solutions = robot.ik(target_pose)
        
        if len(ik_solutions) == 0:
            print(f"Error: No IK solution found for waypoint {i+1}")
            continue
        
        # Choose solution closest to current configuration
        closest_q = min(ik_solutions, key=lambda q: np.linalg.norm(q - current_q))
        
        # Move to target configuration
        robot.move_to_q(closest_q)
        robot.wait_for_motion_stop()
        
        current_q = closest_q
        
        import time
        time.sleep(wait_time)
    
    print("\nAll waypoints reached!")


def main():
    """Main execution function."""
    # Initialize robot
    robot = CRS97()
    
    print("Initializing robot...")
    input("Press Enter to initialize motors...")
    robot.initialize()
    
    print("\nRobot initialized!")
    print("Current position:", robot.fk(robot.get_q())[:3, 3])
    
    # Choose execution mode
    print("\nExecution modes:")
    print("1. Waypoints only (move to each defined point)")
    print("2. Full interpolated trajectory")
    
    mode = input("\nSelect mode (1 or 2): ").strip()
    
    try:
        if mode == "1":
            execute_waypoints_only(robot, TRAJECTORY_POINTS)
        elif mode == "2":
            num_points = int(input("Enter number of points per segment (default 30): ") or "30")
            execute_trajectory(robot, TRAJECTORY_POINTS, num_points_per_segment=num_points)
        else:
            print("Invalid mode selected")
    except Exception as e:
        print(f"Error during execution: {e}")
    finally:
        robot.close()
        print("Robot connection closed")


if __name__ == "__main__":
    main()
