#!/usr/bin/env python3
"""
Robot Calibration Automation Script
====================================
This script automatically moves the robot to predefined calibration positions,
captures images, and saves the data (joint positions, transformation matrices, images).

The script:
1. Loads 9 calibration positions from calibration_positions.yaml
2. Moves the robot to each position using IK and shortest path selection
3. Captures an image at each position
4. Saves robot_q, transformation matrix, and image to files

Author: David
Date: 2025-10-18
"""

import numpy as np
import yaml
from pathlib import Path
import sys

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from ctu_crs import CRS93
from select_shortest_path import find_shortest_path
from camera.dataset_creator import uloz_data


def pose_to_transformation_matrix(pose: np.ndarray) -> np.ndarray:
    """
    Convert pose [x, y, z, roll, pitch, yaw] to 4x4 transformation matrix.
    
    Args:
        pose: 6D array [x, y, z, roll, pitch, yaw] where x,y,z in meters, r,p,y in radians
        
    Returns:
        4x4 homogeneous transformation matrix
    """
    x, y, z, roll, pitch, yaw = pose
    
    # Rotation matrix from RPY (ZYX convention)
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    
    # ZYX rotation matrix
    R = np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp, cp*sr, cp*cr]
    ])
    
    # Build 4x4 transformation matrix
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    
    return T


def load_calibration_positions(yaml_file: str) -> list:
    """
    Load calibration positions from YAML file.
    
    Args:
        yaml_file: Path to the YAML file containing calibration positions
        
    Returns:
        List of tuples (position_name, pose array [x,y,z,r,p,y])
    """
    with open(yaml_file, 'r') as f:
        data = yaml.safe_load(f)
    
    positions = []
    for key in sorted(data.keys()):  # Sort to ensure consistent order
        if 'position_' in key:
            pose = np.array(data[key]['pose'])
            positions.append((key, pose))
    
    return positions


def move_to_position_safe(robot, target_pose: np.ndarray, joint_weights: np.ndarray):
    """
    Safely move robot to target position using IK and shortest path selection.
    
    Args:
        robot: Robot instance (CRS93)
        target_pose: Target pose [x, y, z, roll, pitch, yaw]
        joint_weights: Weights for distance calculation
    
    Returns:
        bool: True if movement successful, False otherwise
    """
    try:
        # Get current robot position
        current_q = robot.get_q()
        
        # Convert pose to transformation matrix
        target_T = pose_to_transformation_matrix(target_pose)
        
        print(f"  Target pose [x,y,z,r,p,y]: [{target_pose[0]:.3f}, {target_pose[1]:.3f}, {target_pose[2]:.3f}, {target_pose[3]:.3f}, {target_pose[4]:.3f}, {target_pose[5]:.3f}]")
        
        # Get all IK solutions
        ik_solutions = robot.ik(target_T)
        
        if len(ik_solutions) == 0:
            print(f"  ⚠️  No IK solutions found for target pose")
            return False
        
        print(f"  Found {len(ik_solutions)} IK solutions")
        
        # Find shortest path
        sorted_distances = find_shortest_path(current_q, ik_solutions, joint_weights)
        
        # Select the closest solution
        idx, distance, _ = sorted_distances[0]
        best_q = ik_solutions[idx]
        
        print(f"  → Selected IK solution {idx} with distance {distance:.4f}")
        print(f"  → Target joints: {best_q}")
        
        # Move to position
        robot.move_to_q(best_q)
        robot.wait_for_motion_stop()
        
        return True
        
    except Exception as e:
        print(f"  ❌ Error during movement: {e}")
        return False


def main():
    """Main calibration automation function."""
    
    print("="*70)
    print("  ROBOT CALIBRATION AUTOMATION")
    print("="*70)
    print()
    
    # Setup paths
    script_dir = Path(__file__).parent
    positions_file = script_dir / "calibration_positions.yaml"
    output_folder = str(script_dir / "calibration_data")
    
    # Create output folder if it doesn't exist
    Path(output_folder).mkdir(exist_ok=True, parents=True)
    
    # Configuration
    joint_weights = np.array([1.0, 1.0, 1.0, 10.0, 1.0, 10.0])
    
    # Load calibration positions
    print(f"📂 Loading calibration positions from: {positions_file}")
    try:
        positions = load_calibration_positions(str(positions_file))
        print(f"✅ Loaded {len(positions)} calibration positions")
        print()
    except Exception as e:
        print(f"❌ Failed to load positions: {e}")
        return
    
    # Initialize robot
    print("🤖 Initializing robot...")
    try:
        robot = CRS93()
        robot.initialize(home=False)
        print("✅ Robot initialized successfully")
        print()
    except Exception as e:
        print(f"❌ Failed to initialize robot: {e}")
        return
    
    # Process each calibration position
    successful_captures = 0
    failed_captures = 0
    
    for i, (pos_name, target_pose) in enumerate(positions, 1):
        print(f"{'='*70}")
        print(f"Position {i}/{len(positions)}: {pos_name}")
        print(f"{'='*70}")
        
        # Move to position
        print(f"Moving to position...")
        if not move_to_position_safe(robot, target_pose, joint_weights):
            print(f"❌ Failed to move to position {pos_name}")
            failed_captures += 1
            continue
        
        # Get actual robot position after movement
        actual_q = robot.get_q()
        print(f"Actual joint configuration: {actual_q}")
        
        # Calculate transformation matrix
        transformation_matrix = robot.fk(actual_q)
        print(f"Transformation matrix calculated")
        
        # Capture image
        print(f"📸 Capturing image...")
        try:
            image = robot.grab_image()
            if image is None:
                print(f"❌ Failed to capture image at position {pos_name}")
                failed_captures += 1
                continue
            print(f"✅ Image captured: {image.shape}")
        except Exception as e:
            print(f"❌ Error capturing image: {e}")
            failed_captures += 1
            continue
        
        # Save data
        print(f"💾 Saving data...")
        try:
            yaml_file, png_file = uloz_data(
                actual_q, 
                transformation_matrix, 
                image,
                folder=output_folder,
                prefix=f"calib_{pos_name}"
            )
            print(f"✅ Data saved successfully")
            successful_captures += 1
        except Exception as e:
            print(f"❌ Error saving data: {e}")
            failed_captures += 1
            continue
        
        print()
    
    # Summary
    print("="*70)
    print("  CALIBRATION SUMMARY")
    print("="*70)
    print(f"Total positions: {len(positions)}")
    print(f"✅ Successful captures: {successful_captures}")
    print(f"❌ Failed captures: {failed_captures}")
    print(f"Output folder: {output_folder}")
    print()
    
    # Release robot
    print("🤖 Releasing robot...")
    try:
        robot.release()
        print("✅ Robot released successfully")
    except Exception as e:
        print(f"⚠️  Warning during robot release: {e}")
    
    print()
    print("="*70)
    print("  CALIBRATION COMPLETE")
    print("="*70)


if __name__ == "__main__":
    main()
