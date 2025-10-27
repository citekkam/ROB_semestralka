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

from numpy.typing import ArrayLike

# Add project root to sys.path before importing local packages
ROOT_DIR = Path(__file__).resolve().parent.parent
SRC_DIR = ROOT_DIR / "src"
if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))
if SRC_DIR.exists() and str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

# # Correct module imports
# from test_ikt6.ikt6_python.robot import ikt6_robot_init
# from test_ikt6.ikt6_python.kinematics import ikt6_ikt, ikt6_dkt_T, ikt6_dkt

from ctu_crs import CRS93
from select_shortest_path import find_shortest_path
from camera.dataset_creator import uloz_data

prev_pose = None  # Global variable to store previous pose for test function


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
    
    # Sort keys numerically by extracting the position number
    position_keys = [key for key in data.keys() if 'position_' in key]
    sorted_keys = sorted(position_keys, key=lambda x: int(x.split('_')[1]))
    
    for key in sorted_keys:
        pose = np.array(data[key]['pose'])
        positions.append((key, pose))
    
    return positions

def move_to_position(robot, target_pose: np.ndarray, joint_weights: np.ndarray = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])) -> bool:
    """
    Safely move robot to target position using IK and shortest path selection.
    
    Args:
        robot: Robot instance (CRS93)
        target_pose: Target pose [x, y, z, roll, pitch, yaw]
        joint_weights: Weights for distance calculation
    
    Returns:
        bool: True if movement successful, False otherwise
    """
    # Get current robot position
    current_q = robot.get_q()
    
    # Convert pose to transformation matrix
    target_T = pose_to_transformation_matrix(target_pose)

    print(target_T)
    
    print(f"  Target pose [x,y,z,r,p,y]: [{target_pose[0]:.3f}, {target_pose[1]:.3f}, {target_pose[2]:.3f}, {target_pose[3]:.3f}, {target_pose[4]:.3f}, {target_pose[5]:.3f}]")
    
    # Get all IK solutions
    ik_solutions = robot.ik(target_T)
    
    if len(ik_solutions) == 0:
        print(f"  ⚠️  No IK solutions found for target pose")
        return False
    
    print(f"  Found {len(ik_solutions)} IK solutions")
    
    # Find shortest path
    sorted_distances = find_shortest_path(current_q, ik_solutions, joint_weights)
    
    # # Select the closest solution
    # idx, distance, _ = sorted_distances[0]
    # best_q = ik_solutions[idx]
    
    # print(f"  → Selected IK solution {idx} with distance {distance:.4f}")
    # print(f"  → Target joints: {best_q}")

    for idx,distance, _ in sorted_distances:
        q = ik_solutions[idx]
        # check robot limit
        if robot.in_limits(q):
            # Move to position
            print("q is:", q)
            robot.move_to_q(q)
            print("start moving")
            robot.wait_for_motion_stop()
            print("stop moving")
            break
        else:
            print("robot limit are false ty debile!!!!!!!!!!!!", idx)
                
    return True

def move_to_pos_T(robot, target_pose_T: np.ndarray, joint_weights: np.ndarray = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])) -> bool:
    """
    Safely move robot to target position using IK and shortest path selection.
    
    Args:
        robot: Robot instance (CRS93)
        target_pose: Target pose [x, y, z, roll, pitch, yaw]
        joint_weights: Weights for distance calculation
    
    Returns:
        bool: True if movement successful, False otherwise
    """
    # Get current robot position
    current_q = robot.get_q()
    
    # Convert pose to transformation matrix
    target_T = target_pose_T

    print(target_T)
    
    # Get all IK solutions
    ik_solutions = robot.ik(target_T)
    
    if len(ik_solutions) == 0:
        print(f"  ⚠️  No IK solutions found for target pose")
        return False
    
    print(f"  Found {len(ik_solutions)} IK solutions")
    
    # Find shortest path
    sorted_distances = find_shortest_path(current_q, ik_solutions, joint_weights)

    for idx,distance, _ in sorted_distances:
        q = ik_solutions[idx]
        # check robot limit
        if robot.in_limits(q):
            # Move to position
            print("q is:", q)
            robot.move_to_q(q)
            print("start moving")
            robot.wait_for_motion_stop()
            print("stop moving")
            break
        else:
            print("robot limit are false ty debile!!!!!!!!!!!!", idx)
                
    return True

def robot_calibration(soft_home=True) -> tuple:
    """
    Robot calibration function that collects images and transformation matrices.
    
    Returns:
        tuple: (imgs, transformation_matrixes) where:
               - imgs: list of images captured at each calibration position
               - transformation_matrixes: list of 4x4 transformation matrices
               Returns ([], []) if calibration fails.
    """
    print("="*70)
    print("  ROBOT CALIBRATION")
    print("="*70)
    print()
    
    # Setup paths
    script_dir = Path(__file__).parent
    positions_file = script_dir / "robot_calibration" / "calibration_positions.yaml"
    
    # Configuration
    joint_weights = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
    
    # Load calibration positions
    print(f"📂 Loading calibration positions from: {positions_file}")
    try:
        positions = load_calibration_positions(str(positions_file))
        print(f"✅ Loaded {len(positions)} calibration positions")
        print()
    except Exception as e:
        print(f"❌ Failed to load positions: {e}")
        return [], []
    
    # Initialize robot
    print("🤖 Initializing robot...")
    try:
        robot = CRS93()
        if soft_home:
            robot.initialize(home=False)
            robot.soft_home()
        else:
            robot.initialize()
        print("✅ Robot initialized successfully")
        print()
    except Exception as e:
        print(f"❌ Failed to initialize robot: {e}")
        return [], []
    
    # Arrays to store images and transformation matrices separately
    imgs = []
    transformation_matrixes = []
    
    # Process each calibration position
    successful_captures = 0
    failed_captures = 0
    
    for i, (pos_name, target_pose) in enumerate(positions, 1):
        print(f"{'='*70}")
        print(f"Position {i}/{len(positions)}: {pos_name}")
        print(f"{'='*70}")
        
        # Move to position
        print(f"Moving to position...")
        if not move_to_position(robot, target_pose, joint_weights):
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
        
        # Append image and transformation_matrix to separate arrays
        imgs.append(image)
        transformation_matrixes.append(transformation_matrix)
        print(f"✅ Data added to calibration arrays (entry {len(imgs)})")
        successful_captures += 1
        print()
    
    # Summary
    print("="*70)
    print("  CALIBRATION SUMMARY")
    print("="*70)
    print(f"Total positions: {len(positions)}")
    print(f"✅ Successful captures: {successful_captures}")
    print(f"❌ Failed captures: {failed_captures}")
    print(f"📊 Images collected: {len(imgs)}")
    print(f"📊 Transformation matrices collected: {len(transformation_matrixes)}")
    print()
    
    # # Release robot
    # print("🤖 Releasing robot...")
    # try:
    #     robot.release()
    #     print("✅ Robot released successfully")
    # except Exception as e:
    #     print(f"⚠️  Warning during robot release: {e}")
    
    print()
    print("="*70)
    print("  CALIBRATION COMPLETE")
    print("="*70)
    
    return (imgs, transformation_matrixes)

def main():
    """Main calibration automation function."""
    
    print("="*70)
    print("  ROBOT CALIBRATION AUTOMATION")
    print("="*70)
    print()
    
    # Setup paths
    script_dir = Path(__file__).parent
    positions_file = script_dir/ "robot_calibration" / "calibration_positions.yaml"
    output_folder = Path(script_dir / "calibration_data")
    
    # Clear old data from calibration_data folder
    if output_folder.exists():
        print(f"🧹 Cleaning old calibration data from: {output_folder}")
        # Remove all files in the folder
        for file in output_folder.glob("*"):
            if file.is_file():
                file.unlink()
                print(f"   Removed: {file.name}")
        print(f"✅ Old data cleared")
        print()
    
    # Create output folder if it doesn't exist
    output_folder.mkdir(exist_ok=True, parents=True)
    output_folder = str(output_folder)  # Convert back to string for compatibility
    
    # Configuration
    joint_weights = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
    
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
        # Define robot parameters - CRS93
        robot = CRS93()
        robot.initialize(home=False)
        robot.soft_home()

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
        if not move_to_position(robot, target_pose, joint_weights):
            print(f"❌ Failed to move to position {pos_name}")
            failed_captures += 1
            continue
        
        # Get actual robot position after movement
        # test
        actual_q = robot.get_q()
        print(f"Actual joint configuration: {actual_q}")
        # actual_q = prev_pose  # Use prev_pose from test function
        
        # Calculate transformation matrix
        # test
        transformation_matrix = robot.fk(actual_q)
        print(f"Transformation matrix calculated")
        # transformation_matrix = ikt6_dkt_T(robot, actual_q)
        
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
    # main()

    imgs, pos = robot_calibration()
