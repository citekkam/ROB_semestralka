#!/usr/bin/env python3
"""
Single Position Test
====================
Test moving to a single position and capturing data.
Useful for debugging and verifying setup before full calibration run.

Author: David
Date: 2025-10-18
"""

import numpy as np
from pathlib import Path
import sys

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from ctu_crs import CRS93
from select_shortest_path import find_shortest_path
from camera.dataset_creator import uloz_data


def pose_to_transformation_matrix(pose: np.ndarray) -> np.ndarray:
    """Convert pose [x, y, z, roll, pitch, yaw] to 4x4 transformation matrix."""
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


def main():
    """Test single position capture."""
    
    print("="*70)
    print("  SINGLE POSITION TEST")
    print("="*70)
    print()
    
    # Test pose (center position from calibration grid)
    # [x, y, z, roll, pitch, yaw]
    test_pose = np.array([0.40, 0.00, 0.20, 0.0, -1.57, 0.0])
    joint_weights = np.array([1.0, 1.0, 1.0, 10.0, 1.0, 10.0])
    
    print(f"Test pose [x,y,z,r,p,y]: {test_pose}")
    print(f"  Position: x={test_pose[0]:.3f}m, y={test_pose[1]:.3f}m, z={test_pose[2]:.3f}m")
    print(f"  Orientation: roll={test_pose[3]:.3f}rad, pitch={test_pose[4]:.3f}rad, yaw={test_pose[5]:.3f}rad")
    print()
    
    # Initialize robot
    print("🤖 Initializing robot...")
    robot = CRS93()
    robot.initialize(home=False)
    print("✅ Robot initialized")
    print()
    
    # Get current position
    current_q = robot.get_q()
    print(f"Current position: {current_q}")
    print()
    
    # Calculate target pose
    print("Converting pose to transformation matrix...")
    target_T = pose_to_transformation_matrix(test_pose)
    print(f"Transformation matrix:")
    print(target_T)
    print()
    
    # Get IK solutions
    print("Computing IK solutions...")
    ik_solutions = robot.ik(target_T)
    print(f"Found {len(ik_solutions)} IK solutions")
    
    if len(ik_solutions) == 0:
        print("❌  No IK solutions found for this pose")
        robot.release()
        return
    else:
        # Find best solution
        sorted_distances = find_shortest_path(current_q, ik_solutions, joint_weights)
        idx, distance, _ = sorted_distances[0]
        best_q = ik_solutions[idx]
        
        print(f"Selected solution {idx} with distance {distance:.4f}")
        print(f"Target q: {best_q}")
        print()
        
        # Move to position
        print("Moving robot...")
        robot.move_to_q(best_q)
    
    robot.wait_for_motion_stop()
    print("✅ Movement complete")
    print()
    
    # Get actual position
    actual_q = robot.get_q()
    print(f"Actual position: {actual_q}")
    print()
    
    # Calculate transformation
    transformation = robot.fk(actual_q)
    print("Transformation matrix:")
    print(transformation)
    print()
    
    # Capture image
    print("📸 Capturing image...")
    image = robot.grab_image()
    
    if image is not None:
        print(f"✅ Image captured: {image.shape}")
        print()
        
        # Save data
        print("💾 Saving test data...")
        output_folder = str(Path(__file__).parent / "test_output")
        yaml_file, png_file = uloz_data(
            actual_q,
            transformation,
            image,
            folder=output_folder,
            prefix="test"
        )
        print(f"✅ Test data saved")
    else:
        print("❌ Failed to capture image")
    
    print()
    
    # Release robot
    print("🤖 Releasing robot...")
    robot.release()
    print("✅ Robot released")
    
    print()
    print("="*70)
    print("  TEST COMPLETE")
    print("="*70)


if __name__ == "__main__":
    main()
