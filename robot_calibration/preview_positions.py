#!/usr/bin/env python3
"""
Calibration Position Preview
=============================
This script previews the calibration positions without moving the robot.
Use this to verify positions before running the actual calibration.

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


def load_calibration_positions(yaml_file: str) -> list:
    """Load calibration positions from YAML file."""
    with open(yaml_file, 'r') as f:
        data = yaml.safe_load(f)
    
    positions = []
    for key in sorted(data.keys()):
        if 'position_' in key:
            pose = np.array(data[key]['pose'])
            positions.append((key, pose))
    
    return positions


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


def check_pose_reachable(robot, pose: np.ndarray) -> tuple:
    """Check if pose is reachable and get IK solutions."""
    try:
        T = pose_to_transformation_matrix(pose)
        ik_solutions = robot.ik(T)
        return True, ik_solutions
    except Exception as e:
        return False, []


def main():
    """Preview calibration positions."""
    
    print("="*70)
    print("  CALIBRATION POSITION PREVIEW")
    print("="*70)
    print()
    
    # Setup paths
    script_dir = Path(__file__).parent
    positions_file = script_dir / "calibration_positions.yaml"
    
    # Load calibration positions
    print(f"📂 Loading calibration positions from: {positions_file}")
    try:
        positions = load_calibration_positions(str(positions_file))
        print(f"✅ Loaded {len(positions)} calibration positions")
        print()
    except Exception as e:
        print(f"❌ Failed to load positions: {e}")
        return
    
    # Initialize robot for limits checking (no connection needed)
    robot = CRS93(tty_dev=None)
    
    print("="*70)
    print("  POSITION DETAILS")
    print("="*70)
    print()
    
    valid_positions = 0
    invalid_positions = 0
    
    for i, (pos_name, pose) in enumerate(positions, 1):
        print(f"Position {i}: {pos_name}")
        print(f"  Pose [x,y,z,r,p,y]: [{pose[0]:.3f}, {pose[1]:.3f}, {pose[2]:.3f}, {pose[3]:.3f}, {pose[4]:.3f}, {pose[5]:.3f}]")
        print(f"  Position [m]: x={pose[0]:.3f}, y={pose[1]:.3f}, z={pose[2]:.3f}")
        print(f"  Orientation [rad]: roll={pose[3]:.3f}, pitch={pose[4]:.3f}, yaw={pose[5]:.3f}")
        
        # Check if reachable
        reachable, ik_solutions = check_pose_reachable(robot, pose)
        
        if reachable and len(ik_solutions) > 0:
            print(f"  ✅ Pose is reachable ({len(ik_solutions)} IK solutions found)")
            valid_positions += 1
            
            # Show first solution
            print(f"  Example joint config: {ik_solutions[0]}")
        else:
            print(f"  ❌ Pose is NOT reachable (no IK solutions found)")
            invalid_positions += 1
        
        print()
    
    print("="*70)
    print("  SUMMARY")
    print("="*70)
    print(f"Total positions: {len(positions)}")
    print(f"✅ Valid positions: {valid_positions}")
    print(f"❌ Invalid positions: {invalid_positions}")
    
    if invalid_positions > 0:
        print()
        print("⚠️  WARNING: Some positions are not reachable!")
        print("   Please adjust calibration_positions.yaml before running calibration.")
    else:
        print()
        print("✅ All positions are valid and ready for calibration!")
        print("   Run calibration_automation.py to start the calibration process.")
    
    print()
    print("="*70)


if __name__ == "__main__":
    main()
