"""
Custom robot test example - Python version
Equivalent to my_custom_test.cpp
"""

import numpy as np
import sys
import os

# Add parent directories to path
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
grandparent_dir = os.path.dirname(parent_dir)
sys.path.insert(0, grandparent_dir)

from ikt6_python import Robot, ikt6_robot_init, ikt6_dkt, ikt6_dkt_T, ikt6_ikt
from ikt6_python.tests.test_utils import run_test, print_test_stats


def main():
    print("=" * 80)
    print("Custom Robot Test Example - Python Version")
    print("=" * 80)
    
    # Define robot parameters - CRS93
    lengths = np.array([440, 0, 305, 0, 330, 211], dtype=float)  # Link lengths in mm
    offsets = np.array([0, 0, 0, 0, 0, 0], dtype=float)  # Joint offsets in radians
    directions = np.array([1, -1, -1, 1, -1, 1], dtype=float)  # Joint directions
    limits_max = np.deg2rad([175, 90, 110, 180, 105, 180])  # Max limits
    limits_min = np.deg2rad([-175, -90, -110, -180, -105, -180])  # Min limits
    
    # Define base and tool transforms (identity for now)
    base = np.eye(4)
    tool = np.eye(4)
    
    # Initialize robot
    robot = ikt6_robot_init(
        name="CRS93",
        lengths=lengths,
        offsets=offsets,
        directions=directions,
        limits_max=limits_max,
        limits_min=limits_min,
        base=base,
        tool=tool
    )
    
    print(f"\nRobot initialized: {robot.name}")
    print(f"Link lengths: {robot.lengths}")
    
    # Test 1: Normal operation (no singularities)
    print("\n=== Test 1: Normal Operation ===")
    zerocheck1 = np.array([0, 0, 0, 0, 0, 0])
    zeroset1 = np.array([0, 0, 0, 0, 0, 0])
    
    ntests = 1000
    debug = False
    
    stats1 = run_test(robot, zerocheck1, zeroset1, ntests, debug)
    print_test_stats(stats1)
    
    # Test 2: With J5 singularity
    print("\n=== Test 2: J5 Singularity (J5 = 0) ===")
    zerocheck2 = np.array([0, 0, 0, 0, 1, 0])
    zeroset2 = np.array([0, 0, 0, 0, 1, 0])
    
    stats2 = run_test(robot, zerocheck2, zeroset2, ntests, debug)
    print_test_stats(stats2)
    
    # Test 3: Manual single test with known values
    print("\n=== Test 3: Manual Single Configuration ===")
    
    J_test = np.array([1.28114148e-01, -1.05680035e+00, -1.94032616e+00, 
                       -6.65644384e-03, -1.38230077e-01, 1.34653016e-01])
    print(f"Input joints: {J_test}")
    
    # Forward kinematics
    P_test = ikt6_dkt(robot, J_test)
    print(f"Forward kinematics result (x,y,z,r,p,y): {P_test}")
    
    T_test = ikt6_dkt_T(robot, J_test)
    print(f"Forward kinematics transformation matrix T:")
    print(T_test)
    
    # Inverse kinematics
    P_test_mod = P_test.copy()
    P_test_mod[1] -= 100  # Slightly perturb to avoid singularity issues
    
    J_solutions = ikt6_ikt(robot, P=P_test_mod)
    
    # Count valid solutions
    n_valid = 0
    for i in range(J_solutions.shape[1]):
        if not np.any(np.isnan(J_solutions[:, i])):
            n_valid += 1
    
    print(f"Number of IK solutions found: {n_valid}")
    
    # Verify each solution
    for i in range(J_solutions.shape[1]):
        if not np.any(np.isnan(J_solutions[:, i])):
            P_verify = ikt6_dkt(robot, J_solutions[:, i])
            error = np.linalg.norm(P_test_mod - P_verify)
            print(f"Solution {i+1}: {J_solutions[:, i]} (error: {error:.6e})")
    
    print("\n" + "=" * 80)
    print("Tests completed!")
    print("=" * 80)


if __name__ == "__main__":
    main()
