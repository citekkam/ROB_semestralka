"""
Testing utilities for IKT6 Python library
Port from test.h
"""

import numpy as np
import time
from dataclasses import dataclass
from typing import Tuple
import sys
import os

# Add parent directories to path
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
grandparent_dir = os.path.dirname(parent_dir)
sys.path.insert(0, grandparent_dir)

from ikt6_python import Robot, ikt6_dkt, ikt6_ikt


@dataclass
class TestStats:
    """Statistics from kinematics testing"""
    total_tested: int = 0
    total_solutions: int = 0
    correct: int = 0
    incorrect: int = 0
    nosolution: int = 0
    dkt_time_ms: float = 0.0
    ikt_time_ms: float = 0.0


def check_limits(robot: Robot, J: np.ndarray) -> bool:
    """
    Check if joint angles are within robot limits
    
    Args:
        robot: Robot parameters
        J: Joint angles (6 elements)
    
    Returns:
        True if all joints are within limits
    """
    return np.all(J > robot.limits_min) and np.all(J < robot.limits_max)


def run_test(
    robot: Robot,
    zerocheck: np.ndarray,
    zeroset: np.ndarray,
    ntests: int = 1000,
    debug: bool = False,
    eps: float = 1e-6
) -> TestStats:
    """
    Run comprehensive kinematics test
    
    Tests the round-trip: J -> DKT -> P -> IKT -> J' -> DKT -> P'
    and verifies that P ≈ P'
    
    Args:
        robot: Robot parameters
        zerocheck: Which joints to check for zero (singularity)
        zeroset: Which joints to force to zero
        ntests: Number of random configurations to test
        debug: Print detailed failure messages
        eps: Tolerance for comparisons
    
    Returns:
        TestStats object with results
    """
    stats = TestStats()
    
    for _ in range(ntests * 10):  # Try more to account for rejections
        if stats.total_tested >= ntests:
            break
        
        # Generate random joint angles within limits
        J_in = np.zeros(6)
        for j in range(6):
            J_in[j] = ((np.random.rand() - 0.5) * 
                      (robot.limits_max[j] - robot.limits_min[j]))
        
        # Check for singularities or set purposefully
        for j in range(6):
            if zerocheck[j] == 1 and abs(J_in[3]) < eps:
                continue
            if zeroset[j] == 1:
                J_in[j] = 0
        
        # Check limits
        if not check_limits(robot, J_in):
            continue
        
        stats.total_tested += 1
        
        # Direct kinematics
        t0 = time.perf_counter()
        P_in = ikt6_dkt(robot, J_in)
        t1 = time.perf_counter()
        stats.dkt_time_ms += (t1 - t0) * 1000
        
        # Inverse kinematics
        t0 = time.perf_counter()
        J_out = ikt6_ikt(robot, P=P_in)
        t1 = time.perf_counter()
        stats.ikt_time_ms += (t1 - t0) * 1000
        
        # Count valid solutions
        n_solutions = 0
        for i in range(J_out.shape[1]):
            if not np.any(np.isnan(J_out[:, i])):
                n_solutions += 1
        
        if n_solutions == 0:
            stats.nosolution += 1
            if debug:
                print("!!! No solution !!!")
                print(f"J_in  = {J_in}")
                print(f"P_in  = {P_in}")
            continue
        
        stats.total_solutions += n_solutions
        
        # Check each solution
        for j in range(J_out.shape[1]):
            if np.any(np.isnan(J_out[:, j])):
                continue
            
            # Verify solution
            P_out = ikt6_dkt(robot, J_out[:, j])
            error = np.linalg.norm(P_in - P_out)
            
            if error < eps:
                stats.correct += 1
            else:
                stats.incorrect += 1
                if debug:
                    print(f"!!! Incorrect solution (error={error:.6e}) !!!")
                    print(f"J_in  = {J_in}")
                    print(f"P_in  = {P_in}")
                    print(f"J_out = {J_out[:, j]}")
                    print(f"P_out = {P_out}")
    
    return stats


def print_test_stats(stats: TestStats):
    """Print formatted test statistics"""
    print(f"Total tested configurations                {stats.total_tested:8d}")
    print(f"Total solutions                            {stats.total_solutions:8d}")
    print(f"No solution count                          {stats.nosolution:8d}")
    print(f"Correct count                              {stats.correct:8d}")
    print(f"Incorrect count                            {stats.incorrect:8d}")
    if stats.total_solutions > 0:
        success_rate = stats.correct / stats.total_solutions * 100.0
        print(f"Successfulness:                            {success_rate:9.2f} %")
    print(f"Total dkt time:                            {stats.dkt_time_ms:9.4f} ms")
    if stats.total_tested > 0:
        avg_dkt = stats.dkt_time_ms / stats.total_tested
        print(f"Single dkt time (average):                 {avg_dkt:9.4f} ms")
    print(f"Total ikt time:                            {stats.ikt_time_ms:9.4f} ms")
    if stats.total_tested > 0:
        avg_ikt = stats.ikt_time_ms / stats.total_tested
        print(f"Single ikt time (average):                 {avg_ikt:9.4f} ms")
