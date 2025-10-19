"""
Kinematics functions - Direct (Forward) and Inverse Kinematics
Direct port from ikt6.cpp
"""

import numpy as np
from numpy.typing import NDArray
from typing import List
from .robot import Robot
from .util import (
    mtx_dh, mtx_translate, mtx_rotate_x, mtx_rotate_y, mtx_rotate_z,
    mtx_to_angles, pmp, sgn, EPS
)


def apply_offset_and_direction(robot: Robot, J: NDArray) -> NDArray:
    """
    Apply joint offsets and directions to joint angles
    
    Args:
        robot: Robot parameters
        J: Joint angles (6 elements)
    
    Returns:
        Modified joint angles
    """
    J_new = np.zeros(6)
    for i in range(6):
        J_new[i] = (J[i] - robot.offsets[i]) * robot.directions[i]
    return J_new


def remove_offset_and_direction(robot: Robot, J: NDArray) -> NDArray:
    """
    Remove joint offsets and directions from joint angles
    
    Args:
        robot: Robot parameters
        J: Joint angles (6 elements)
    
    Returns:
        Original joint angles
    """
    J_new = np.zeros(6)
    for i in range(6):
        J_new[i] = J[i] * robot.directions[i] + robot.offsets[i]
    return J_new


def test_limits(robot: Robot, j: float, i: int) -> bool:
    """
    Test if a joint angle is within limits
    
    Args:
        robot: Robot parameters
        j: Joint angle (with offset and direction applied)
        i: Joint index (0-5)
    
    Returns:
        True if within limits
    """
    j = j * robot.directions[i] + robot.offsets[i]
    return robot.limits_min[i] < j < robot.limits_max[i]


def ikt6_dkt_T(robot: Robot, J: NDArray) -> NDArray:
    """
    Direct (Forward) kinematics - returns transformation matrix
    
    Args:
        robot: Robot parameters
        J: Joint angles [j1, j2, j3, j4, j5, j6] in radians
    
    Returns:
        4x4 homogeneous transformation matrix from base to tool
    """
    T = robot.base.copy()
    
    # Apply offsets and direction changes
    J = apply_offset_and_direction(robot, J)
    
    # Forward kinematics chain
    for i in range(6):
        T = T @ mtx_dh(robot.DH[:, i] + (J[i] * robot.DH_Param[:, i]))
    
    T = T @ robot.tool
    
    return T


def ikt6_dkt(robot: Robot, J: NDArray) -> NDArray:
    """
    Direct (Forward) kinematics - returns position and orientation
    
    Args:
        robot: Robot parameters
        J: Joint angles [j1, j2, j3, j4, j5, j6] in radians
    
    Returns:
        6D vector [x, y, z, roll, pitch, yaw]
    """
    T = ikt6_dkt_T(robot, J)
    
    # Extract position
    t = T @ np.array([0, 0, 0, 1])
    position = t[:3]
    
    # Extract orientation (Euler angles)
    angles = mtx_to_angles(T)
    
    return np.concatenate([position, angles])


def solve_two_circles(
    c1: NDArray, r1: float,
    c2: NDArray, r2: float,
    theta1: float
) -> NDArray:
    """
    Solve intersection of two circles in 2D plane
    
    Args:
        c1: Center of first circle [x, y]
        r1: Radius of first circle
        c2: Center of second circle [x, y]
        r2: Radius of second circle
        theta1: Theta1 value to include in solution
    
    Returns:
        5x2 matrix with solutions [x, y, theta1, c2_x, c2_y]
    """
    s = np.full((5, 2), np.nan)
    
    # Distance between circle centers
    a = np.linalg.norm(c2 - c1)
    
    # Calculate intersection points
    x = (r1**2 + a**2 - r2**2) / (2 * a)
    sqr_y = r1**2 - x**2
    
    if abs(sqr_y) < EPS:
        # One solution (circles touch)
        phi = np.arctan2(c2[1] - c1[1], c2[0] - c1[0])
        s[:, 0] = [c1[0] + np.cos(phi)*x, c1[1] + np.sin(phi)*x, theta1, c2[0], c2[1]]
        s[:, 1] = [c1[0] + np.cos(phi)*x, c1[1] + np.sin(phi)*x, theta1, c2[0], c2[1]]
    elif sqr_y > 0:
        # Two solutions
        y = np.sqrt(sqr_y)
        phi = np.arctan2(c2[1] - c1[1], c2[0] - c1[0])
        
        s[:, 0] = [
            c1[0] + np.cos(phi)*x - np.sin(phi)*y,
            c1[1] + np.sin(phi)*x + np.cos(phi)*y,
            theta1, c2[0], c2[1]
        ]
        s[:, 1] = [
            c1[0] + np.cos(phi)*x + np.sin(phi)*y,
            c1[1] + np.sin(phi)*x - np.cos(phi)*y,
            theta1, c2[0], c2[1]
        ]
    
    return s


def ikt6_ikt(robot: Robot, P: NDArray = None, T: NDArray = None) -> NDArray:
    """
    Inverse kinematics
    
    Args:
        robot: Robot parameters
        P: Target pose [x, y, z, roll, pitch, yaw] (optional if T is given)
        T: Target transformation matrix 4x4 (optional if P is given)
    
    Returns:
        6x8 matrix with up to 8 IK solutions (columns with NaN are invalid)
    """
    # Convert P to T if needed
    if T is None and P is not None:
        T = (mtx_translate(P[:3]) @ 
             mtx_rotate_z(P[5]) @ 
             mtx_rotate_y(P[4]) @ 
             mtx_rotate_x(P[3]))
    elif T is None:
        raise ValueError("Either P or T must be provided")
    
    # Initialize solution matrix
    J = np.full((6, 8), np.nan)
    
    # Remove tool and base transformations
    T = T @ np.linalg.inv(robot.tool)
    T = T @ np.linalg.inv(robot.A76)
    T = np.linalg.inv(robot.base) @ T
    
    W = T @ np.array([0, 0, 0, 1])
    
    # Calculate theta1 solutions
    theta1a = np.arctan2(W[1], W[0])
    theta1b = theta1a + np.pi
    if theta1b > np.pi:
        theta1b = theta1b - 2 * np.pi
    
    # Two circles problem for theta2 and theta3
    r1 = robot.lengths[2]
    r2 = np.sqrt(robot.lengths[4]**2 + robot.lengths[3]**2)
    
    c1 = np.array([robot.lengths[1], 0])
    absW = np.sqrt(W[0]**2 + W[1]**2)
    
    c2a = np.array([absW, W[2] - robot.lengths[0]])
    c2b = np.array([-absW, W[2] - robot.lengths[0]])
    
    s1 = solve_two_circles(c1, r1, c2a, r2, theta1a)
    s2 = solve_two_circles(c1, r1, c2b, r2, theta1b)
    
    s = np.hstack([s1, s2])
    
    # If no solution, return empty
    if s.shape[1] == 0:
        return J[:, :0]
    
    # Calculate first three joints
    J3 = np.full((3, 4), np.nan)
    
    for i in range(s.shape[1]):
        if np.any(np.isnan(s[:, i])):
            continue
        
        theta1 = s[2, i]
        theta2 = np.arctan2(s[0, i] - c1[0], s[1, i] - c1[1])
        theta3 = (np.arctan2(s[3, i] - s[0, i], s[4, i] - s[1, i]) - 
                  theta2 + 
                  np.arctan2(robot.lengths[3], robot.lengths[4]))
        
        # Normalize to (-π, π]
        theta3 = pmp(theta3)
        
        # Check limits
        if (test_limits(robot, theta1, 0) and
            test_limits(robot, theta2, 1) and
            test_limits(robot, theta3, 2)):
            J3[:, i] = [theta1, theta2, theta3]
    
    # Calculate joints 4, 5, 6 for each solution
    n_sol_J = 0
    for i in range(J3.shape[1]):
        if np.any(np.isnan(J3[:, i])):
            n_sol_J += 2
            continue
        
        M = robot.base.copy()
        for j in range(3):
            M = M @ mtx_dh(robot.DH[:, j] + (J3[j, i] * robot.DH_Param[:, j]))
        
        mtx = np.linalg.inv(M) @ T
        c5 = mtx[2, 2]
        
        # Case: c5 ≈ 1 (singularity)
        if c5 > (1 - EPS):
            theta4 = 0
            theta5 = 0
            theta6 = np.arctan2(-mtx[0, 1], mtx[1, 1]) - np.pi
            theta6 = pmp(theta6)
            
            if test_limits(robot, theta6, 5):
                J[:, n_sol_J] = np.concatenate([J3[:, i], [theta4, theta5, theta6]])
                J[:, n_sol_J+1] = np.concatenate([J3[:, i], [theta4, theta5, theta6]])
            
            n_sol_J += 2
        
        # Case: c5 ≈ -1 (no solution)
        elif c5 < (-1 + EPS):
            n_sol_J += 2
            continue
        
        # General case: two solutions
        else:
            theta5a = np.arccos(c5)
            theta5b = -theta5a
            
            sg5a = sgn(np.sin(theta5a))
            sg5b = -sg5a
            
            # Solution A
            theta4a = np.arctan2(-mtx[1, 2] * sg5a, -mtx[0, 2] * sg5a)
            theta6a = np.arctan2(-mtx[2, 1] * sg5a, mtx[2, 0] * sg5a) - np.pi
            theta6a = pmp(theta6a)
            
            if (test_limits(robot, theta4a, 3) and
                test_limits(robot, theta5a, 4) and
                test_limits(robot, theta6a, 5)):
                J[:, n_sol_J] = np.concatenate([J3[:, i], [theta4a, theta5a, theta6a]])
            n_sol_J += 1
            
            # Solution B
            theta4b = np.arctan2(-mtx[1, 2] * sg5b, -mtx[0, 2] * sg5b)
            theta6b = np.arctan2(-mtx[2, 1] * sg5b, mtx[2, 0] * sg5b) - np.pi
            theta6b = pmp(theta6b)
            
            if (test_limits(robot, theta4b, 3) and
                test_limits(robot, theta5b, 4) and
                test_limits(robot, theta6b, 5)):
                J[:, n_sol_J] = np.concatenate([J3[:, i], [theta4b, theta5b, theta6b]])
            n_sol_J += 1
    
    # Remove offsets and directions
    for i in range(8):
        if not np.any(np.isnan(J[:, i])):
            J[:, i] = remove_offset_and_direction(robot, J[:, i])
    
    return J
