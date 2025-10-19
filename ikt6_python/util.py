"""
Utility functions for IKT6 library
Direct port from util.cpp
"""

import numpy as np
from numpy.typing import NDArray

# Epsilon for floating point comparisons
EPS = np.finfo(float).eps * 10000


def sgn(val: float) -> int:
    """Sign function"""
    return int((0.0 < val)) - int((val < 0.0))


def pmp(angle: float) -> float:
    """
    Plus minus PI - normalizes angle to range (-π, π]
    """
    if angle <= -np.pi:
        angle = angle + 2 * np.pi
    
    if angle > np.pi:
        angle = angle - 2 * np.pi
    
    return angle


def mtx_translate(t: NDArray) -> NDArray:
    """
    4x4 transformation matrix representing translation
    
    Args:
        t: 3D translation vector [x, y, z]
    
    Returns:
        4x4 homogeneous transformation matrix
    """
    T = np.eye(4)
    T[:3, 3] = t
    return T


def mtx_rotate_x(angle: float) -> NDArray:
    """
    4x4 transformation matrix representing rotation around X axis
    
    Args:
        angle: Rotation angle in radians
    
    Returns:
        4x4 homogeneous transformation matrix
    """
    c = np.cos(angle)
    s = np.sin(angle)
    R = np.eye(4)
    R[1:3, 1:3] = np.array([[c, -s],
                             [s,  c]])
    return R


def mtx_rotate_y(angle: float) -> NDArray:
    """
    4x4 transformation matrix representing rotation around Y axis
    
    Args:
        angle: Rotation angle in radians
    
    Returns:
        4x4 homogeneous transformation matrix
    """
    c = np.cos(angle)
    s = np.sin(angle)
    R = np.eye(4)
    R[0, 0] = c
    R[0, 2] = s
    R[2, 0] = -s
    R[2, 2] = c
    return R


def mtx_rotate_z(angle: float) -> NDArray:
    """
    4x4 transformation matrix representing rotation around Z axis
    
    Args:
        angle: Rotation angle in radians
    
    Returns:
        4x4 homogeneous transformation matrix
    """
    c = np.cos(angle)
    s = np.sin(angle)
    R = np.eye(4)
    R[:2, :2] = np.array([[c, -s],
                          [s,  c]])
    return R


def mtx_dh(dhcol: NDArray) -> NDArray:
    """
    4x4 transformation matrix from DH parameters
    
    Args:
        dhcol: DH parameters [alpha, a, theta, d]
    
    Returns:
        4x4 homogeneous transformation matrix
    """
    alpha = dhcol[0]
    a = dhcol[1]
    theta = dhcol[2]
    d = dhcol[3]
    
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    
    T = np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,   sa,       ca,      d],
        [0,   0,        0,       1]
    ])
    
    return T


def mtx_to_angles(T: NDArray) -> NDArray:
    """
    Convert rotation matrix to Euler angles (XYZ convention)
    
    Args:
        T: 4x4 homogeneous transformation matrix
    
    Returns:
        3D vector of Euler angles [roll, pitch, yaw]
    """
    s2 = -T[2, 0]
    
    if abs(s2 - 1) < EPS:
        A = 0
        B = np.pi / 2
        C = np.arctan2(T[1, 2], T[0, 2])
    elif abs(s2 + 1) < EPS:
        A = 0
        B = -np.pi / 2
        C = np.arctan2(-T[1, 2], T[0, 2])
    else:
        s2 = np.clip(s2, -1.0, 1.0)
        B = np.arcsin(s2)
        c2 = sgn(np.cos(B))
        C = np.arctan2(T[1, 0] * c2, T[0, 0] * c2)
        A = np.arctan2(T[2, 1] * c2, T[2, 2] * c2)
    
    return np.array([A, B, C])
