"""
Robot class and initialization
Direct port from ikt6.h Robot struct
"""

import numpy as np
from numpy.typing import NDArray
from dataclasses import dataclass
from .util import mtx_translate


@dataclass
class Robot:
    """
    Robot parameters for 6-DOF manipulator
    
    Attributes:
        name: Robot name/identifier
        lengths: Link lengths [L1, L2, L3, L4, L5, L6] (6 elements)
        offsets: Joint angle offsets in radians (6 elements)
        directions: Joint rotation directions +1 or -1 (6 elements)
        limits_max: Maximum joint limits in radians (6 elements)
        limits_min: Minimum joint limits in radians (6 elements)
        base: Base transformation (4x4 matrix)
        tool: Tool transformation (4x4 matrix)
        DH: DH parameters matrix (4x6)
        DH_Param: DH parameter flags (4x6)
        A76: Tool offset transformation (4x4 matrix)
    """
    name: str
    lengths: NDArray  # 6 elements
    offsets: NDArray  # 6 elements
    directions: NDArray  # 6 elements
    limits_max: NDArray  # 6 elements
    limits_min: NDArray  # 6 elements
    base: NDArray  # 4x4 matrix
    tool: NDArray  # 4x4 matrix
    DH: NDArray  # 4x6 matrix
    DH_Param: NDArray  # 4x6 matrix
    A76: NDArray  # 4x4 matrix


def ikt6_robot_init(
    name: str,
    lengths: NDArray,
    offsets: NDArray,
    directions: NDArray,
    limits_max: NDArray,
    limits_min: NDArray,
    base: NDArray = None,
    tool: NDArray = None
) -> Robot:
    """
    Initialize robot parameters
    
    Args:
        name: Robot name
        lengths: Link lengths [L1, L2, L3, L4, L5, L6] in mm or m
        offsets: Joint offsets in radians
        directions: Joint directions (+1 or -1)
        limits_max: Maximum joint limits in radians
        limits_min: Minimum joint limits in radians
        base: Base transformation (default: identity)
        tool: Tool transformation (default: identity)
    
    Returns:
        Initialized Robot object
    """
    # Default transformations
    if base is None:
        base = np.eye(4)
    if tool is None:
        tool = np.eye(4)
    
    # Convert to numpy arrays if needed
    lengths = np.asarray(lengths, dtype=float)
    offsets = np.asarray(offsets, dtype=float)
    directions = np.asarray(directions, dtype=float)
    limits_max = np.asarray(limits_max, dtype=float)
    limits_min = np.asarray(limits_min, dtype=float)
    
    # DH parameters matrix: [alpha, a, theta, d] for each joint (6 columns)
    DH = np.array([
        [-np.pi/2,       0,    -np.pi/2,  np.pi/2,  -np.pi/2,       0],  # alpha
        [lengths[1], lengths[2], lengths[3],      0,         0,       0],  # a
        [        0,    -np.pi/2,  -np.pi/2,      0,         0,  np.pi],  # theta
        [lengths[0],           0,         0, lengths[4],     0, lengths[5]]   # d
    ])
    
    # DH parameter flags (which parameters are modified by joint angles)
    DH_Param = np.array([
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [1, 1, 1, 1, 1, 1],  # theta is modified by joint angle
        [0, 0, 0, 0, 0, 0]
    ])
    
    # Tool offset transformation
    A76 = mtx_translate(np.array([0, 0, lengths[5]]))
    
    robot = Robot(
        name=name,
        lengths=lengths,
        offsets=offsets,
        directions=directions,
        limits_max=limits_max,
        limits_min=limits_min,
        base=base,
        tool=tool,
        DH=DH,
        DH_Param=DH_Param,
        A76=A76
    )
    
    return robot
