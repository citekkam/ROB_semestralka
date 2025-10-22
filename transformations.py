# File for transformations

from numpy.typing import ArrayLike
import numpy as np
from se3 import SE3
from so3 import SO3

from homofraphy import hom2se3, CRC_OFF


def get_puzzle_base(aruco_ids: ArrayLike, aruco_positions: ArrayLike) -> np.ndarray:
    if aruco_ids is None:
        raise ValueError("At least two ArUco markers are required to determine the puzzle base.")
    elif len(aruco_ids) == 1:
        raise ValueError("Not yet implemented for one ArUco marker.")
    elif len(aruco_ids) == 2:
        pos1 = aruco_positions[0]
        pos2 = aruco_positions[1]

        center = (pos1 + pos2) / 2.0
    else:
        raise NotImplementedError("False positives detected, more than two ArUco markers found.")
    return center[:2]



def get_puzzle_orientation(aruco_ids: ArrayLike, aruco_positions: ArrayLike) -> SO3:   
    pass


def get_base_T(base_pos: np.ndarray, orientation: SO3) -> np.ndarray:
    # CRC_OFF * trans
    T = SE3(orientation, np.array([base_pos[0], base_pos[1], 0.05]))
    print(T)
    return T
    
