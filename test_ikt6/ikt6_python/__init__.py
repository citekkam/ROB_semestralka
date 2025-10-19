"""
IKT6 Python Library
===================

Python implementation of the IKT6 inverse kinematics library for 6-DOF robots.
This is a direct port of the C++ implementation.

Main components:
- Robot: Robot parameter class
- ikt6_robot_init: Initialize robot with parameters
- ikt6_dkt: Direct (forward) kinematics
- ikt6_ikt: Inverse kinematics

Author: Python port from C++ implementation
"""

from .robot import Robot, ikt6_robot_init
from .kinematics import ikt6_dkt, ikt6_dkt_T, ikt6_ikt
from .util import *

__version__ = "1.0.0"

__all__ = [
    'Robot',
    'ikt6_robot_init',
    'ikt6_dkt',
    'ikt6_dkt_T',
    'ikt6_ikt',
]
