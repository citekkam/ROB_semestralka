#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2024-10-31
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
import numpy as np
from ctu_crs import CRS97

robot = CRS97()
robot.initialize(robots_home=False)
robot.soft_home()

q0 = robot.q_home
q0[1] = 0   
robot.move_to_q(q0)

robot.close()
