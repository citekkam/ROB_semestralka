import numpy as np
from ctu_crs import CRS93

robot = CRS93()
robot.initialize(home = False)


#q0 = np.array([0, -0.5, -1.05, 0, -0.735, 0])
#robot.move_to_q(q0)
print(robot.get_q())
# print(robot.fk(robot.get_q()))

pose = robot.fk(robot.get_q())
print(pose)
pose[2][3] = 8.23489053e-01

qs = robot.ik(pose)
print(qs)
#print(qs[0])
robot.move_to_q(qs[0])



# robot.release()