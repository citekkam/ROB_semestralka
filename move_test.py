import numpy as np
from ctu_crs import CRS93
from select_shortest_path import find_shortest_path

robot = CRS93()
robot.initialize(home = False)

joint_weights = np.array([1.0, 1.0, 1.0, 10.0, 1.0, 10.0])



# q0 = np.array([ 1.27558277e-04, -1.06018865e+00, -1.91550815e+00,  1.29470034e-04,
#        -1.59282805e-01, -5.57399683e-05])
# robot.move_to_q(q0)

# print(robot.fk(robot.get_q()))


# print(robot.get_q())


# qs = robot.ik(pose)
# print(qs)
# #print(qs[0])
# robot.move_to_q(qs[4])
# robot.move_to_q([ 1.27558277e-04, -1.06018865e+00, -1.91550815e+00,  1.29470034e-04,
#        -1.59282805e-01, -5.57399683e-05])


robot_q = robot.get_q()
# print(robot_q)

pose = robot.fk(robot.get_q())
print(pose)
pose[1][3] += 0.05
print(pose)
robot_qs = robot.ik(pose)
print(robot_qs)
sorted_distances = find_shortest_path(robot_q, robot_qs, joint_weights)

# for idx, dist, normalized_diff in sorted_distances:
#     print(f"Index: {idx}, Vzdálenost: {dist:.6f} rad, Δq[3]: {normalized_diff[3]:+.6f} rad, Δq[5]: {normalized_diff[5]:+.6f} rad")

idx,_,_ = sorted_distances[0]
print(idx)
print(robot_qs[idx])

robot.move_to_q(robot_qs[idx])

robot_q = robot.get_q()
print("Robot_pos_q: "robot_q)

pose = robot.fk(robot.get_q())
print("tramsformacni matic:", pose)





# robot.release()