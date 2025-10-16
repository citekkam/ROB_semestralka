import numpy as np
from ctu_crs import CRS93
from select_shortest_path import find_shortest_path
from dataset_creator import next_filename, uloz_data

robot = CRS93()
robot.initialize(home=False)

joint_weights = np.array([1.0, 1.0, 1.0, 10.0, 1.0, 10.0])



# q0 = np.array([ 0.58205858, -1.09545765, -1.75684144, -0.01269079, -0.28491104,  0.59186361])
# robot.move_to_q(q0)



# robot_q = robot.get_q()
# # print(robot_q)

# pose = robot.fk(robot.get_q())
# print(pose)
# pose[0][3] += 0.0
# pose[1][3] += 0.0
# pose[2][3] += 0.0
# print(pose)
# robot_qs = robot.ik(pose)
# print(robot_qs)
# sorted_distances = find_shortest_path(robot_q, robot_qs, joint_weights)


# idx,_,_ = sorted_distances[0]
# print(idx)
# print(robot_qs[idx])

# robot.move_to_q(robot_qs[idx])
# robot.wait_for_motion_stop()

# robot_q = robot.get_q()
# print("Robot_pos_q: ",robot_q)

# pose = robot.fk(robot.get_q())
# print("tramsformacni matic:", pose)

# uloz_data(robot_q, pose, robot.grab_image())




# robot.release()