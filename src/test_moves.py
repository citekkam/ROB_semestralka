from robot_move import RobotMove
import homofraphy as hm
import numpy as np

import cv2
from se3 import SE3
from so3 import SO3
our_robot = RobotMove.create("CRS93", soft_home=True)

H = hm.get_H()

our_robot.robot.soft_home()


img = our_robot.robot.grab_image()


ids, corners = hm.find_aruco(img)


puzzle_base = hm.get_puzzle_base(ids, corners, H, img)

show_img = cv2.resize(img, (1200, 800))
cv2.imshow(f"ArUco", show_img)
cv2.waitKey(0)
cv2.destroyAllWindows()

# trajectory, main_points_idx = our_robot.trajectory.get_trajectory_se3('A', segment_length=0.1)
# trajectory, main_points_idx = our_robot.trajectory.get_trajectory_se3('B', segment_length=0.01)
# trajectory, main_points_idx = our_robot.trajectory.get_trajectory_se3('C', segment_length=0.01)
# trajectory, main_points_idx = our_robot.trajectory.get_trajectory_se3('D', segment_length=0.005)
trajectory, main_points_idx = our_robot.trajectory.get_trajectory_se3('E', segment_length=0.005)



puzzle_base = SE3(rotation = puzzle_base.rotation, translation = puzzle_base.translation)
print("base", puzzle_base)
print("computed", (puzzle_base * trajectory[0]))


matrcies_puzzle = our_robot.valid_traj(puzzle_base, trajectory, main_points_idx)

print("----------------")

our_robot.go_traj(matrcies_puzzle, hm.CRC_OFF)



show_img = cv2.resize(img, (1200, 800))
cv2.imshow(f"ArUco", show_img)
cv2.waitKey(0)
cv2.destroyAllWindows()