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

positions = hm.get_aruco_center(corners, img)


puzzle_base = hm.get_puzzle_base(ids, corners, H, img)

print("Position of puzzle base: ", puzzle_base)
trans = hm.hom2se3(our_robot.robot.fk(our_robot.robot.get_q()))
print("Forward kinematics: ",trans)

T = SE3(rotation = SO3().ry(np.pi), translation = puzzle_base.translation)
print(T)


## ------------------ A puzzle ----------------
# trajectory = our_robot.trajectory.get_trajectory_se3('A', segment_length=0.1)

# print("matrices", trajectory, sep = "\n")

# puzzle_base = SE3(rotation = puzzle_base.rotation.ry(np.pi), translation = puzzle_base.translation)
# print(puzzle_base)
# print("computed", puzzle_base * trajectory[0])


# matrcies_puzzle = our_robot.trajectory.to_puzzle_matrice(puzzle_base, trajectory)

# print(matrcies_puzzle)

# our_robot.go_traj(matrcies_puzzle, hm.CRC_OFF)


## ------------------------- B puzzle ---------------
# TODO fix balet
trajectory = our_robot.trajectory.get_trajectory_se3('B', segment_length=0.01)

print("matrices", trajectory, sep = "\n")

puzzle_base = SE3(rotation = puzzle_base.rotation, translation = puzzle_base.translation)
print(puzzle_base)
print("computed", (puzzle_base * trajectory[0]))

matrcies_puzzle = our_robot.trajectory.to_puzzle_matrice(puzzle_base, trajectory)
print(matrcies_puzzle)

our_robot.go_traj(matrcies_puzzle, hm.CRC_OFF)

## ------------------------ C puzzle ---------------

show_img = cv2.resize(img, (1200, 800))
cv2.imshow(f"ArUco", show_img)
cv2.waitKey(0)
cv2.destroyAllWindows()