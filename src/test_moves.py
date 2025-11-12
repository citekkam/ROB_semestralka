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

print("Position of puzzle base: ", puzzle_base)
trans = hm.hom2se3(our_robot.robot.fk(our_robot.robot.get_q()))
print("Forward kinematics: ",trans)

T = SE3(rotation = SO3().ry(np.pi), translation = puzzle_base.translation)
print(T)

# trajectory = our_robot.trajectory.get_trajectory_se3('A', segment_length=0.1)
# trajectory = our_robot.trajectory.get_trajectory_se3('B', segment_length=0.01)
# trajectory = our_robot.trajectory.get_trajectory_se3('C', segment_length=0.01)
# trajectory = our_robot.trajectory.get_trajectory_se3('D', segment_length=0.01)
trajectory = our_robot.trajectory.get_trajectory_se3('E', segment_length=0.01)

puzzle_base = SE3(rotation = puzzle_base.rotation, translation = puzzle_base.translation)
print("base", puzzle_base)
print("computed", (puzzle_base * trajectory[0]))


matrcies_puzzle = our_robot.valid_traj(puzzle_base, trajectory)

print("----------------")
# print(*matrcies_puzzle, sep = "\n")

our_robot.go_traj(matrcies_puzzle, hm.CRC_OFF)

## ------------------ A puzzle ----------------
# trajectory = our_robot.trajectory.get_trajectory_se3('A', segment_length=0.1)
# trajectory = our_robot.trajectory.get_trajectory_se3('A', segment_length=0.1)

# print("matrices", trajectory, sep = "\n")

# puzzle_base = SE3(rotation = puzzle_base.rotation, translation = puzzle_base.translation)
# print(puzzle_base)
# print("computed", puzzle_base * trajectory[0])

# # z_rot = SE3(rotation = puzzle_base.rotation.inverse())
# # matrcies_puzzle = our_robot.trajectory.to_puzzle_matrice(puzzle_base, trajectory, z_rot)

# matrcies_puzzle = our_robot.valid_traj(puzzle_base, trajectory)

# print(matrcies_puzzle)

# our_robot.go_traj(matrcies_puzzle, hm.CRC_OFF)

# center check
# for i in range(8):
#     z_rot = z_rot = SE3(rotation = SO3().rz(np.pi * (2*i / 8)))
#     print(z_rot)
# for i in range(8):
#     z_rot = z_rot = SE3(rotation = SO3().rz(np.pi * (2*i / 8)))
#     print(z_rot)
    

## ------------------------- B puzzle ---------------
# TODO fix balet
# trajectory = our_robot.trajectory.get_trajectory_se3('B', segment_length=0.01)


# puzzle_base = SE3(rotation = puzzle_base.rotation, translation = puzzle_base.translation)
# print(puzzle_base)
# print("computed", (puzzle_base * trajectory[0]))

# # z_rot = SE3(rotation = SO3().rz(np.pi/2))

# # matrcies_puzzle = our_robot.trajectory.to_puzzle_matrice(puzzle_base, trajectory, z_rot)

# matrcies_puzzle = our_robot.valid_traj(puzzle_base, trajectory)

# our_robot.go_traj(matrcies_puzzle, hm.CRC_OFF)


## ------------------------ C puzzle ---------------
# trajectory = our_robot.trajectory.get_trajectory_se3('C', segment_length=0.01)


# puzzle_base = SE3(rotation = puzzle_base.rotation, translation = puzzle_base.translation)
# print(puzzle_base)
# print("computed", (puzzle_base * trajectory[0]))

# # z_rot = SE3(rotation = SO3().rz(np.pi/2))

# # matrcies_puzzle = our_robot.trajectory.to_puzzle_matrice(puzzle_base, trajectory, z_rot)

# matrcies_puzzle = our_robot.valid_traj(puzzle_base, trajectory)

# our_robot.go_traj(matrcies_puzzle, hm.CRC_OFF)



## ------------------------ D puzzle ---------------
# trajectory = our_robot.trajectory.get_trajectory_se3('D', segment_length=0.01)


# puzzle_base = SE3(rotation = puzzle_base.rotation, translation = puzzle_base.translation)
# print("base", puzzle_base)
# print("computed", (puzzle_base * trajectory[0]))

# for t in trajectory:
#     print(t)
# matrcies_puzzle = our_robot.valid_traj(puzzle_base, trajectory)

# print("----------------")
# print(*matrcies_puzzle, sep = "\n")

# our_robot.go_traj(matrcies_puzzle, hm.CRC_OFF)



show_img = cv2.resize(img, (1200, 800))
cv2.imshow(f"ArUco", show_img)
cv2.waitKey(0)
cv2.destroyAllWindows()