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


## ------------------ A puzzle ----------------
trajectory = our_robot.trajectory.get_trajectory_se3('A', segment_length=0.1)

print("matrices", trajectory, sep = "\n")

puzzle_base = SE3(rotation = puzzle_base.rotation, translation = puzzle_base.translation)
print(puzzle_base)
print("computed", puzzle_base * trajectory[0])

z_rot = SE3(rotation = puzzle_base.rotation.inverse())
z_rot = SE3(rotation = SO3().rz(-np.pi/2))

matrcies_puzzle = our_robot.trajectory.to_puzzle_matrice(puzzle_base, trajectory, z_rot)

print(matrcies_puzzle)

our_robot.go_traj(matrcies_puzzle, hm.CRC_OFF)

# center check
# pos = SE3(puzzle_base.translation + [0,0,0.22], SO3().ry(np.pi))
# print(pos)
# for i in range(12):
#     z_rot = z_rot = SE3(rotation = SO3().rz(-np.pi * (2*i / 12)))
#     print(z_rot)
#     our_robot.move_to_pose_T((pos * z_rot * hm.CRC_OFF.inverse()).homogeneous())
#     img = our_robot.robot.grab_image()
#     show_img = cv2.resize(img, (1200, 800))
#     cv2.imshow(f"ArUco", show_img)
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()

## ------------------------- B puzzle ---------------
# TODO fix balet
# trajectory = our_robot.trajectory.get_trajectory_se3('B', segment_length=0.01)


# puzzle_base = SE3(rotation = puzzle_base.rotation, translation = puzzle_base.translation)
# print(puzzle_base)
# print("computed", (puzzle_base * trajectory[0]))

# z_rot = SE3(rotation = SO3().rz(np.pi/2))

# matrcies_puzzle = our_robot.trajectory.to_puzzle_matrice(puzzle_base, trajectory, z_rot)

# our_robot.go_traj(matrcies_puzzle, hm.CRC_OFF)

## ------------------------ C puzzle ---------------
# trajectory = our_robot.trajectory.get_trajectory_se3('C', segment_length=0.01)

# print("matrices", trajectory, sep = "\n")

# puzzle_base = SE3(rotation = puzzle_base.rotation, translation = puzzle_base.translation)
# print("base", puzzle_base)
# print("computed", (puzzle_base * trajectory[0]))


# z_rot = SE3(rotation = SO3().rz(-np.pi))
# z_rot = SE3(rotation = SO3().rz(-np.pi/2))

# matrcies_puzzle = our_robot.trajectory.to_puzzle_matrice(puzzle_base, trajectory, z_rot)

# print(matrcies_puzzle)
# our_robot.go_traj(matrcies_puzzle, hm.CRC_OFF)



show_img = cv2.resize(img, (1200, 800))
cv2.imshow(f"ArUco", show_img)
cv2.waitKey(0)
cv2.destroyAllWindows()