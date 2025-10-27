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

show_img = cv2.resize(img, (1200, 800))
cv2.imshow(f"ArUco", show_img)
cv2.waitKey(0)
cv2.destroyAllWindows()

ids, corners = hm.find_aruco(img)

positions = hm.get_aruco_center(corners, img)


puzzle_base = hm.get_puzzle_base(ids, corners, H, img)

print("Position of puzzle base: ", puzzle_base)
trans = hm.hom2se3(our_robot.robot.fk(our_robot.robot.get_q()))
print("Forward kinematics: ",trans)

T = SE3(rotation = SO3().ry(np.pi), translation = puzzle_base.translation)
print(T)

our_robot.move_to_pose_T((T * hm.CRC_OFF.inverse()).homogeneous())

# H_check = hm.homography_check(img, H)
# print("Homography check: ", H_check)


show_img = cv2.resize(img, (1200, 800))
cv2.imshow(f"ArUco", show_img)
cv2.waitKey(0)
cv2.destroyAllWindows()