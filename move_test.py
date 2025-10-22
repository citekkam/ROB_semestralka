import numpy as np
from ctu_crs import CRS93
from select_shortest_path import find_shortest_path
import cv2
# from ROB_semestralka.camera.dataset_creator import next_filename, uloz_data
from homofraphy import hom2se3, load_image_yaml_pairs, correct_eff_pos, find_hoop_homography, find_aruco, get_aruco_world_pos, get_puzzle_base, CRC_OFF
from robot_calibration.calibration_move import move_to_pos_T, robot_calibration

robot = CRS93()
robot.initialize(home = False)
robot.soft_home()

FILE = "/home/nguyexu7/Documents/ROB/ROB_semestralka/robot_calibration/calibration_data"

img = robot.grab_image()


print(CRC_OFF)
imgs, hoop_pos = load_image_yaml_pairs()

# print(imgs, hoop_pos)
H = find_hoop_homography(imgs, hoop_pos)
print(H)


ids, corners = find_aruco(img)
positions = get_aruco_world_pos(corners, H)
fk = robot.fk(robot.get_q())

puzzle_base = get_puzzle_base(ids, positions)
print(puzzle_base)
print(np.hstack((puzzle_base[:2], 0.0, 1.0)))
print(CRC_OFF.inverse().act(np.hstack((puzzle_base[:2], 0.0))))


T = robot.fk(robot.get_q())
print("fk",T)
print(CRC_OFF.inverse())
print(CRC_OFF.inverse().homogeneous() @ T)
# T[0,3] = puzzle_base[0]
# T[1,3] = puzzle_base[1]
# T[2,3] = 0.3

# print(T)

# T = T[0,3] += puzzle_base[0]

T = np.array([
  [-9.99999517e-01, 4.36878670e-04, -8.80522963e-04, 0.537],
  [4.36546069e-04, 9.99999833e-01, 3.77887523e-04, -0.0146],
  [8.80687907e-04, 3.77502951e-04, -9.99999541e-01, 3.0000000e-01],
  [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
])


move_to_pos_T(robot, T) 
img = robot.grab_image()

cv2.imshow("img", img)
cv2.waitKey(0)


# robot.release()