import numpy as np
from ctu_crs import CRS93
from select_shortest_path import find_shortest_path
# from ROB_semestralka.camera.dataset_creator import next_filename, uloz_data
from homofraphy import hom2se3, load_image_yaml_pairs, correct_eff_pos, find_hoop_homography, find_aruco, get_aruco_world_pos, get_puzzle_base, CRC_OFF

robot = CRS93()
robot.initialize(home = False)
robot.soft_home()

FILE = "/home/nguyexu7/Documents/ROB/ROB_semestralka/robot_calibration/calibration_data"

print(CRC_OFF)
imgs, hoop_pos = load_image_yaml_pairs()

# print(imgs, hoop_pos)
H = find_hoop_homography(imgs, hoop_pos)
print(H)

img = robot.grab_image()

ids, corners = find_aruco(img)
positions = get_aruco_world_pos(corners, H)

puzzle_base = get_puzzle_base(ids, positions)
print(puzzle_base)

# T = robot.fk(robot.get_q())
# print(T)

# T[0,3] = puzzle_base[0]
# T[1,3] = puzzle_base[1]
# T[2,3] = 0.3

# print(T)

# T = T[0,3] += puzzle_base[0]

T = np.array([
  [-9.99999517e-01, 4.36878670e-04, -8.80522963e-04, 0.26545822],
  [4.36546069e-04, 9.99999833e-01, 3.77887523e-04, -0.00275656],
  [8.80687907e-04, 3.77502951e-04, -9.99999541e-01, 2.30000000e-01],
  [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
])

ik_sol = robot.ik(T)
T_go = find_shortest_path(robot.get_q(),ik_sol)
idx, dis, _ = T_go[0]
print(idx)
ik = ik_sol[idx]
print(ik)
robot.move_to_q(ik)

# robot.release()