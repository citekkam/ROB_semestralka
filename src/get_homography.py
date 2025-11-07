from robot_move import RobotMove
import homofraphy as hm

our_robot = RobotMove.create("CRS93", soft_home=False)

calibration_file = "/home/nguyexu7/Documents/ROB/ROB_semestralka/src/calibration_positions.yaml"
imgs, transforms = our_robot.calibration_move(calibration_file)

H = hm.find_hoop_homography(imgs, transforms)

H_load = hm.get_H()

print(H, H_load)

our_robot.robot.soft_home()
# our_robot.robot.soft_home()

# img = our_robot.robot.grab_image()

# ids, corners = hm.find_aruco(img)

# positions = get_aruco_center