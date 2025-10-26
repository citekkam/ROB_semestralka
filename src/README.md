
# RobotMove class:   
## Create RobotMove instance
    robot_mover = RobotMove.create("CRS93")

### or the older method by creating robot insance first and add to RobotMove like input 
    robot = CRS93()
    robot.initialize(home=False)
    robot.soft_home()
    robot_mover = RobotMove(robot)
    
## access robot methods through robot_mover.robot
    q = robot_mover.robot.get_q()
    robot_mover.robot.move_to_q(q)
    robot_mover.robot.wait_for_motion_stop()
    robot_mover.robot.grab_image()

## robot move to target T by choosing the nearest path (shortest path)
    success = robot_mover.move_to_pose_T(target_T)

### alternativ way for robot move in order of target pose [x,y,z,r,p,y]
    target_pose = np.array([0.4, 0.0, 0.5, 0.0, np.pi, 0.0])
    success = robot_mover.move_to_pose(target_pose)

### robot calibration
    calibration_file = "./robot_calibration/robot_calibration/calibration_positions.yaml"
    imgs, transforms = robot_mover.calibration_move(calibration_file)



# RobotTrajectory class:
## to get robot trajectories in form of se3
    trajectory = RobotTrajectory.get_trajectory_se3('C', num_points=100)
    ## or 
    trajectory = RobotTrajectory.get_trajectory_se3('B', segment_length=0.005)

# Convert to homogeneous matrices
    matrices = RobotTrajectory.to_homogeneous_matrices(trajectory)