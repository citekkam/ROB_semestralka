#!/usr/bin/env python3
"""
RobotMove Class
===============
This class provides methods for robot movement including calibration routines
and path planning using shortest path selection.

Author: Xuan Dinh Nguyen
Date: 2025-10-25
"""

import numpy as np
import yaml
from pathlib import Path

from ctu_crs import CRS93, CRS97
from homofraphy import CRC_OFF
from robot_trajectory import RobotTrajectory
from se3 import SE3
from so3 import SO3
from robot_model.check_collision.collision import Collision


class RobotMove:
    """
    A class for controlling robot movements with calibration and path planning capabilities.
    """

    def __init__(self, robot: CRS93 | CRS97, joint_weights: np.ndarray | None = None):
        """
        Initialize RobotMove controller.
        
        Args:
            robot: CRS93 or CRS97 robot instance
            joint_weights: Weights for joint distance calculation (default: [1,1,1,1,1,1])
        """
        self.robot = robot
        self.trajectory = RobotTrajectory
        self.joint_weights = joint_weights if joint_weights is not None else np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        self.collision = Collision()
        self.current_q = self.robot.get_q()
    
    @classmethod
    def create(cls, robot_type: str = "CRS93", soft_home: bool = False, 
               joint_weights: np.ndarray | None = None) -> 'RobotMove':
        """
        Factory method to create RobotMove with robot initialization.
        
        Args:
            robot_type: Either "CRS93" or "CRS97"
            soft_home: Whether to use soft homing (True) or full homing (False)
            joint_weights: Weights for joint distance calculation
        
        Returns:
            RobotMove instance with initialized robot
            
        Example:
            # Quick way to create and initialize
            robot_mover = RobotMove.create("CRS93")
            
            # Or with custom weights
            weights = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
            robot_mover = RobotMove.create("CRS97", joint_weights=weights)
        """
        # Create robot based on type
        if robot_type.upper() == "CRS93":
            robot = CRS93()
        elif robot_type.upper() == "CRS97":
            robot = CRS97()
        else:
            raise ValueError(f"Unknown robot type: {robot_type}. Use 'CRS93' or 'CRS97'")
        
        # Initialize if requested
        if soft_home:
            robot.initialize(home=False)
            robot.soft_home()
        else:
                robot.initialize()
        
        # Create and return RobotMove instance
        return cls(robot, joint_weights)

    def select_shortest_path(self, q_current: np.ndarray, q_targets: list, debug: bool = False) -> list:
        """
        Find the shortest path from current position to one of the target positions.
        Uses Euclidean distance in joint space with angle normalization.
        
        Args:
            q_current: Current robot joint configuration (6 joints)
            q_targets: List of possible target joint configurations
            debug: If True, prints detailed distance information
        
        Returns:
            List of tuples (index, distance, normalized_diff) sorted by distance
            where:
                - index: index of the configuration in q_targets
                - distance: weighted Euclidean distance
                - normalized_diff: normalized angular differences for each joint
        """
        
        distances = []
        
        for i, q_target in enumerate(q_targets):
            # Compute difference
            diff = np.array(q_target) - np.array(q_current)
            
            # Normalize angles to range [-pi, pi]
            normalized_diff = np.array([self._normalize_angle(d) for d in diff])
            
            # Apply joint weights
            weighted_diff = normalized_diff * self.joint_weights
            
            # Calculate weighted Euclidean distance
            dist = np.linalg.norm(weighted_diff)
            
            distances.append((i, dist, normalized_diff))

        # Sort by distance (ascending)
        sorted_distances = sorted(distances, key=lambda x: x[1])
        return sorted_distances
    
    def sort_shortest_path(self, q_current: np.ndarray, q_targets: list, debug: bool = False) -> np.ndarray:
        """
        Sort target configurations by distance from current position.
        Returns the configurations themselves sorted by distance (closest first).
        
        Args:
            q_current: Current robot joint configuration (6 joints)
            q_targets: Array-like of possible target joint configurations
            debug: If True, prints detailed distance information
        
        Returns:
            NumPy array of joint configurations sorted by distance from q_current (closest first)
        """
        # Get sorted distances using select_shortest_path
        sorted_distances = self.select_shortest_path(q_current, q_targets, debug)
        
        # Convert to array and extract configurations in sorted order
        q_targets_array = np.array(q_targets)
        sorted_q_targets = np.array([q_targets_array[idx] for idx, _, _ in sorted_distances])
        
        return sorted_q_targets
    
    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """
        Normalize angle to range [-pi, pi].
        
        Args:
            angle: Angle in radians
            
        Returns:
            Normalized angle in range [-pi, pi]
        """
        return np.arctan2(np.sin(angle), np.cos(angle))
    
    @staticmethod
    def _pose_to_transformation_matrix(pose: np.ndarray) -> np.ndarray:
        """Convert pose [x, y, z, roll, pitch, yaw] to 4x4 transformation matrix."""
        x, y, z, roll, pitch, yaw = pose
        
        # Use SO3 for rotation (ZYX Euler angles)
        rotation = SO3.from_euler_angles([yaw, pitch, roll], ['z', 'y', 'x'])
        
        # Use SE3 for complete transformation
        transform = SE3(translation=np.array([x, y, z]), rotation=rotation)
        
        return transform.homogeneous()
    
    def move_to_pose_T(self, target_T: np.ndarray, check_limits: bool = True) -> bool:
        """
        Move robot to target transformation matrix using IK and shortest path selection.
        
        Args:
            target_T: Target 4x4 transformation matrix
            check_limits: Whether to check robot limits before moving
        
        Returns:
            bool: True if movement successful, False otherwise
        """
        # Get current robot position
        current_q = self.robot.get_q()
        
        # Get all IK solutions
        ik_solutions = self.robot.ik(target_T)
        
        if len(ik_solutions) == 0:
            print(f"⚠️  No IK solutions found for target transformation")
            return False
        
        # print(f"Found {len(ik_solutions)} IK solutions")
        
        # Find shortest path
        sorted_distances = self.select_shortest_path(current_q, ik_solutions)
        
        # Try each solution in order of distance
        for idx, distance, _ in sorted_distances:
            q = ik_solutions[idx]
            
            # Check robot limits if requested
            if check_limits and not self.robot.in_limits(q):
                print(f"Configuration with idx {idx} and q {q} exceeds robot limits, trying next...")
                continue
            
            # Move to position
            self.robot.move_to_q(q)
            # self.collision.visualize(q, wait_for_input=True)
            self.robot.wait_for_motion_stop()
            return True
        
        print("❌ No valid configuration found within robot limits")
        return False
    
    def calibration_move(self, positions_file: str, soft_home: bool = True) -> tuple:
        """
        Execute robot calibration routine by moving through predefined positions.
        Captures images and transformation matrices at each position.
        
        Args:
            positions_file: Path to YAML file containing calibration positions
            soft_home: Whether to use soft home (True) or full homing sequence (False)
        
        Returns:
            tuple: (imgs, transformation_matrices) where:
                   - imgs: list of images captured at each calibration position
                   - transformation_matrices: list of 4x4 transformation matrices
                   Returns ([], []) if calibration fails.
        """
        print("  ROBOT CALIBRATION")
        
        # Load calibration positions
        print(f"Loading calibration positions from: {positions_file}")
        try:
            positions = self._load_calibration_positions(positions_file)
            print(f"Loaded {len(positions)} calibration positions")
            print()
        except Exception as e:
            print(f"Failed to load positions: {e}")
            return [], []
        
        # Initialize robot if needed
        print("Checking robot initialization...")
        try:
            if not hasattr(self.robot, '_initialized') or not self.robot._initialized:
                if soft_home:
                    self.robot.initialize(home=False)
                    self.robot.soft_home()
                else:
                    self.robot.initialize()
            print("Robot ready")
            print()
        except Exception as e:
            print(f"Failed to initialize robot: {e}")
            return [], []
        
        # Arrays to store images and transformation matrices
        imgs = []
        transformation_matrices = []
        
        # Process each calibration position
        successful_captures = 0
        failed_captures = 0
        
        for i, (pos_name, target_pose) in enumerate(positions, 1):

            print(f"Position {i}/{len(positions)}: {pos_name}")
            
            # Move to position
            print(f"Moving to position...")
            if not self.move_to_pose(target_pose):
                print(f"Failed to move to position {pos_name}")
                failed_captures += 1
                continue
            
            # Get actual robot position after movement
            actual_q = self.robot.get_q()
            print(f"Actual joint configuration: {actual_q}")
            
            # Calculate transformation matrix
            transformation_matrix = self.robot.fk(actual_q)
            print(f"Transformation matrix calculated")
            
            # Capture image
            print(f"Capturing image...")
            try:
                image = self.robot.grab_image()
                if image is None:
                    print(f"Failed to capture image at position {pos_name}")
                    failed_captures += 1
                    continue
                print(f"Image captured: {image.shape}")
            except Exception as e:
                print(f"Error capturing image: {e}")
                failed_captures += 1
                continue
            
            # Append to arrays
            imgs.append(image)
            transformation_matrices.append(transformation_matrix)
            print(f"Data added to calibration arrays (entry {len(imgs)})")
            successful_captures += 1
            print()
        
        return (imgs, transformation_matrices)
    
    @staticmethod
    def _load_calibration_positions(yaml_file: str) -> list:
        """
        Load calibration positions from YAML file.
        
        Args:
            yaml_file: Path to the YAML file containing calibration positions
            
        Returns:
            List of tuples (position_name, pose array [x,y,z,r,p,y])
        """
        with open(yaml_file, 'r') as f:
            data = yaml.safe_load(f)
        
        positions = []
        
        # Sort keys numerically by extracting the position number
        position_keys = [key for key in data.keys() if 'position_' in key]
        sorted_keys = sorted(position_keys, key=lambda x: int(x.split('_')[1]))
        
        for key in sorted_keys:
            pose = np.array(data[key]['pose'])
            positions.append((key, pose))
        
        return positions

    def dif_angle_check(self, current_q : np.ndarray, target_q : np.ndarray, rng : float) -> bool:
        # print(current_q[-1])
        print(current_q)
        # if current_q[-1] > 0.1:
        #     input()
        for i in range(len(current_q)):
            diff = abs(current_q[i] - target_q[i])
            if i == len(current_q) or i == 3:
                print(diff)
                if diff > rng / 2:
                    print("Angle issue in EE")
                    return False
            else:
                if diff > rng:
                    print("Angle issue")
                    return False
        return True


<<<<<<< HEAD
    def ik_sol_check(self, current_q: np.ndarray, target_T: np.ndarray, seq : list, coli_seq : list, check_angle = True, prev_q) -> [bool, int]:
=======
    def ik_sol_check(self, target_T: np.ndarray, seq : list, coli_seq : list, check_angle = True) -> [bool, int]:
>>>>>>> test_branch
        """check if there is an ik solution within range pi/2 from current_q,
            check if it is within robot limits and
            check the collisions
        """
        ik_solutions = self.robot.ik(target_T)
        
        sorted_distances = self.select_shortest_path(self.current_q, ik_solutions)
        for idx, distance, _ in sorted_distances:
            q = ik_solutions[idx]
            # todo collision check
            if check_angle:
<<<<<<< HEAD
                if self.dif_angle_check(current_q, q, np.pi* 2/4) and self.robot.in_limits(q) and not self.collision.in_collision(q, coli_seq):
                    prev_q = q
=======
                if self.dif_angle_check(self.current_q, q, np.pi * 5/4) and self.robot.in_limits(q) and not self.collision.in_collision(q, coli_seq):
                    self.current_q = q
>>>>>>> test_branch
                    return True, idx
            else:
                print("checking start")
                if self.robot.in_limits(q) and not self.collision.in_collision(q, coli_seq):
<<<<<<< HEAD
                    prev_q = q
=======
                    self.current_q = q
>>>>>>> test_branch
                    print("Robot not in limits or collision")
                    return True, idx
        return False, None
    
    def seq_check (self, seq: list, CRC_OFF, coli_seq, check_angle = True) -> bool:
        # print("target T : ", seq[0] * CRC_OFF.inverse())
        # print("seq :", *seq, sep="\n")
        # print(seq)
<<<<<<< HEAD
        prev_q = current_q
        for T in seq:
            target_T = SE3(translation = [0,0,-0.02]) * T * CRC_OFF.inverse()
            # is_valid, idx = self.ik_sol_check(current_q, target_T.homogeneous())
            is_valid, idx = self.ik_sol_check(current_q, target_T.homogeneous(), seq, coli_seq, check_angle, prev_q)
=======
        

        for T in seq:
            target_T = SE3(translation = [0,0,-0.02]) * T * CRC_OFF.inverse()
            # is_valid, idx = self.ik_sol_check(current_q, target_T.homogeneous())
            is_valid, idx = self.ik_sol_check(target_T.homogeneous(), seq, coli_seq, check_angle)
>>>>>>> test_branch
            if not is_valid:
                print("Cant get to :", target_T)
                return False
        return True

    # def valid_traj(self, puzzle_base : SE3, matrices : list, main_points_idx : list) -> None | list:
    #     for i in range(16):
    #         angle = np.pi * (2*i / 16)
    #         z_rot = SE3(rotation = SO3().rz(angle))
    #         # z_rot = SE3(rotation = puzzle_base.rotation.inverse())
    #         # print(z_rot)
    #         # print(SE3(rotation = puzzle_base.rotation.inverse()))
    #         seq = self.trajectory.to_puzzle_matrice(puzzle_base, matrices, z_rot)
    #         seq.insert(0, seq[0] * SE3(translation = [0,0,-0.03]))
    #         current_q = self.robot.get_q()

    #         if self.seq_check(current_q, seq, CRC_OFF):
    #             return seq

    #     return None

    def valid_traj(self, puzzle_base : SE3, matrices : list, main_points_idx : list) -> None | list:
        new_seq = []
        coli_seq = self.trajectory.to_puzzle_matrice(puzzle_base, matrices, SE3())
        self.current_q = self.robot.get_q()
        prev_angle = 0
        for i in range(len(main_points_idx)-1):
            prev_angle_idx = 0
            start_idx = main_points_idx[i]
            end_idx = main_points_idx[i+1]
            
            matrices_segment = matrices[start_idx:end_idx]
            valid_segment_found = False

            for j in range(16):
                angle = prev_angle + np.pi * (2*j / 16)
                z_rot = SE3(rotation = SO3().rz(angle))
                seq_segment = self.trajectory.to_puzzle_matrice(puzzle_base, matrices_segment, z_rot)
                
                if i == 0:
                    start_point_2 = seq_segment[0] * SE3(translation = [0,0,-0.03])
                    start_point_1 = SE3(translation = [0,0,0.04]) * start_point_2
                    seq_segment.insert(0, start_point_1)
                    seq_segment.insert(1, start_point_2)

                    if not self.seq_check(seq_segment[:2], CRC_OFF, coli_seq, False):
                        print("Cant get to starting position")
                        continue
                elif angle != prev_angle :
                    # find the angle change from previous segment with count angle change in 16 steps
                    diff_angle_idx = j
                    T_start = new_seq[-1]
                    T_end = seq_segment[0]
                    angle_seq_seg = self.trajectory.generate_segment(T_start, T_end, num_points=diff_angle_idx + 2)
                    old_seq_seg = seq_segment[1:]
                    seq_segment = list(angle_seq_seg[1:]) + list(old_seq_seg)
                #todo seq_segment[1:] nebude fungovat pokud nejsem na zacatku i == 0
                start_idx = 2 if i == 0 else 0
                if self.seq_check(seq_segment[start_idx:], CRC_OFF, coli_seq ,True):
                    new_seq.extend(seq_segment[:])
                    valid_segment_found = True
                    prev_angle = angle
                    break
            if not valid_segment_found:
                return None  

        return new_seq
    


    def go_traj(self, seq : SE3, CRC_OFF) -> bool:
        if seq == None:
            print("WARN: No trajectory found!")
            return

        for T in seq:
            e_pos = SE3(translation = [0,0,-0.0166]) * T * CRC_OFF.inverse()
            print(e_pos)
            self.move_to_pose_T(e_pos.homogeneous())
            self.robot.wait_for_motion_stop()
            
        seq_i = seq[::-1]
        for T in seq_i:
            e_pos = SE3(translation = [0,0,-0.0166]) * T * CRC_OFF.inverse()
            print(e_pos)
            self.move_to_pose_T(e_pos.homogeneous())
            self.robot.wait_for_motion_stop()


        self.robot.soft_home()
            
# # Example usage
# if __name__ == "__main__":
#     """Example usage of RobotMove class."""
    
#     print("="*70)
#     print("ROBOTMOVE CLASS - USAGE EXAMPLES")
#     print("="*70)
#     print()
    
#     # ========================================================================
#     # METHOD 1: Use the factory method (Easiest way!)
#     # ========================================================================
#     print("Method 1: Using factory method")
#     print("-" * 70)
    
#     # Simply specify robot type - everything else is handled automatically
#     robot_mover = RobotMove.create("CRS93")  # or "CRS97"

#     print("Robot initialized and ready to use!")
#     print()

#     # Access robot methods through robot_mover.robot
#     q = robot_mover.robot.get_q()
#     robot_mover.robot.move_to_q(q)
#     robot_mover.robot.wait_for_motion_stop()
#     robot_mover.robot.grab_image()
#     # ========================================================================


#     # ========================================================================
#     # METHOD 2: Manual initialization (More control)
#     # ========================================================================
#     print("Method 2: Manual initialization")
#     print("-" * 70)
    
#     # Choose robot type
#     ROBOT_TYPE = "CRS93"  # Change to "CRS97" to use the other robot
    
#     # Initialize robot
#     print(f"Initializing {ROBOT_TYPE}...")
#     if ROBOT_TYPE == "CRS93":
#         robot = CRS93()
#     elif ROBOT_TYPE == "CRS97":
#         robot = CRS97()
#     else:
#         raise ValueError(f"Unknown robot type: {ROBOT_TYPE}")
    
#     robot.initialize(home=False)
#     robot.soft_home()
    
#     # Create RobotMove with custom weights
#     # joint_weights = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
#     # robot_mover = RobotMove(robot, joint_weights)
#     robot_mover = RobotMove(robot)
#     print(f"{ROBOT_TYPE} initialized successfully")
#     print(f"   Joint limits: q_min={robot.q_min}")
#     print(f"                 q_max={robot.q_max}")
#     print()
#     # ========================================================================


#     # ========================================================================
#     # USAGE EXAMPLES
#     # ========================================================================
#     print("="*70)
#     print("USAGE EXAMPLES")
#     print("="*70)
    
#     # ========================================================================
#     # Example 1: Move to a specific pose
#     # ========================================================================  
#     print("\nExample 1: Moving to target pose")
#     print("-" * 70)
#     target_pose = np.array([0.4, 0.0, 0.5, np.pi, 0.0, np.pi])
#     print(f"Target: x={target_pose[0]}, y={target_pose[1]}, z={target_pose[2]}")
#     # success = robot_mover.move_to_pose(target_pose)
#     print("(Commented out for safety)")
#     # ========================================================================
    

#     # ========================================================================
#     # Example 2: Run calibration routine
#     # ========================================================================
#     print("\nExample 2: Calibration routine")
#     print("-" * 70)
#     calibration_file = "./robot_calibration/robot_calibration/calibration_positions.yaml"
#     print(f"Calibration file: {calibration_file}")
#     # imgs, transforms = robot_mover.calibration_move(calibration_file)
#     # print(f"Collected {len(imgs)} images and {len(transforms)} matrices")
#     print("(Commented out for safety)")
#     # ========================================================================


#     # ========================================================================
#     # Example 3: Move to a target transformation matrix
#     # ========================================================================
#     print("\nExample 3: Moving to target pose using transformation matrix")
#     print("-" * 70)
#     target_T = np.array([
#         [-1.0,  0.0,  0.0,  0.4],
#         [ 0.0,  1.0,  0.0,  0.0],
#         [ 0.0,  0.0, -1.0,  0.5],
#         [ 0.0,  0.0,  0.0,  1.0]
#     ])
#     print(f"Target Transformation Matrix:\n{target_T}")
#     # success = robot_mover.move_to_pose_T(target_T)
#     print("(Commented out for safety)")
#     # ========================================================================


#     # Release robot
#     print("\n" + "="*70)
#     robot_mover.robot.release()
#     print("Robot released. Done!")
