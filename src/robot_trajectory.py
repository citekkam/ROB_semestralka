#!/usr/bin/env python3
"""
RobotTrajectory Class
=====================
This class provides trajectory generation and management using SE3 transformations
with SO3 rotations and interpolation methods.

Author: David
Date: 2025-10-25
"""

import numpy as np
from pathlib import Path
import sys

# Add parent directory to path
sys.path.append(str(Path(__file__).parent.parent))

from se3 import SE3
from so3 import SO3
from trajectory_points import (
    TRAJECTORY_POINTS_PUZZLE_A,
    TRAJECTORY_POINTS_PUZZLE_B,
    TRAJECTORY_POINTS_PUZZLE_C,
    TRAJECTORY_POINTS_PUZZLE_D,
    TRAJECTORY_POINTS_PUZZLE_E,
)
from numpy.typing import ArrayLike
from utils import visualize_trajectory, plot_trajectory_3d


class RobotTrajectory:
    """
    A class for robot trajectory generation and management using SE3/SO3.
    Supports linear position interpolation and SLERP rotation interpolation.
    """

    def __init__(self, waypoints: list[SE3] | None = None, puzzle: str | None = None):
        """
        Initialize RobotTrajectory with optional waypoints or puzzle name.
        
        Args:
            waypoints: List of SE3 transformation waypoints (optional)
            puzzle: Puzzle name ('A', 'B', 'C', 'D', or 'E') to load predefined trajectory (optional)
        """
        if puzzle is not None:
            self.waypoints = self._load_puzzle_waypoints(puzzle)
        elif waypoints is not None:
            self.waypoints = waypoints
        else:
            self.waypoints = []
        
        self.trajectory = []
    
    @staticmethod
    def _load_puzzle_waypoints(puzzle: str) -> list[SE3]:
        """
        Load predefined trajectory waypoints for a specific puzzle.
        
        Args:
            puzzle: Puzzle identifier ('A', 'B', 'C', 'D', or 'E')
        
        Returns:
            List of SE3 waypoints for the specified puzzle
        
        Raises:
            ValueError: If puzzle name is invalid
        """
        puzzle_map = {
            'A': TRAJECTORY_POINTS_PUZZLE_A,
            'B': TRAJECTORY_POINTS_PUZZLE_B,
            'C': TRAJECTORY_POINTS_PUZZLE_C,
            'D': TRAJECTORY_POINTS_PUZZLE_D,
            'E': TRAJECTORY_POINTS_PUZZLE_E,
        }
        
        puzzle_upper = puzzle.upper()
        if puzzle_upper not in puzzle_map:
            raise ValueError(f"Invalid puzzle '{puzzle}'. Choose from: A, B, C, D, E")
        
        return puzzle_map[puzzle_upper]
    
    def add_waypoint(self, waypoint: SE3) -> None:
        """
        Add a single waypoint to the trajectory.
        
        Args:
            waypoint: SE3 transformation to add
        """
        self.waypoints.append(waypoint)
    
    def add_waypoints(self, waypoints: list[SE3]) -> None:
        """
        Add multiple waypoints to the trajectory.
        
        Args:
            waypoints: List of SE3 transformations to add
        """
        self.waypoints.extend(waypoints)
    
    def clear_waypoints(self) -> None:
        """Clear all waypoints."""
        self.waypoints = []
        self.trajectory = []
    
    @staticmethod
    def interpolate_position_linear(p1: np.ndarray, p2: np.ndarray, t: float) -> np.ndarray:
        """
        Linear interpolation between two positions.
        
        Args:
            p1: Starting position (3D vector)
            p2: Ending position (3D vector)
            t: Interpolation parameter [0, 1]
        
        Returns:
            Interpolated position (3D numpy array)
        """
        return (1 - t) * p1 + t * p2
    
    @staticmethod
    def interpolate_rotation_slerp(rot1: SO3, rot2: SO3, t: float) -> SO3:
        """
        Spherical linear interpolation (SLERP) between two SO3 rotations.
        
        Args:
            rot1: Starting rotation (SO3 object)
            rot2: Ending rotation (SO3 object)
            t: Interpolation parameter [0, 1]
        
        Returns:
            Interpolated rotation (SO3 object)
        """
        # Compute the relative rotation: R_rel = R1^T * R2
        R_rel = rot1.inverse() * rot2
        
        # Get the rotation vector (logarithm) of the relative rotation
        omega = R_rel.log()
        
        # Scale by t and convert back to rotation
        R_interp_rel = SO3.exp(t * omega)
        
        # Compose with the starting rotation
        R_interp = rot1 * R_interp_rel
        
        return R_interp
    
    def interpolate_transformation(self, T1: SE3, T2: SE3, t: float) -> SE3:
        """
        Interpolate between two SE3 transformations.
        
        Args:
            T1: Starting transformation (SE3 object)
            T2: Ending transformation (SE3 object)
            t: Interpolation parameter [0, 1]
        
        Returns:
            Interpolated transformation (SE3 object)
        """
        # Interpolate position using linear interpolation
        p_interp = self.interpolate_position_linear(T1.translation, T2.translation, t)
        
        # Interpolate rotation using SLERP
        R_interp = self.interpolate_rotation_slerp(T1.rotation, T2.rotation, t)
        
        # Create interpolated SE3 transformation
        return SE3(translation=p_interp, rotation=R_interp)
    
    def generate_segment(self, T_start: SE3, T_end: SE3, num_points: int = 50) -> list[SE3]:
        """
        Generate a trajectory segment between two poses with fixed point count.
        
        Args:
            T_start: Starting transformation (SE3 object)
            T_end: Ending transformation (SE3 object)
            num_points: Number of interpolation points (default: 50)
        
        Returns:
            List of SE3 objects representing the trajectory segment
        """
        trajectory_segment = []
        for i in range(num_points):
            t = i / (num_points - 1) if num_points > 1 else 0
            T_interp = self.interpolate_transformation(T_start, T_end, t)
            trajectory_segment.append(T_interp)
        
        return trajectory_segment
    
    def generate_segment_by_length(self, T_start: SE3, T_end: SE3, 
                                   segment_length: float = 0.01) -> list[SE3]:
        """
        Generate a trajectory segment with fixed spacing between points.
        
        Args:
            T_start: Starting transformation (SE3 object)
            T_end: Ending transformation (SE3 object)
            segment_length: Desired spacing between points in meters (default: 0.01m)
        
        Returns:
            List of SE3 objects representing the trajectory segment
        """
        # Calculate Euclidean distance between start and end positions
        distance = np.linalg.norm(T_end.translation - T_start.translation)
        
        # Calculate number of points based on segment length
        num_points = max(2, int(np.ceil(distance / segment_length)) + 1)
        
        return self.generate_segment(T_start, T_end, num_points)
    
    def generate_by_count(self, num_points_per_segment: int = 50) -> list[SE3]:
        """
        Generate full trajectory with fixed point count per segment.
        
        Args:
            num_points_per_segment: Number of interpolation points between waypoints
        
        Returns:
            List of SE3 objects representing the full trajectory
        """
        if len(self.waypoints) < 2:
            print("⚠️  Need at least 2 waypoints to generate trajectory")
            return []
        
        self.trajectory = []
        
        for i in range(len(self.waypoints) - 1):
            segment = self.generate_segment(
                self.waypoints[i], 
                self.waypoints[i + 1], 
                num_points_per_segment
            )
            
            # Avoid duplicating points at segment boundaries
            if i > 0:
                segment = segment[1:]
            
            self.trajectory.extend(segment)
        
        return self.trajectory
    
    def generate_by_length(self, segment_length: float = 0.01) -> list[SE3]:
        """
        Generate full trajectory with fixed spacing between points.
        
        Args:
            segment_length: Desired spacing between points in meters (default: 0.01m)
        
        Returns:
            List of SE3 objects representing the full trajectory
        """
        if len(self.waypoints) < 2:
            print("⚠️  Need at least 2 waypoints to generate trajectory")
            return []
        
        self.trajectory = []
        
        for i in range(len(self.waypoints) - 1):
            segment = self.generate_segment_by_length(
                self.waypoints[i], 
                self.waypoints[i + 1], 
                segment_length
            )
            
            # Avoid duplicating points at segment boundaries
            if i > 0:
                segment = segment[1:]
            
            self.trajectory.extend(segment)
        
        return self.trajectory
    
    def get_trajectory_as_matrices(self) -> list[np.ndarray]:
        """
        Get trajectory as list of 4x4 homogeneous transformation matrices.
        
        Returns:
            List of 4x4 numpy arrays
        """
        return [T.homogeneous() for T in self.trajectory]
    
    def get_trajectory_as_poses(self) -> np.ndarray:
        """
        Get trajectory as array of poses [x, y, z, roll, pitch, yaw].
        
        Returns:
            Nx6 numpy array where each row is [x, y, z, roll, pitch, yaw]
        """
        poses = []
        for T in self.trajectory:
            # Extract position
            position = T.translation
            
            # Extract euler angles from rotation (ZYX convention)
            R = T.rotation.rot
            
            # Convert rotation matrix to euler angles (ZYX)
            # This is the inverse of the from_euler_angles method
            if R[2, 0] < 1:
                if R[2, 0] > -1:
                    pitch = np.arcsin(-R[2, 0])
                    roll = np.arctan2(R[2, 1], R[2, 2])
                    yaw = np.arctan2(R[1, 0], R[0, 0])
                else:
                    pitch = np.pi / 2
                    roll = -np.arctan2(-R[1, 2], R[1, 1])
                    yaw = 0
            else:
                pitch = -np.pi / 2
                roll = np.arctan2(-R[1, 2], R[1, 1])
                yaw = 0
            
            poses.append([position[0], position[1], position[2], roll, pitch, yaw])
        
        return np.array(poses)
    
    def get_trajectory_positions(self) -> np.ndarray:
        """
        Get trajectory positions only (without rotation).
        
        Returns:
            Nx3 numpy array of positions
        """
        return np.array([T.translation for T in self.trajectory])
    
    def compute_trajectory_length(self) -> float:
        """
        Compute total trajectory length (sum of segment distances).
        
        Returns:
            Total trajectory length in meters
        """
        if len(self.trajectory) < 2:
            return 0.0
        
        total_length = 0.0
        for i in range(len(self.trajectory) - 1):
            distance = np.linalg.norm(
                self.trajectory[i + 1].translation - self.trajectory[i].translation
            )
            total_length += distance
        
        return total_length
    
    def print_info(self) -> None:
        """Print trajectory information."""
        print("=" * 70)
        print("ROBOT TRAJECTORY INFO")
        print("=" * 70)
        print(f"Number of waypoints: {len(self.waypoints)}")
        print(f"Number of trajectory points: {len(self.trajectory)}")
        if self.trajectory:
            print(f"Trajectory length: {self.compute_trajectory_length():.4f} m")
        print()
    
    @classmethod
    def from_puzzle(cls, puzzle: str) -> 'RobotTrajectory':
        """
        Create RobotTrajectory from predefined puzzle waypoints.
        
        Args:
            puzzle: Puzzle identifier ('A', 'B', 'C', 'D', or 'E')
        
        Returns:
            RobotTrajectory instance with loaded puzzle waypoints
        
        Example:
            traj = RobotTrajectory.from_puzzle('B')
            traj.generate_by_length(0.01)
        """
        return cls(puzzle=puzzle)


# Example usage
if __name__ == "__main__":
    """Example usage of RobotTrajectory class with predefined puzzles."""
    
    print("=" * 70)
    print("ROBOT TRAJECTORY CLASS - USAGE EXAMPLES")
    print("=" * 70)
    print()
    
    # ========================================================================
    # METHOD 1: Load predefined puzzle trajectory
    # ========================================================================
    print("Method 1: Load predefined puzzle trajectory")
    print("-" * 70)
    
    # Option 1a: Using constructor with puzzle parameter
    traj_a = RobotTrajectory(puzzle='A')
    print(f"Puzzle A - Waypoints: {len(traj_a.waypoints)}")
    
    # Option 1b: Using from_puzzle class method
    traj_b = RobotTrajectory.from_puzzle('B')
    print(f"Puzzle B - Waypoints: {len(traj_b.waypoints)}")
    print()
    
    # ========================================================================
    # METHOD 2: Generate trajectory for a puzzle
    # ========================================================================
    print("Method 2: Generate trajectory for Puzzle B")
    print("-" * 70)
    
    # Generate trajectory with fixed point count
    traj_b.generate_by_count(num_points_per_segment=20)
    traj_b.print_info()
    
    # ========================================================================
    # METHOD 3: Generate trajectory with fixed spacing
    # ========================================================================
    print("\nMethod 3: Generate trajectory for Puzzle C with fixed spacing")
    print("-" * 70)
    
    traj_c = RobotTrajectory.from_puzzle('C')
    traj_c.generate_by_length(segment_length=0.005)  # 5mm spacing
    traj_c.print_info()
    
    # ========================================================================
    # METHOD 4: Custom waypoints (manual definition)
    # ========================================================================
    print("\nMethod 4: Create trajectory from custom waypoints")
    print("-" * 70)
    
    # Create rotation matrix (180° around Y-axis)
    rotation_matrix = np.array([
        [-1.0,  0.0,  0.0],
        [ 0.0,  1.0,  0.0],
        [ 0.0,  0.0, -1.0]
    ])
    rotation = SO3(rotation_matrix)
    
    # Define custom waypoints
    custom_waypoints = [
        SE3(translation=np.array([0.0, 0.0, 0.2]), rotation=rotation),
        SE3(translation=np.array([0.1, 0.0, 0.15]), rotation=rotation),
        SE3(translation=np.array([0.1, 0.1, 0.1]), rotation=rotation),
    ]
    
    traj_custom = RobotTrajectory(waypoints=custom_waypoints)
    traj_custom.generate_by_count(num_points_per_segment=10)
    traj_custom.print_info()
    
    # ========================================================================
    # METHOD 5: Get trajectory in different formats
    # ========================================================================
    print("\nMethod 5: Export trajectory in different formats")
    print("-" * 70)
    
    # Use Puzzle A for demonstration
    traj_demo = RobotTrajectory.from_puzzle('A')
    traj_demo.generate_by_length(segment_length=0.01)
    
    matrices = traj_demo.get_trajectory_as_matrices()
    print(f"As matrices: {len(matrices)} transformation matrices (4x4)")
    
    poses = traj_demo.get_trajectory_as_poses()
    print(f"As poses: {poses.shape} array [x, y, z, roll, pitch, yaw]")
    
    positions = traj_demo.get_trajectory_positions()
    print(f"As positions: {positions.shape} array [x, y, z]")
    
    # ========================================================================
    # METHOD 6: Visualize trajectory
    # ========================================================================
    print("\nMethod 6: Visualize trajectory")
    print("-" * 70)
    
    # Create trajectory for visualization
    traj_viz = RobotTrajectory.from_puzzle('B')
    traj_viz.generate_by_length(segment_length=0.01)
    
    # Text-based visualization (always works)
    print("\nText visualization of trajectory:")
    visualize_trajectory(traj_viz.trajectory)
    
    # 3D plot visualization (requires matplotlib)
    print("\n📊 Opening 3D plot visualization...")
    print("(Close the plot window to continue)")
    try:
        plot_trajectory_3d(traj_viz.trajectory)
    except ImportError:
        print("⚠️  Matplotlib not available. Install with: pip install matplotlib")
    except Exception as e:
        print(f"⚠️  Could not create plot: {e}")
    
    # ========================================================================
    # QUICK REFERENCE
    # ========================================================================
    print("\n" + "=" * 70)
    print("QUICK REFERENCE")
    print("=" * 70)
    print("# Load puzzle and generate trajectory:")
    print("traj = RobotTrajectory.from_puzzle('B')")
    print("traj.generate_by_length(0.01)  # 1cm spacing")
    print()
    print("# Get trajectory as transformation matrices:")
    print("matrices = traj.get_trajectory_as_matrices()")
    print()
    print("# Get trajectory as poses [x,y,z,roll,pitch,yaw]:")
    print("poses = traj.get_trajectory_as_poses()")
    print()
    print("# Visualize trajectory:")
    print("from utils import visualize_trajectory, plot_trajectory_3d")
    print("visualize_trajectory(traj.trajectory)  # Text output")
    print("plot_trajectory_3d(traj.trajectory)    # 3D plot")
    print()
    print("Available puzzles: A, B, C, D, E")
    print("=" * 70)
