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
from utils import visualize_trajectory, plot_trajectory_3d




class RobotTrajectory:
    """
    A class for robot trajectory generation and management using SE3/SO3.
    Supports linear position interpolation and SLERP rotation interpolation.
    """

    def __init__(self, waypoints: list | None = None, puzzle: str | None = None):
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
        self.main_points_idx = []
        self.curr_puzzle = puzzle
    
    @staticmethod
    def _load_puzzle_waypoints(puzzle: str) -> list:
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
    
    def add_waypoints(self, waypoints: list) -> None:
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
        self.main_points_idx = []
    
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
            T1: Ending transformation (SE3 object)
            T2: Starting transformation (SE3 object)
            t: Interpolation parameter [0, 1]
        
        Returns:
            Interpolated transformation (SE3 object)
        """
        if not (T1.rotation == T2.rotation) and (self.curr_puzzle in ['D', 'E']):
            # For puzzles D and E, use circular interpolation
            p1 = T1.translation
            p2 = T2.translation
            radius = 0.05  # 5cm radius for both D and E
            center = p1- [0,0,radius]
            ret = self.generate_circle_segment(center, T1, T2, radius, t)
        else:
            # Interpolate position using linear interpolation
            p_interp = self.interpolate_position_linear(T1.translation, T2.translation, t)
            
            # Interpolate rotation using SLERP
            R_interp = self.interpolate_rotation_slerp(T1.rotation, T2.rotation, t)
        
            ret = SE3(translation=p_interp, rotation=R_interp)
            
        return ret
    
    def generate_circle_segment(self, center: np.ndarray, T1: SE3, T2: SE3, 
                            radius: float, t: float) -> SE3:
        """
        Generate a single SE3 transformation along a circular path rotating about the x-axis.
        Points move in the Y-Z plane (rotation axis = x).

        center : 3-element center [x,y,z]
        T1, T2 : SE3 start/end (only Y,Z used for arc)
        radius : desired radius in meters (if <=0, radius computed from T1)
        t      : interpolation parameter in [0,1]
        """
        center = np.array(center, dtype=float)

        T_center = SE3(translation=center)
        if self.curr_puzzle == "D":
            rot = SE3(rotation=SO3().ry(-(t) * np.pi/2))
        elif self.curr_puzzle == "E":
            rot = SE3(rotation=SO3().rx((t) * np.pi/2))

        res = T_center * rot * T_center.inverse()  * T1
        return res



    def generate_segment(self, T_start: SE3, T_end: SE3, num_points: int = 50) -> list:
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
                                   segment_length: float = 0.01) -> list:
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
    
    def generate_by_count(self, num_points_per_segment: int = 50) -> list:
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
    
    def generate_by_length(self, segment_length: float = 0.01) -> list:
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
        self.main_points_idx = [0]
        
        for i in range(len(self.waypoints) - 1):
            segment = self.generate_segment_by_length(
                self.waypoints[i], 
                self.waypoints[i + 1], 
                segment_length
            )

            # Avoid duplicating points at segment boundaries
            if i > 0:
                segment = segment[1:]

            self.main_points_idx.append(self.main_points_idx[-1] + len(segment))
            self.trajectory.extend(segment)
            
        return self.trajectory, self.main_points_idx
    
    @classmethod
    def from_puzzle(cls, puzzle: str) -> 'RobotTrajectory':
        """
        Create RobotTrajectory from predefined puzzle waypoints.
        
        Args:
            puzzle: Puzzle identifier ('A', 'B', 'C', 'D', or 'E')
        
        Returns:
            RobotTrajectory instance with loaded puzzle waypoints
            main_points_idx initialized to track key waypoints.
        
        Example:
            traj = RobotTrajectory.from_puzzle('B')
            traj.generate_by_length(0.01)
        """
        return cls(puzzle=puzzle)
    
    # ========================================================================
    # SIMPLIFIED STATIC METHOD - ONE-CALL GENERATION
    # ========================================================================
    
    @staticmethod
    def get_trajectory_se3(puzzle: str, segment_length: float = 0.01,
                          num_points: int | None = None) -> list:
        """
        Generate and return trajectory as SE3 objects in one call.
        
        Args:
            puzzle: Puzzle identifier ('A', 'B', 'C', 'D', or 'E')
            segment_length: Spacing between points in meters (default: 0.01m)
            num_points: If provided, uses fixed point count instead of segment_length
        
        Returns:
            List of SE3 transformation objects
        
        Example:
            trajectory = RobotTrajectory.get_trajectory_se3('D', segment_length=0.02)
        """
        traj = RobotTrajectory(puzzle=puzzle)
        

        if num_points is not None:
            return traj.generate_by_count(num_points)
        else:
            return traj.generate_by_length(segment_length)
    
    @staticmethod
    def to_homogeneous_matrices(trajectory: list) -> list:
        """
        Convert SE3 trajectory to list of 4x4 homogeneous transformation matrices.
        
        Args:
            trajectory: List of SE3 transformation objects
        
        Returns:
            List of 4x4 numpy arrays (homogeneous transformation matrices)
        
        Example:
            trajectory = RobotTrajectory.get_trajectory_se3('A', segment_length=0.01)
            matrices = RobotTrajectory.to_homogeneous_matrices(trajectory)
        """
        return [T.homogeneous() for T in trajectory]

    def to_puzzle_matrice(puzzle_base : SE3, matrices : list, z_rot : SO3) -> list:
        ret = []
        for T in matrices:
            # --- TODO - fix tranforamtions
            T = puzzle_base * T
            T_ee = T * SE3(rotation = SO3().ry(np.pi)) * z_rot
            # T = SE3(translation = T.translation, rotation = SO3().ry(np.pi))
            
            # print(T_ee)
            ret.append(T_ee)

        return ret


# Example usage
if __name__ == "__main__":
    """Example usage of RobotTrajectory class with simplified API."""
    
    print("=" * 70)
    print("ROBOT TRAJECTORY CLASS - SIMPLIFIED USAGE")
    print("=" * 70)
    print()
    
    # ========================================================================
    # SIMPLIFIED API - ONE-LINE TRAJECTORY GENERATION
    # ========================================================================
    print("✨ Generate trajectory in one call:")
    print("-" * 70)
    
    # Generate trajectory as SE3 objects with segment_length
    # trajectory = RobotTrajectory.get_trajectory_se3('A', segment_length=0.01)
    # print(f"✅ Puzzle A: Generated {len(trajectory)} SE3 transformations (1cm spacing)")
    
    # trajectory = RobotTrajectory.get_trajectory_se3('B', segment_length=0.005)
    # print(f"✅ Puzzle B: Generated {len(trajectory)} SE3 transformations (5mm spacing)")
    
    # # Or use fixed number of points
    # trajectory = RobotTrajectory.get_trajectory_se3('C', num_points=100)
    # print(f"✅ Puzzle C: Generated {len(trajectory)} SE3 transformations (100 points)")
    
    # # Convert to homogeneous matrices
    # matrices = RobotTrajectory.to_homogeneous_matrices(trajectory)
    # print(f"✅ Converted to {len(matrices)} homogeneous matrices (4x4)")
    # print(f"   Example matrix:\n{matrices[0]}")
    # print()
    
    # ========================================================================
    # VISUALIZATION
    # ========================================================================
    print("📊 Visualize trajectory:")
    print("-" * 70)
    
    # Generate trajectory for visualization
    viz_trajectory = RobotTrajectory.get_trajectory_se3('E', segment_length=0.01)
    
    print("Text visualization:")
    visualize_trajectory(viz_trajectory)
    
    print("\n3D plot visualization (close window to continue)...")
    try:
        plot_trajectory_3d(viz_trajectory)
    except ImportError:
        print("⚠️  Matplotlib not available. Install: pip install matplotlib")
    except Exception as e:
        print(f"⚠️  Could not create plot: {e}")
    
    # ========================================================================
    # QUICK REFERENCE
    # ========================================================================
    print("\n" + "=" * 70)
    print("⚡ QUICK REFERENCE")
    print("=" * 70)
    print("# Generate trajectory in one line:")
    print("trajectory = RobotTrajectory.get_trajectory_se3('A', segment_length=0.01)")
    print("trajectory = RobotTrajectory.get_trajectory_se3('B', num_points=100)")
    print()
    print("# Convert to homogeneous matrices:")
    print("matrices = RobotTrajectory.to_homogeneous_matrices(trajectory)")
    print()
    print("# Visualize:")
    print("from utils import visualize_trajectory, plot_trajectory_3d")
    print("visualize_trajectory(trajectory)  # Text output")
    print("plot_trajectory_3d(trajectory)    # 3D plot")
    print()
    print("Available puzzles: A, B, C, D, E")
    print("=" * 70)
