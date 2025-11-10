#!/usr/bin/env python3
"""
Collision Testing for Robot Trajectories
=========================================
Class-based collision testing using Pinocchio with trajectory spheres.

Author: David
Date: 2025-11-06
"""

import pinocchio as pin
from pathlib import Path
from pinocchio.visualize import MeshcatVisualizer
import numpy as np
import hppfcl
import sys

# Add parent directory to path
sys.path.append(str(Path(__file__).resolve().parent.parent.parent))
from so3 import SO3
from robot_trajectory import RobotTrajectory
from se3 import SE3


class Collision:
    """Class for testing collision between robot and trajectory spheres."""
    
    def __init__(self, urdf_path: str = None, mesh_dirs: list = None):
        """
        Initialize collision tester with robot model.
        
        Args:
            urdf_path: Path to URDF file (optional, uses default if None)
            mesh_dirs: List of mesh directory paths (optional)
        """
        # Setup paths
        self.robot_model_dir = Path(__file__).resolve().parent.parent
        
        if urdf_path is None:
            urdf_path = str((self.robot_model_dir / "urdf" / "crs_a465.urdf").resolve())
        
        if mesh_dirs is None:
            mesh_dirs = [str(self.robot_model_dir.resolve())]
        
        # Load robot model
        self.model, self.collision_model, self.visual_model = pin.buildModelsFromUrdf(
            urdf_path, mesh_dirs
        )
        
        # Create data structures
        self.data = self.model.createData()
        self.geom_data = None
        
        # Storage for added spheres
        self.trajectory_spheres = []
        
        print(f"✓ Robot loaded: {self.model.nq} DOF, "
              f"{len(self.collision_model.geometryObjects)} collision objects")
    
    def add_sphere(self, position: np.ndarray, radius: float = 0.05, 
                   name_suffix: str = "", offset: SE3 = None) -> tuple:
        """
        Add collision and visual sphere at given position.
        
        Args:
            position: 3D position [x, y, z]
            radius: Sphere radius in meters (default: 0.05)
            name_suffix: Unique name suffix
            offset: Optional SE3 transformation to apply
        
        Returns:
            Tuple of (collision_object, visual_object)
        """
        # Apply offset if provided
        if offset is not None:
            position = offset.act(position)
        
        # Create sphere geometry
        sphere_geom = hppfcl.Sphere(radius)
        sphere_pose = pin.SE3(np.eye(3), position)
        
        # Add collision sphere
        sphere_collision = pin.GeometryObject(
            f"sphere_collision_{name_suffix}",
            0,  # world frame
            sphere_pose,
            sphere_geom
        )
        self.collision_model.addGeometryObject(sphere_collision)
        
        # Add visual sphere
        sphere_visual = pin.GeometryObject(
            f"sphere_visual_{name_suffix}",
            0,
            sphere_pose,
            sphere_geom
        )
        self.visual_model.addGeometryObject(sphere_visual)
        
        return sphere_collision, sphere_visual
    
    def add_trajectory_spheres(self, trajectory: list, segment_length: float = 0.05,
                              radius: float = 0.03, offset: SE3 = None) -> list:
        """
        Add spheres along trajectory for collision checking.
        
        Args:
            trajectory: List of SE3 transformations representing the trajectory
            segment_length: Distance between spheres in meters (used only if resampling)
            radius: Sphere radius in meters
            offset: Optional SE3 transformation for all spheres
        
        Returns:
            List of added sphere pairs
        """
        offset_str = " with offset" if offset is not None else ""
        print(f"\n=== Adding {len(trajectory)} spheres from trajectory{offset_str} ===")
        
        spheres = []
        for i, T in enumerate(trajectory):
            position = T.translation
            sphere_pair = self.add_sphere(
                position,
                radius=radius,
                name_suffix=f"traj_{i}",
                offset=offset
            )
            spheres.append(sphere_pair)
        
        self.trajectory_spheres.extend(spheres)
        print(f"✓ Added {len(spheres)} spheres (radius={radius}m)")
        
        return spheres
    
    def setup_collision_pairs(self, srdf_path: str = None) -> None:
        """
        Setup collision pairs with SRDF filtering.
        
        Args:
            srdf_path: Path to SRDF file (optional, uses default if None)
        """
        # Apply SRDF collision filters
        if srdf_path is None:
            srdf_path = str(self.robot_model_dir / "srdf" / "crs.srdf")
        
        try:
            pin.removeCollisionPairs(self.model, self.collision_model, srdf_path)
            print("✓ SRDF collision filters applied")
        except Exception as e:
            print(f"⚠️  Warning: SRDF not found or failed: {e}")
        
        # Add all collision pairs
        self.collision_model.addAllCollisionPairs()
        print(f"✓ Collision pairs: {len(self.collision_model.collisionPairs)}")
        
        # Create geometry data
        self.geom_data = pin.GeometryData(self.collision_model)
    
    def clean(self) -> None:
        """
        Clean trajectory spheres by reloading the robot model.
        This is the safest way to remove added geometries.
        """
        # Reload the robot model from scratch
        urdf_path = str((self.robot_model_dir / "urdf" / "crs_a465.urdf").resolve())
        mesh_dirs = [str(self.robot_model_dir.resolve())]
        
        self.model, self.collision_model, self.visual_model = pin.buildModelsFromUrdf(
            urdf_path, mesh_dirs
        )
        
        # Reset data structures
        self.data = self.model.createData()
        self.geom_data = None
        self.trajectory_spheres.clear()
        
        # print("✓ Trajectory spheres cleaned (model reloaded)")
    
    def is_in_collision(self, q: np.ndarray) -> bool:
        """
        Simple collision check - returns True if robot is in collision.
        
        Args:
            q: Robot joint configuration
        
        Returns:
            True if collision detected, False otherwise
        """
        if self.geom_data is None:
            raise RuntimeError("Call setup_collision_pairs() first!")
        
        # Update geometry placements and compute collisions
        pin.updateGeometryPlacements(self.model, self.data, 
                                    self.collision_model, self.geom_data, q)
        pin.computeCollisions(self.collision_model, self.geom_data, False)
        
        # Check for any collision with trajectory spheres
        for k, pair in enumerate(self.collision_model.collisionPairs):
            g1 = self.collision_model.geometryObjects[pair.first].name
            g2 = self.collision_model.geometryObjects[pair.second].name
            
            # Only check trajectory sphere collisions
            if not ('sphere_collision_traj' in g1 or 'sphere_collision_traj' in g2):
                continue
            
            if self.geom_data.collisionResults[k].isCollision():
                print(f"IN COLLISION: {g1} ↔ {g2}")
                # self.visualize(q, wait_for_input=True)
                return True
        
        return False
    
    def check_collisions(self, q: np.ndarray, verbose: bool = True) -> int:
        """
        Check collisions at given robot configuration.
        
        Args:
            q: Robot joint configuration
            verbose: Print collision details (default: True)
        
        Returns:
            Number of collisions detected
        """
        if self.geom_data is None:
            raise RuntimeError("Call setup_collision_pairs() first!")
        
        # Update geometry placements and compute collisions
        pin.updateGeometryPlacements(self.model, self.data, 
                                    self.collision_model, self.geom_data, q)
        pin.computeCollisions(self.collision_model, self.geom_data, False)
        
        # Count and report collisions
        collision_count = 0
        
        if verbose:
            print("\n=== Collision Results ===")
        
        for k, pair in enumerate(self.collision_model.collisionPairs):
            g1 = self.collision_model.geometryObjects[pair.first].name
            g2 = self.collision_model.geometryObjects[pair.second].name
            
            # Only check trajectory sphere collisions
            if not ('sphere_collision_traj' in g1 or 'sphere_collision_traj' in g2):
                continue
            
            cr = self.geom_data.collisionResults[k]
            if cr.isCollision():
                collision_count += 1
                if verbose:
                    print(f"IN  COLLISION: {g1} ↔ {g2}")
        
        if verbose:
            print(f"\nTotal collisions: {collision_count}")
            print("=" * 50)
        
        return collision_count
    
    def visualize(self, q: np.ndarray, wait_for_input: bool = True) -> None:
        """
        Visualize robot and trajectory spheres in MeshCat.
        
        Args:
            q: Robot joint configuration
            wait_for_input: Wait for user input before closing
        """
        try:
            viz = MeshcatVisualizer(self.model, self.collision_model, self.visual_model)
            viz.initViewer(open=True)
            viz.loadViewerModel()
            viz.displayCollisions(True)
            viz.displayVisuals(True)
            viz.display(q)
            
            print("\n✓ MeshCat viewer opened")
            print(f"  Robot config q = {q}")
            print(f"  Trajectory spheres: {len(self.trajectory_spheres)}")
            
            if wait_for_input:
                input("Press Enter to close...")
            
            try:
                viz.viewer.delete()
            except Exception:
                pass
                
        except Exception as e:
            print(f"⚠️  Viewer error: {e}")

    def in_collision(self, q: np.ndarray, traj: list, radius: float = 0.007, offset: SE3 = None) -> bool:
        """
        Check if robot is in collision along a trajectory.
        
        Args:
            q: Robot joint configuration
            traj: List of SE3 transformations representing the trajectory
            radius: Sphere radius in meters
            offset: Optional SE3 transformation for all spheres
        Returns:
            True if collision detected, False otherwise
        """
        # Clean previous trajectory spheres
        if len(self.trajectory_spheres) > 0:
            self.clean()
        
        # Add trajectory spheres
        self.add_trajectory_spheres(traj, radius=radius, offset=offset)
        
        # Setup collision pairs
        self.setup_collision_pairs()
        
        # Check for collision
        return self.is_in_collision(q)
        
        
def main():
    """Main test function."""
    
    print("COLLISION TESTER - Robot Trajectory Collision Detection")
    
    # Initialize tester
    tester = Collision()
    
    # Define offset transformation
    offset = SE3(
        rotation=SO3.rz(np.pi),
        translation=np.array([0.45, -0.15, 0.05])
    )
    
    # Get trajectory for specific puzzles
    trajectory_A = RobotTrajectory.get_trajectory_se3('A', segment_length=0.01)
    trajectory_B = RobotTrajectory.get_trajectory_se3('B', segment_length=0.01)
    # trajectory_C = RobotTrajectory.get_trajectory_se3('C', segment_length=0.01)
    
    # Test configuration
    q = np.array([
        -4.06427842e-01, -1.11658486e+00, -1.69536048e+00,
         0.00000000e+00, -3.28547760e-01, -4.04954404e-01
    ])

    # Test with trajectory A
    in_collision_A = tester.in_collision(q, trajectory_A, radius=0.007, offset=offset)
    print(f" Trajectory A: {'COLLISION' if in_collision_A else 'NO COLLISION'}")

    # Test with trajectory B (clean() is called automatically)
    in_collision_B = tester.in_collision(q, trajectory_B, radius=0.007, offset=offset)
    print(f" Trajectory B: {'COLLISION' if in_collision_B else 'NO COLLISION'}")
    
    # Or manually clean and test
    # tester.clean()
    # tester.add_trajectory_spheres(trajectory_C, radius=0.007, offset=offset)
    # tester.setup_collision_pairs()
    # in_collision_C = tester.is_in_collision(q)
    
    # Visualize the last trajectory
    # num_collisions = tester.check_collisions(q, verbose=True)
    tester.visualize(q, wait_for_input=True)
    
    print(f"\n✅ Test complete")


if __name__ == "__main__":
    main()
