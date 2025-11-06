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


class CollisionTester:
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
    
    def add_trajectory_spheres(self, puzzle: str = 'A', segment_length: float = 0.05,
                              radius: float = 0.03, offset: SE3 = None) -> list:
        """
        Add spheres along trajectory for collision checking.
        
        Args:
            puzzle: Puzzle identifier ('A', 'B', 'C', 'D', 'E')
            segment_length: Distance between spheres in meters
            radius: Sphere radius in meters
            offset: Optional SE3 transformation for all spheres
        
        Returns:
            List of added sphere pairs
        """
        # Get trajectory using RobotTrajectory
        trajectory = RobotTrajectory.get_trajectory_se3(puzzle, segment_length=segment_length)
        
        offset_str = " with offset" if offset is not None else ""
        print(f"\n=== Adding {len(trajectory)} spheres from Puzzle {puzzle} trajectory{offset_str} ===")
        
        spheres = []
        for i, T in enumerate(trajectory):
            position = T.translation
            sphere_pair = self.add_sphere(
                position,
                radius=radius,
                name_suffix=f"traj_{puzzle}_{i}",
                offset=offset
            )
            spheres.append(sphere_pair)
        
        self.trajectory_spheres.extend(spheres)
        print(f"✓ Added {len(spheres)} spheres (radius={radius}m, spacing={segment_length}m)")
        
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
                    print(f"⚠️  COLLISION: {g1} ↔ {g2}")
        
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


def main():
    """Main test function."""
    
    print("COLLISION TESTER - Robot Trajectory Collision Detection")
    
    # Initialize tester
    tester = CollisionTester()
    
    # Define offset transformation
    offset = SE3(
        rotation=SO3.rz(np.pi),
        translation=np.array([0.45, -0.15, 0.05])
    )
    
    # Add trajectory spheres
    tester.add_trajectory_spheres(
        puzzle='C',
        segment_length=0.01,  # 1cm spacing
        radius=0.007,         # 7mm radius
        offset=offset
    )
    
    # Setup collision pairs
    tester.setup_collision_pairs()
    
    # Test configuration
    q = np.array([
        -4.06427842e-01, -1.11658486e+00, -1.69536048e+00,
         0.00000000e+00, -3.28547760e-01, -4.04954404e-01
    ])

    # Simple collision check
    in_collision = tester.is_in_collision(q)
    print(f"Robot in collision: {in_collision}")

    # Detailed collision check
    num_collisions = tester.check_collisions(q, verbose=True)
    
    # Visualize
    tester.visualize(q, wait_for_input=True)
    
    print(f"\n✅ Test complete: {'COLLISION' if in_collision else 'NO COLLISION'} ({num_collisions} collision pairs)")


if __name__ == "__main__":
    main()
