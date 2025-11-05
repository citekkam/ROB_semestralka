#!/usr/bin/env python3
"""
Simple test script to verify that URDF and collision model loads correctly.
"""

from pathlib import Path
import sys

def test_urdf_loading():
    """Test if URDF loads without errors"""
    try:
        import pinocchio as pin
        print("✓ Pinocchio imported successfully")
    except ImportError as e:
        print(f"✗ Failed to import pinocchio: {e}")
        print("Run: python install_dependencies.py")
        return False
    
    try:
        # Test URDF loading
        urdf_path = Path(__file__).resolve().parent / "urdf" / "crs_a465.urdf"
        package_dirs = [str(Path(__file__).resolve().parent)]
        
        print(f"Loading URDF from: {urdf_path}")
        print(f"Package directories: {package_dirs}")
        
        model = pin.buildModelFromUrdf(str(urdf_path))
        print(f"✓ Kinematic model loaded: {model.nq} DOF, {model.njoints} joints")
        
        collision_model = pin.buildGeomFromUrdf(
            model,
            str(urdf_path),
            geom_type=pin.GeometryType.COLLISION,
            package_dirs=package_dirs,
        )
        print(f"✓ Collision model loaded: {collision_model.ngeoms} geometries")
        
        visual_model = pin.buildGeomFromUrdf(
            model,
            str(urdf_path),
            geom_type=pin.GeometryType.VISUAL,
            package_dirs=package_dirs,
        )
        print(f"✓ Visual model loaded: {visual_model.ngeoms} geometries")
        
        return True
        
    except Exception as e:
        print(f"✗ Failed to load URDF: {e}")
        return False

def test_puzzle_geometry():
    """Test if puzzle STL files can be loaded"""
    try:
        import hppfcl
        print("✓ hpp-fcl imported successfully")
    except ImportError as e:
        print(f"✗ Failed to import hppfcl: {e}")
        return False
    
    puzzle_path = Path(__file__).resolve().parent / "puzzles" / "A.stl"
    if not puzzle_path.exists():
        print(f"✗ Puzzle STL not found: {puzzle_path}")
        return False
    
    try:
        loader = hppfcl.MeshLoader()
        shape = loader.load(str(puzzle_path))
        print(f"✓ Puzzle geometry loaded: {puzzle_path}")
        return True
    except Exception as e:
        print(f"✗ Failed to load puzzle geometry: {e}")
        return False

def main():
    print("=== Robot Model Test ===\n")
    
    urdf_ok = test_urdf_loading()
    print()
    puzzle_ok = test_puzzle_geometry()
    
    print("\n=== Test Summary ===")
    if urdf_ok and puzzle_ok:
        print("✓ All tests passed! Collision detection should work.")
        print("Run: python test_collision/collision_check.py")
    else:
        print("✗ Some tests failed. Check dependencies and file paths.")
        if not urdf_ok:
            print("  - URDF loading failed")
        if not puzzle_ok:
            print("  - Puzzle geometry loading failed")

if __name__ == "__main__":
    main()