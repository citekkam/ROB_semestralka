#!/usr/bin/env python3
"""
Improved collision detection that properly handles self-collisions.
"""

from copy import deepcopy
from pathlib import Path
import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
import hppfcl

# Import the basic setup from collision_check
from collision_check import (
    PUZZLE_CATEGORY_TO_STL,
    load_puzzle_geometry,
    _extract_penetration_depth,
    model,
    q1, q2, q11
)

def setup_collision_model_smart():
    """
    Set up collision model with proper filtering of self-collisions.
    Only keeps collisions between robot and observer (puzzle).
    """
    urdf_path = Path(__file__).resolve().parents[1] / "urdf" / "crs_a465.urdf"
    package_dirs = [str(Path(__file__).resolve().parents[1])]
    
    # Build models
    collision_model = pin.buildGeomFromUrdf(
        model,
        str(urdf_path),
        geom_type=pin.GeometryType.COLLISION,
        package_dirs=package_dirs,
    )
    visual_model = pin.buildGeomFromUrdf(
        model,
        str(urdf_path),
        geom_type=pin.GeometryType.VISUAL,
        package_dirs=package_dirs,
    )
    
    # Add puzzle geometry
    observer_pose = pin.SE3(np.eye(3), np.array([5.35066584e-01, -1.19730794e-01, 4.98366670e-02]))
    observer_category = "A"
    observer_collision = load_puzzle_geometry(observer_category, parent_joint=0, placement=observer_pose)
    observer_visual = deepcopy(observer_collision)
    observer_visual.name = observer_visual.name.replace("_collision", "_visual")
    
    collision_model.addGeometryObject(observer_collision)
    visual_model.addGeometryObject(observer_visual)
    
    # Only add collision pairs between robot parts and the observer
    observer_idx = collision_model.ngeoms - 1  # Last added geometry
    
    for i in range(collision_model.ngeoms - 1):  # All robot geometries
        pair = pin.CollisionPair(i, observer_idx)
        collision_model.addCollisionPair(pair)
    
    print(f"Collision model setup:")
    print(f"  - {collision_model.ngeoms} geometries")
    print(f"  - {len(collision_model.collisionPairs)} collision pairs")
    print(f"  - Observer geometry: {observer_collision.name}")
    
    return collision_model, visual_model

def compute_collision_contacts_smart(q: np.ndarray, collision_model, *, stop_at_first_collision: bool = False):
    """
    Smart collision detection that only checks robot vs observer.
    """
    q = np.asarray(q, dtype=float)
    if q.shape != (model.nq,):
        raise ValueError(f"Expected q shape {(model.nq,)}, received {q.shape}.")

    data = model.createData()
    collision_data = pin.GeometryData(collision_model)

    pin.forwardKinematics(model, data, q)
    pin.updateGeometryPlacements(model, data, collision_model, collision_data)
    pin.computeCollisions(collision_model, collision_data, stop_at_first_collision=stop_at_first_collision)

    contacts = []
    for pair, result in zip(collision_model.collisionPairs, collision_data.collisionResults):
        if not result.isCollision():
            continue
        first = collision_model.geometryObjects[pair.first].name
        second = collision_model.geometryObjects[pair.second].name
        penetration = _extract_penetration_depth(result)
        contacts.append({"first": first, "second": second, "penetration": penetration})
        if stop_at_first_collision:
            print("Early exit after first collision.")
            break
    return contacts

def main():
    print("Setting up smart collision detection...")
    collision_model, visual_model = setup_collision_model_smart()
    
    print(f"\nTesting configuration q2...")
    q = q2
    print("q =", q)

    contacts = compute_collision_contacts_smart(q, collision_model)
    print(f"Collision detected? {bool(contacts)}")

    if contacts:
        print("Contact summary:")
        for contact in contacts:
            print(f"  {contact['first']} ↔ {contact['second']}, penetration {contact['penetration']:.4f} m")
    else:
        print("No collisions detected with observer!")

    # Test other configurations
    test_configs = {"q1": q1, "q11": q11}
    for name, q_test in test_configs.items():
        contacts_test = compute_collision_contacts_smart(q_test, collision_model, stop_at_first_collision=True)
        status = "COLLISION" if contacts_test else "free"
        print(f"{name}: {status}")

    # Visualization
    print(f"\nStarting visualization...")
    viz = MeshcatVisualizer(model, collision_model, visual_model)
    viz.initViewer(open=True)
    viz.loadViewerModel()
    viz.display(q)
    print("Check the MeshCat viewer in your browser!")
    input("Press Enter to exit...")

if __name__ == "__main__":
    main()