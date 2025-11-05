from copy import deepcopy
from pathlib import Path

import numpy as np

import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
import hppfcl


# STL files used for the puzzle observer per category label.
# Update the relative paths if your meshes live elsewhere.
PUZZLE_CATEGORY_TO_STL = {
    "A": Path("puzzles") / "A.stl",
    "B": Path("puzzles") / "B.stl",
    "C": Path("puzzles") / "C.stl",
    "D": Path("puzzles") / "D.stl",
    "E": Path("puzzles") / "E.stl",
}


def load_puzzle_geometry(category: str, parent_joint: int, placement: pin.SE3, scale: float = 0.001) -> pin.GeometryObject:
    """
    Load the STL mesh assigned to a puzzle category and wrap it as a Pinocchio GeometryObject.

    Parameters
    ----------
    category : str
        Puzzle identifier (A–E).
    parent_joint : int
        Joint index the geometry should follow. Use 0 for a fixed obstacle in world frame.
    placement : pin.SE3
        Pose of the mesh relative to the parent joint.
    scale : float, optional
        Uniform mesh scaling factor.
    """
    category = category.upper()
    try:
        rel_path = PUZZLE_CATEGORY_TO_STL[category]
    except KeyError as exc:
        raise ValueError(f"Unknown puzzle category '{category}'. Expected one of {sorted(PUZZLE_CATEGORY_TO_STL)}.") from exc

    mesh_path = (Path(__file__).resolve().parents[1] / rel_path).resolve()
    if not mesh_path.exists():
        raise FileNotFoundError(f"STL for puzzle '{category}' not found at {mesh_path}. Update PUZZLE_CATEGORY_TO_STL.")

    loader = hppfcl.MeshLoader()
    shape = loader.load(str(mesh_path))

    geometry = pin.GeometryObject(
        f"puzzle_{category}_collision",
        parent_joint,
        placement,
        shape,
    )
    geometry.meshPath = str(mesh_path)
    geometry.meshScale = np.array([scale, scale, scale])
    return geometry


def _extract_penetration_depth(result) -> float:
    """Best-effort penetration depth extraction that works across FCL bindings."""
    depth = float("nan")
    if hasattr(result, "contacts"):
        contacts = result.contacts
        if contacts:
            depth = contacts[0].penetration_depth
    elif hasattr(result, "numContacts") and result.numContacts() > 0:
        depth = result.getContact(0).penetration_depth
    return depth


# Resolve URDF and mesh search paths relative to the repository root.
urdf_path = Path(__file__).resolve().parents[1] / "urdf" / "crs_a465.urdf"
package_dirs = [str(Path(__file__).resolve().parents[1])]  # robot_model root directory

# Build the kinematic model alongside collision and visual geometry models.
model = pin.buildModelFromUrdf(str(urdf_path))
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

# Attach the puzzle observer geometry to the world.
observer_pose = pin.SE3(np.eye(3), np.array([5.35066584e-01, 1.49730794e-01, 4.98366670e-02]))
observer_category = "A"
observer_collision = load_puzzle_geometry(observer_category, parent_joint=0, placement=observer_pose)
observer_visual = deepcopy(observer_collision)
observer_visual.name = observer_visual.name.replace("_collision", "_visual")

collision_model.addGeometryObject(observer_collision)
visual_model.addGeometryObject(observer_visual)

# Add collision pairs only between robot and observer (not self-collisions)
collision_model.addAllCollisionPairs()

# Remove self-collision pairs (adjacent links)
for i in range(collision_model.ngeoms):
    for j in range(i, collision_model.ngeoms):
        geom1 = collision_model.geometryObjects[i]
        geom2 = collision_model.geometryObjects[j]
        
        # Skip observer collisions (we want those)
        if "puzzle" in geom1.name or "puzzle" in geom2.name:
            continue
            
        # Remove self-collisions between robot parts
        if (geom1.name != geom2.name and 
            not "puzzle" in geom1.name and 
            not "puzzle" in geom2.name):
            pair = pin.CollisionPair(i, j)
            if collision_model.existCollisionPair(pair):
                collision_model.removeCollisionPair(pair)

# Use the neutral configuration as a starting configuration.
# q = np.array([0, 0, 0, 0, 0, 0])
q1 = np.array([-4.06427842e-01, -1.11658486e+00, -1.69536048e+00,  0.00000000e+00, -3.28547760e-01, -4.04954404e-01])
q2 = np.array([-3.58125855e-01, -1.17652645e+00, -1.50760319e+00,  0.00000000e+00, -4.58609696e-01, -3.58763660e-01])
q11 = np.array([3.58801297e-01, -1.17655786e+00, -1.50760319e+00,  0.00000000e+00, -4.58672527e-01,  3.58701450e-01])


def compute_collision_contacts(q: np.ndarray, *, stop_at_first_collision: bool = False):
    """
    Evaluate the robot configuration for collisions against the observer mesh and return contact info.

    Parameters
    ----------
    q : np.ndarray
        Joint configuration (size model.nq).
    stop_at_first_collision : bool, optional
        Early exit when the first collision is detected to speed up queries.
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
    print(f"Total collisions detected: {len(contacts)}")
    return contacts


def is_configuration_in_collision(q: np.ndarray) -> bool:
    """Return True if the provided configuration is in collision with the observer mesh."""
    return bool(compute_collision_contacts(q, stop_at_first_collision=True))


def main():
    q = q2
    print("Checking for collisions with observer geometry...")
    print("q =", q)

    contacts = compute_collision_contacts(q)
    print(f"Collision detected? {bool(contacts)}")

    if contacts:
        print("Contact summary:")
        for contact in contacts:
            print(f"  {contact['first']} ↔ {contact['second']}, penetration {contact['penetration']:.4f} m")

    viz = MeshcatVisualizer(model, collision_model, visual_model)
    viz.initViewer(open=True)
    viz.loadViewerModel()
    viz.display(q)
    input("Press Enter to exit...")
    # viz.quit()  # This method doesn't exist in newer versions


if __name__ == "__main__":
    main()
