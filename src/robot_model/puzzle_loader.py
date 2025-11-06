"""
Utilities to load puzzle STL meshes (A–E) as Pinocchio GeometryObjects
and add them into collision and visual models.

Meshes are expected under: <robot_model_dir>/puzzles/{A,B,C,D,E}.stl
"""

from __future__ import annotations

from pathlib import Path
from typing import Tuple, Iterable

import numpy as np
import pinocchio as pin
import hppfcl


# Map category to STL relative path under robot_model_dir
PUZZLE_CATEGORY_TO_STL = {
    "A": Path("puzzles") / "A.stl",
    "B": Path("puzzles") / "B.stl",
    "C": Path("puzzles") / "C.stl",
    "D": Path("puzzles") / "D.stl",
    "E": Path("puzzles") / "E.stl",
}


def _robot_model_dir() -> Path:
    """Return the absolute path to the robot_model directory."""
    # This file lives in <...>/src/robot_model/, so its parent is robot_model
    return Path(__file__).resolve().parent


def load_puzzle_geometry(
    category: str,
    *,
    parent_joint: int = 0,
    placement: pin.SE3 | None = None,
    scale: float = 0.001,
) -> Tuple[pin.GeometryObject, pin.GeometryObject]:
    """
    Load the STL for a puzzle category and return (collision_go, visual_go).

    Parameters
    ----------
    category : str
        One of "A", "B", "C", "D", "E" (case-insensitive).
    parent_joint : int
        Joint index to attach the geometry to. 0 means world frame.
    placement : pin.SE3 | None
        Pose of the mesh w.r.t. parent joint. Identity when None.
    scale : float
        Uniform mesh scale (meters: STL often in mm so 0.001 is common).
    """
    cat = category.upper()
    if cat not in PUZZLE_CATEGORY_TO_STL:
        raise ValueError(f"Unknown puzzle category '{category}'. Expected one of {sorted(PUZZLE_CATEGORY_TO_STL)}")

    mesh_rel = PUZZLE_CATEGORY_TO_STL[cat]
    mesh_path = (_robot_model_dir() / mesh_rel).resolve()
    if not mesh_path.exists():
        raise FileNotFoundError(f"Puzzle mesh not found: {mesh_path}")

    if placement is None:
        placement = pin.SE3.Identity()

    loader = hppfcl.MeshLoader()
    shape = loader.load(str(mesh_path))

    # Collision geometry object
    go_coll = pin.GeometryObject(
        f"puzzle_{cat}_collision",
        parent_joint,
        placement,
        shape,
    )
    go_coll.meshPath = str(mesh_path)
    go_coll.meshScale = np.array([scale, scale, scale])

    # Visual geometry object (duplicate with a different name)
    go_vis = go_coll.copy()
    go_vis.name = f"puzzle_{cat}_visual"

    return go_coll, go_vis


def add_puzzle_to_scene(
    collision_model: pin.GeometryModel,
    visual_model: pin.GeometryModel,
    category: str,
    position_xyz: Iterable[float] = (0.3, 0.2, 0.05),
    *,
    scale: float = 0.001,
    parent_joint: int = 0,
) -> tuple[pin.GeometryObject, pin.GeometryObject]:
    """
    Convenience helper: create placement from XYZ, load puzzle geo, and add to models.

    Returns the (collision_go, visual_go) that were added.
    """
    xyz = np.asarray(list(position_xyz), dtype=float)
    if xyz.shape != (3,):
        raise ValueError("position_xyz must be length-3 iterable [x,y,z]")

    placement = pin.SE3(np.eye(3), xyz)
    go_coll, go_vis = load_puzzle_geometry(
        category,
        parent_joint=parent_joint,
        placement=placement,
        scale=scale,
    )

    collision_model.addGeometryObject(go_coll)
    visual_model.addGeometryObject(go_vis)

    return go_coll, go_vis
