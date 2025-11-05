from __future__ import annotations

import numpy as np
import time

from pinocchio.visualize import MeshcatVisualizer

# Reuse the collision pipeline set up in collision_check.py
from collision_check import (
    collision_model,
    compute_collision_contacts,
    is_configuration_in_collision,
    model,
    q1,
    q11,
    q2,
    visual_model,
)


def sample_trajectory(start: np.ndarray, goal: np.ndarray, num_samples: int) -> np.ndarray:
    """Linearly interpolate `num_samples` configurations between start and goal."""
    alphas = np.linspace(0.0, 1.0, num_samples)
    return np.array([(1.0 - a) * start + a * goal for a in alphas])


def evaluate_trajectory(trajectory: np.ndarray):
    """Check each configuration for collision and return boolean results plus timing."""
    start_time = time.perf_counter()
    collision_flags = [is_configuration_in_collision(q) for q in trajectory]
    elapsed = time.perf_counter() - start_time
    return collision_flags, elapsed


def main():
    num_samples = 50
    start_config = q1
    goal_config = q11

    print(f"Sampling {num_samples} configurations between q1 and q11.")
    trajectory = sample_trajectory(start_config, goal_config, num_samples)

    collision_flags, elapsed = evaluate_trajectory(trajectory)
    num_colliding = sum(collision_flags)

    print(f"Collision queries completed in {elapsed:.4f} s.")
    print(f"Average per configuration: {elapsed / num_samples:.6f} s.")
    print(f"{num_colliding}/{num_samples} configurations are in collision.")

    viz = MeshcatVisualizer(model, collision_model, visual_model)
    viz.initViewer(open=True)
    viz.loadViewerModel()

    animation_dt = 0.1  # seconds between frames

    for idx, (q, in_collision) in enumerate(zip(trajectory, collision_flags)):
        status = "COLLISION" if in_collision else "free"
        print(f"[{idx:02d}] {status}: {q}")
        viz.display(q)
        time.sleep(animation_dt)
        if in_collision:
            contacts = compute_collision_contacts(q)
            for contact in contacts:
                print(
                    f"     - {contact['first']} ↔ {contact['second']}, "
                    f"penetration {contact['penetration']:.4f} m"
                )

    viz.display(goal_config)
    input("Press Enter to close the MeshCat viewer...")
    # viz.quit()  # This method doesn't exist in newer versions


if __name__ == "__main__":
    main()
