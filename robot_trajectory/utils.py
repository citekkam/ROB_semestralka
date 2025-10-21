import numpy as np
import sys
from pathlib import Path

# Add parent directory to path to import SE3 and SO3
sys.path.append(str(Path(__file__).parent.parent))

from se3 import SE3
from so3 import SO3


def print_trajectory_info(trajectory_points, puzzle_name="Default"):
    """Print information about the trajectory points."""
    
    print(f"Trajectory Points for {puzzle_name}:")
    print("=" * 60)
    for i, point in enumerate(trajectory_points, 1):
        print(f"\nPoint {i}:")
        print(f"Position: {point.translation}")
        print(f"Rotation matrix:\n{point.rotation.rot}")
        print(f"Homogeneous matrix:\n{point.homogeneous()}")
    print("=" * 60)


def visualize_trajectory(full_trajectory):
    """
    Visualize the trajectory by printing positions.
    
    Args:
        full_trajectory: List of SE3 transformations representing the full trajectory
    """
    print(f"Trajectory with {len(full_trajectory)} points")
    print("\nSample points:")
    print("-" * 60)
    
    # Print every 10th point
    step = max(1, len(full_trajectory) // 10)
    for i in range(0, len(full_trajectory), step):
        T = full_trajectory[i]
        pos = T.translation
        print(f"Point {i:3d}: Position = [{pos[0]:7.4f}, {pos[1]:7.4f}, {pos[2]:7.4f}]")


def plot_trajectory_3d(full_trajectory):
    """
    Plot the trajectory in 3D using matplotlib.
    
    Args:
        full_trajectory: List of SE3 transformations representing the full trajectory
    """
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    
    # Extract positions
    trajectory_positions = np.array([T.translation for T in full_trajectory])
    
    # Create 3D plot
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot interpolated trajectory
    ax.plot(trajectory_positions[:, 0], 
            trajectory_positions[:, 1], 
            trajectory_positions[:, 2], 
            'b-', linewidth=2, label='Trajectory', alpha=0.6)
    
    # Plot interpolated points
    ax.scatter(trajectory_positions[:, 0], 
               trajectory_positions[:, 1], 
               trajectory_positions[:, 2], 
               c='cyan', s=20, marker='.', label='Interpolated points', alpha=0.5)
    
    # Labels and title
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    ax.set_title('Robot Trajectory')
    ax.legend()
    ax.grid(True)
    
    plt.tight_layout()
    plt.show()

    