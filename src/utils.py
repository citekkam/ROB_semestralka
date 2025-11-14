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


def plot_trajectory_3d(
    full_trajectory,
    show_orientations: bool = True,
    orientation_stride: int = 1,
    orientation_scale: float = 0.01,
    show_euler_plots: bool = False,
    euler_sequence: str = "zyx",
    keep_equal_scale: bool = True,
):
    """Vykreslí trajektorii v 3D prostoru (pozice + volitelně orientace).

    Args:
        full_trajectory: list[SE3] sekvence transformací.
        show_orientations: zda vykreslit lokální osy (x,y,z) podél trajektorie.
        orientation_stride: každá n-tá pozice pro vykreslení orientace (řídí hustotu).
        orientation_scale: délka vykreslených os (šířka šipek / délka vektorů).
        show_euler_plots: navíc vykreslí grafy Eulerových úhlů vs index.
        euler_sequence: pořadí os pro převod na Eulerovy úhly (např. "zyx").
    """
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (potřebné pro 3D projekci)

    if not full_trajectory:
        print("Prázdná trajektorie – nic k vykreslení.")
        return

    # Pozice z transformací
    trajectory_positions = np.array([T.translation for T in full_trajectory])

    fig = plt.figure(figsize=(11, 8))
    ax = fig.add_subplot(111, projection="3d")

    # Křivka trajektorie
    ax.plot(
        trajectory_positions[:, 0],
        trajectory_positions[:, 1],
        trajectory_positions[:, 2],
        "b-",
        linewidth=2,
        label="Trajectory",
        alpha=0.7,
    )

    # Body trajektorie (jemnější vzorek)
    ax.scatter(
        trajectory_positions[:, 0],
        trajectory_positions[:, 1],
        trajectory_positions[:, 2],
        c="cyan",
        s=14,
        marker=".",
        label="Samples",
        alpha=0.55,
    )

    if show_orientations:
        # Vykreslení orientačních os pro každou n-tou transformaci
        xs_o, ys_o, zs_o = [], [], []  # počátky
        ux_all, uy_all, uz_all = [], [], []  # x osa
        vx_all, vy_all, vz_all = [], [], []  # y osa
        wx_all, wy_all, wz_all = [], [], []  # z osa

        for i, T in enumerate(full_trajectory):
            if i % max(1, orientation_stride) != 0:
                continue
            R = T.rotation.rot  # 3x3 rotační matice
            p = T.translation
            # Lokální osy v globálním systému
            x_axis = R[:, 0]
            y_axis = R[:, 1]
            z_axis = R[:, 2]
            xs_o.append(p[0]); ys_o.append(p[1]); zs_o.append(p[2])
            ux_all.append(x_axis[0]); uy_all.append(x_axis[1]); uz_all.append(x_axis[2])
            vx_all.append(y_axis[0]); vy_all.append(y_axis[1]); vz_all.append(y_axis[2])
            wx_all.append(z_axis[0]); wy_all.append(z_axis[1]); wz_all.append(z_axis[2])

        if xs_o:
            # Quivery pro tři osy – barvy X=red, Y=green, Z=blue
            ax.quiver(xs_o, ys_o, zs_o, ux_all, uy_all, uz_all,
                      length=orientation_scale, normalize=True, color="r", linewidth=1,
                      label="X axis")
            ax.quiver(xs_o, ys_o, zs_o, vx_all, vy_all, vz_all,
                      length=orientation_scale, normalize=True, color="g", linewidth=1,
                      label="Y axis")
            ax.quiver(xs_o, ys_o, zs_o, wx_all, wy_all, wz_all,
                      length=orientation_scale, normalize=True, color="b", linewidth=1,
                      label="Z axis")

    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.set_title("Robot trajectory (positions + orientations)")
    
    # Zajištění stejného měřítka ve všech třech osách (aspect ratio 1:1:1)
    if keep_equal_scale:
        try:
            # Novější Matplotlib (>=3.4) má set_box_aspect
            ax.set_box_aspect((1, 1, 1))
        except Exception:
            # Fallback ruční úprava limitů
            mins = trajectory_positions.min(axis=0)
            maxs = trajectory_positions.max(axis=0)
            ranges = maxs - mins
            max_range = ranges.max()
            centers = (maxs + mins) / 2.0
            ax.set_xlim(centers[0] - max_range / 2, centers[0] + max_range / 2)
            ax.set_ylim(centers[1] - max_range / 2, centers[1] + max_range / 2)
            ax.set_zlim(centers[2] - max_range / 2, centers[2] + max_range / 2)
    ax.grid(True)
    ax.legend(loc="upper left", fontsize=9)

    plt.tight_layout()
    plt.show()

    if show_euler_plots:
        # Druhý obrázek s průběhy Eulerových úhlů v radianech
        import math

        def rot_to_euler(R: np.ndarray, seq: str):
            # Jednoduchý převod pro běžné pořadí 'zyx' (yaw-pitch-roll)
            if seq == "zyx":
                yaw = math.atan2(R[1, 0], R[0, 0])
                pitch = math.asin(-R[2, 0])
                roll = math.atan2(R[2, 1], R[2, 2])
                return roll, pitch, yaw
            # Můžeme rozšířit dle potřeby – fallback: nulové
            return 0.0, 0.0, 0.0

        eulers = np.array([rot_to_euler(T.rotation.rot, euler_sequence) for T in full_trajectory])
        fig2, ax2 = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
        labels = ["Roll", "Pitch", "Yaw"]
        for i in range(3):
            ax2[i].plot(eulers[:, i], label=labels[i], color=["r", "g", "b"][i])
            ax2[i].set_ylabel(f"{labels[i]} [rad]")
            ax2[i].grid(True)
        ax2[0].set_title(f"Euler úhly sekvence '{euler_sequence}'")
        ax2[-1].set_xlabel("Index bodu trajektorie")
        plt.tight_layout()
        plt.show()

    