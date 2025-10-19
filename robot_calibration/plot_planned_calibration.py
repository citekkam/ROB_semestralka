#!/usr/bin/env python3
"""
3D Visualization of Planned Calibration Positions
==================================================
Visualizes the 9 positions defined in calibration_positions.yaml
Shows the 3x3 grid pattern planned for hand-eye calibration.

Author: David
Date: 2025-10-19
"""

import numpy as np
import yaml
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from pathlib import Path


def load_planned_positions(yaml_file):
    """
    Load planned calibration positions from YAML file.
    
    Args:
        yaml_file: Path to calibration_positions.yaml
        
    Returns:
        List of dictionaries with 'name', 'pose', 'position', 'orientation'
    """
    with open(yaml_file, 'r') as f:
        data = yaml.safe_load(f)
    
    positions_list = []
    
    # Sort keys numerically by extracting the position number
    position_keys = [key for key in data.keys() if 'position_' in key]
    sorted_keys = sorted(position_keys, key=lambda x: int(x.split('_')[1]))
    
    for key in sorted_keys:
        pose = np.array(data[key]['pose'])
        position = pose[:3]  # x, y, z
        orientation = pose[3:]  # roll, pitch, yaw
        
        positions_list.append({
            'name': key,
            'pose': pose,
            'position': position,
            'orientation': orientation
        })
    
    return positions_list


def pose_to_transformation_matrix(pose):
    """
    Convert pose [x, y, z, roll, pitch, yaw] to 4x4 transformation matrix.
    
    Args:
        pose: 6D array [x, y, z, roll, pitch, yaw]
        
    Returns:
        4x4 homogeneous transformation matrix
    """
    x, y, z, roll, pitch, yaw = pose
    
    # Rotation matrix from RPY (ZYX convention)
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    
    R = np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp, cp*sr, cp*cr]
    ])
    
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    
    return T


def plot_coordinate_frame(ax, T, scale=0.03, alpha=0.8, linewidth=2):
    """
    Plot a coordinate frame (RGB = XYZ) at the given transformation.
    
    Args:
        ax: Matplotlib 3D axis
        T: 4x4 transformation matrix
        scale: Length of axes arrows
        alpha: Transparency
        linewidth: Line width
    """
    origin = T[:3, 3]
    x_axis = T[:3, 0] * scale
    y_axis = T[:3, 1] * scale
    z_axis = T[:3, 2] * scale
    
    # X axis (red)
    ax.quiver(origin[0], origin[1], origin[2],
              x_axis[0], x_axis[1], x_axis[2],
              color='r', arrow_length_ratio=0.3, linewidth=linewidth, alpha=alpha)
    
    # Y axis (green)
    ax.quiver(origin[0], origin[1], origin[2],
              y_axis[0], y_axis[1], y_axis[2],
              color='g', arrow_length_ratio=0.3, linewidth=linewidth, alpha=alpha)
    
    # Z axis (blue)
    ax.quiver(origin[0], origin[1], origin[2],
              z_axis[0], z_axis[1], z_axis[2],
              color='b', arrow_length_ratio=0.3, linewidth=linewidth, alpha=alpha)


def plot_3d_grid(positions_list):
    """
    Create 3D plot showing the planned calibration grid.
    
    Args:
        positions_list: List of position dictionaries
    """
    fig = plt.figure(figsize=(16, 12))
    
    # Main 3D plot with coordinate frames
    ax1 = fig.add_subplot(221, projection='3d')
    
    positions = np.array([p['position'] for p in positions_list])
    
    # Plot positions as spheres
    ax1.scatter(positions[:, 0], positions[:, 1], positions[:, 2],
               c='cyan', marker='o', s=200, alpha=0.6, edgecolors='blue', linewidth=2,
               label='Planned positions')
    
    # Plot coordinate frames at each position
    for i, pos_data in enumerate(positions_list):
        T = pose_to_transformation_matrix(pos_data['pose'])
        plot_coordinate_frame(ax1, T, scale=0.025, alpha=0.8, linewidth=2)
        
        # Add position labels
        ax1.text(positions[i, 0], positions[i, 1], positions[i, 2] + 0.01,
                f"{i+1}", fontsize=12, fontweight='bold', ha='center')
    
    # Draw grid lines to show 3x3 structure
    # Identify rows and columns based on y and z coordinates
    unique_y = sorted(list(set(positions[:, 1])))
    unique_z = sorted(list(set(positions[:, 2])))
    
    # Draw horizontal lines (same Z level)
    for z_val in unique_z:
        z_positions = positions[np.abs(positions[:, 2] - z_val) < 0.001]
        z_positions = z_positions[np.argsort(z_positions[:, 1])]
        ax1.plot(z_positions[:, 0], z_positions[:, 1], z_positions[:, 2],
                'k--', alpha=0.3, linewidth=1)
    
    # Draw vertical lines (same Y level)
    for y_val in unique_y:
        y_positions = positions[np.abs(positions[:, 1] - y_val) < 0.001]
        y_positions = y_positions[np.argsort(y_positions[:, 2])]
        ax1.plot(y_positions[:, 0], y_positions[:, 1], y_positions[:, 2],
                'k--', alpha=0.3, linewidth=1)
    
    ax1.set_xlabel('X (m)', fontsize=12, fontweight='bold')
    ax1.set_ylabel('Y (m)', fontsize=12, fontweight='bold')
    ax1.set_zlabel('Z (m)', fontsize=12, fontweight='bold')
    ax1.set_title('Planned Calibration Grid - 3D View', fontsize=14, fontweight='bold')
    ax1.legend(fontsize=10)
    ax1.grid(True, alpha=0.3)
    
    # Set viewing angle
    ax1.view_init(elev=20, azim=45)
    
    # XY plane (Top view)
    ax2 = fig.add_subplot(222)
    
    # Color code by Z level
    colors = plt.cm.viridis(np.linspace(0, 1, len(unique_z)))
    for i, z_val in enumerate(unique_z):
        mask = np.abs(positions[:, 2] - z_val) < 0.001
        z_positions = positions[mask]
        ax2.scatter(z_positions[:, 0], z_positions[:, 1], 
                   c=[colors[i]], s=150, alpha=0.7, edgecolors='black', linewidth=2,
                   label=f'Z = {z_val:.2f}m')
        
        # Add labels
        for j, pos in enumerate(z_positions):
            pos_idx = np.where((positions == pos).all(axis=1))[0][0]
            ax2.text(pos[0], pos[1], f" {pos_idx+1}", fontsize=10, fontweight='bold')
    
    # Draw grid
    for z_val in unique_z:
        z_positions = positions[np.abs(positions[:, 2] - z_val) < 0.001]
        z_positions = z_positions[np.argsort(z_positions[:, 1])]
        ax2.plot(z_positions[:, 0], z_positions[:, 1], 'k--', alpha=0.3, linewidth=1)
    
    for y_val in unique_y:
        y_positions = positions[np.abs(positions[:, 1] - y_val) < 0.001]
        y_positions = y_positions[np.argsort(y_positions[:, 2])]
        ax2.plot(y_positions[:, 0], y_positions[:, 1], 'k--', alpha=0.3, linewidth=1)
    
    ax2.set_xlabel('X (m)', fontsize=12, fontweight='bold')
    ax2.set_ylabel('Y (m)', fontsize=12, fontweight='bold')
    ax2.set_title('Top View (XY plane)', fontsize=14, fontweight='bold')
    ax2.legend(fontsize=9, loc='best')
    ax2.grid(True, alpha=0.3)
    ax2.axis('equal')
    
    # YZ plane (Front view)
    ax3 = fig.add_subplot(223)
    
    for i, z_val in enumerate(unique_z):
        mask = np.abs(positions[:, 2] - z_val) < 0.001
        z_positions = positions[mask]
        ax3.scatter(z_positions[:, 1], z_positions[:, 2],
                   c=[colors[i]], s=150, alpha=0.7, edgecolors='black', linewidth=2,
                   label=f'Z = {z_val:.2f}m')
        
        # Add labels
        for j, pos in enumerate(z_positions):
            pos_idx = np.where((positions == pos).all(axis=1))[0][0]
            ax3.text(pos[1], pos[2], f" {pos_idx+1}", fontsize=10, fontweight='bold')
    
    # Draw grid
    for z_val in unique_z:
        z_positions = positions[np.abs(positions[:, 2] - z_val) < 0.001]
        z_positions = z_positions[np.argsort(z_positions[:, 1])]
        ax3.plot(z_positions[:, 1], z_positions[:, 2], 'k--', alpha=0.3, linewidth=1)
    
    for y_val in unique_y:
        y_positions = positions[np.abs(positions[:, 1] - y_val) < 0.001]
        y_positions = y_positions[np.argsort(y_positions[:, 2])]
        ax3.plot(y_positions[:, 1], y_positions[:, 2], 'k--', alpha=0.3, linewidth=1)
    
    ax3.set_xlabel('Y (m)', fontsize=12, fontweight='bold')
    ax3.set_ylabel('Z (m)', fontsize=12, fontweight='bold')
    ax3.set_title('Front View (YZ plane)', fontsize=14, fontweight='bold')
    ax3.legend(fontsize=9, loc='best')
    ax3.grid(True, alpha=0.3)
    ax3.axis('equal')
    
    # XZ plane (Side view)
    ax4 = fig.add_subplot(224)
    
    # Color code by Y level
    colors_y = plt.cm.plasma(np.linspace(0, 1, len(unique_y)))
    for i, y_val in enumerate(unique_y):
        mask = np.abs(positions[:, 1] - y_val) < 0.001
        y_positions = positions[mask]
        ax4.scatter(y_positions[:, 0], y_positions[:, 2],
                   c=[colors_y[i]], s=150, alpha=0.7, edgecolors='black', linewidth=2,
                   label=f'Y = {y_val:.2f}m')
        
        # Add labels
        for j, pos in enumerate(y_positions):
            pos_idx = np.where((positions == pos).all(axis=1))[0][0]
            ax4.text(pos[0], pos[2], f" {pos_idx+1}", fontsize=10, fontweight='bold')
    
    # Draw grid
    for z_val in unique_z:
        z_positions = positions[np.abs(positions[:, 2] - z_val) < 0.001]
        z_positions = z_positions[np.argsort(z_positions[:, 1])]
        ax4.plot(z_positions[:, 0], z_positions[:, 2], 'k--', alpha=0.3, linewidth=1)
    
    for y_val in unique_y:
        y_positions = positions[np.abs(positions[:, 1] - y_val) < 0.001]
        y_positions = y_positions[np.argsort(y_positions[:, 2])]
        ax4.plot(y_positions[:, 0], y_positions[:, 2], 'k--', alpha=0.3, linewidth=1)
    
    ax4.set_xlabel('X (m)', fontsize=12, fontweight='bold')
    ax4.set_ylabel('Z (m)', fontsize=12, fontweight='bold')
    ax4.set_title('Side View (XZ plane)', fontsize=14, fontweight='bold')
    ax4.legend(fontsize=9, loc='best')
    ax4.grid(True, alpha=0.3)
    ax4.axis('equal')
    
    plt.tight_layout()
    
    return fig


def print_grid_info(positions_list):
    """
    Print information about the calibration grid.
    
    Args:
        positions_list: List of position dictionaries
    """
    positions = np.array([p['position'] for p in positions_list])
    orientations = np.array([p['orientation'] for p in positions_list])
    
    print("\n" + "="*70)
    print("PLANNED CALIBRATION GRID INFORMATION")
    print("="*70)
    print(f"Total positions: {len(positions_list)}")
    print()
    
    print("Grid structure:")
    unique_x = sorted(list(set(positions[:, 0])))
    unique_y = sorted(list(set(positions[:, 1])))
    unique_z = sorted(list(set(positions[:, 2])))
    
    print(f"  X levels: {len(unique_x)} → {unique_x}")
    print(f"  Y levels: {len(unique_y)} → {unique_y}")
    print(f"  Z levels: {len(unique_z)} → {unique_z}")
    print(f"  Grid pattern: {len(unique_z)}×{len(unique_y)} (Z×Y)")
    print()
    
    print("Position ranges:")
    print(f"  X: [{positions[:, 0].min():.3f}, {positions[:, 0].max():.3f}] m")
    print(f"  Y: [{positions[:, 1].min():.3f}, {positions[:, 1].max():.3f}] m")
    print(f"  Z: [{positions[:, 2].min():.3f}, {positions[:, 2].max():.3f}] m")
    print()
    
    print("Spacing:")
    if len(unique_y) > 1:
        y_spacing = np.diff(sorted(unique_y))
        print(f"  Y spacing: {y_spacing[0]:.3f} m")
    if len(unique_z) > 1:
        z_spacing = np.diff(sorted(unique_z))
        print(f"  Z spacing: {z_spacing[0]:.3f} m")
    print()
    
    print("Orientation (all positions):")
    print(f"  Roll:  {np.rad2deg(orientations[0, 0]):.1f}°")
    print(f"  Pitch: {np.rad2deg(orientations[0, 1]):.1f}°")
    print(f"  Yaw:   {np.rad2deg(orientations[0, 2]):.1f}°")
    print()
    
    print("Position list:")
    for i, pos_data in enumerate(positions_list, 1):
        pos = pos_data['position']
        print(f"  {i:2d}. {pos_data['name']:12s} → X={pos[0]:.2f}, Y={pos[1]:+.2f}, Z={pos[2]:.2f} m")
    
    print("="*70)


def main():
    """Main function."""
    print("="*70)
    print("3D VISUALIZATION OF PLANNED CALIBRATION POSITIONS")
    print("="*70)
    print()
    
    # Setup paths
    script_dir = Path(__file__).parent
    positions_file = script_dir / "robot_calibration" / "calibration_positions.yaml"
    
    if not positions_file.exists():
        print(f"❌ File {positions_file} does not exist!")
        return
    
    # Load positions
    print(f"📂 Loading positions from: {positions_file}")
    positions_list = load_planned_positions(positions_file)
    
    if not positions_list:
        print("❌ No positions found in file!")
        return
    
    print(f"✅ Loaded {len(positions_list)} planned positions")
    
    # Print info
    print_grid_info(positions_list)
    
    # Create plot
    print("\n📊 Creating 3D visualization...")
    fig = plot_3d_grid(positions_list)
    
    # Save plot
    output_dir = script_dir / "calibration_visualization"
    output_dir.mkdir(exist_ok=True)
    
    output_file = output_dir / "planned_positions_3d.png"
    fig.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"💾 Saved: {output_file}")
    
    print("\n✅ Visualization complete!")
    print("\n🖼️  Close the plot window to exit...")
    plt.show()


if __name__ == "__main__":
    main()
