#!/usr/bin/env python3
"""
3D Visualization of Robot Calibration Positions
================================================
Reads all calibration data from exporty/ folder and plots:
- End-effector positions in 3D space
- Coordinate frames showing orientation
- Robot joint configurations
- Interactive 3D plot with matplotlib

Author: David
Date: 2025-10-19
"""

import numpy as np
import yaml
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from pathlib import Path
import re


def load_calibration_data(exporty_folder):
    """
    Load all calibration data from YAML files.
    
    Args:
        exporty_folder: Path to folder containing data_*.yaml files
        
    Returns:
        List of dictionaries with 'name', 'robot_q', 'transformation_matrix', 'position', 'orientation'
    """
    exporty_path = Path(exporty_folder)
    yaml_files = sorted(exporty_path.glob("data_*.yaml"), 
                       key=lambda p: int(re.search(r'data_(\d+)', p.name).group(1)))
    
    data_list = []
    
    for yaml_file in yaml_files:
        with open(yaml_file, 'r') as f:
            content = f.read()
        
        # Parse Robot_pos_q
        robot_q_match = re.search(r'Robot_pos_q\s*:\s*\[(.*?)\]', content)
        if robot_q_match:
            robot_q_str = robot_q_match.group(1)
            robot_q = np.array([float(x.strip()) for x in robot_q_str.split(',')])
        else:
            continue
        
        # Parse transformation matrix - find all numbers after transformacni_matic
        matrix_start = content.find('transformacni_matic')
        if matrix_start != -1:
            matrix_section = content[matrix_start:]
            # Extract all numbers from the matrix section
            numbers = re.findall(r'[-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?', matrix_section)
            numbers = [float(x) for x in numbers[:16]]  # Take first 16 numbers
            if len(numbers) == 16:
                T = np.array(numbers).reshape(4, 4)
            else:
                print(f"Warning: {yaml_file.name} - found {len(numbers)} numbers instead of 16")
                continue
        else:
            continue
        
        # Extract position and orientation
        position = T[:3, 3]
        rotation_matrix = T[:3, :3]
        
        # Convert rotation matrix to Euler angles (ZYX convention)
        # Extract roll, pitch, yaw from rotation matrix
        if abs(rotation_matrix[2, 0]) < 0.99999:
            pitch = -np.arcsin(rotation_matrix[2, 0])
            roll = np.arctan2(rotation_matrix[2, 1] / np.cos(pitch), 
                             rotation_matrix[2, 2] / np.cos(pitch))
            yaw = np.arctan2(rotation_matrix[1, 0] / np.cos(pitch), 
                            rotation_matrix[0, 0] / np.cos(pitch))
        else:
            # Gimbal lock case
            yaw = 0
            if rotation_matrix[2, 0] < 0:
                pitch = np.pi / 2
                roll = np.arctan2(rotation_matrix[0, 1], rotation_matrix[0, 2])
            else:
                pitch = -np.pi / 2
                roll = np.arctan2(-rotation_matrix[0, 1], -rotation_matrix[0, 2])
        
        orientation = np.array([roll, pitch, yaw])
        
        data_list.append({
            'name': yaml_file.stem,
            'robot_q': robot_q,
            'transformation_matrix': T,
            'position': position,
            'orientation': orientation
        })
    
    return data_list


def plot_coordinate_frame(ax, T, scale=0.05, alpha=0.7):
    """
    Plot a coordinate frame (RGB = XYZ) at the given transformation.
    
    Args:
        ax: Matplotlib 3D axis
        T: 4x4 transformation matrix
        scale: Length of axes
        alpha: Transparency
    """
    origin = T[:3, 3]
    x_axis = T[:3, 0] * scale
    y_axis = T[:3, 1] * scale
    z_axis = T[:3, 2] * scale
    
    # X axis (red)
    ax.plot([origin[0], origin[0] + x_axis[0]], 
            [origin[1], origin[1] + x_axis[1]], 
            [origin[2], origin[2] + x_axis[2]], 
            'r-', linewidth=2, alpha=alpha)
    
    # Y axis (green)
    ax.plot([origin[0], origin[0] + y_axis[0]], 
            [origin[1], origin[1] + y_axis[1]], 
            [origin[2], origin[2] + y_axis[2]], 
            'g-', linewidth=2, alpha=alpha)
    
    # Z axis (blue)
    ax.plot([origin[0], origin[0] + z_axis[0]], 
            [origin[1], origin[1] + z_axis[1]], 
            [origin[2], origin[2] + z_axis[2]], 
            'b-', linewidth=2, alpha=alpha)


def plot_3d_positions(data_list):
    """
    Create 3D plot of robot positions.
    
    Args:
        data_list: List of calibration data dictionaries
    """
    fig = plt.figure(figsize=(15, 10))
    
    # Main 3D plot
    ax1 = fig.add_subplot(221, projection='3d')
    
    # Extract positions
    positions = np.array([d['position'] for d in data_list])
    
    # Plot positions as points
    ax1.scatter(positions[:, 0], positions[:, 1], positions[:, 2], 
               c='blue', marker='o', s=100, label='End-effector positions')
    
    # Plot coordinate frames at each position
    for i, data in enumerate(data_list):
        plot_coordinate_frame(ax1, data['transformation_matrix'], scale=0.03, alpha=0.6)
        
        # Add labels
        if i % 3 == 0:  # Label every 3rd point to avoid clutter
            ax1.text(positions[i, 0], positions[i, 1], positions[i, 2], 
                    f"  {i+1}", fontsize=8)
    
    # Connect points in order
    ax1.plot(positions[:, 0], positions[:, 1], positions[:, 2], 
            'k--', alpha=0.3, linewidth=0.5, label='Path')
    
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('Robot End-Effector Positions')
    ax1.legend()
    ax1.grid(True)
    
    # Set equal aspect ratio
    max_range = np.array([positions[:, 0].max()-positions[:, 0].min(),
                         positions[:, 1].max()-positions[:, 1].min(),
                         positions[:, 2].max()-positions[:, 2].min()]).max() / 2.0
    mid_x = (positions[:, 0].max()+positions[:, 0].min()) * 0.5
    mid_y = (positions[:, 1].max()+positions[:, 1].min()) * 0.5
    mid_z = (positions[:, 2].max()+positions[:, 2].min()) * 0.5
    ax1.set_xlim(mid_x - max_range, mid_x + max_range)
    ax1.set_ylim(mid_y - max_range, mid_y + max_range)
    ax1.set_zlim(mid_z - max_range, mid_z + max_range)
    
    # XY plane view
    ax2 = fig.add_subplot(222)
    ax2.scatter(positions[:, 0], positions[:, 1], c='blue', marker='o', s=50)
    ax2.plot(positions[:, 0], positions[:, 1], 'k--', alpha=0.3, linewidth=0.5)
    for i in range(0, len(positions), 3):
        ax2.text(positions[i, 0], positions[i, 1], f"{i+1}", fontsize=8)
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_title('Top View (XY plane)')
    ax2.grid(True)
    ax2.axis('equal')
    
    # XZ plane view
    ax3 = fig.add_subplot(223)
    ax3.scatter(positions[:, 0], positions[:, 2], c='blue', marker='o', s=50)
    ax3.plot(positions[:, 0], positions[:, 2], 'k--', alpha=0.3, linewidth=0.5)
    for i in range(0, len(positions), 3):
        ax3.text(positions[i, 0], positions[i, 2], f"{i+1}", fontsize=8)
    ax3.set_xlabel('X (m)')
    ax3.set_ylabel('Z (m)')
    ax3.set_title('Side View (XZ plane)')
    ax3.grid(True)
    ax3.axis('equal')
    
    # YZ plane view
    ax4 = fig.add_subplot(224)
    ax4.scatter(positions[:, 1], positions[:, 2], c='blue', marker='o', s=50)
    ax4.plot(positions[:, 1], positions[:, 2], 'k--', alpha=0.3, linewidth=0.5)
    for i in range(0, len(positions), 3):
        ax4.text(positions[i, 1], positions[i, 2], f"{i+1}", fontsize=8)
    ax4.set_xlabel('Y (m)')
    ax4.set_ylabel('Z (m)')
    ax4.set_title('Front View (YZ plane)')
    ax4.grid(True)
    ax4.axis('equal')
    
    plt.tight_layout()
    
    return fig


def plot_joint_configurations(data_list):
    """
    Plot joint angles over different positions.
    
    Args:
        data_list: List of calibration data dictionaries
    """
    fig, axes = plt.subplots(3, 2, figsize=(15, 10))
    axes = axes.flatten()
    
    positions_indices = np.arange(1, len(data_list) + 1)
    
    for joint_idx in range(6):
        ax = axes[joint_idx]
        joint_angles = [np.rad2deg(d['robot_q'][joint_idx]) for d in data_list]
        
        ax.plot(positions_indices, joint_angles, 'o-', linewidth=2, markersize=6)
        ax.set_xlabel('Position Index')
        ax.set_ylabel('Angle (degrees)')
        ax.set_title(f'Joint {joint_idx + 1} Configuration')
        ax.grid(True, alpha=0.3)
        ax.set_xticks(positions_indices[::3])  # Show every 3rd position
    
    plt.tight_layout()
    return fig


def print_statistics(data_list):
    """
    Print statistics about the calibration positions.
    
    Args:
        data_list: List of calibration data dictionaries
    """
    positions = np.array([d['position'] for d in data_list])
    
    print("\n" + "="*70)
    print("CALIBRATION POSITION STATISTICS")
    print("="*70)
    print(f"Total positions: {len(data_list)}")
    print()
    
    print("Position range:")
    print(f"  X: [{positions[:, 0].min():.4f}, {positions[:, 0].max():.4f}] m  (range: {positions[:, 0].max() - positions[:, 0].min():.4f} m)")
    print(f"  Y: [{positions[:, 1].min():.4f}, {positions[:, 1].max():.4f}] m  (range: {positions[:, 1].max() - positions[:, 1].min():.4f} m)")
    print(f"  Z: [{positions[:, 2].min():.4f}, {positions[:, 2].max():.4f}] m  (range: {positions[:, 2].max() - positions[:, 2].min():.4f} m)")
    print()
    
    print("Average position:")
    print(f"  X: {positions[:, 0].mean():.4f} m")
    print(f"  Y: {positions[:, 1].mean():.4f} m")
    print(f"  Z: {positions[:, 2].mean():.4f} m")
    print()
    
    # Calculate distances between consecutive positions
    distances = []
    for i in range(1, len(positions)):
        dist = np.linalg.norm(positions[i] - positions[i-1])
        distances.append(dist)
    
    if distances:
        print("Distance between consecutive positions:")
        print(f"  Min: {min(distances):.4f} m")
        print(f"  Max: {max(distances):.4f} m")
        print(f"  Mean: {np.mean(distances):.4f} m")
        print(f"  Std: {np.std(distances):.4f} m")
    
    print("="*70)


def main():
    """Main function."""
    print("="*70)
    print("3D VISUALIZATION OF ROBOT CALIBRATION POSITIONS")
    print("="*70)
    print()
    
    # Setup paths
    script_dir = Path(__file__).parent
    project_root = script_dir.parent  # Go up to ROB_semestralka folder
    exporty_folder = project_root / "exporty"
    
    if not exporty_folder.exists():
        print(f"❌ Folder {exporty_folder} does not exist!")
        return
    
    # Load data
    print(f"📂 Loading calibration data from: {exporty_folder}")
    data_list = load_calibration_data(exporty_folder)
    
    if not data_list:
        print("❌ No calibration data found!")
        return
    
    print(f"✅ Loaded {len(data_list)} calibration positions")
    
    # Print statistics
    print_statistics(data_list)
    
    # Create plots
    print("\n📊 Creating 3D visualization...")
    fig1 = plot_3d_positions(data_list)
    
    print("📊 Creating joint configuration plot...")
    fig2 = plot_joint_configurations(data_list)
    
    # Save plots
    output_dir = project_root / "calibration_visualization"
    output_dir.mkdir(exist_ok=True)
    
    fig1.savefig(output_dir / "positions_3d.png", dpi=300, bbox_inches='tight')
    print(f"💾 Saved: {output_dir / 'positions_3d.png'}")
    
    fig2.savefig(output_dir / "joint_configurations.png", dpi=300, bbox_inches='tight')
    print(f"💾 Saved: {output_dir / 'joint_configurations.png'}")
    
    print("\n✅ Visualization complete!")
    print("\n🖼️  Close the plot windows to exit...")
    plt.show()


if __name__ == "__main__":
    main()
