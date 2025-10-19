#!/usr/bin/env python3
"""
Calibration Grid Generator
===========================
Creates a customizable grid of calibration positions for robot hand-eye calibration.

Features:
- Choose grid size (e.g., 3x3, 4x4, 5x3)
- Choose fixed plane (XY, XZ, or YZ)
- Set grid spacing
- Set center position
- Choose orientation (no rotation or custom)
- Export to YAML format

Author: David
Date: 2025-10-19
"""

import numpy as np
import yaml
from pathlib import Path
import argparse


def generate_grid_positions(
    grid_rows: int = 3,
    grid_cols: int = 3,
    plane: str = 'XY',
    fixed_axis_value: float = 0.05,  # 5cm
    spacing: float = None,  # 10cm (or calculated from grid_size)
    grid_size: tuple = None,  # (width, height) in meters, e.g., (0.10, 0.20) for 10cm x 20cm
    center: tuple = (0.40, 0.0, 0.20),
    orientation: str = 'down',  # 'down', 'front', 'none', or custom [roll, pitch, yaw]
):
    """
    Generate a grid of calibration positions.
    
    Args:
        grid_rows: Number of rows in the grid
        grid_cols: Number of columns in the grid
        plane: Plane for the grid ('XY', 'XZ', 'YZ')
        fixed_axis_value: Value for the fixed axis (offset from origin)
        spacing: Distance between adjacent grid points (ignored if grid_size is specified)
        grid_size: Physical size of grid (width, height) in meters - overrides spacing
        center: Center position of the grid (x, y, z)
        orientation: Camera orientation ('down', 'front', 'side', 'none', or [r,p,y])
    
    Returns:
        List of poses [x, y, z, roll, pitch, yaw]
    """
    
    # Calculate spacing from grid_size if provided
    if grid_size is not None:
        width, height = grid_size
        # Calculate spacing needed to fit the grid
        col_spacing = width / (grid_cols - 1) if grid_cols > 1 else 0
        row_spacing = height / (grid_rows - 1) if grid_rows > 1 else 0
    else:
        # Use uniform spacing
        if spacing is None:
            spacing = 0.10  # Default 10cm
        col_spacing = spacing
        row_spacing = spacing
    
    # Calculate grid offsets from center
    row_offsets = np.linspace(-(grid_rows-1)/2, (grid_rows-1)/2, grid_rows) * row_spacing
    col_offsets = np.linspace(-(grid_cols-1)/2, (grid_cols-1)/2, grid_cols) * col_spacing
    
    positions = []
    
    # Set orientation based on preset or custom
    if orientation == 'down':
        # Camera pointing down (pitch = -90°)
        roll, pitch, yaw = 0.0, -np.pi/2, 0.0
    elif orientation == 'front':
        # Camera pointing forward
        roll, pitch, yaw = 0.0, 0.0, 0.0
    elif orientation == 'side':
        # Camera pointing to the side (yaw = 90°)
        roll, pitch, yaw = 0.0, 0.0, np.pi/2
    elif orientation == 'none':
        # No rotation
        roll, pitch, yaw = 0.0, 0.0, 0.0
    elif isinstance(orientation, (list, tuple)) and len(orientation) == 3:
        # Custom orientation [roll, pitch, yaw] in radians
        roll, pitch, yaw = orientation
    else:
        raise ValueError(f"Invalid orientation: {orientation}")
    
    # Generate grid based on plane
    plane = plane.upper()
    
    for i, row_offset in enumerate(row_offsets):
        for j, col_offset in enumerate(col_offsets):
            
            if plane == 'XY':
                # Grid in XY plane, Z is fixed
                x = center[0] + col_offset
                y = center[1] + row_offset
                z = fixed_axis_value
                
            elif plane == 'XZ':
                # Grid in XZ plane, Y is fixed
                x = center[0] + col_offset
                y = fixed_axis_value
                z = center[2] + row_offset
                
            elif plane == 'YZ':
                # Grid in YZ plane, X is fixed
                x = fixed_axis_value
                y = center[1] + col_offset
                z = center[2] + row_offset
                
            else:
                raise ValueError(f"Invalid plane: {plane}. Choose 'XY', 'XZ', or 'YZ'")
            
            pose = [x, y, z, roll, pitch, yaw]
            positions.append(pose)
    
    return positions


def save_to_yaml(positions, output_file, description=None):
    """
    Save positions to YAML file in calibration format.
    
    Args:
        positions: List of poses [x, y, z, roll, pitch, yaw]
        output_file: Path to output YAML file
        description: Optional description for the file
    """
    
    data = {}
    
    # Add header comment
    if description:
        data['_description'] = description
    
    # Add positions
    for i, pose in enumerate(positions, 1):
        data[f'position_{i}'] = {
            'pose': [float(x) for x in pose]
        }
    
    # Save to file
    with open(output_file, 'w') as f:
        if description:
            f.write(f"# {description}\n")
            f.write("# Format: [x, y, z, roll, pitch, yaw]\n")
            f.write("# - x, y, z in meters\n")
            f.write("# - roll, pitch, yaw in radians\n\n")
        
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)
    
    print(f"✅ Saved {len(positions)} positions to: {output_file}")


def print_grid_preview(positions, grid_rows, grid_cols, plane):
    """
    Print a preview of the generated grid.
    
    Args:
        positions: List of poses
        grid_rows: Number of rows
        grid_cols: Number of columns
        plane: Grid plane
    """
    print("\n" + "="*70)
    print("GRID PREVIEW")
    print("="*70)
    print(f"Grid size: {grid_rows}×{grid_cols} ({len(positions)} positions)")
    print(f"Plane: {plane}")
    print()
    
    # Extract positions
    pos_array = np.array([[p[0], p[1], p[2]] for p in positions])
    
    # Calculate physical grid size
    x_size = pos_array[:, 0].max() - pos_array[:, 0].min()
    y_size = pos_array[:, 1].max() - pos_array[:, 1].min()
    z_size = pos_array[:, 2].max() - pos_array[:, 2].min()
    
    print("Physical grid size:")
    if plane == 'XY':
        print(f"  {x_size*100:.1f} cm × {y_size*100:.1f} cm  (X × Y)")
    elif plane == 'XZ':
        print(f"  {x_size*100:.1f} cm × {z_size*100:.1f} cm  (X × Z)")
    elif plane == 'YZ':
        print(f"  {y_size*100:.1f} cm × {z_size*100:.1f} cm  (Y × Z)")
    print()
    
    print("Position ranges:")
    print(f"  X: [{pos_array[:, 0].min():.3f}, {pos_array[:, 0].max():.3f}] m")
    print(f"  Y: [{pos_array[:, 1].min():.3f}, {pos_array[:, 1].max():.3f}] m")
    print(f"  Z: [{pos_array[:, 2].min():.3f}, {pos_array[:, 2].max():.3f}] m")
    print()
    
    print("Orientation (all positions):")
    roll, pitch, yaw = positions[0][3:]
    print(f"  Roll:  {np.rad2deg(roll):+.1f}°")
    print(f"  Pitch: {np.rad2deg(pitch):+.1f}°")
    print(f"  Yaw:   {np.rad2deg(yaw):+.1f}°")
    print()
    
    print("Grid layout (position numbers):")
    idx = 1
    for i in range(grid_rows):
        row_str = "  "
        for j in range(grid_cols):
            row_str += f"{idx:3d} "
            idx += 1
        print(row_str)
    
    print("="*70)


def interactive_mode():
    """
    Interactive mode to configure grid parameters.
    """
    print("\n" + "="*70)
    print("CALIBRATION GRID GENERATOR - INTERACTIVE MODE")
    print("="*70)
    print()
    
    # Grid size
    print("Grid size:")
    grid_rows = int(input("  Number of rows [3]: ") or "3")
    grid_cols = int(input("  Number of columns [3]: ") or "3")
    print()
    
    # Plane selection
    print("Grid plane:")
    print("  1. XY plane (horizontal, Z fixed)")
    print("  2. XZ plane (vertical front, Y fixed)")
    print("  3. YZ plane (vertical side, X fixed)")
    plane_choice = input("  Choose plane [1]: ") or "1"
    plane_map = {'1': 'XY', '2': 'XZ', '3': 'YZ'}
    plane = plane_map.get(plane_choice, 'XY')
    print(f"  → Selected: {plane} plane")
    print()
    
    # Fixed axis value
    fixed_axis_name = {'XY': 'Z', 'XZ': 'Y', 'YZ': 'X'}[plane]
    fixed_axis_value = float(input(f"  {fixed_axis_name} offset (fixed axis value in meters) [0.05]: ") or "0.05")
    print()
    
    # Spacing
    spacing = float(input("  Grid spacing (distance between points in meters) [0.10]: ") or "0.10")
    print()
    
    # Center position
    print("Grid center position:")
    center_x = float(input("  X center [0.40]: ") or "0.40")
    center_y = float(input("  Y center [0.00]: ") or "0.00")
    center_z = float(input("  Z center [0.20]: ") or "0.20")
    center = (center_x, center_y, center_z)
    print()
    
    # Orientation
    print("Camera orientation:")
    print("  1. Pointing down (pitch = -90°)")
    print("  2. Pointing forward (no rotation)")
    print("  3. Pointing to side (yaw = 90°)")
    print("  4. No rotation")
    print("  5. Custom (enter roll, pitch, yaw)")
    orientation_choice = input("  Choose orientation [1]: ") or "1"
    
    if orientation_choice == '1':
        orientation = 'down'
    elif orientation_choice == '2':
        orientation = 'front'
    elif orientation_choice == '3':
        orientation = 'side'
    elif orientation_choice == '4':
        orientation = 'none'
    elif orientation_choice == '5':
        roll = float(input("    Roll (degrees): "))
        pitch = float(input("    Pitch (degrees): "))
        yaw = float(input("    Yaw (degrees): "))
        orientation = [np.deg2rad(roll), np.deg2rad(pitch), np.deg2rad(yaw)]
    else:
        orientation = 'down'
    print()
    
    # Output file
    output_file = input("  Output file [calibration_positions.yaml]: ") or "calibration_positions.yaml"
    print()
    
    return {
        'grid_rows': grid_rows,
        'grid_cols': grid_cols,
        'plane': plane,
        'fixed_axis_value': fixed_axis_value,
        'spacing': spacing,
        'center': center,
        'orientation': orientation,
        'output_file': output_file
    }


def main():
    """Main function."""
    parser = argparse.ArgumentParser(
        description='Generate calibration grid positions',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Interactive mode
  python create_3x3_grid.py
  
  # 3x3 grid in XY plane at Z=0.05m with 10cm spacing
  python create_3x3_grid.py --rows 3 --cols 3 --plane XY --offset 0.05 --spacing 0.10
  
  # 3x3 grid with 10cm × 20cm physical size (spacing calculated automatically)
  python create_3x3_grid.py --rows 3 --cols 3 --plane XY --offset 0.05 --grid-size 0.10 0.20
  
  # 4x4 grid with 30cm × 30cm size in XZ plane at Y=0.0m
  python create_3x3_grid.py --rows 4 --cols 4 --plane XZ --offset 0.0 --grid-size 0.30 0.30
  
  # 5x3 grid with custom center
  python create_3x3_grid.py --rows 5 --cols 3 --center 0.35 0.05 0.25 --grid-size 0.20 0.15
        """
    )
    
    parser.add_argument('--rows', type=int, help='Number of rows')
    parser.add_argument('--cols', type=int, help='Number of columns')
    parser.add_argument('--plane', type=str, choices=['XY', 'XZ', 'YZ'], help='Grid plane')
    parser.add_argument('--offset', type=float, help='Fixed axis offset (meters)')
    parser.add_argument('--spacing', type=float, help='Grid spacing (meters) - ignored if --grid-size is used')
    parser.add_argument('--grid-size', type=float, nargs=2, metavar=('WIDTH', 'HEIGHT'),
                       help='Physical grid size in meters (e.g., 0.10 0.20 for 10cm×20cm). Overrides --spacing.')
    parser.add_argument('--center', type=float, nargs=3, metavar=('X', 'Y', 'Z'), 
                       help='Grid center position (x y z)')
    parser.add_argument('--orientation', type=str, 
                       choices=['down', 'front', 'side', 'none'],
                       help='Camera orientation preset')
    parser.add_argument('--output', type=str, help='Output YAML file')
    parser.add_argument('--yes', '-y', action='store_true', 
                       help='Skip confirmation prompt')
    
    args = parser.parse_args()
    
    # Use interactive mode if no arguments provided
    if not any(vars(args).values()):
        config = interactive_mode()
    else:
        # Use command line arguments with defaults
        config = {
            'grid_rows': args.rows or 3,
            'grid_cols': args.cols or 3,
            'plane': args.plane or 'XY',
            'fixed_axis_value': args.offset if args.offset is not None else 0.05,
            'spacing': args.spacing,
            'grid_size': tuple(args.grid_size) if args.grid_size else None,
            'center': tuple(args.center) if args.center else (0.40, 0.0, 0.20),
            'orientation': args.orientation or 'down',
            'output_file': args.output or 'calibration_positions.yaml'
        }
    
    # Generate grid
    print("\nGenerating grid...")
    positions = generate_grid_positions(**{k: v for k, v in config.items() if k != 'output_file'})
    
    # Print preview
    print_grid_preview(positions, config['grid_rows'], config['grid_cols'], config['plane'])
    
    # Confirm
    if args.yes if hasattr(args, 'yes') else False:
        confirm = 'y'
    else:
        confirm = input("\nSave this grid? [Y/n]: ").lower()
    
    if confirm in ['', 'y', 'yes']:
        # Prepare output path
        output_path = Path(config['output_file'])
        if not output_path.is_absolute():
            # Save to robot_calibration folder by default
            script_dir = Path(__file__).parent
            calib_dir = script_dir / "robot_calibration"
            calib_dir.mkdir(exist_ok=True)
            output_path = calib_dir / config['output_file']
        
        # Create description
        if config.get('grid_size'):
            size_desc = f"Grid size: {config['grid_size'][0]*100:.0f}cm×{config['grid_size'][1]*100:.0f}cm"
        elif config.get('spacing'):
            size_desc = f"Spacing: {config['spacing']:.3f}m"
        else:
            size_desc = "Spacing: 0.100m"
        
        description = (
            f"Calibration grid: {config['grid_rows']}×{config['grid_cols']} in {config['plane']} plane\n"
            f"# Fixed axis: {config['fixed_axis_value']:.3f}m, {size_desc}\n"
            f"# Center: ({config['center'][0]:.2f}, {config['center'][1]:.2f}, {config['center'][2]:.2f})"
        )
        
        # Save
        save_to_yaml(positions, output_path, description)
        
        print(f"\n✅ Grid saved successfully!")
        print(f"\nYou can visualize it with:")
        print(f"  python plot_planned_calibration.py")
    else:
        print("\n❌ Grid not saved.")


if __name__ == "__main__":
    main()
    # 3×3 grid with 30cm × 30cm size


# python ROB_semestralka/robot_calibration/create_grid.py --rows 8 --cols 3 --center 0.4 0.0 0.5 --grid-size 0.2 0.4
