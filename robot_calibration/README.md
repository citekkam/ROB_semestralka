# Robot Calibration Automation

This folder contains scripts and data for automated robot hand-eye calibration.

## Files


## Usage

### 1. Configure Calibration Positions

Edit `calibration_positions.yaml` to set the desired calibration positions in Cartesian space. The default configuration includes 9 positions in a 3x3 grid:

```yaml
position_1:
  pose: [0.35, 0.10, 0.25, 0.0, -1.57, 0.0]  # [x, y, z, roll, pitch, yaw]
position_2:
  pose: [0.40, 0.00, 0.25, 0.0, -1.57, 0.0]
...
```

**Format:**
- `x, y, z`: Position in meters (relative to robot base)
- `roll, pitch, yaw`: Orientation in radians (ZYX Euler angles)

### 2. Run Calibration

Execute the calibration script:

```bash
python calibration_automation.py
```

or make it executable and run directly:

```bash
chmod +x calibration_automation.py
./calibration_automation.py
```

### 3. Output Data

For each calibration position, the script saves:

- **calib_position_N.yaml** - Joint positions and transformation matrix
- **calib_position_N.png** - Captured image

Output format (YAML):
```yaml
Robot_pos_q: [q0, q1, q2, q3, q4, q5]
transformacni_matic: [[r11, r12, r13, tx],
                      [r21, r22, r23, ty],
                      [r31, r32, r33, tz],
                      [0,   0,   0,   1]]
```

## How It Works

1. **Load Positions**: Reads 9 calibration positions from YAML file
2. **Initialize Robot**: Connects to CRS93 robot (without homing)
3. **For Each Position**:
   - Calculate target pose using forward kinematics
   - Compute IK solutions
   - Select shortest path using joint weights
   - Move robot to position
   - Wait for motion to stop
   - Capture image from camera
   - Save joint configuration, transformation matrix, and image
4. **Summary**: Displays success/failure statistics

## Joint Weights

The script uses weighted distance calculation for selecting the best IK solution:

```python
joint_weights = [1.0, 1.0, 1.0, 10.0, 1.0, 10.0]
```

This prioritizes moving larger joints (0-2) over smaller joints (3-5).

## Safety Features

- Validates IK solutions before movement
- Falls back to direct movement if no IK solutions found
- Error handling for each capture
- Continues to next position if one fails
- Provides detailed progress output

## Dependencies

- numpy
- yaml (PyYAML)
- ctu_crs (CRS93 robot driver)
- select_shortest_path
- camera.dataset_creator

## Troubleshooting

**No IK solutions found:**
- Check if target positions are within robot workspace
- Adjust positions in calibration_positions.yaml

**Image capture fails:**
- Verify camera connection
- Check camera initialization in CRS93

**Robot movement errors:**
- Ensure robot is properly initialized
- Check joint limits
- Verify robot is not in emergency stop

## Customization

### Change Number of Positions

Edit `calibration_positions.yaml` and add/remove positions:

```yaml
position_10:
  robot_q: [0.1, -0.9, -1.5, 0.0, -0.6, 0.0]
```

### Change Output Folder

Modify the `output_folder` variable in `main()`:

```python
output_folder = str(script_dir / "my_calibration_data")
```

### Adjust Joint Weights

Modify `joint_weights` in `main()` to change path selection priority:

```python
joint_weights = np.array([2.0, 2.0, 2.0, 5.0, 1.0, 5.0])
```

## Example Output

```
======================================================================
  ROBOT CALIBRATION AUTOMATION
======================================================================

📂 Loading calibration positions from: calibration_positions.yaml
✅ Loaded 9 calibration positions

🤖 Initializing robot...
✅ Robot initialized successfully

======================================================================
Position 1/9: position_1
======================================================================
Target joint configuration: [ 0.3 -0.8 -1.6  0.  -0.7  0. ]
Moving to position...
  → Selected IK solution 0 with distance 0.1234
Actual joint configuration: [ 0.3001 -0.8002 -1.5998  0.0001 -0.7001  0.0001]
Transformation matrix calculated
📸 Capturing image...
✅ Image captured: (1200, 1600, 3)
💾 Saving data...
✅ Matice uloženy do: calibration_data/calib_position_1.yaml
✅ Obrázek uložen jako: calibration_data/calib_position_1.png
✅ Data saved successfully

...

======================================================================
  CALIBRATION SUMMARY
======================================================================
Total positions: 9
✅ Successful captures: 9
❌ Failed captures: 0
Output folder: /path/to/robot_calibration/calibration_data

🤖 Releasing robot...
✅ Robot released successfully

======================================================================
  CALIBRATION COMPLETE
======================================================================
```
