# CRS robots control at CTU/CIIRC
Package to control CTU/CIIRC robots CRS93 and CRS97 via MARS control unit.
For the complete documentation see https://robotics-robot-stations.readthedocs.io/

## Installation

```
pip install ctu_crs
```

## Command-Line Scripts

This package includes command-line scripts for direct robot control from your terminal.

### `move_crs93` and `move_crs97`

After installing the package, you can use the `move_crs93` or `move_crs97` scripts to move the robot by a relative offset for specified joints.

**Usage:**

```sh
move_crs93 [--nohome] <joint> <angle> [<joint> <angle> ...]
move_crs97 [--nohome] <joint> <angle> [<joint> <angle> ...]
```

**Arguments:**

*   `<joint> <angle>`: One or more pairs of joint names (q0-q5) and their corresponding relative angles in degrees.
*   `--nohome`: Optional flag to initialize the robot without performing the homing sequence.

**Examples:**

To move the `crs93`'s first joint (`q0`) by 10 degrees and the second joint (`q1`) by -5 degrees:

```sh
move_crs93 q0 10 q1 -5
```

To move the `crs97`'s second joint (`q2`) by 20.5 degrees without homing the robot first:

```sh
move_crs97 --nohome q2 20.5
```

## Simple python script to control the robot

```python
from ctu_crs import CRS93 # or CRS97
robot = CRS93()  # set argument tty_dev=None if you are not connected to robot,
# it will allow you to compute FK and IK offline
robot.initialize()  # initialize connection to the robot, perform hard and soft home
q = robot.get_q()  # get current joint configuration
robot.move_to_q(q + [0.1, 0.0, 0.0, 0.0, 0.0, 0.0])  # move robot all values in radians
robot.wait_for_motion_stop() # wait until the robot stops
robot.close()  # close the connection
```

## Example of Grabbing a Camera Picture

You can easily capture an image from the camera associated with the robot using the robot interface itself. The library handles the camera connection automatically.

```python
from ctu_crs import CRS97 # or CRS93

# Initialize the robot interface
robot = CRS97()
robot.initialize()

# Grab an image from the camera
# This will automatically connect to and open the camera on the first call
img = robot.grab_image()

# 'img' is a NumPy array containing the image data.
# You can now process it, for example, using a library like OpenCV.
if img is not None:
    print(f"Successfully grabbed image with shape: {img.shape}")

# Don't forget to close the connection when you're done
robot.close()
```

## Step-by-Step Procedure for Operating the Robot

- **Power On the Robot**
Turn on the robot using the red switch on the front panel of the control unit (the switch will light up).
Press the yellow Arm Power button (a yellow LED will light up above it).
- **Initialize Communication and Setup in Python**
Run the following commands to initiate communication and perform necessary setup:
```python
from ctu_crs import CRS93 # or CRS97
robot = CRS93()  # set argument tty_dev=None if you are not connected to robot,
# it will allow you to compute FK and IK offline
robot.initialize()  # initialize connection to the robot, perform hard and soft home
```
- **Move the Robot** After initialization, you can continue with your desired commands for operating the robot.

- **End of Session - Return Robot to Home Position**
To finish working and return the robot to the home position, execute:
```python
robot.soft_home()
robot.close()  # close the connection
```
Turn off the robot by switching off the red rocker switch on the front panel.

### Accessing the Robot Within the Safety Cage
If you need to work inside the protective cage:
Stop the robot (wait until movement stops completely). You can check with:
```python
robot.in_motion()
```
Run the following command to release the robot:
```python
robot.release()
```
This will engage the brakes and disconnect feedback. Be cautious, as the robot arm might slightly drop, so avoid performing this command directly above the work surface.
Open the cage door (the yellow Arm Power LED will turn off).

## Emergency Stop Procedure
If the robot behaves unexpectedly, immediately press the Emergency Stop button (red mushroom-shaped button).

### Restoring Operation After Emergency Stop or Working Inside the Cage
Unlock the emergency stop by pressing the blue button on the side of the emergency button, or close the cage door if open.
Press the Motion Stop button.
Press the yellow Arm Power button (the yellow Arm Power LED will turn on).
Resume sending movement commands to the robot.
If the motor enters an error state (indicated by a flashing green status LED above the motor’s letter and a red error LED), reset the motor state with:
```python
robot.reset_motors()
```
Note: Re-homing the robot is not necessary, even after activating the emergency stop or opening the cage door, as long as the control unit is not powered off.


## For developers
```
git clone git@github.com:CTURobotics/ctu_crs.git
cd ctu_crs
uv sync
uv pip install -e ".[dev]"
pre-commit install
```
