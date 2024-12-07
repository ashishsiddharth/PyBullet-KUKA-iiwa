# PyBullet-KUKA-iiwa
KUKA iiwa Robot Simulation with PyBullet<br />
<br />

This project provides a framework to simulate and control the KUKA iiwa 7-DOF robotic arm in a 3D environment using the PyBullet physics engine. The robot interacts with the environment, tracks spheres, and can store joint positions into a CSV file. The class pyRobot serves as the core of the simulation, allowing you to load the robot, manipulate it, and manage the simulation environment.

Notes:<br />
1. Ensure you have PyBullet and NumPy installed to run this code.<br />
2. This project uses PyBullet for physics simulation and URDF models for the robot and environment objects.<br />
3. The simulation is set to real-time mode, so you can interact with the robot and objects dynamically.<br />
4. You can optionally log joint states by enabling the storeData function.<br />

#Installation
* 1. Install the required dependencies:
```
pip install pybullet numpy
```

* 2. Clone this repository:
```
git clone https://github.com/your-username/pyDualArm.git
cd pyDualArm
```

### Sample program
```
from api import pyRobot
import time
import numpy as np

# Initialize the robot and environment
rbt = pyRobot()
rbt.connect()
rbt.enableGravity()
rbt.loadUrdf()

# Start real-time simulation and move robot to home position
rbt.enableRealTimeSim()
rbt.home()

# Move sphere and robot arm
rbt.move()

# Optionally, store joint data
# rbt.storeData(True)
```

### API Functions Description
```
__init__(self)
```
* Initializes the simulation environment and sets default values for various settings, such as gravity and object parameters.

```
connect(self)
```
* Establishes a connection to the PyBullet simulation and resets the simulation environment.

```
disconnect(self)
```
* Disconnects from the PyBullet simulation.

```
resetSim(self)
```
* Resets the simulation, clearing all objects and returning to the initial state.

```
enableRealTimeSim(self)
```
* Enables real-time simulation where the simulation runs in sync with the system clock, providing interactive control.

```
enableGravity(self, gX=0, gY=0, gZ=0)
```
* Enables gravity in the simulation with customizable gravity values along the X, Y, and Z axes.

```
disableGravity(self, gX=0, gY=0, gZ=0)
```
* Disables gravity in the simulation.

```
loadPlane(self)
```
* Loads a plane in the simulation to serve as the ground surface for the robot.

```
load_iiwa(self)
```
* Loads a KUKA iiwa 7-DOF robot arm model into the simulation.

```
loadSphere1(self)
```
* Loads a red sphere into the simulation at a specified location (used as a target for the robot to interact with).

```
loadSphere3(self)
```
* Loads a blue sphere into the simulation at a specified location (used as a secondary target for the robot to track).

```
showArmAxis(self, bodyID=None, index=None)
```
* Displays the axes of the robot arm (or any body) in the simulation for visual tracking. This helps in debugging and tracking the movement of the arm.

```
showBodyAxis(self, bodyID=None)
```
* Displays the axes of a specified body in the simulation (e.g., spheres or robot parts).

```
loadRobot(self)
```
* Loads the robot into the simulation and displays the arm axes for visual reference.

```
loadUrdf(self)
```
Loads all the required URDF models (plane, KUKA robot, and spheres) into the simulation.

```
joint_command(self, lArm=[0, 0, 0, 0, 0, 0, 0])
```
* Sends joint position commands to the KUKA robot arm. This method allows for controlling the robot’s arm movement by providing a list of joint angles.

```
ikin(self, lpose=[0, 0, 0, 0, 0, 0])
```
* Calculates the inverse kinematics for the KUKA arm based on the desired target position and orientation of the robot's end-effector. This method calculates joint angles required to reach a specific pose.
```
pose(self)
```
* Displays the forward kinematics of the robot based on the current joint angles. This helps visualize the current position of the robot's end-effector.

```
home(self)
```
* Moves the KUKA robot to its default home position. This is typically the "rest" position for the robot's joints.

```
getState(self)
```
* Fetches and prints the current state of the robot’s joints, showing their positions, velocities, and forces.

```
enableStepSim(self)
```
* Runs the simulation step by step. This mode is useful for debugging or when you want to have more granular control over the simulation.

```
sphere_slider(self)
```
* Displays user-debug sliders for adjusting the position and orientation of sphere1 in the simulation.

```
moveSphere(self)
```
* Moves the sphere1 based on user-defined values from the sliders (for interactive control).

```
trackSphere1(self)
```
* Tracks the position of sphere1 by calculating inverse kinematics and sending joint position commands to the robot to follow the sphere.

```
move(self, x=0, y=0, z=0, roll=0, pitch=0, yaw=0)
```
* Moves sphere3 incrementally in 3D space, updating its position and orientation. It then makes the robot arm track the sphere's position.

```
move_abs(self, x=0, y=0, z=0, roll=0, pitch=0, yaw=0)
```
* Moves sphere3 to an absolute position in 3D space and updates its pose accordingly. The robot arm then tracks this new position.

```
trackSphere3(self, x=0, y=0, z=0)
```
* Tracks sphere3 by calculating the inverse kinematics and sending joint position commands to the robot to follow the sphere’s position.

```
storeData(self, data=True)
```
* Enables or disables saving the robot's joint data to a CSV file.

```
write_data(self, data)
```
* Writes joint data into a CSV file (only if storeData is enabled). This can be used for logging joint positions over time.


# Conclusion
By combining forward kinematics (to determine where the robot's end-effector is) and inverse kinematics (to calculate how to move the robot's joints to reach a desired position), this project enables intuitive control of the KUKA iiwa robot within the PyBullet simulation environment.
