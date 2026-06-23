# UR5e GELLO State Publisher

ROS 2 Python package for reading the STS3215-based GELLO controller and publishing UR5e-compatible joint commands.

Published topics:

- `gello/joint_states` (`sensor_msgs/JointState`)
- `gello/gripper_position` (`std_msgs/Float32`)
- `scaled_joint_trajectory_controller/joint_trajectory` (`trajectory_msgs/JointTrajectory`)

## Build

From the ROS 2 workspace:

```bash
cd gello_software/ros2
colcon build --packages-select ur5e_gello_state_publisher
source install/setup.bash
```

On Windows PowerShell, use:

```powershell
cd gello_software\ros2
colcon build --packages-select ur5e_gello_state_publisher
.\install\setup.ps1
```

Install Python serial support in the same Python environment if it is not already available:

```bash
pip install pyserial
```

## Configure

Edit `config/ur5e_gello.yaml`.

Important parameters:

- `port`: serial port for the URT-2/STS3215 adapter, for example `COM3` on Windows or `/dev/ttyUSB0` on Linux.
- `offset_file`: `servo_offsets.json`, or an absolute path to your calibrated offsets file.
- `trajectory_topic`: UR controller command topic. The default matches the common scaled joint trajectory controller.
- `publish_trajectory`: set to `false` if you only want to publish joint states.

## Run

```bash
ros2 launch ur5e_gello_state_publisher ur5e_gello.launch.py
```

With a custom config:

```bash
ros2 launch ur5e_gello_state_publisher ur5e_gello.launch.py config_file:=/absolute/path/to/ur5e_gello.yaml
```

## Simulation Smoke Test

Run a fake GELLO publisher without STS3215 hardware:

```bash
ros2 launch ur5e_gello_state_publisher fake_ur5e_gello.launch.py
```

Check the published messages:

```bash
ros2 topic echo /gello/joint_states
ros2 topic echo /joint_trajectory_controller/joint_trajectory
```

To drive the UR fake hardware controller in `UR_OnRobot_ROS2`, launch the robot with fake hardware and the unscaled controller:

```bash
ros2 launch ur_onrobot_control start_robot.launch.py \
  use_fake_hardware:=true \
  initial_joint_controller:=joint_trajectory_controller
```

The fake publisher defaults to `joint_trajectory_controller/joint_trajectory`, because the scaled controller is not used by the fake hardware launch path.

## Real GELLO With UR Fake Hardware

Use this when the physical STS3215 GELLO controller is connected, but the UR5e robot should be simulated with ROS 2 fake hardware.

Terminal 1:

```bash
ros2 launch ur_onrobot_control start_robot.launch.py \
  ur_type:=ur5e \
  use_fake_hardware:=true \
  initial_joint_controller:=joint_trajectory_controller
```

Terminal 2:

```bash
ros2 launch ur5e_gello_state_publisher ur5e_gello_sim.launch.py
```

If your serial port is not `/dev/ttyUSB0`, pass it as a launch argument:

```bash
ros2 launch ur5e_gello_state_publisher ur5e_gello_sim.launch.py port:=/dev/ttyACM0
```

Useful checks:

```bash
ros2 topic echo /gello/joint_states
ros2 topic echo /joint_trajectory_controller/joint_trajectory
ros2 control list_controllers
```
