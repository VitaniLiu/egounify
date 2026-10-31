# sharpa_driver

ROS 2 command, state, and tactile bridge for one or two Sharpa Wave hands.
Device discovery, SDK loading, firmware initialization, lifecycle cleanup,
joint validation, and tactile channel selection remain in
`litchi_hardware.hardware.sharpa`; this package only translates between that
HAL contract and standard ROS messages.

## Install and build

Install the private platform wheel built for the ROS interpreter ABI before
building an external workspace. Version 0.1.4 is the minimum supported HAL
because it contains the Setting-mode initialization sequence required by Wave
firmware 3.0.6:

```bash
python3 -m pip install 'litchi-hardware[sharpa]>=0.1.4'
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select sharpa_driver
source install/setup.bash
```

The wheel contains proprietary vendor assets and must come from the team's
private index. It must match the active CPython ABI (for example, use the
`cp312` wheel with CPython 3.12). The ROS package does not import the vendor SDK
or repeat its initialization sequence.

## Run without hardware

Exercise the same HAL and ROS interfaces without device discovery:

```bash
ros2 run sharpa_driver sharpa_driver --ros-args \
  -p use_fake_hardware:=true \
  -p hand_side:=left
```

The full dual-hand launch surface is provided by
`manus_sharpa_teleop/teleop.launch.py`; its `use_fake_hardware`,
`enable_left`, `enable_right`, serial, coefficient, rate, tactile, and command
timeout arguments are forwarded to this node.

## ROS contract

The default per-hand interfaces are:

- Subscribe `/sharpa/{left,right}/command` as `sensor_msgs/JointState`.
- Publish `/sharpa/{left,right}/joint_states` as `sensor_msgs/JointState`.
- Optionally publish `/sharpa/{side}/tactile/{finger}/{raw,deformation}` as
  `sensor_msgs/Image` and `/wrench` as `geometry_msgs/WrenchStamped`.

Commands without names must contain all 22 positions in Sharpa HAL order.
Named commands may use generic joint names or names prefixed with `left_` or
`right_`; the driver reorders them before passing one `DeviceAction` to the
HAL. For real hardware, provide explicit serials when both hands are present
and verify the installation before sending motion commands:

```bash
python3 -c 'from litchi_hardware import sharpa; print(sharpa.load_sdk().__file__)'
```
