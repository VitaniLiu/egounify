# xynova_hand

ROS 2 Humble Python driver for the Xynova Dexterous Hand with SDK 2.x. It uses the
core `HandInterface` implementation and native runtimes bundled by
`litchi-hardware`. The ROS node never calls the vendor manager, device, or FFI
directly.

## Required packages

- ROS 2 Humble with `rclpy`, `sensor_msgs`, `std_msgs`, `launch`, `launch_ros`
  and `ament_index_python`.
- Python 3.10 or newer.
- PyYAML 6 or newer.
- `litchi-hardware[xynova]>=0.1.0`.

The ROS package does not copy or locate vendor libraries itself. Runtime
loading, lifecycle, command validation, state conversion, and SDK error
translation are owned by `litchi_hardware.hardware.xynova`, including bundled
Linux x86_64 and aarch64 SDKs or an external library selected through
`XYNOVA_LIB`.

## Build as a standalone ROS workspace

From the `litchi_hardware` repository:

```text
litchi_hardware/
├── src/litchi_hardware/hardware/xynova/
└── ros2/src/xynova_hand/
```

Install the Python SDK first, then build the ROS package:

```bash
python3 -m pip install '.[xynova]'
cd ros2
colcon build --symlink-install --packages-select xynova_hand
source install/setup.bash
```

Dry-run without touching hardware:

```bash
ros2 launch xynova_hand xynova_hand.launch.py dry_run:=true
```

Real hardware with the bundled x86_64 runtime:

```bash
ros2 launch xynova_hand xynova_hand.launch.py
```

For ARM64 or a separately installed vendor runtime:

```bash
export XYNOVA_LIB=/opt/xynova-arm64/lib/libxynova.so.2.0.0
ros2 launch xynova_hand xynova_hand.launch.py
```

Default ROS interfaces, under namespace `/xynova_hand`:

- Subscribe: `/xynova_hand/joint_command` (`sensor_msgs/msg/JointState`)
- Publish: `/xynova_hand/joint_states` (`sensor_msgs/msg/JointState`)
- Optional publish: `/xynova_hand/tactile_forces` (`sensor_msgs/msg/Image`)

Commands and states use radians at the ROS/core boundary. The YAML
`joint_calibration` table configures the HAL conversion
`sdk_value = gain * radians + offset`.

SDK 2.x reports 23 joint angles and no longer exposes the old raw joint-torque
feedback API, so `joint_states.effort` is empty.

Set `params.enable_tactile: true` to publish a `32FC6` image with `height=15`
and `width=1`. Each row is one tactile region and its channels are
`Fx, Fy, Fz, Mx, My, Mz`. Components beyond the region's reported dimension
are unused by the vendor SDK and published as zero. The canonical row order is:

```text
TT1, TT2,
FT1, FT2, FT3,
MT1, MT2, MT3,
RT1, RT2, RT3,
LT1, LT2, LT3,
PT1
```

Tactile feedback is disabled by default. Its topic and poll rate can be changed
with `tactile_topic` and `tactile_publish_hz`.

Before the first feedback sample, the first real command must contain all 23
joints. Afterwards partial named commands retain the most recent complete
target. Dry-run uses `MockXynovaHand`, which implements the same core interface.
