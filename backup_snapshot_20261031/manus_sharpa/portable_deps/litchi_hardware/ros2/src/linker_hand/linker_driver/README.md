# linker_driver

ROS 2 bridge for Linker Hand G20 and O30I devices. Hardware lifecycle,
calibration, SDK calls, and command validation remain in `litchi_hardware`;
this package only translates `sensor_msgs/JointState` and tactile images.

Build and run without hardware:

```bash
python3 -m pip install -e '.[linker]'
cd ros2
colcon build --symlink-install --packages-select linker_driver
source install/setup.bash
ros2 launch linker_driver linker_hand.launch.py use_fake_hardware:=true
```

The default relative topics under namespace `/linker/right` are:

- `joint_command` (`sensor_msgs/JointState`): complete 20-joint position
  commands in radians. Names may be omitted, use the model schema, or carry a
  `left_`/`right_` prefix.
- `joint_states` (`sensor_msgs/JointState`): position and derived velocity.
- `tactile/{finger}` (`sensor_msgs/Image`, `32FC1`): optional matrix tactile.

For O30I transparent SocketCAN adapters, configure `config/o30i.yaml`, bring up
`can0`, and launch normally. Set `auto_setup_can: true` only when passwordless
`sudo ip` is configured. For the metal CAN-FD adapter, set
`transport: libcanbus`; the wheel selects its bundled runtime automatically.
G20 uses `config/g20.yaml` and SocketCAN.

No movement is sent while connecting. Before real commands, calibrate every
joint's affine map (`vendor_tick = gain * radians + offset`) and verify the
open/close direction on the physical hand. `use_fake_hardware` exercises the
same HAL and ROS topics without opening CAN.
