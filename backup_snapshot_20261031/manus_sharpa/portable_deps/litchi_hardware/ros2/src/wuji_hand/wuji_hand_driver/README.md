# Wuji Hand 2 ROS driver

This package is a thin ROS 2 bridge around the official `wuji-sdk` Python API.
It accepts 20-joint `sensor_msgs/msg/JointState` commands and publishes online
joint state feedback. Position and velocity use radians and radians per second;
effort uses the SDK current/torque proxy in amperes.

Start a right hand discovered by handedness:

```bash
ros2 launch wuji_hand_driver wuji_hand_2.launch.py
```

Start both hands using explicit serial numbers:

```bash
ros2 launch wuji_hand_driver wuji_hand_2.launch.py \
  enable_left:=true enable_right:=true \
  left_serial:=LEFT_SERIAL right_serial:=RIGHT_SERIAL
```

Topics default to `/wuji/<side>/command` and
`/wuji/<side>/joint_states`. The hand must have all 20 joints online by
default. The driver applies the configured MIT gains and effort limit, enables
the motors, waits for the enabled state, and disables the motors on shutdown.

Wuji Hand 2 uses a static IP. Configure the host network adapter on the same
subnet as the hand before launching the driver. Keep the hand clear at startup,
and tune `mit_kp`, `mit_kd`, and `effort_limit` for the deployed hardware.
