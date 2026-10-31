# MANUS to Wuji Hand 2 teleoperation

This package connects the existing MANUS ROS driver to Wuji Hand 2. It uses the
official `wuji-sdk` `RetargetSession` implementation; ROS only transports the
MANUS keypoints and the resulting 20-joint command.

The MANUS stream contains 25 points. The bridge converts it to the 21-point
MediaPipe landmark order required by Wuji by retaining the wrist and thumb and
dropping the extra metacarpal-base point from each of the other four fingers.
The SDK output is already in Wuji Hand 2 firmware order and is sent unchanged.

Launch one right hand:

```bash
ros2 launch manus_wuji_teleop teleop.launch.py \
  manus_calibration_directory:=/absolute/path/to/manus/calibration
```

Launch both hands with explicit serial numbers:

```bash
ros2 launch manus_wuji_teleop teleop.launch.py \
  enable_left:=true enable_right:=true \
  left_wuji_serial:=LEFT_SERIAL right_wuji_serial:=RIGHT_SERIAL \
  manus_calibration_directory:=/absolute/path/to/manus/calibration
```

The data path is:

```text
/manus/{side}/keypoints
  -> official Wuji RetargetSession
  -> /wuji/{side}/command
  -> wuji-sdk WujiHand2
```

The launch defaults to the in-process MANUS backend, right hand only, and a
250 ms stale-input limit. Read the Wuji Hand 2 safety and network setup
instructions before enabling motors. Keep the hand clear during startup.

The SDK output is filtered independently for each hand before publication.
`smoothing_alpha` defaults to `0.35`; the first 16 joints are limited to
`max_joint_velocity_rad_s:=12.0`, while the four pinky joints use the stricter
`pinky_max_joint_velocity_rad_s:=5.0`. If the pinky still reacts too sharply,
lower the pinky limit without slowing the other fingers:

```bash
ros2 launch manus_wuji_teleop teleop.launch.py \
  pinky_max_joint_velocity_rad_s:=3.0
```
