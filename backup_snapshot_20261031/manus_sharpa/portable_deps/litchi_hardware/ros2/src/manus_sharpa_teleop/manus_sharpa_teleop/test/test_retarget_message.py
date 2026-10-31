from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose, PoseArray
from manus_driver.node import frame_to_pose_array
from manus_sharpa_teleop.retarget_node import pose_array_to_frame

from litchi_hardware.core.types import (
    HandSide,
    HandTrackingFrame,
)
from litchi_hardware.core.types import (
    Pose as CorePose,
)
from litchi_hardware.hardware.manus import MANUS_SHARPA_25_SCHEMA


def test_pose_array_conversion_preserves_vendor_order() -> None:
    message = PoseArray()
    message.header.stamp.sec = 10
    message.header.stamp.nanosec = 250_000_000
    message.header.frame_id = "manus"
    for index in range(25):
        pose = Pose()
        pose.position.x = float(index)
        pose.orientation.w = 1.0
        message.poses.append(pose)

    frame = pose_array_to_frame(message, HandSide.RIGHT)
    assert frame.timestamp_s == 10.25
    assert frame.side is HandSide.RIGHT
    assert frame.keypoints[-1].position[0] == 24.0


def test_manus_ros_message_uses_ros_receipt_stamp() -> None:
    frame = HandTrackingFrame(
        side=HandSide.LEFT,
        keypoints=tuple(CorePose.identity() for _ in range(25)),
        timestamp_s=123.5,
        schema=MANUS_SHARPA_25_SCHEMA,
    )
    receipt_stamp = Time(sec=456, nanosec=789)

    message = frame_to_pose_array(frame, stamp=receipt_stamp)

    assert message.header.stamp == receipt_stamp
