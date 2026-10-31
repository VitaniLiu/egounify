from geometry_msgs.msg import Pose, PoseArray
from manus_wuji_teleop.retarget_node import pose_array_to_frame

from litchi_hardware.core.types import HandSide


def test_pose_array_conversion_preserves_manus_order() -> None:
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
    assert frame.frame_id == "manus"
    assert frame.keypoints[-1].position[0] == 24.0
