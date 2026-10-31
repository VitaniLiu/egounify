from __future__ import annotations

from functools import partial

import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from wuji_hand_driver.driver import joint_names

from litchi_hardware.core.types import HandSide, HandTrackingFrame, Pose
from litchi_hardware.hardware.manus import MANUS_SHARPA_25_SCHEMA

from .retargeter import RetargetingFilterConfig, WujiSdkRetargeter


def pose_array_to_frame(message: PoseArray, side: HandSide) -> HandTrackingFrame:
    timestamp_s = float(message.header.stamp.sec) + (
        float(message.header.stamp.nanosec) * 1e-9
    )
    keypoints = tuple(
        Pose(
            (pose.position.x, pose.position.y, pose.position.z),
            (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
        )
        for pose in message.poses
    )
    return HandTrackingFrame(
        side=side,
        keypoints=keypoints,
        timestamp_s=timestamp_s,
        schema=MANUS_SHARPA_25_SCHEMA,
        frame_id=message.header.frame_id or "manus_hand",
    )


class RetargetNode(Node):
    def __init__(self) -> None:
        super().__init__("manus_wuji_retarget")
        self.declare_parameter("enable_left", False)
        self.declare_parameter("enable_right", True)
        self.declare_parameter("left_input_topic", "/manus/left/keypoints")
        self.declare_parameter("right_input_topic", "/manus/right/keypoints")
        self.declare_parameter("left_command_topic", "/wuji/left/command")
        self.declare_parameter("right_command_topic", "/wuji/right/command")
        self.declare_parameter("max_input_age_s", 0.25)
        self.declare_parameter("smoothing_alpha", 0.35)
        self.declare_parameter("max_joint_velocity_rad_s", 12.0)
        self.declare_parameter("pinky_max_joint_velocity_rad_s", 5.0)
        self.declare_parameter("nominal_input_rate_hz", 60.0)
        self.declare_parameter("max_filter_dt_s", 0.05)

        enabled_sides = tuple(
            side
            for side in HandSide
            if bool(self.get_parameter(f"enable_{side.value}").value)
        )
        self._retargeter = WujiSdkRetargeter(
            enabled_sides,
            filter_config=RetargetingFilterConfig(
                smoothing_alpha=float(
                    self.get_parameter("smoothing_alpha").value
                ),
                max_joint_velocity_rad_s=float(
                    self.get_parameter("max_joint_velocity_rad_s").value
                ),
                pinky_max_joint_velocity_rad_s=float(
                    self.get_parameter("pinky_max_joint_velocity_rad_s").value
                ),
                nominal_input_rate_hz=float(
                    self.get_parameter("nominal_input_rate_hz").value
                ),
                max_filter_dt_s=float(
                    self.get_parameter("max_filter_dt_s").value
                ),
            ),
        )
        self._max_input_age_s = float(self.get_parameter("max_input_age_s").value)
        if self._max_input_age_s < 0.0:
            raise ValueError("max_input_age_s must be non-negative")

        self._side_publishers = {}
        self._side_subscriptions = []
        self._last_warning_ns = {side: 0 for side in enabled_sides}
        for side in enabled_sides:
            input_topic = str(
                self.get_parameter(f"{side.value}_input_topic").value
            )
            command_topic = str(
                self.get_parameter(f"{side.value}_command_topic").value
            )
            self._side_publishers[side] = self.create_publisher(
                JointState, command_topic, 10
            )
            self._side_subscriptions.append(
                self.create_subscription(
                    PoseArray,
                    input_topic,
                    partial(self._on_keypoints, side),
                    qos_profile_sensor_data,
                )
            )

        self.get_logger().info(
            "MANUS-to-Wuji official SDK retargeting active for "
            + ", ".join(side.value for side in enabled_sides)
        )

    def _on_keypoints(self, side: HandSide, message: PoseArray) -> None:
        try:
            frame = pose_array_to_frame(message, side)
            now_s = self.get_clock().now().nanoseconds * 1e-9
            if (
                self._max_input_age_s > 0.0
                and frame.timestamp_s > 0.0
                and now_s - frame.timestamp_s > self._max_input_age_s
            ):
                self._warn_throttled(
                    side,
                    f"dropped stale {side.value} MANUS frame "
                    f"({now_s - frame.timestamp_s:.3f}s old)",
                )
                return

            positions = self._retargeter.retarget(frame)
            command = JointState()
            command.header = message.header
            command.name = list(joint_names(side.value))
            command.position = list(positions)
            self._side_publishers[side].publish(command)
        except Exception as exc:
            self._warn_throttled(
                side,
                f"failed to retarget {side.value} MANUS hand: {exc}",
            )

    def _warn_throttled(self, side: HandSide, message: str) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_warning_ns[side] > 1_000_000_000:
            self.get_logger().warning(message)
            self._last_warning_ns[side] = now_ns


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = RetargetNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
