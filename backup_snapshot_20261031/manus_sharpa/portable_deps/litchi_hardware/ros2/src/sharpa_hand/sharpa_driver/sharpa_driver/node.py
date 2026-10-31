"""Consume joint targets and publish feedback for one or two Sharpa Wave hands."""

from __future__ import annotations

import time
import json
import math
from collections.abc import Sequence
from functools import partial

import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState as RosJointState
from std_msgs.msg import String

from litchi_hardware.core.types import DeviceAction, HandSide
from litchi_hardware.hardware.sharpa.schema import SHARPA_WAVE_JOINT_SCHEMA
from litchi_hardware.hardware.sharpa import (
    SHARPA_FINGERS,
    SHARPA_JOINT_NAMES,
    SharpaConfig,
    create_hand,
)

from .messages import (
    tactile_image_message,
    tactile_topic_prefix,
    tactile_wrench_message,
    validate_tactile_route,
)


def normalize_command(
    names: Sequence[str], positions: Sequence[float], side: HandSide
) -> tuple[float, ...]:
    """Accept SDK-order vectors or reorder named generic/side-prefixed joints."""

    if not names:
        if len(positions) != 22:
            raise ValueError(f"expected 22 unnamed joint positions, received {len(positions)}")
        return tuple(float(value) for value in positions)
    if len(names) != len(positions):
        raise ValueError("joint command name and position lengths differ")
    by_name = dict(zip(names, positions, strict=True))
    result = []
    missing = []
    for generic_name in SHARPA_JOINT_NAMES:
        prefixed_name = f"{side.value}_{generic_name}"
        if generic_name in by_name:
            result.append(float(by_name[generic_name]))
        elif prefixed_name in by_name:
            result.append(float(by_name[prefixed_name]))
        else:
            missing.append(generic_name)
    if missing:
        raise ValueError("joint command is missing: " + ", ".join(missing))
    return tuple(result)


class SharpaDriverNode(Node):
    def __init__(self) -> None:
        super().__init__("sharpa_driver")
        self.declare_parameter("use_fake_hardware", False)
        self.declare_parameter("hand_side", "left")
        self.declare_parameter("serial", "")
        self.declare_parameter("enable_left", False)
        self.declare_parameter("enable_right", False)
        self.declare_parameter("left_serial", "")
        self.declare_parameter("right_serial", "")
        self.declare_parameter("speed_coefficient", 0.3)
        self.declare_parameter("current_coefficient", 0.6)
        self.declare_parameter("interpolate", True)
        self.declare_parameter("discovery_timeout_s", 10.0)
        self.declare_parameter("sdk_python_path", "")
        self.declare_parameter("command_topic", "")
        self.declare_parameter("state_topic", "")
        self.declare_parameter("left_command_topic", "/sharpa/left/command")
        self.declare_parameter("right_command_topic", "/sharpa/right/command")
        self.declare_parameter(
            "left_state_topic", "/sharpa/left/joint_states"
        )
        self.declare_parameter(
            "right_state_topic", "/sharpa/right/joint_states"
        )
        self.declare_parameter("state_rate_hz", 30.0)
        self.declare_parameter("publish_tactile", True)
        self.declare_parameter("tactile_poll_rate_hz", 60.0)
        self.declare_parameter("stop_on_command_timeout_s", 0.0)

        enable_left = bool(self.get_parameter("enable_left").value)
        enable_right = bool(self.get_parameter("enable_right").value)
        if enable_left or enable_right:
            sides = tuple(
                side
                for side, enabled in (
                    (HandSide.LEFT, enable_left),
                    (HandSide.RIGHT, enable_right),
                )
                if enabled
            )
        else:
            side_value = self.get_parameter("hand_side").value.lower()
            try:
                sides = (HandSide(side_value),)
            except ValueError as exc:
                raise ValueError("hand_side must be 'left' or 'right'") from exc

        use_fake_hardware = self.get_parameter("use_fake_hardware").value
        common_serial = self.get_parameter("serial").value
        self._hands = {}
        self._state_publishers = {}
        self._tactile_publishers = {}
        self._result_publishers = {}
        self._command_subscriptions = []
        publish_tactile = bool(self.get_parameter("publish_tactile").value)
        try:
            for side in sides:
                serial = (
                    self.get_parameter(f"{side.value}_serial").value
                    if len(sides) > 1
                    else common_serial
                    or self.get_parameter(f"{side.value}_serial").value
                )
                config = SharpaConfig(
                    name=f"{self.get_name()}_{side.value}",
                    serial=serial or None,
                    side=side,
                    speed_coefficient=self.get_parameter(
                        "speed_coefficient"
                    ).value,
                    current_coefficient=self.get_parameter(
                        "current_coefficient"
                    ).value,
                    interpolate=self.get_parameter("interpolate").value,
                    enable_tactile=publish_tactile,
                    discovery_timeout_s=self.get_parameter(
                        "discovery_timeout_s"
                    ).value,
                    sdk_python_path=(
                        self.get_parameter("sdk_python_path").value or None
                    ),
                )
                hand = create_hand(
                    config, use_fake_hardware=use_fake_hardware
                )
                hand.connect()
                self._hands[side] = hand

                command_topic = self.get_parameter(
                    f"{side.value}_command_topic"
                ).value
                state_topic = self.get_parameter(
                    f"{side.value}_state_topic"
                ).value
                if len(sides) == 1:
                    command_topic = (
                        self.get_parameter("command_topic").value or command_topic
                    )
                    state_topic = (
                        self.get_parameter("state_topic").value or state_topic
                    )
                self._state_publishers[side] = self.create_publisher(
                    RosJointState, state_topic, qos_profile_sensor_data
                )
                self._result_publishers[side] = self.create_publisher(
                    String, f"/sharpa/{side.value}/command_result",
                    QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE)
                )
                self._command_subscriptions.append(
                    self.create_subscription(
                        RosJointState,
                        command_topic,
                        partial(self._on_command, side),
                        10,
                    )
                )
                if publish_tactile and getattr(
                    hand, "tactile_available", False
                ):
                    for finger in SHARPA_FINGERS:
                        prefix = tactile_topic_prefix(side, finger)
                        self._tactile_publishers[(side, finger)] = {
                            "raw": self.create_publisher(
                                Image,
                                f"{prefix}/raw",
                                qos_profile_sensor_data,
                            ),
                            "deformation": self.create_publisher(
                                Image,
                                f"{prefix}/deformation",
                                qos_profile_sensor_data,
                            ),
                            "deformation_se": self.create_publisher(
                                Image,
                                f"{prefix}/deformation_se",
                                qos_profile_sensor_data,
                            ),
                            "wrench": self.create_publisher(
                                WrenchStamped,
                                f"{prefix}/wrench",
                                qos_profile_sensor_data,
                            ),
                        }
                elif publish_tactile:
                    self.get_logger().warning(
                        f"Sharpa {side.value} does not report fingertip tactile support"
                    )
        except Exception:
            self._disconnect_all()
            raise

        state_rate_hz = float(self.get_parameter("state_rate_hz").value)
        if state_rate_hz <= 0:
            raise ValueError("state_rate_hz must be positive")
        self._state_timer = self.create_timer(1.0 / state_rate_hz, self._publish_state)
        tactile_rate_hz = float(
            self.get_parameter("tactile_poll_rate_hz").value
        )
        if tactile_rate_hz <= 0:
            raise ValueError("tactile_poll_rate_hz must be positive")
        self._tactile_timer = (
            self.create_timer(
                1.0 / tactile_rate_hz, self._publish_tactile
            )
            if self._tactile_publishers
            else None
        )
        self._last_tactile_warning_at = {
            side: 0.0 for side in sides
        }
        self._command_timeout_s = float(
            self.get_parameter("stop_on_command_timeout_s").value
        )
        self._last_command_at = {side: time.monotonic() for side in sides}
        self._has_received_command = {side: False for side in sides}
        self._timeout_stopped = {side: False for side in sides}
        self.get_logger().info(
            "Sharpa connected in one SDK process for "
            + ", ".join(side.value for side in sides)
            + f"; fake={use_fake_hardware}"
        )

    def _on_command(self, side: HandSide, message: RosJointState) -> None:
        status, reason = "sdk_call_succeeded", ""
        target_info = {}
        try:
            requested = normalize_command(message.name, message.position, side)
            if not all(math.isfinite(v) for v in requested):
                raise ValueError("Joint targets must contain only finite values")
            # Use the exact schema enforced by the hand's final send_action guard.
            # Clip each finite target; never widen or bypass the underlying limits.
            positions = tuple(min(upper, max(lower, value)) for value, (lower, upper)
                              in zip(requested, SHARPA_WAVE_JOINT_SCHEMA.limits, strict=True))
            target_info = dict(names=list(SHARPA_JOINT_NAMES), requested_target=list(requested),
                               target=list(positions), clamped_joints=[name for name, raw, safe in
                               zip(SHARPA_JOINT_NAMES, requested, positions, strict=True) if raw != safe])
            self._hands[side].send_action(
                DeviceAction({"joint_position": positions})
            )
            self._last_command_at[side] = time.monotonic()
            self._has_received_command[side] = True
            self._timeout_stopped[side] = False
        except Exception as exc:
            status, reason = "rejected", str(exc)
            self.get_logger().error(
                f"rejected {side.value} Sharpa joint command: {exc}"
            )
        # Observability only: failure to publish this receipt must never change
        # command delivery, timeout bookkeeping, limits, or hand enable state.
        try:
            report = dict(schema="sharpa_command_result_v2", side=side.value,
                          stamp_ns=self.get_clock().now().nanoseconds,
                          command_stamp_ns=message.header.stamp.sec*10**9+message.header.stamp.nanosec,
                          status=status, reason=reason[:2048], execution_confirmed=False, **target_info)
            self._result_publishers[side].publish(String(data=json.dumps(report, allow_nan=False)))
        except Exception as exc:
            self.get_logger().error(f"failed to publish {side.value} command receipt: {exc}")

    def _publish_state(self) -> None:
        for side, hand in self._hands.items():
            try:
                if (
                    self._command_timeout_s > 0
                    and self._has_received_command[side]
                    and not self._timeout_stopped[side]
                    and time.monotonic() - self._last_command_at[side]
                    > self._command_timeout_s
                ):
                    hand.stop()
                    self._timeout_stopped[side] = True
                    self.get_logger().warning(
                        f"Sharpa {side.value} stopped after command timeout"
                    )
                state = hand.read_joint_state()
                message = RosJointState()
                message.header.stamp = self.get_clock().now().to_msg()
                message.name = [
                    f"{side.value}_{name}" for name in state.names
                ]
                message.position = list(state.position)
                message.velocity = list(state.velocity)
                message.effort = list(state.effort)
                self._state_publishers[side].publish(message)
            except Exception as exc:
                self.get_logger().error(
                    f"failed to service {side.value} Sharpa hand: {exc}"
                )

    def _publish_tactile(self) -> None:
        for (side, finger), publishers in self._tactile_publishers.items():
            hand = self._hands[side]
            try:
                frame = hand.read_tactile(finger, timeout_s=0.0)
                if frame is None:
                    continue
                stamp = self.get_clock().now().to_msg()
                frame_id = validate_tactile_route(
                    frame,
                    expected_side=side,
                    expected_finger=finger,
                )
                if frame.raw is not None:
                    publishers["raw"].publish(
                        tactile_image_message(
                            frame.raw, frame_id=frame_id, stamp=stamp
                        )
                    )
                if frame.deformation is not None:
                    publishers["deformation"].publish(
                        tactile_image_message(
                            frame.deformation,
                            frame_id=frame_id,
                            stamp=stamp,
                        )
                    )
                if frame.deformation_se is not None:
                    publishers["deformation_se"].publish(
                        tactile_image_message(
                            frame.deformation_se, frame_id=frame_id, stamp=stamp
                        )
                    )
                if frame.wrench:
                    publishers["wrench"].publish(
                        tactile_wrench_message(frame, stamp=stamp)
                    )
            except Exception as exc:
                now = time.monotonic()
                if now - self._last_tactile_warning_at[side] > 2.0:
                    self.get_logger().error(
                        f"failed to publish {side.value} tactile: {exc}"
                    )
                    self._last_tactile_warning_at[side] = now

    def _disconnect_all(self) -> None:
        for hand in getattr(self, "_hands", {}).values():
            try:
                hand.stop()
            finally:
                hand.disconnect()
        if hasattr(self, "_hands"):
            self._hands.clear()

    def destroy_node(self) -> bool:
        self._disconnect_all()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = SharpaDriverNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
