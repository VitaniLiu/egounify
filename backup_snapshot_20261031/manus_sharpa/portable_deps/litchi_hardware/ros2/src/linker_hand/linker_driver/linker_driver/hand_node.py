"""ROS 2 transport for the core-aligned Linker Hand backend."""

from __future__ import annotations

import struct
from pathlib import Path
from typing import Any

import rclpy
import yaml
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState as RosJointState

from linker_driver.config_loader import build_hand_config
from linker_driver.joint_mapping import JointMapping
from litchi_hardware.core.interfaces import HandInterface
from litchi_hardware.core.types import DeviceAction
from litchi_hardware.hardware.linker import create_hand, schema_for_model

_FINGERS = ("thumb", "index", "middle", "ring", "little")


class HandNode(Node):
    def __init__(self) -> None:
        super().__init__("linker_hand_node")
        self._declare_parameters()
        params = self._collect_parameters()
        state_hz = float(params["state_publish_hz"])
        tactile_hz = float(params["tactile_publish_hz"])
        if state_hz <= 0.0 or tactile_hz <= 0.0:
            raise ValueError("publish rates must be positive")

        config = build_hand_config(params)
        self._mapping = JointMapping(
            names=schema_for_model(config.model).names,
            side=config.side.value,
        )
        self._enable_tactile = config.enable_tactile
        self._hand: HandInterface | None = None
        try:
            self._hand = create_hand(
                config,
                use_fake_hardware=bool(params["use_fake_hardware"]),
            )
            self._hand.connect()
        except Exception:
            self._disconnect_hand()
            raise

        self._command_sub = self.create_subscription(
            RosJointState,
            str(params["command_topic"]),
            self._on_command,
            int(params["command_queue_depth"]),
        )
        self._state_pub = self.create_publisher(RosJointState, str(params["state_topic"]), 10)
        self._state_timer = self.create_timer(1.0 / state_hz, self._publish_state)
        self._tactile_publishers: dict[str, Any] = {}
        if self._enable_tactile:
            for finger in _FINGERS:
                topic = f"{str(params['tactile_topic_prefix']).rstrip('/')}/{finger}"
                self._tactile_publishers[finger] = self.create_publisher(Image, topic, 10)
            self._tactile_timer = self.create_timer(1.0 / tactile_hz, self._publish_tactile)
        self.get_logger().info(
            f"Linker Hand ready: model={config.model.value} side={config.side.value} "
            f"transport={self._hand.info.metadata['transport']} "
            f"fake={bool(params['use_fake_hardware'])}"
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "use_fake_hardware": False,
            "config_file": "",
            "name": "linker_hand",
            "model": "O30I",
            "side": "right",
            "transport": "socketcan",
            "channel": "can0",
            "bitrate": 1_000_000,
            "data_bitrate": 5_000_000,
            "auto_setup_can": False,
            "frame_id": -1,
            "canfd_device": 0,
            "canfd_channel": 0,
            "libcanbus_path": "",
            "initial_speed": [],
            "enable_tactile": False,
            "verify_identity": True,
            "command_topic": "joint_command",
            "state_topic": "joint_states",
            "tactile_topic_prefix": "tactile",
            "state_publish_hz": 100.0,
            "tactile_publish_hz": 20.0,
            "command_queue_depth": 10,
        }
        self._parameter_names = tuple(defaults)
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _collect_parameters(self) -> dict[str, Any]:
        params = {name: self.get_parameter(name).value for name in self._parameter_names}
        params["joint_calibration"] = {}
        config_file = str(params["config_file"]).strip()
        if config_file:
            with Path(config_file).open(encoding="utf-8") as stream:
                document = yaml.safe_load(stream) or {}
            params.update(document.get("params", {}) or {})
            params["joint_calibration"] = document.get("joint_calibration", {}) or {}
        return params

    def _on_command(self, message: RosJointState) -> None:
        try:
            positions = self._mapping.ros_positions_to_core(
                list(message.name), list(message.position)
            )
            if self._hand is None:
                raise RuntimeError("Linker Hand is unavailable")
            self._hand.send_action(DeviceAction({"joint_position": positions}))
        except Exception as exc:
            self.get_logger().warning(f"rejecting Linker Hand command: {exc}")

    def _publish_state(self) -> None:
        if self._hand is None:
            return
        try:
            state = self._hand.read_joint_state()
            message = RosJointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = list(state.names)
            message.position = list(state.position)
            message.velocity = list(state.velocity)
            message.effort = list(state.effort)
            self._state_pub.publish(message)
        except Exception as exc:
            self.get_logger().warning(
                f"failed to read Linker Hand state: {exc}",
                throttle_duration_sec=2.0,
            )

    def _publish_tactile(self) -> None:
        if self._hand is None:
            return
        for finger, publisher in self._tactile_publishers.items():
            try:
                frame = self._hand.read_tactile(finger, timeout_s=0.0)
                height, width = frame.shape
                message = Image()
                message.header.stamp = self.get_clock().now().to_msg()
                message.header.frame_id = frame.frame_id
                message.height = height
                message.width = width
                message.encoding = "32FC1"
                message.is_bigendian = False
                message.step = width * 4
                message.data = struct.pack(f"<{len(frame.values)}f", *frame.values)
                publisher.publish(message)
            except Exception as exc:
                self.get_logger().warning(
                    f"failed to read {finger} tactile data: {exc}",
                    throttle_duration_sec=2.0,
                )

    def _disconnect_hand(self) -> None:
        hand = getattr(self, "_hand", None)
        if hand is None:
            return
        try:
            hand.stop()
        except Exception as exc:
            self.get_logger().warning(f"failed to stop Linker Hand: {exc}")
        try:
            hand.disconnect()
        except Exception as exc:
            self.get_logger().warning(f"failed to disconnect Linker Hand: {exc}")
        self._hand = None

    def destroy_node(self) -> bool:  # type: ignore[override]
        self._disconnect_hand()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: HandNode | None = None
    try:
        node = HandNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == "__main__":
    main()
