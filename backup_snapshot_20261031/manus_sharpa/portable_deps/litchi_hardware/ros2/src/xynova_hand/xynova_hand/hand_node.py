"""ROS 2 transport for a core-aligned Xynova hand.

All lifecycle, validation, unit conversion, and vendor calls live in
``litchi_hardware.hardware.xynova``. This node only translates ROS
``sensor_msgs/JointState`` messages to and from the shared ``HandInterface``.
"""

from __future__ import annotations

import math
import struct
import threading
from typing import Any

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState as RosJointState

from litchi_hardware.core.interfaces import HandInterface, TactileCapable
from litchi_hardware.core.types import DeviceAction
from litchi_hardware.hardware.xynova import (
    XYNOVA_JOINT_NAMES,
    create_hand,
    is_sdk_loaded,
)
from xynova_hand.config_loader import build_hand_config
from xynova_hand.joint_mapping import (
    JointMapping,
    UnknownJointError,
)

COMMAND_TOPIC_DEFAULT = "joint_command"
STATE_TOPIC_DEFAULT = "joint_states"
TACTILE_TOPIC_DEFAULT = "tactile_forces"


class HandNode(Node):
    def __init__(self) -> None:
        super().__init__("xynova_hand_node")
        self._declare_params()
        params = self._collect_params()

        explicit_dry_run = bool(params["dry_run"])
        self._dry_run = explicit_dry_run or not is_sdk_loaded()
        if explicit_dry_run and is_sdk_loaded():
            self.get_logger().warning(
                "dry_run=True but SDK is loaded; using the core mock backend."
            )
        elif not is_sdk_loaded():
            self.get_logger().warning(
                "Xynova SDK runtime is unavailable; using the core mock backend."
            )

        self._mapping = JointMapping()
        self._last_targets: list[float] = [0.0] * len(XYNOVA_JOINT_NAMES)
        self._have_target_baseline = self._dry_run
        self._lock = threading.Lock()
        state_hz = float(params["state_publish_hz"])
        tactile_hz = float(params["tactile_publish_hz"])
        if state_hz <= 0 or tactile_hz <= 0:
            raise ValueError("state_publish_hz and tactile_publish_hz must be positive")

        self._hand: HandInterface | None = None
        try:
            config = build_hand_config(params)
            self._hand = create_hand(config, use_fake_hardware=self._dry_run)
            self._hand.connect()
        except Exception:
            self._disconnect_hand()
            raise

        cmd_qos = QoSProfile(
            depth=int(params["command_queue_depth"]),
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._cmd_sub = self.create_subscription(
            RosJointState,
            str(params["command_topic"]),
            self._on_command,
            cmd_qos,
        )
        self._state_pub = self.create_publisher(RosJointState, str(params["state_topic"]), 10)
        self._state_timer = self.create_timer(1.0 / state_hz, self._publish_state)
        self._tactile_pub = None
        self._tactile_timer = None
        if config.enable_tactile:
            if not isinstance(self._hand, TactileCapable):
                self._disconnect_hand()
                raise RuntimeError("configured Xynova backend does not support tactile feedback")
            self._tactile_pub = self.create_publisher(
                Image, str(params["tactile_topic"]), 10
            )
            self._tactile_timer = self.create_timer(
                1.0 / tactile_hz, self._publish_tactile
            )

        self.get_logger().info(
            f"xynova_hand_node ready (dry_run={self._dry_run}, "
            f"transport={self._hand.info.metadata['transport']}). "
            f"cmd={params['command_topic']!r} state={params['state_topic']!r} "
            f"state_hz={state_hz} tactile={config.enable_tactile}"
        )

    def _declare_params(self) -> None:
        self.declare_parameter("dry_run", False)
        self.declare_parameter("command_topic", COMMAND_TOPIC_DEFAULT)
        self.declare_parameter("state_topic", STATE_TOPIC_DEFAULT)
        self.declare_parameter("tactile_topic", TACTILE_TOPIC_DEFAULT)
        self.declare_parameter("state_publish_hz", 100.0)
        self.declare_parameter("tactile_publish_hz", 100.0)
        self.declare_parameter("command_queue_depth", 10)
        self.declare_parameter("enable_tactile", False)
        self.declare_parameter("config_file", "")
        self.declare_parameter("sdk_log_level", "info")

        for key, value in [
            ("master.name", "xynova_master"),
            ("master.period_us", 1_000),
            ("master.cpu_index", -1),
            ("comm.sub_type", "ChuangXinCAN"),
            ("comm.logger_name", "xynova_hand"),
            ("can.iface", "0.0"),
            ("can.manager_can", False),
            ("can.bitrate", 1_000_000),
            ("can.dbitrate", 5_000_000),
            ("can.sample_point", 0.8),
            ("can.dsample_point", 0.8),
            ("slave.type", "DexterousHandSlave"),
            ("slave.id", 0xFFFFFFFF),
            ("slave.logger_name", "xynova_hand"),
            ("device.type", "DexterousHandDevice"),
            ("device.name", "dexterous_hand"),
        ]:
            self.declare_parameter(key, value)

    def _collect_params(self) -> dict[str, Any]:
        keys = [
            "dry_run",
            "command_topic",
            "state_topic",
            "tactile_topic",
            "state_publish_hz",
            "tactile_publish_hz",
            "command_queue_depth",
            "enable_tactile",
            "config_file",
            "sdk_log_level",
            "master.name",
            "master.period_us",
            "master.cpu_index",
            "comm.sub_type",
            "comm.logger_name",
            "can.iface",
            "can.manager_can",
            "can.bitrate",
            "can.dbitrate",
            "can.sample_point",
            "can.dsample_point",
            "slave.type",
            "slave.id",
            "slave.logger_name",
            "device.type",
            "device.name",
        ]
        params = {key: self.get_parameter(key).value for key in keys}
        params["joint_calibration"] = {}

        config_path = str(params.get("config_file", "")).strip()
        if config_path:
            with open(config_path, encoding="utf-8") as stream:
                document = yaml.safe_load(stream) or {}
            if isinstance(document, dict):
                params["joint_calibration"] = document.get("joint_calibration", {}) or {}
                for key, value in (document.get("params") or {}).items():
                    if key in params and value is not None:
                        params[key] = value
        return params

    def _on_command(self, message: RosJointState) -> None:
        try:
            targets = self._mapping.ros_positions_to_core(
                list(message.name), list(message.position)
            )
            with self._lock:
                is_partial = any(math.isnan(value) for value in targets)
                if is_partial and not self._have_target_baseline:
                    raise ValueError(
                        "first Xynova command must contain all 23 joints; "
                        "no feedback baseline is available yet"
                    )
                for index, value in enumerate(targets):
                    if not math.isnan(value):
                        self._last_targets[index] = value
                snapshot = tuple(self._last_targets)
                self._have_target_baseline = True

            if self._hand is None:
                raise RuntimeError("Xynova hand is unavailable")
            self._hand.send_action(DeviceAction({"joint_position": snapshot}))
        except (UnknownJointError, ValueError) as exc:
            self.get_logger().warning(f"rejecting malformed Xynova command: {exc}")
        except Exception as exc:
            self.get_logger().error(f"failed to command Xynova hand: {exc}")

    def _publish_state(self) -> None:
        if self._hand is None:
            return
        try:
            state = self._hand.read_joint_state()
            with self._lock:
                if not self._have_target_baseline:
                    self._last_targets[:] = state.position
                    self._have_target_baseline = True

            message = RosJointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = list(state.names)
            message.position = list(state.position)
            message.velocity = list(state.velocity)
            message.effort = list(state.effort)
            self._state_pub.publish(message)
        except Exception as exc:
            self.get_logger().warning(
                f"failed to read Xynova state: {exc}",
                throttle_duration_sec=2.0,
            )

    def _publish_tactile(self) -> None:
        if self._hand is None or self._tactile_pub is None:
            return
        try:
            if not isinstance(self._hand, TactileCapable):
                raise RuntimeError("Xynova hand does not support tactile feedback")
            frame = self._hand.read_tactile(timeout_s=0.0)
            if frame.shape != (15, 6):
                raise ValueError(
                    f"expected a (15, 6) Xynova tactile frame, got {frame.shape}"
                )
            message = Image()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = frame.frame_id
            message.height = 15
            message.width = 1
            message.encoding = "32FC6"
            message.is_bigendian = False
            message.step = 6 * 4
            message.data = struct.pack(f"<{len(frame.values)}f", *frame.values)
            self._tactile_pub.publish(message)
        except Exception as exc:
            self.get_logger().warning(
                f"failed to read Xynova tactile feedback: {exc}",
                throttle_duration_sec=2.0,
            )

    def _disconnect_hand(self) -> None:
        hand = getattr(self, "_hand", None)
        if hand is None:
            return
        try:
            hand.stop()
        except Exception as exc:
            self.get_logger().warning(f"failed to stop Xynova hand: {exc}")
        try:
            hand.disconnect()
        except Exception as exc:
            self.get_logger().warning(f"failed to disconnect Xynova hand: {exc}")
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
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
