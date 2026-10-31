"""Shared validation and HAL behavior for Xynova robot arms."""

from __future__ import annotations

import math
import time
from abc import abstractmethod
from collections.abc import Sequence

from litchi_hardware.core.config import DeviceInfo
from litchi_hardware.core.errors import CommandRejectedError, ConnectionError
from litchi_hardware.core.interfaces import ArmInterface
from litchi_hardware.core.lifecycle import ConnectionState, LifecycleGuard
from litchi_hardware.core.types import (
    DeviceAction,
    DeviceObservation,
    FeatureSpec,
)

from .config import XynovaArmConfig


class XynovaArmBase(ArmInterface):
    def __init__(self, config: XynovaArmConfig) -> None:
        self.config = config
        self._lifecycle = LifecycleGuard()
        self._serial: str | None = None
        self._model = config.model.value
        self._controller_version: str | None = None
        self._sdk_version: str | None = None
        self._mac: str | None = None

    @property
    @abstractmethod
    def transport_name(self) -> str: ...

    @property
    def info(self) -> DeviceInfo:
        return DeviceInfo(
            name=self.config.name,
            vendor="Xynova",
            model=self._model,
            serial=self._serial,
            metadata={
                "transport": self.transport_name,
                "remote_ip": self.config.remote_ip,
                "local_ip": self.config.local_ip,
                "arm_model": self.config.model.value,
                "joint_count": self.config.joint_count,
                "controller_version": self._controller_version,
                "sdk_version": self._sdk_version,
                "mac": self._mac,
            },
        )

    @property
    def connection_state(self) -> ConnectionState:
        return self._lifecycle.state

    @property
    def observation_features(self) -> tuple[FeatureSpec, ...]:
        count = self.config.joint_count
        features = [
            FeatureSpec("joint_position", count, "rad"),
            FeatureSpec("joint_velocity", count, "rad/s"),
            FeatureSpec("joint_effort", count, "Nm"),
            FeatureSpec("tcp_pose", 7, "m,quaternion"),
        ]
        if self.config.supports_wrench:
            features.append(FeatureSpec("wrench", 6, "N,Nm"))
        return tuple(features)

    @property
    def action_features(self) -> tuple[FeatureSpec, ...]:
        return (FeatureSpec("joint_position", self.config.joint_count, "rad"),)

    def get_observation(self) -> DeviceObservation:
        state = self.read_joint_state()
        timestamp = state.timestamp_s or time.time()
        features: dict[str, object] = {
            "joint_state": state,
            "tcp_pose": self.read_tcp_pose(),
        }
        if self.config.supports_wrench:
            features["wrench"] = self.read_wrench()
        return DeviceObservation(features, timestamp)

    def send_action(self, action: DeviceAction) -> None:
        if "joint_position" not in action.features:
            raise CommandRejectedError("Xynova arm action requires 'joint_position'")
        self.command_joint_position(action.features["joint_position"])

    def command_joint_position(self, positions: Sequence[float]) -> None:
        self._require_connected()
        values = tuple(float(value) for value in positions)
        if len(values) != self.config.joint_count:
            raise CommandRejectedError(
                f"Xynova {self.config.model.value} arm requires "
                f"{self.config.joint_count} joint positions, received {len(values)}"
            )
        if not all(math.isfinite(value) for value in values):
            raise CommandRejectedError("Xynova arm joint positions must be finite")
        self._write_joint_position(values)

    def _require_connected(self) -> None:
        if not self.is_connected:
            raise ConnectionError("Xynova arm is not connected")

    @abstractmethod
    def _write_joint_position(self, positions: tuple[float, ...]) -> None: ...

    @abstractmethod
    def power_on(self) -> None: ...

    @abstractmethod
    def power_off(self) -> None: ...
