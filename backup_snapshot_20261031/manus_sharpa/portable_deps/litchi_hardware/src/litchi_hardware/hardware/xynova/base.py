"""Shared validation and core HAL behavior for Xynova hands."""

from __future__ import annotations

import math
import time
from abc import abstractmethod
from collections.abc import Sequence

from litchi_hardware.core.config import DeviceInfo
from litchi_hardware.core.errors import CommandRejectedError, ConnectionError
from litchi_hardware.core.interfaces import HandInterface
from litchi_hardware.core.lifecycle import ConnectionState, LifecycleGuard
from litchi_hardware.core.types import DeviceAction, DeviceObservation, FeatureSpec

from .config import XynovaConfig
from .schema import XYNOVA_DEXTEROUS_HAND_JOINT_SCHEMA


class XynovaHandBase(HandInterface):
    def __init__(self, config: XynovaConfig) -> None:
        self.config = config
        self._lifecycle = LifecycleGuard()
        self._serial: str | None = None
        self._model = "Dexterous Hand"
        self._hardware_version: str | None = None
        self._software_version: str | None = None

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
                "can_interface": self.config.can_interface,
                "device_id": self.config.device_id,
                "hardware_version": self._hardware_version,
                "software_version": self._software_version,
            },
        )

    @property
    def connection_state(self) -> ConnectionState:
        return self._lifecycle.state

    @property
    def observation_features(self) -> tuple[FeatureSpec, ...]:
        schema = XYNOVA_DEXTEROUS_HAND_JOINT_SCHEMA
        features = [FeatureSpec("joint_position", schema.size, schema.unit)]
        return tuple(features)

    @property
    def action_features(self) -> tuple[FeatureSpec, ...]:
        schema = XYNOVA_DEXTEROUS_HAND_JOINT_SCHEMA
        return (FeatureSpec("joint_position", schema.size, schema.unit),)

    def get_observation(self) -> DeviceObservation:
        state = self.read_joint_state()
        return DeviceObservation({"joint_state": state}, state.timestamp_s or time.time())

    def send_action(self, action: DeviceAction) -> None:
        if "joint_position" not in action.features:
            raise CommandRejectedError("Xynova action requires 'joint_position'")
        self.command_joint_position(action.features["joint_position"])

    def command_joint_position(self, positions: Sequence[float]) -> None:
        if not self.is_connected:
            raise ConnectionError("Xynova hand is not connected")
        values = tuple(float(value) for value in positions)
        expected = XYNOVA_DEXTEROUS_HAND_JOINT_SCHEMA.size
        if len(values) != expected:
            raise CommandRejectedError(
                f"Xynova requires {expected} joint positions, received {len(values)}"
            )
        if not all(math.isfinite(value) for value in values):
            raise CommandRejectedError("Xynova joint positions must be finite")
        self._write_joint_position(values)

    @abstractmethod
    def _write_joint_position(self, positions: tuple[float, ...]) -> None: ...

    def _command_preset(self, preset_name: str, positions: tuple[float, ...] | None) -> None:
        if positions is None:
            raise CommandRejectedError(f"Xynova {preset_name}_position is not configured")
        self.command_joint_position(positions)

    def open(self) -> None:
        self._command_preset("open", self.config.open_position)

    def close(self) -> None:
        self._command_preset("close", self.config.close_position)
