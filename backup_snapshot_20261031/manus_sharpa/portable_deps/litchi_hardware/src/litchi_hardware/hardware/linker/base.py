"""Shared validation and HAL behavior for Linker Hand devices."""

from __future__ import annotations

import math
import time
from abc import abstractmethod
from collections.abc import Sequence

from litchi_hardware.core.config import DeviceInfo
from litchi_hardware.core.errors import CommandRejectedError, ConnectionError
from litchi_hardware.core.interfaces import HandInterface
from litchi_hardware.core.lifecycle import ConnectionState, LifecycleGuard
from litchi_hardware.core.types import (
    DeviceAction,
    DeviceObservation,
    FeatureSpec,
    JointSchema,
)

from .config import LinkerConfig, LinkerModel
from .schema import LINKER_G20_JOINT_SCHEMA, LINKER_O30I_JOINT_SCHEMA


def schema_for_model(model: LinkerModel) -> JointSchema:
    if model is LinkerModel.G20:
        return LINKER_G20_JOINT_SCHEMA
    return LINKER_O30I_JOINT_SCHEMA


class LinkerHandBase(HandInterface):
    def __init__(self, config: LinkerConfig) -> None:
        self.config = config
        self._lifecycle = LifecycleGuard()
        self._serial: str | None = None
        self._firmware_version: str | None = None
        self._stopped = False

    @property
    @abstractmethod
    def transport_name(self) -> str: ...

    @property
    def joint_schema(self) -> JointSchema:
        return schema_for_model(self.config.model)

    @property
    def info(self) -> DeviceInfo:
        return DeviceInfo(
            name=self.config.name,
            vendor="Linker Hand",
            model=self.config.model.value,
            serial=self._serial,
            metadata={
                "transport": self.transport_name,
                "side": self.config.side.value,
                "channel": self.config.channel,
                "firmware_version": self._firmware_version,
            },
        )

    @property
    def connection_state(self) -> ConnectionState:
        return self._lifecycle.state

    @property
    def observation_features(self) -> tuple[FeatureSpec, ...]:
        schema = self.joint_schema
        return (
            FeatureSpec("joint_position", schema.size, schema.unit),
            FeatureSpec("joint_velocity", schema.size, "rad/s"),
        )

    @property
    def action_features(self) -> tuple[FeatureSpec, ...]:
        schema = self.joint_schema
        return (
            FeatureSpec(
                "joint_position",
                schema.size,
                schema.unit,
                schema.minimum,
                schema.maximum,
            ),
        )

    def get_observation(self) -> DeviceObservation:
        state = self.read_joint_state()
        return DeviceObservation({"joint_state": state}, state.timestamp_s or time.time())

    def send_action(self, action: DeviceAction) -> None:
        if "joint_position" not in action.features:
            raise CommandRejectedError("Linker Hand action requires 'joint_position'")
        self.command_joint_position(action.features["joint_position"])

    def command_joint_position(self, positions: Sequence[float]) -> None:
        if not self.is_connected:
            raise ConnectionError("Linker Hand is not connected")
        if self._stopped:
            raise ConnectionError("Linker Hand is stopped")
        values = tuple(float(value) for value in positions)
        schema = self.joint_schema
        if len(values) != schema.size:
            raise CommandRejectedError(
                f"{self.config.model.value} requires {schema.size} joint positions, "
                f"received {len(values)}"
            )
        if not all(math.isfinite(value) for value in values):
            raise CommandRejectedError("Linker Hand joint positions must be finite")
        violations = [
            schema.names[index]
            for index, (value, (lower, upper)) in enumerate(zip(values, schema.limits, strict=True))
            if value < lower or value > upper
        ]
        if violations:
            raise CommandRejectedError(
                "Linker Hand joint limits exceeded: " + ", ".join(violations)
            )
        self._write_joint_position(values)

    @abstractmethod
    def _write_joint_position(self, positions: tuple[float, ...]) -> None: ...

    def open(self) -> None:
        self.command_joint_position(self.config.open_position)

    def close(self) -> None:
        self.command_joint_position(self.config.close_position)
