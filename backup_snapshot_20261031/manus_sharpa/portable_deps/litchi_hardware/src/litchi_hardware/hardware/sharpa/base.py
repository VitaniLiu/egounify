"""Shared Sharpa Wave validation and HAL behavior."""

from __future__ import annotations

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
)

from .config import SharpaConfig
from .schema import (
    SHARPA_CLOSE_POSITION,
    SHARPA_OPEN_POSITION,
    SHARPA_WAVE_JOINT_SCHEMA,
)


class SharpaHandBase(HandInterface):
    def __init__(self, config: SharpaConfig) -> None:
        self.config = config
        self._lifecycle = LifecycleGuard()
        self._serial = config.serial
        self._side = config.side

    @property
    @abstractmethod
    def transport_name(self) -> str: ...

    @property
    def info(self) -> DeviceInfo:
        return DeviceInfo(
            name=self.config.name,
            vendor="Sharpa",
            model="Wave",
            serial=self._serial,
            metadata={
                "transport": self.transport_name,
                "side": self._side.value if self._side else "unknown",
            },
        )

    @property
    def connection_state(self) -> ConnectionState:
        return self._lifecycle.state

    @property
    def observation_features(self) -> tuple[FeatureSpec, ...]:
        schema = SHARPA_WAVE_JOINT_SCHEMA
        return (
            FeatureSpec("joint_position", schema.size, schema.unit),
            FeatureSpec("joint_velocity", schema.size, "rad/s"),
            FeatureSpec("joint_effort", schema.size, "sdk_torque"),
        )

    @property
    def action_features(self) -> tuple[FeatureSpec, ...]:
        schema = SHARPA_WAVE_JOINT_SCHEMA
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
            raise CommandRejectedError("Sharpa action requires 'joint_position'")
        self.command_joint_position(action.features["joint_position"])

    def command_joint_position(self, positions: Sequence[float]) -> None:
        if not self.is_connected:
            raise ConnectionError("Sharpa hand is not connected")
        values = tuple(float(value) for value in positions)
        schema = SHARPA_WAVE_JOINT_SCHEMA
        if len(values) != schema.size:
            raise CommandRejectedError(
                f"Sharpa requires {schema.size} joint positions, received {len(values)}"
            )
        violations = [
            schema.names[index]
            for index, (value, (lower, upper)) in enumerate(
                zip(values, schema.limits, strict=True)
            )
            if value < lower or value > upper
        ]
        if violations:
            raise CommandRejectedError(
                "Sharpa joint limits exceeded: " + ", ".join(violations)
            )
        self._write_joint_position(values)

    @abstractmethod
    def _write_joint_position(self, positions: tuple[float, ...]) -> None: ...

    def open(self) -> None:
        self.command_joint_position(SHARPA_OPEN_POSITION)

    def close(self) -> None:
        self.command_joint_position(SHARPA_CLOSE_POSITION)
