"""Deterministic in-memory Xynova arm implementation."""

from __future__ import annotations

import time

from litchi_hardware.core.errors import CommandRejectedError
from litchi_hardware.core.lifecycle import ConnectionState
from litchi_hardware.core.types import JointState, Pose

from .base import XynovaArmBase
from .config import XynovaArmConfig
from .schema import arm_joint_schema


class MockXynovaArm(XynovaArmBase):
    def __init__(self, config: XynovaArmConfig) -> None:
        super().__init__(config)
        self._position = (0.0,) * config.joint_count
        self._velocity = (0.0,) * config.joint_count
        self._effort = (0.0,) * config.joint_count
        self._pose = Pose.identity()
        self._wrench = (0.0,) * 6
        self._powered = False

    @property
    def transport_name(self) -> str:
        return "mock"

    def connect(self) -> None:
        if self.is_connected:
            return
        if self.connection_state is ConnectionState.FAILED:
            self._lifecycle.reset()
        self._lifecycle.begin_connect()
        self._serial = f"MOCK-{self.config.model.value.upper()}"
        self._model = f"Mock {self.config.model.value}"
        self._controller_version = "mock"
        self._sdk_version = "0.7.1"
        self._lifecycle.connected()
        if self.config.auto_power_on:
            self.power_on()

    def disconnect(self) -> None:
        if self.connection_state is ConnectionState.DISCONNECTED:
            return
        if self.config.power_off_on_disconnect:
            self._powered = False
        self._lifecycle.disconnected()

    def power_on(self) -> None:
        self._require_connected()
        self._powered = True

    def power_off(self) -> None:
        self._require_connected()
        self._powered = False

    def read_joint_state(self) -> JointState:
        self._require_connected()
        schema = arm_joint_schema(self.config.model)
        return JointState(
            names=schema.names,
            position=self._position,
            velocity=self._velocity,
            effort=self._effort,
            timestamp_s=time.time(),
            schema=schema,
        )

    def read_tcp_pose(self) -> Pose:
        self._require_connected()
        return self._pose

    def read_wrench(self) -> tuple[float, float, float, float, float, float]:
        self._require_connected()
        return self._wrench  # type: ignore[return-value]

    def _write_joint_position(self, positions: tuple[float, ...]) -> None:
        if not self._powered:
            raise CommandRejectedError("Xynova arm is not powered on")
        self._position = positions

    def stop(self) -> None:
        self._require_connected()
        self._velocity = (0.0,) * self.config.joint_count
