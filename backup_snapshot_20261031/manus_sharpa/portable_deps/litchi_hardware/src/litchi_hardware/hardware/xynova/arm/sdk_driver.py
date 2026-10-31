"""HAL adapter over the Xynova Arm SDK 0.7.1 native bridge."""

from __future__ import annotations

import math
import time
from collections.abc import Callable
from typing import Any

from litchi_hardware.core.errors import ConnectionError
from litchi_hardware.core.lifecycle import ConnectionState
from litchi_hardware.core.types import JointState, Pose

from ._native import NativeArm
from .base import XynovaArmBase
from .config import XynovaArmConfig
from .schema import arm_joint_schema


def _rpy_to_quaternion(
    roll: float, pitch: float, yaw: float
) -> tuple[float, float, float, float]:
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class XynovaSdkArm(XynovaArmBase):
    def __init__(
        self,
        config: XynovaArmConfig,
        *,
        native_factory: Callable[..., Any] = NativeArm,
    ) -> None:
        super().__init__(config)
        self._native_factory = native_factory
        self._native: Any | None = None

    @property
    def transport_name(self) -> str:
        return "xynova-arm-native-sdk"

    def connect(self) -> None:
        if self.is_connected:
            return
        if self.connection_state is ConnectionState.FAILED:
            self._lifecycle.reset()
        self._lifecycle.begin_connect()
        try:
            native = self._native_factory(
                self.config.native_model_code,
                self.config.remote_ip,
                self.config.local_ip,
            )
            info = native.info
            if info.joint_count != self.config.joint_count:
                native.close()
                raise ConnectionError(
                    f"Xynova arm reports {info.joint_count} joints; "
                    f"configured model expects {self.config.joint_count}"
                )
            self._native = native
            self._serial = info.id or None
            self._mac = info.mac or None
            self._model = info.model or self._model
            self._controller_version = info.controller_version or None
            self._sdk_version = info.sdk_version or None
            self._lifecycle.connected()
            if self.config.auto_power_on:
                self.power_on()
        except Exception:
            self._cleanup()
            self._lifecycle.failed()
            raise

    def _cleanup(self) -> None:
        native, self._native = self._native, None
        if native is None:
            return
        try:
            if self.config.power_off_on_disconnect:
                native.set_power(False)
        finally:
            native.close()

    def disconnect(self) -> None:
        if self.connection_state is ConnectionState.DISCONNECTED:
            return
        try:
            self._cleanup()
        finally:
            self._lifecycle.disconnected()

    def _device(self) -> Any:
        self._require_connected()
        if self._native is None:
            raise ConnectionError("Xynova arm native handle is unavailable")
        return self._native

    def power_on(self) -> None:
        self._device().set_power(True)

    def power_off(self) -> None:
        self._device().set_power(False)

    def read_joint_state(self) -> JointState:
        position, velocity, effort = self._device().read_state()
        schema = arm_joint_schema(self.config.model)
        for name, values in (
            ("position", position),
            ("velocity", velocity),
            ("effort", effort),
        ):
            if len(values) != schema.size:
                raise ConnectionError(
                    f"Xynova arm {name} feedback has {len(values)} values; "
                    f"expected {schema.size}"
                )
        return JointState(
            names=schema.names,
            position=position,
            velocity=velocity,
            effort=effort,
            timestamp_s=time.time(),
            schema=schema,
        )

    def read_tcp_pose(self) -> Pose:
        x, y, z, roll, pitch, yaw = self._device().read_tcp_pose()
        return Pose((x, y, z), _rpy_to_quaternion(roll, pitch, yaw))

    def read_wrench(self) -> tuple[float, float, float, float, float, float]:
        return self._device().read_wrench()

    def _write_joint_position(self, positions: tuple[float, ...]) -> None:
        self._device().command_joint_position(
            positions,
            self.config.default_speed,
            self.config.default_zone,
        )

    def stop(self) -> None:
        self._device().stop()
