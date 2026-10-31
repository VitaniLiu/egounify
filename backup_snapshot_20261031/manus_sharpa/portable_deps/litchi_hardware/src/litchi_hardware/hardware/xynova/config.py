"""Configuration for the core-aligned Xynova SDK 2.x hand adapter."""

from __future__ import annotations

import math
from dataclasses import dataclass

from litchi_hardware.core.config import DeviceConfig

from .hand.dexterous_hand import JOINT_NAMES

_JOINT_COUNT = len(JOINT_NAMES)
_IDENTITY_GAIN = (1.0,) * _JOINT_COUNT
_ZERO_OFFSET = (0.0,) * _JOINT_COUNT


def _joint_vector(
    values: tuple[float, ...] | None,
    field_name: str,
    *,
    allow_none: bool,
) -> tuple[float, ...] | None:
    if values is None:
        if allow_none:
            return None
        raise ValueError(f"{field_name} must not be None")
    result = tuple(float(value) for value in values)
    if len(result) != _JOINT_COUNT:
        raise ValueError(f"{field_name} must contain {_JOINT_COUNT} values")
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{field_name} must contain only finite values")
    return result


@dataclass(frozen=True, slots=True)
class XynovaConfig(DeviceConfig):
    """Connection, transport, and unit-conversion settings for one hand.

    The core interface and the SDK 2.x Angle API both exchange radians. Optional
    per-joint calibration is applied as
    ``sdk_value = servo_gain * radians + servo_offset``.
    """

    can_interface: str = "0.0"
    can_driver: str = "ChuangXinCAN"
    manager_can: bool = False
    device_id: int = 0xFFFFFFFF
    bitrate: int = 1_000_000
    data_bitrate: int = 5_000_000
    sample_point: float = 0.8
    data_sample_point: float = 0.8
    master_name: str = "xynova_master"
    master_period_us: int = 1_000
    master_cpu_index: int = -1
    native_device_name: str | None = None
    native_device_type: str = "DexterousHandDevice"
    native_slave_type: str = "DexterousHandSlave"
    logger_name: str | None = None
    log_level: str = "info"
    state_report_period_ms: int = 5
    enable_joint_torque: bool = False
    enable_tactile: bool = False
    async_control: bool = True
    servo_gains: tuple[float, ...] = _IDENTITY_GAIN
    servo_offsets: tuple[float, ...] = _ZERO_OFFSET
    open_position: tuple[float, ...] | None = None
    close_position: tuple[float, ...] | None = None

    def __post_init__(self) -> None:
        super(XynovaConfig, self).__post_init__()
        for field_name, value in (
            ("can_interface", self.can_interface),
            ("can_driver", self.can_driver),
            ("master_name", self.master_name),
            ("native_device_type", self.native_device_type),
            ("native_slave_type", self.native_slave_type),
        ):
            if not value:
                raise ValueError(f"{field_name} must not be empty")
        if self.native_device_name == "":
            raise ValueError("native_device_name must not be empty")
        if self.logger_name == "":
            raise ValueError("logger_name must not be empty")
        if self.log_level.lower() not in {
            "trace",
            "debug",
            "info",
            "warn",
            "error",
            "fatal",
        }:
            raise ValueError("log_level is not a supported Xynova log level")
        if not 0 <= self.device_id <= 0xFFFFFFFF:
            raise ValueError("device_id must be a uint32 value")
        if self.bitrate <= 0 or self.data_bitrate <= 0:
            raise ValueError("CAN bitrates must be positive")
        if not 0.0 <= self.sample_point <= 1.0:
            raise ValueError("sample_point must be in [0, 1]")
        if not 0.0 <= self.data_sample_point <= 1.0:
            raise ValueError("data_sample_point must be in [0, 1]")
        if self.master_period_us <= 0:
            raise ValueError("master_period_us must be positive")
        if self.master_cpu_index < -1:
            raise ValueError("master_cpu_index must be -1 or a non-negative index")
        if not 0 < self.state_report_period_ms <= 0xFFFF:
            raise ValueError("state_report_period_ms must be in [1, 65535]")
        if self.enable_joint_torque:
            raise ValueError("enable_joint_torque is not supported by Xynova SDK 2.x")

        gains = _joint_vector(self.servo_gains, "servo_gains", allow_none=False)
        assert gains is not None
        if any(gain == 0.0 for gain in gains):
            raise ValueError("servo_gains must not contain zero")
        offsets = _joint_vector(self.servo_offsets, "servo_offsets", allow_none=False)
        object.__setattr__(self, "servo_gains", gains)
        object.__setattr__(self, "servo_offsets", offsets)
        object.__setattr__(
            self,
            "open_position",
            _joint_vector(self.open_position, "open_position", allow_none=True),
        )
        object.__setattr__(
            self,
            "close_position",
            _joint_vector(self.close_position, "close_position", allow_none=True),
        )

    @property
    def resolved_device_name(self) -> str:
        return self.native_device_name or self.name

    @property
    def resolved_logger_name(self) -> str:
        return self.logger_name or self.name
