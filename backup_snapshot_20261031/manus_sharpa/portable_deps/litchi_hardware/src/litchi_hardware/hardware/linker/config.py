"""Configuration for Linker Hand G20 and O30I devices."""

from __future__ import annotations

import math
from dataclasses import dataclass

from litchi_hardware.core._compat import StrEnum
from litchi_hardware.core.config import DeviceConfig
from litchi_hardware.core.types import HandSide

from .schema import LINKER_CLOSE_POSITION, LINKER_OPEN_POSITION

_JOINT_COUNT = 20
_DEFAULT_POSITION_GAINS = (255.0,) * _JOINT_COUNT
_DEFAULT_POSITION_OFFSETS = (0.0,) * _JOINT_COUNT


class LinkerModel(StrEnum):
    G20 = "G20"
    O30I = "O30I"


class LinkerTransport(StrEnum):
    SOCKETCAN = "socketcan"
    LIBCANBUS = "libcanbus"


def _vector(
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
class LinkerConfig(DeviceConfig):
    """Connection and calibration settings for one Linker Hand.

    The HAL uses radians in a calibrated 0..1 range. Vendor ticks are converted
    per joint as ``tick = position_gain * radians + position_offset``.
    """

    model: LinkerModel = LinkerModel.O30I
    side: HandSide = HandSide.RIGHT
    transport: LinkerTransport = LinkerTransport.SOCKETCAN
    channel: str = "can0"
    bitrate: int = 1_000_000
    data_bitrate: int = 5_000_000
    auto_setup_can: bool = False
    frame_id: int | None = None
    canfd_device: int = 0
    canfd_channel: int = 0
    libcanbus_path: str | None = None
    position_gains: tuple[float, ...] = _DEFAULT_POSITION_GAINS
    position_offsets: tuple[float, ...] = _DEFAULT_POSITION_OFFSETS
    open_position: tuple[float, ...] = LINKER_OPEN_POSITION
    close_position: tuple[float, ...] = LINKER_CLOSE_POSITION
    initial_speed: tuple[int, ...] | None = None
    enable_tactile: bool = False
    verify_identity: bool = True

    def __post_init__(self) -> None:
        super(LinkerConfig, self).__post_init__()
        model = LinkerModel(self.model)
        side = HandSide(self.side)
        transport = LinkerTransport(self.transport)
        if not self.channel:
            raise ValueError("channel must not be empty")
        if self.bitrate <= 0 or self.data_bitrate <= 0:
            raise ValueError("CAN bitrates must be positive")
        if self.frame_id is not None and not 0 < self.frame_id <= 0x7FF:
            raise ValueError("frame_id must be in [1, 2047]")
        if self.canfd_device < 0 or self.canfd_channel < 0:
            raise ValueError("CAN-FD device and channel indices must not be negative")
        if self.libcanbus_path == "":
            raise ValueError("libcanbus_path must not be empty")
        if model is LinkerModel.G20 and transport is LinkerTransport.LIBCANBUS:
            raise ValueError("G20 supports the socketcan transport only")

        gains = _vector(self.position_gains, "position_gains", allow_none=False)
        offsets = _vector(self.position_offsets, "position_offsets", allow_none=False)
        assert gains is not None and offsets is not None
        if any(gain == 0.0 for gain in gains):
            raise ValueError("position_gains must not contain zero")
        for index, (gain, offset) in enumerate(zip(gains, offsets, strict=True)):
            endpoints = (offset, gain + offset)
            if min(endpoints) < 0.0 or max(endpoints) > 255.0:
                raise ValueError(
                    f"calibration for joint {index} maps the 0..1 rad range "
                    "outside vendor ticks [0, 255]"
                )

        initial_speed = self.initial_speed
        if initial_speed is not None:
            initial_speed = tuple(int(value) for value in initial_speed)
            expected_speed_size = 5 if model is LinkerModel.G20 else _JOINT_COUNT
            if len(initial_speed) != expected_speed_size:
                raise ValueError(
                    f"{model.value} initial_speed must contain {expected_speed_size} vendor values"
                )
            if any(value < 0 or value > 255 for value in initial_speed):
                raise ValueError("initial_speed values must be in [0, 255]")

        object.__setattr__(self, "model", model)
        object.__setattr__(self, "side", side)
        object.__setattr__(self, "transport", transport)
        object.__setattr__(self, "position_gains", gains)
        object.__setattr__(self, "position_offsets", offsets)
        object.__setattr__(
            self,
            "open_position",
            _vector(self.open_position, "open_position", allow_none=False),
        )
        object.__setattr__(
            self,
            "close_position",
            _vector(self.close_position, "close_position", allow_none=False),
        )
        object.__setattr__(self, "initial_speed", initial_speed)
