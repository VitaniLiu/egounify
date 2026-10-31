"""Python wrapper for the Xynova SDK 2.x Dexterous Hand C API."""

from __future__ import annotations

import contextlib
import ctypes as C
from dataclasses import dataclass
from enum import IntEnum
from typing import Final

from litchi_hardware.hardware.xynova import _ffi
from litchi_hardware.hardware.xynova._ffi import JOINT_NUM
from litchi_hardware.hardware.xynova.errors import check


class ControlMode(IntEnum):
    DISABLE = _ffi.CONTROL_MODE_DISABLE
    ANGLE = _ffi.CONTROL_MODE_ANGLE
    SERVO = ANGLE  # SDK 1.x compatibility alias


class HandType(IntEnum):
    UNKNOWN = 0
    LEFT = 1
    RIGHT = 2


class JointID(IntEnum):
    """1-based joint IDs in the SDK 2.x canonical order."""

    FF1 = 1
    FF2 = 2
    FF3 = 3
    FF4 = 4
    MF1 = 5
    MF2 = 6
    MF3 = 7
    MF4 = 8
    RF1 = 9
    RF2 = 10
    RF3 = 11
    RF4 = 12
    LF1 = 13
    LF2 = 14
    LF3 = 15
    LF4 = 16
    LF5 = 17
    TH1 = 18
    TH2 = 19
    TH3 = 20
    TH4 = 21
    WR1 = 22
    WR2 = 23


JOINT_NAMES: Final[tuple[str, ...]] = tuple(joint.name for joint in JointID)


class TactileID(IntEnum):
    TT1 = 0x01
    TT2 = 0x02
    FT1 = 0x03
    FT2 = 0x04
    FT3 = 0x05
    MT1 = 0x06
    MT2 = 0x07
    MT3 = 0x08
    RT1 = 0x09
    RT2 = 0x0A
    RT3 = 0x0B
    LT1 = 0x0C
    LT2 = 0x0D
    LT3 = 0x0E
    PT1 = 0x0F


TACTILE_NAMES: Final[tuple[str, ...]] = tuple(sensor.name for sensor in TactileID)


class WrenchDimension(IntEnum):
    ONE_D = 1
    THREE_D = 3
    SIX_D = 6


class RangePointStatus(IntEnum):
    NO_TARGET = 0
    WEAK_SIGNAL = 1
    VALID = 2
    INVALID = 3


@dataclass(frozen=True, slots=True)
class HandErrorState:
    timestamp_us: int
    warning_flags: int
    fault_flags: int
    joint_error_codes: tuple[int, ...]


@dataclass(frozen=True, slots=True)
class TactileWrench:
    timestamp_us: int
    region: TactileID
    dimension: WrenchDimension
    values: tuple[float, float, float, float, float, float]


@dataclass(frozen=True, slots=True)
class TactileTaxel:
    timestamp_us: int
    region: TactileID
    taxel_id: int
    dimension: WrenchDimension
    values: tuple[float, float, float, float, float, float]


@dataclass(frozen=True, slots=True)
class RangePoint:
    timestamp_us: int
    id: int
    status: RangePointStatus
    distance_m: float


def _typed_float(value: _ffi.XynovaTypedValueC) -> float:
    if value.type != _ffi.DATA_TYPE_FLOAT:
        raise ValueError(f"expected SDK float value, received data type {value.type}")
    return float(value.value.f32)


def _wrench_values(value: _ffi.XynovaTypedValueArray6C) -> tuple[float, ...]:
    if value.type != _ffi.DATA_TYPE_FLOAT:
        raise ValueError(f"expected SDK float wrench, received data type {value.type}")
    return tuple(float(component.f32) for component in value.values)


class Device:
    """RAII-style wrapper around an SDK 2.x ``XynovaDeviceHandle``."""

    def __init__(self, handle: int) -> None:
        self._h = C.c_void_p(handle if isinstance(handle, int) else handle.value)  # type: ignore[union-attr]
        self._running = False

    def valid(self) -> bool:
        return bool(self._h and self._h.value)

    def run(self) -> None:
        check(_ffi.lib.XynovaDexterousHand_run(self._h), "Device.run")
        self._running = True

    def stop(self) -> None:
        check(_ffi.lib.XynovaDexterousHand_stop(self._h), "Device.stop")
        self._running = False

    def close(self) -> None:
        if not self.valid():
            return
        if self._running:
            self.stop()
        _ffi.lib.XynovaCommunicationManager_freeDevice(self._h)
        self._h = C.c_void_p()

    def __enter__(self) -> Device:
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()

    def __del__(self) -> None:
        with contextlib.suppress(Exception):
            self.close()

    def _get_string(self, symbol: str, *, capacity: int = 256) -> str:
        out = C.create_string_buffer(capacity)
        check(getattr(_ffi.lib, symbol)(self._h, out, capacity), symbol)
        return out.value.decode("utf-8", errors="replace")

    def get_product_model(self) -> str:
        return self._get_string("XynovaDexterousHand_getProductModel")

    def get_product_sn(self) -> str:
        return self._get_string("XynovaDexterousHand_getProductSN")

    def get_hardware_version(self) -> str:
        return self._get_string("XynovaDexterousHand_getHardwareVersion")

    def get_software_version(self) -> str:
        return self._get_string("XynovaDexterousHand_getSoftwareVersion")

    def get_hand_type(self) -> HandType:
        result = C.c_uint8()
        check(_ffi.lib.XynovaDexterousHand_getHandType(self._h, C.byref(result)), "getHandType")
        return HandType(result.value)

    def get_error_state(self) -> HandErrorState:
        result = _ffi.DexterousHandErrorStateC()
        check(
            _ffi.lib.XynovaDexterousHand_getErrorState(self._h, C.byref(result)),
            "getErrorState",
        )
        errors = sorted(result.joint_error_code, key=lambda value: int(value.id))
        return HandErrorState(
            timestamp_us=int(result.timestamp_us),
            warning_flags=int(result.warning_flags),
            fault_flags=int(result.fault_flags),
            joint_error_codes=tuple(int(value.data.value.u16) for value in errors),
        )

    def set_joint_control_mode_uniform(self, mode: ControlMode) -> None:
        check(
            _ffi.lib.XynovaDexterousHand_setJointControlAllMode(self._h, int(mode)),
            "setJointControlAllMode",
        )

    @staticmethod
    def _angle_payload(values: list[float]) -> _ffi.JointAngleControlC:
        if len(values) != JOINT_NUM:
            raise ValueError(f"values must have length {JOINT_NUM}, got {len(values)}")
        payload = _ffi.JointAngleControlC()
        for index, value in enumerate(values):
            payload.values[index].id = index + 1
            payload.values[index].data.type = _ffi.DATA_TYPE_FLOAT
            payload.values[index].data.value.f32 = float(value)
        return payload

    def set_joint_angle_async(self, values: list[float]) -> None:
        payload = self._angle_payload(values)
        check(
            _ffi.lib.XynovaDexterousHand_setJointAngle_async(self._h, C.byref(payload)),
            "setJointAngle_async",
        )

    def set_joint_angle(self, values: list[float]) -> None:
        payload = self._angle_payload(values)
        check(
            _ffi.lib.XynovaDexterousHand_setJointAngle(self._h, C.byref(payload)),
            "setJointAngle",
        )

    set_joint_servo_async = set_joint_angle_async
    set_joint_servo = set_joint_angle

    def get_joint_angle(self) -> tuple[list[float], int]:
        result = _ffi.JointAngleStatesC()
        timestamp = C.c_uint64()
        check(
            _ffi.lib.XynovaDexterousHand_getJointAngle(
                self._h, C.byref(result), C.byref(timestamp)
            ),
            "getJointAngle",
        )
        values = sorted(result.values, key=lambda value: int(value.id))
        return [_typed_float(value.data) for value in values], int(timestamp.value)

    get_current_joint_angle = get_joint_angle

    def enable_tactile_wrench(self, enable: bool = True) -> None:
        check(
            _ffi.lib.XynovaDexterousHand_enableTactileWrench(self._h, enable),
            "enableTactileWrench",
        )

    def get_tactile_wrenches(self) -> tuple[list[TactileWrench], int]:
        result = _ffi.TactileWrenchStatesC()
        timestamp = C.c_uint64()
        check(
            _ffi.lib.XynovaDexterousHand_getTactileWrench(
                self._h, C.byref(result), C.byref(timestamp)
            ),
            "getTactileWrench",
        )
        values = sorted(result.values, key=lambda value: int(value.region))
        return [
            TactileWrench(
                timestamp_us=int(value.timestamp_us),
                region=TactileID(value.region),
                dimension=WrenchDimension(value.dimension),
                values=_wrench_values(value.wrench),  # type: ignore[arg-type]
            )
            for value in values
        ], int(timestamp.value)

    def enable_tactile_taxel(self, enable: bool = True) -> None:
        check(
            _ffi.lib.XynovaDexterousHand_enableTactileTaxel(self._h, enable),
            "enableTactileTaxel",
        )

    def get_tactile_taxels(self) -> tuple[list[TactileTaxel], int]:
        buffer = (_ffi.TactileTaxelC * _ffi.TACTILE_TOTAL_TAXEL_NUM)()
        result = _ffi.TactileTaxelStatesC(
            count=_ffi.TACTILE_TOTAL_TAXEL_NUM,
            taxels=C.cast(buffer, C.POINTER(_ffi.TactileTaxelC)),
        )
        timestamp = C.c_uint64()
        check(
            _ffi.lib.XynovaDexterousHand_getTactileTaxel(
                self._h, C.byref(result), C.byref(timestamp)
            ),
            "getTactileTaxel",
        )
        return [
            TactileTaxel(
                timestamp_us=int(value.timestamp_us),
                region=TactileID(value.region),
                taxel_id=int(value.taxel_id),
                dimension=WrenchDimension(value.dimension),
                values=_wrench_values(value.wrench),  # type: ignore[arg-type]
            )
            for value in buffer[: result.count]
        ], int(timestamp.value)

    def enable_range(self, enable: bool = True) -> None:
        check(_ffi.lib.XynovaDexterousHand_enableRange(self._h, enable), "enableRange")

    def get_range(self) -> tuple[list[RangePoint], tuple[int, int], int]:
        buffer = (_ffi.RangePointC * _ffi.RANGE_POINT_NUM)()
        result = _ffi.RangeStatesC(
            count=_ffi.RANGE_POINT_NUM,
            points=C.cast(buffer, C.POINTER(_ffi.RangePointC)),
        )
        timestamp = C.c_uint64()
        check(
            _ffi.lib.XynovaDexterousHand_getRange(
                self._h, C.byref(result), C.byref(timestamp)
            ),
            "getRange",
        )
        points = [
            RangePoint(
                timestamp_us=int(value.timestamp_us),
                id=int(value.id),
                status=RangePointStatus(value.status),
                distance_m=float(value.distance),
            )
            for value in buffer[: result.count]
        ]
        return points, (int(result.height), int(result.width)), int(timestamp.value)
