"""Type-safe ctypes adapter over the project-owned Xynova Arm bridge."""

from __future__ import annotations

import ctypes
from dataclasses import dataclass
from pathlib import Path

from litchi_hardware.core.errors import ConnectionError, DependencyUnavailableError

from .vendor import load_arm_bridge

_BRIDGE_ABI_VERSION = 1
_MAX_JOINTS = 7


class _NativeState(ctypes.Structure):
    _fields_ = [
        ("joint_count", ctypes.c_uint32),
        ("position", ctypes.c_double * _MAX_JOINTS),
        ("velocity", ctypes.c_double * _MAX_JOINTS),
        ("effort", ctypes.c_double * _MAX_JOINTS),
    ]


class _NativeInfo(ctypes.Structure):
    _fields_ = [
        ("joint_count", ctypes.c_uint32),
        ("id", ctypes.c_char * 128),
        ("mac", ctypes.c_char * 64),
        ("controller_version", ctypes.c_char * 64),
        ("model", ctypes.c_char * 128),
        ("sdk_version", ctypes.c_char * 64),
    ]


@dataclass(frozen=True, slots=True)
class NativeArmInfo:
    joint_count: int
    id: str
    mac: str
    controller_version: str
    model: str
    sdk_version: str


def _decode(value: bytes) -> str:
    return value.split(b"\0", 1)[0].decode("utf-8", errors="replace")


class NativeArm:
    """One connected robot handle owned by the native bridge."""

    def __init__(
        self,
        model_code: int,
        remote_ip: str,
        local_ip: str = "",
        *,
        library_path: str | None = None,
    ) -> None:
        try:
            library = (
                load_arm_bridge()
                if library_path is None
                else ctypes.CDLL(str(Path(library_path).expanduser().resolve()))
            )
        except (OSError, DependencyUnavailableError) as exc:
            raise DependencyUnavailableError(
                f"Xynova Arm native backend is not loadable: {exc}"
            ) from exc
        self._library = library
        self._configure_signatures()
        self._validate_abi()
        self._handle = self._library.litchi_xynova_arm_connect(
            model_code, remote_ip.encode(), local_ip.encode()
        )
        if not self._handle:
            raise ConnectionError(f"Xynova Arm connection failed: {self._last_error()}")

    def _configure_signatures(self) -> None:
        library = self._library
        library.litchi_xynova_arm_bridge_abi_version.argtypes = []
        library.litchi_xynova_arm_bridge_abi_version.restype = ctypes.c_uint32
        library.litchi_xynova_arm_state_size.argtypes = []
        library.litchi_xynova_arm_state_size.restype = ctypes.c_uint32
        library.litchi_xynova_arm_info_size.argtypes = []
        library.litchi_xynova_arm_info_size.restype = ctypes.c_uint32
        library.litchi_xynova_arm_connect.argtypes = [
            ctypes.c_int32,
            ctypes.c_char_p,
            ctypes.c_char_p,
        ]
        library.litchi_xynova_arm_connect.restype = ctypes.c_void_p
        for name in ("disconnect", "destroy", "stop"):
            function = getattr(library, f"litchi_xynova_arm_{name}")
            function.argtypes = [ctypes.c_void_p]
            function.restype = ctypes.c_int32
        library.litchi_xynova_arm_get_info.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(_NativeInfo),
        ]
        library.litchi_xynova_arm_get_info.restype = ctypes.c_int32
        library.litchi_xynova_arm_read_state.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(_NativeState),
        ]
        library.litchi_xynova_arm_read_state.restype = ctypes.c_int32
        library.litchi_xynova_arm_read_tcp_pose.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_double),
        ]
        library.litchi_xynova_arm_read_tcp_pose.restype = ctypes.c_int32
        library.litchi_xynova_arm_read_wrench.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_double),
        ]
        library.litchi_xynova_arm_read_wrench.restype = ctypes.c_int32
        library.litchi_xynova_arm_set_power.argtypes = [
            ctypes.c_void_p,
            ctypes.c_int32,
        ]
        library.litchi_xynova_arm_set_power.restype = ctypes.c_int32
        library.litchi_xynova_arm_command_joint_position.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_double),
            ctypes.c_uint32,
            ctypes.c_double,
            ctypes.c_double,
        ]
        library.litchi_xynova_arm_command_joint_position.restype = ctypes.c_int32
        library.litchi_xynova_arm_last_error.argtypes = []
        library.litchi_xynova_arm_last_error.restype = ctypes.c_char_p

    def _validate_abi(self) -> None:
        abi = int(self._library.litchi_xynova_arm_bridge_abi_version())
        if abi != _BRIDGE_ABI_VERSION:
            raise DependencyUnavailableError(
                f"unsupported Xynova Arm bridge ABI {abi}; expected {_BRIDGE_ABI_VERSION}"
            )
        state_size = int(self._library.litchi_xynova_arm_state_size())
        info_size = int(self._library.litchi_xynova_arm_info_size())
        if state_size != ctypes.sizeof(_NativeState) or info_size != ctypes.sizeof(_NativeInfo):
            raise DependencyUnavailableError("Xynova Arm native bridge ABI size mismatch")

    def _last_error(self) -> str:
        value = self._library.litchi_xynova_arm_last_error()
        return value.decode("utf-8", errors="replace") if value else "unknown error"

    def _check(self, result: int, operation: str) -> None:
        if result != 0:
            raise ConnectionError(f"Xynova Arm {operation} failed: {self._last_error()}")

    @property
    def info(self) -> NativeArmInfo:
        value = _NativeInfo()
        self._check(
            self._library.litchi_xynova_arm_get_info(self._handle, ctypes.byref(value)),
            "get_info",
        )
        return NativeArmInfo(
            joint_count=int(value.joint_count),
            id=_decode(value.id),
            mac=_decode(value.mac),
            controller_version=_decode(value.controller_version),
            model=_decode(value.model),
            sdk_version=_decode(value.sdk_version),
        )

    def read_state(self) -> tuple[tuple[float, ...], tuple[float, ...], tuple[float, ...]]:
        value = _NativeState()
        self._check(
            self._library.litchi_xynova_arm_read_state(self._handle, ctypes.byref(value)),
            "read_state",
        )
        count = int(value.joint_count)
        return (
            tuple(value.position[:count]),
            tuple(value.velocity[:count]),
            tuple(value.effort[:count]),
        )

    def read_tcp_pose(self) -> tuple[float, float, float, float, float, float]:
        value = (ctypes.c_double * 6)()
        self._check(
            self._library.litchi_xynova_arm_read_tcp_pose(self._handle, value),
            "read_tcp_pose",
        )
        return tuple(value)  # type: ignore[return-value]

    def read_wrench(self) -> tuple[float, float, float, float, float, float]:
        value = (ctypes.c_double * 6)()
        self._check(
            self._library.litchi_xynova_arm_read_wrench(self._handle, value),
            "read_wrench",
        )
        return tuple(value)  # type: ignore[return-value]

    def set_power(self, enabled: bool) -> None:
        self._check(
            self._library.litchi_xynova_arm_set_power(self._handle, int(enabled)),
            "set_power",
        )

    def command_joint_position(
        self, positions: tuple[float, ...], speed: float, zone: float
    ) -> None:
        values = (ctypes.c_double * len(positions))(*positions)
        self._check(
            self._library.litchi_xynova_arm_command_joint_position(
                self._handle, values, len(positions), speed, zone
            ),
            "command_joint_position",
        )

    def stop(self) -> None:
        self._check(self._library.litchi_xynova_arm_stop(self._handle), "stop")

    def close(self) -> None:
        handle = getattr(self, "_handle", None)
        if not handle:
            return
        try:
            self._check(
                self._library.litchi_xynova_arm_disconnect(handle), "disconnect"
            )
        finally:
            self._library.litchi_xynova_arm_destroy(handle)
            self._handle = None
