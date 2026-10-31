"""ctypes binding for the project-owned Manus bridge ABI."""

from __future__ import annotations

import ctypes
import math
from pathlib import Path

from litchi_hardware.core.errors import (
    ConnectionError,
    DependencyUnavailableError,
)
from litchi_hardware.core.types import HandSide, Pose

from .assets import load_native_bridge
from .models import NativeSkeletonFrame

_BRIDGE_ABI_VERSION = 1
_KEYPOINT_COUNT = 25
_SIDE_CODES = {HandSide.LEFT: 1, HandSide.RIGHT: 2}
_CODE_SIDES = {code: side for side, code in _SIDE_CODES.items()}


class _NativePose(ctypes.Structure):
    _fields_ = [
        ("position_x", ctypes.c_float),
        ("position_y", ctypes.c_float),
        ("position_z", ctypes.c_float),
        ("orientation_x", ctypes.c_float),
        ("orientation_y", ctypes.c_float),
        ("orientation_z", ctypes.c_float),
        ("orientation_w", ctypes.c_float),
    ]


class _NativeFrame(ctypes.Structure):
    _fields_ = [
        ("side", ctypes.c_int32),
        ("keypoint_count", ctypes.c_uint32),
        ("timestamp_ns", ctypes.c_uint64),
        ("sequence", ctypes.c_uint64),
        ("keypoints", _NativePose * _KEYPOINT_COUNT),
    ]


class CtypesManusBridge:
    """Type-safe Python adapter over the versioned bridge C ABI."""

    def __init__(self, library_path: str | None = None) -> None:
        try:
            library = (
                load_native_bridge()
                if library_path is None
                else ctypes.CDLL(str(Path(library_path).expanduser().resolve()))
            )
        except (OSError, DependencyUnavailableError) as exc:
            raise DependencyUnavailableError(
                f"Manus native backend is not loadable: {exc}"
            ) from exc
        self._library = library
        self._configure_signatures()
        abi_version = int(self._library.litchi_manus_bridge_abi_version())
        if abi_version != _BRIDGE_ABI_VERSION:
            raise DependencyUnavailableError(
                "unsupported Manus native bridge ABI "
                f"{abi_version}; expected {_BRIDGE_ABI_VERSION}"
            )
        frame_size = int(self._library.litchi_manus_frame_size())
        if frame_size != ctypes.sizeof(_NativeFrame):
            raise DependencyUnavailableError(
                "Manus native bridge frame ABI mismatch: "
                f"C++={frame_size}, Python={ctypes.sizeof(_NativeFrame)}"
            )

    def _configure_signatures(self) -> None:
        library = self._library
        library.litchi_manus_bridge_abi_version.argtypes = []
        library.litchi_manus_bridge_abi_version.restype = ctypes.c_uint32
        library.litchi_manus_frame_size.argtypes = []
        library.litchi_manus_frame_size.restype = ctypes.c_uint32
        library.litchi_manus_connect.argtypes = [
            ctypes.c_uint32,
            ctypes.c_int32,
            ctypes.c_char_p,
        ]
        library.litchi_manus_connect.restype = ctypes.c_int32
        library.litchi_manus_disconnect.argtypes = []
        library.litchi_manus_disconnect.restype = ctypes.c_int32
        library.litchi_manus_read_frame.argtypes = [
            ctypes.c_int32,
            ctypes.c_uint32,
            ctypes.POINTER(_NativeFrame),
        ]
        library.litchi_manus_read_frame.restype = ctypes.c_int32
        library.litchi_manus_available_sides.argtypes = []
        library.litchi_manus_available_sides.restype = ctypes.c_uint32
        library.litchi_manus_last_error.argtypes = []
        library.litchi_manus_last_error.restype = ctypes.c_char_p

    def _last_error(self) -> str:
        message = self._library.litchi_manus_last_error()
        return message.decode("utf-8", errors="replace") if message else "unknown error"

    def connect(
        self,
        discovery_wait_seconds: int,
        loopback_only: bool,
        calibration_directory: str,
    ) -> None:
        result = self._library.litchi_manus_connect(
            discovery_wait_seconds,
            int(loopback_only),
            calibration_directory.encode(),
        )
        if result != 0:
            raise ConnectionError(
                f"Manus native SDK connection failed: {self._last_error()}"
            )

    def disconnect(self) -> None:
        if self._library.litchi_manus_disconnect() != 0:
            raise ConnectionError(
                f"Manus native SDK shutdown failed: {self._last_error()}"
            )

    def read_frame(
        self, side: HandSide, timeout_s: float
    ) -> NativeSkeletonFrame | None:
        native = _NativeFrame()
        timeout_ms = min(math.ceil(timeout_s * 1000.0), 0xFFFFFFFF)
        result = self._library.litchi_manus_read_frame(
            _SIDE_CODES[side], timeout_ms, ctypes.byref(native)
        )
        if result < 0:
            raise ConnectionError(
                f"Manus native SDK frame read failed: {self._last_error()}"
            )
        if result == 0:
            return None
        returned_side = _CODE_SIDES.get(native.side)
        if returned_side is None:
            raise ConnectionError(
                f"Manus native bridge returned unexpected side code {native.side}"
            )
        if returned_side is not side:
            raise ConnectionError(
                "Manus native bridge returned "
                f"{returned_side.value} for a {side.value} read"
            )
        if native.keypoint_count != _KEYPOINT_COUNT:
            raise ConnectionError(
                "Manus native bridge returned "
                f"{native.keypoint_count} keypoints; expected {_KEYPOINT_COUNT}"
            )
        keypoints = tuple(
            Pose(
                (pose.position_x, pose.position_y, pose.position_z),
                (
                    pose.orientation_x,
                    pose.orientation_y,
                    pose.orientation_z,
                    pose.orientation_w,
                ),
            )
            for pose in native.keypoints
        )
        return NativeSkeletonFrame(
            side=returned_side,
            keypoints=keypoints,
            timestamp_s=native.timestamp_ns / 1_000_000_000.0,
            sequence=native.sequence,
        )

    @property
    def available_sides(self) -> frozenset[HandSide]:
        mask = int(self._library.litchi_manus_available_sides())
        return frozenset(
            side
            for side, code in _SIDE_CODES.items()
            if mask & (1 << (code - 1))
        )
