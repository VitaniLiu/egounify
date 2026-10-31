"""Low-level ctypes FFI layer against the Xynova C SDK.

Only this module talks to ``ctypes``. Higher layers
(:mod:`litchi_hardware.hardware.xynova.manager` and
:mod:`litchi_hardware.hardware.xynova.hand.dexterous_hand`) call in via the
:data:`lib` singleton. The authoritative vendor headers are shipped beside this module under
``_vendor/include/xynova/c``.
"""

from __future__ import annotations

import ctypes as C
import ctypes.util
import os
import platform
import sys
from ctypes import (
    POINTER,
    Structure,
    Union,
    c_bool,
    c_char,
    c_char_p,
    c_double,
    c_float,
    c_int8,
    c_int16,
    c_int32,
    c_int64,
    c_size_t,
    c_uint8,
    c_uint16,
    c_uint32,
    c_uint64,
    c_void_p,
)
from pathlib import Path
from typing import Final

from litchi_hardware.hardware.xynova.errors import XynovaSDKNotLoadedError

# ---------------------------------------------------------------------------
# Runtime constants (verified against real header on SDK arrival)
# ---------------------------------------------------------------------------

JOINT_NUM: Final[int] = int(os.environ.get("XYNOVA_JOINT_NUM", "23"))
TACTILE_NUM: Final[int] = int(os.environ.get("XYNOVA_TACTILE_NUM", "15"))
TACTILE_TOTAL_TAXEL_NUM: Final[int] = int(
    os.environ.get("XYNOVA_TACTILE_TOTAL_TAXEL_NUM", "312")
)
RANGE_POINT_NUM: Final[int] = int(os.environ.get("XYNOVA_RANGE_POINT_NUM", "16"))

# Compatibility name retained for callers of the 1.x wrapper. In SDK 2.x the
# feedback is a six-component wrench rather than a three-component force.
TACTILE_FORCE_NUM: Final[int] = TACTILE_NUM

# ---------------------------------------------------------------------------
# Enums as plain integers (matches C ABI). See _vendor/include/xynova/c/dexterous_hand_types.h
# ---------------------------------------------------------------------------

CAN_FRAME_TYPE_STANDARD: Final[int] = 0
CAN_FRAME_TYPE_EXTENDED: Final[int] = 1

COMM_TYPE_CAN: Final[int] = 0

CONTROL_MODE_DISABLE: Final[int] = 0x00
CONTROL_MODE_ANGLE: Final[int] = 0x02

DATA_TYPE_NONE: Final[int] = 0
DATA_TYPE_BOOL: Final[int] = 1
DATA_TYPE_U8: Final[int] = 2
DATA_TYPE_S8: Final[int] = 3
DATA_TYPE_U16: Final[int] = 4
DATA_TYPE_S16: Final[int] = 5
DATA_TYPE_U32: Final[int] = 6
DATA_TYPE_S32: Final[int] = 7
DATA_TYPE_U64: Final[int] = 8
DATA_TYPE_S64: Final[int] = 9
DATA_TYPE_FLOAT: Final[int] = 10
DATA_TYPE_DOUBLE: Final[int] = 11

# ---------------------------------------------------------------------------
# Struct definitions — see headers/xynova_c_api.h
# ---------------------------------------------------------------------------


class CanConfigC(Structure):
    _fields_ = [
        ("iface", c_char_p),
        ("manager_can", c_bool),
        ("bitrate", c_uint32),
        ("dbitrate", c_uint32),
        ("sample_point", c_float),
        ("dsample_point", c_float),
    ]


class _CommUnion(Union):
    _fields_ = [("can", CanConfigC)]


class CommConfigC(Structure):
    """Exact layout of the vendor's ``XynovaCommunicationCommConfig``."""

    _anonymous_ = ("config",)

    _fields_ = [
        ("type", c_int32),
        ("sub_type", c_char_p),
        ("config", _CommUnion),
        ("logger_name", c_char_p),
    ]


class MasterConfigC(Structure):
    _fields_ = [
        ("name", c_char_p),
        ("comm", CommConfigC),
        ("period_us", c_uint64),
        ("cpu_index", c_int32),
    ]


class LogLoggerBackendConfigC(Structure):
    """Exact layout of ``XynovaLogLoggerBackendConfiguration`` (log_types.h)."""

    _fields_ = [
        ("pool_size", c_uint32),
        ("block_size", c_uint32),
        ("message_size", c_uint32),
        ("max_continuous_repeated_log", c_uint32),
        ("flush_interval_us", c_uint32),
        ("queue_size_multiple", c_float),
    ]


class LogLoggerConfigC(Structure):
    """Exact layout of ``XynovaLogLoggerConfiguration`` (log_types.h)."""

    _fields_ = [
        ("sink_name", c_char_p),
        ("writer_name_list", POINTER(c_char_p)),  # NULL-terminated list
        ("lb_type", c_char_p),
        ("level", c_int32),
        ("lb_config", LogLoggerBackendConfigC),
    ]


class ResponseTimeSettingC(Structure):
    _fields_ = [
        ("user_timeout_us", c_uint32),
        ("user_poll_interval_us", c_uint32),
        ("execute_timeout_us", c_uint32),
        ("execute_max_retry", c_uint32),
        ("execute_poll_interval_us", c_uint32),
    ]


class SlaveConfigC(Structure):
    _fields_ = [
        ("type", c_char_p),
        ("id", c_uint32),
        ("general_time_setting", ResponseTimeSettingC),
        ("fast_time_setting", ResponseTimeSettingC),
        ("logger_name", c_char_p),
    ]


class SchedulingCommandC(Structure):
    _fields_ = [
        ("command", c_char_p),
        ("response_time_setting", ResponseTimeSettingC),
    ]


class DeviceConfigC(Structure):
    _fields_ = [
        ("type", c_char_p),
        ("name", c_char_p),
        ("master_name", c_char_p),
        ("slaves", POINTER(SlaveConfigC)),
        ("slave_count", c_size_t),
        ("commands", POINTER(SchedulingCommandC)),
        ("command_count", c_size_t),
    ]


class ManagerConfigC(Structure):
    _fields_ = [
        ("masters", POINTER(MasterConfigC)),
        ("master_count", c_size_t),
        ("devices", POINTER(DeviceConfigC)),
        ("device_count", c_size_t),
    ]


class RecvFilterC(Structure):
    _fields_ = [
        ("type", c_int32),
        ("id", c_uint32),
        ("mask", c_uint32),
    ]


# --- SDK 2.x generic typed values -------------------------------------------


class XynovaValueC(Union):
    _fields_ = [
        ("boolean", c_bool),
        ("u8", c_uint8),
        ("s8", c_int8),
        ("u16", c_uint16),
        ("s16", c_int16),
        ("u32", c_uint32),
        ("s32", c_int32),
        ("u64", c_uint64),
        ("s64", c_int64),
        ("f32", c_float),
        ("f64", c_double),
    ]


class XynovaTypedValueC(Structure):
    _fields_ = [("type", c_uint8), ("value", XynovaValueC)]


class XynovaTypedValueArray6C(Structure):
    _fields_ = [("type", c_uint8), ("values", XynovaValueC * 6)]


class XynovaIdValueC(Structure):
    _fields_ = [("id", c_uint16), ("data", XynovaTypedValueC)]


class JointAngleControlC(Structure):
    _fields_ = [("values", XynovaIdValueC * JOINT_NUM)]


class JointAngleStatesC(Structure):
    _fields_ = [("values", XynovaIdValueC * JOINT_NUM)]


class DexterousHandErrorStateC(Structure):
    _fields_ = [
        ("timestamp_us", c_uint64),
        ("warning_flags", c_uint32),
        ("fault_flags", c_uint32),
        ("joint_error_code", XynovaIdValueC * JOINT_NUM),
    ]


class TactileWrenchC(Structure):
    _fields_ = [
        ("timestamp_us", c_uint64),
        ("region", c_uint8),
        ("dimension", c_uint8),
        ("wrench", XynovaTypedValueArray6C),
    ]


class TactileWrenchStatesC(Structure):
    _fields_ = [("values", TactileWrenchC * TACTILE_NUM)]


class TactileTaxelC(Structure):
    _fields_ = [
        ("timestamp_us", c_uint64),
        ("region", c_uint8),
        ("taxel_id", c_uint16),
        ("dimension", c_uint8),
        ("reserved", c_uint8),
        ("wrench", XynovaTypedValueArray6C),
    ]


class TactileTaxelStatesC(Structure):
    _fields_ = [("count", c_uint32), ("taxels", POINTER(TactileTaxelC))]


class RangePointC(Structure):
    _fields_ = [
        ("timestamp_us", c_uint64),
        ("id", c_uint16),
        ("status", c_uint8),
        ("reserved", c_uint8),
        ("distance", c_float),
    ]


class RangeStatesC(Structure):
    _fields_ = [
        ("width", c_uint16),
        ("height", c_uint16),
        ("count", c_uint32),
        ("points", POINTER(RangePointC)),
    ]


# ---------------------------------------------------------------------------
# Shared library loader
# ---------------------------------------------------------------------------

_LIB_CANDIDATES = [
    "libxynova.so.2.0.0",
    "libxynova.so.2",
    "libxynova.so",
    "libxynova.dylib",
]
_DEFAULT_SEARCH_DIRS = [
    "/opt/xynova/lib",
    "/usr/local/lib/xynova",
    "/usr/local/lib",
    "/usr/lib",
]


def _platform_tag() -> str | None:
    if not sys.platform.startswith("linux"):
        return None
    machine = platform.machine().lower()
    machine = {"amd64": "x86_64", "arm64": "aarch64"}.get(machine, machine)
    return f"linux-{machine}"


def bundled_library_path() -> Path | None:
    """Return the bundled SDK runtime for this platform, when available."""
    if os.environ.get("XYNOVA_DISABLE_BUNDLED", "").lower() in {"1", "true", "yes"}:
        return None
    tag = _platform_tag()
    if tag is None:
        return None
    lib_dir = Path(__file__).resolve().parent / "_vendor" / "lib" / tag
    for name in _LIB_CANDIDATES:
        candidate = lib_dir / name
        if candidate.is_file():
            return candidate
    return None


def _locate_library() -> Path | str | None:
    override = os.environ.get("XYNOVA_LIB", "").strip()
    if override:
        p = Path(override)
        if p.is_file():
            return p
        return None

    search_dirs: list[str] = []
    explicit_dir = os.environ.get("XYNOVA_LIB_DIR", "").strip()
    if explicit_dir:
        search_dirs.append(explicit_dir)
    search_dirs.extend(filter(None, os.environ.get("LD_LIBRARY_PATH", "").split(os.pathsep)))

    bundled = bundled_library_path()
    if bundled is not None:
        return bundled

    search_dirs.extend(_DEFAULT_SEARCH_DIRS)
    for d in search_dirs:
        for name in _LIB_CANDIDATES:
            p = Path(d) / name
            if p.is_file():
                return p

    system_name = ctypes.util.find_library("xynova")
    if system_name:
        return system_name
    return None


class _StubLib:
    """Stand-in raised at first call when the SDK is not installed.

    Deliberately not iterable / attribute-error-y: every attribute access
    raises the friendly ``XynovaSDKNotLoadedError``. This preserves the ability
    to import :mod:`litchi_hardware.hardware.xynova` in dry-run environments
    such as CI and ROS 2 launch validation.
    """

    def __init__(self, reason: str) -> None:
        self._reason = reason

    def __getattr__(self, name: str):  # noqa: ANN204
        raise XynovaSDKNotLoadedError(f"symbol '{name}' unavailable: {self._reason}")


def _load_library() -> tuple[object, str | None]:
    path = _locate_library()
    if path is None:
        override = os.environ.get("XYNOVA_LIB", "").strip()
        if override:
            reason = f"XYNOVA_LIB does not point to a file: {override}"
        else:
            tag = _platform_tag() or sys.platform
            reason = (
                f"no Xynova SDK runtime found for {tag}; set XYNOVA_LIB to the "
                "matching vendor library"
            )
        return _StubLib(reason), None
    try:
        cdll = C.CDLL(str(path), mode=C.RTLD_GLOBAL)
    except OSError as e:
        return _StubLib(str(e)), None
    return cdll, str(path)


lib, lib_path = _load_library()


def is_loaded() -> bool:
    """True when the real vendor shared library was resolved."""
    return not isinstance(lib, _StubLib)


# ---------------------------------------------------------------------------
# Prototype declarations
# ---------------------------------------------------------------------------

# Ordered as (name, restype, argtypes). Applied lazily so a stubbed lib does
# not crash at import time — we only attempt binding when it's a real CDLL.

_PROTOS: list[tuple[str, object, list[object]]] = [
    # ---- version / logging
    ("xynovaGetSdkVersion", c_char_p, []),
    ("xynovaGetSdkLocalVersion", c_char_p, []),
    ("xynovaGetSdkGitBranch", c_char_p, []),
    ("xynovaGetSdkGitCommit", c_char_p, []),
    ("xynovaGetSdkBuildTime", c_char_p, []),
    ("XynovaLogGlobalLogger_init", None, [c_int32, c_void_p]),
    ("XynovaLogGlobalLogger_setLogLevel", c_int32, [c_int32]),
    ("XynovaLogGlobalLogger_getLogLevel", c_int32, []),
    ("XynovaLogObjectLogger_initDefaultObjects", c_int32, []),
    ("XynovaLogObjectLogger_isInitialized", c_bool, []),
    ("XynovaLogObjectLogger_createLogger", c_int32, [c_char_p, POINTER(LogLoggerConfigC)]),
    ("XynovaLogObjectLogger_hasLogger", c_bool, [c_char_p]),
    # ---- CAN driver registration
    (
        "XynovaCommunicationCan_registerDriver",
        c_int32,
        [c_char_p, c_void_p],  # VTable pointer — struct-of-callbacks left opaque
    ),
    # ---- communication manager
    ("XynovaCommunicationManager_init", c_int32, [POINTER(ManagerConfigC)]),
    ("XynovaCommunicationManager_run", c_int32, [c_char_p]),
    ("XynovaCommunicationManager_stop", c_int32, [c_char_p]),
    (
        "XynovaCommunicationManager_getDevice",
        c_void_p,
        [c_char_p, c_char_p, c_char_p],
    ),
    ("XynovaCommunicationManager_freeDevice", None, [c_void_p]),
    (
        "XynovaCommunicationManager_upgradeFirmware",
        c_int32,
        [c_char_p, c_uint32, c_char_p],
    ),
    # ---- SDK 2.x dexterous hand
    ("XynovaDexterousHand_run", c_int32, [c_void_p]),
    ("XynovaDexterousHand_stop", c_int32, [c_void_p]),
    ("XynovaDexterousHand_getProductModel", c_int32, [c_void_p, POINTER(c_char), c_uint32]),
    ("XynovaDexterousHand_getProductSN", c_int32, [c_void_p, POINTER(c_char), c_uint32]),
    ("XynovaDexterousHand_getHardwareVersion", c_int32, [c_void_p, POINTER(c_char), c_uint32]),
    ("XynovaDexterousHand_getSoftwareVersion", c_int32, [c_void_p, POINTER(c_char), c_uint32]),
    ("XynovaDexterousHand_getHandType", c_int32, [c_void_p, POINTER(c_uint8)]),
    (
        "XynovaDexterousHand_getErrorState",
        c_int32,
        [c_void_p, POINTER(DexterousHandErrorStateC)],
    ),
    ("XynovaDexterousHand_setJointControlAllMode", c_int32, [c_void_p, c_uint8]),
    ("XynovaDexterousHand_enableTactileWrench", c_int32, [c_void_p, c_bool]),
    ("XynovaDexterousHand_enableTactileTaxel", c_int32, [c_void_p, c_bool]),
    ("XynovaDexterousHand_enableRange", c_int32, [c_void_p, c_bool]),
    (
        "XynovaDexterousHand_setJointAngle",
        c_int32,
        [c_void_p, POINTER(JointAngleControlC)],
    ),
    (
        "XynovaDexterousHand_setJointAngle_async",
        c_int32,
        [c_void_p, POINTER(JointAngleControlC)],
    ),
    (
        "XynovaDexterousHand_getJointAngle",
        c_int32,
        [c_void_p, POINTER(JointAngleStatesC), POINTER(c_uint64)],
    ),
    (
        "XynovaDexterousHand_getTactileWrench",
        c_int32,
        [c_void_p, POINTER(TactileWrenchStatesC), POINTER(c_uint64)],
    ),
    (
        "XynovaDexterousHand_getTactileTaxel",
        c_int32,
        [c_void_p, POINTER(TactileTaxelStatesC), POINTER(c_uint64)],
    ),
    (
        "XynovaDexterousHand_getRange",
        c_int32,
        [c_void_p, POINTER(RangeStatesC), POINTER(c_uint64)],
    ),
]


def _bind_prototypes() -> None:
    if not is_loaded():
        return
    for name, restype, argtypes in _PROTOS:
        try:
            fn = getattr(lib, name)
        except AttributeError:
            # Symbol truly missing — leave a lazy stub so callers see a good
            # error on first use rather than at import time.
            continue
        fn.restype = restype
        fn.argtypes = argtypes


_bind_prototypes()


def declared_symbols() -> list[str]:
    """List of every C symbol we bind. Used by test_ffi_symbols."""
    return [name for name, _r, _a in _PROTOS]


def sdk_build_info() -> dict[str, str]:
    """Return build metadata reported by the loaded vendor runtime."""
    fields = {
        "version": "xynovaGetSdkVersion",
        "local_version": "xynovaGetSdkLocalVersion",
        "git_branch": "xynovaGetSdkGitBranch",
        "git_commit": "xynovaGetSdkGitCommit",
        "build_time": "xynovaGetSdkBuildTime",
    }
    result: dict[str, str] = {}
    for key, symbol in fields.items():
        raw = getattr(lib, symbol)()
        result[key] = raw.decode("utf-8", errors="replace") if raw else ""
    return result
