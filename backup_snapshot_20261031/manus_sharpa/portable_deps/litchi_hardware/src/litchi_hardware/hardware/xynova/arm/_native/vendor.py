"""Locations and loading for the bundled Xynova Arm SDK and native bridge."""

from __future__ import annotations

import ctypes
import os
import platform
from pathlib import Path

from litchi_hardware.core.errors import DependencyUnavailableError

ARM_ROOT = Path(__file__).resolve().parents[1]
VENDOR_ROOT = ARM_ROOT / "_vendor"
NATIVE_ROOT = Path(__file__).resolve().parent

_PLATFORM_DIRS = {
    "x86_64": "x86_64",
    "amd64": "x86_64",
    "aarch64": "aarch64",
    "arm64": "aarch64",
}


def arm_include_dir() -> Path:
    return VENDOR_ROOT / "include"


def arm_library_dir() -> Path:
    machine = platform.machine().lower()
    try:
        architecture = _PLATFORM_DIRS[machine]
    except KeyError as exc:
        raise DependencyUnavailableError(
            f"Xynova Arm SDK does not support architecture {machine!r}"
        ) from exc
    return VENDOR_ROOT / "lib" / "Linux" / architecture


def arm_sdk_library() -> Path:
    return arm_library_dir() / "libXynovaSDK.so.0"


def arm_bridge_library() -> Path:
    override = os.environ.get("XYNOVA_ARM_BRIDGE_LIB")
    if override:
        return Path(override).expanduser().resolve()
    return NATIVE_ROOT / "lib" / "liblitchi_xynova_arm_bridge.so"


def load_arm_bridge() -> ctypes.CDLL:
    sdk = arm_sdk_library()
    bridge = arm_bridge_library()
    for name, path in (("SDK", sdk), ("native bridge", bridge)):
        if not path.is_file():
            raise DependencyUnavailableError(f"Xynova Arm {name} is missing: {path}")
    try:
        return ctypes.CDLL(str(bridge))
    except OSError as exc:
        raise DependencyUnavailableError(
            f"failed to load Xynova Arm SDK/bridge: {exc}"
        ) from exc
