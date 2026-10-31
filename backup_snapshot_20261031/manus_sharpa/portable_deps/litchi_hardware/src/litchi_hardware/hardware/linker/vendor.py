"""Paths and lazy loaders for bundled Linker Hand vendor SDKs."""

from __future__ import annotations

import importlib
import os
import platform
from pathlib import Path
from types import ModuleType
from typing import Any

_ROOT = Path(__file__).resolve().parent / "_vendor"


def sdk_root() -> Path:
    return _ROOT


def legacy_sdk_root() -> Path:
    return _ROOT / "legacy"


def o30i_sdk_root() -> Path:
    return _ROOT / "o30i"


def protocol_manual() -> Path:
    return o30i_sdk_root() / "HandProtocol_v1.0.pdf"


def udev_rules_dir() -> Path:
    return o30i_sdk_root() / "udev"


def bundled_libcanbus_path() -> Path | None:
    override = os.environ.get("LINKER_HAND_LIBCANBUS")
    if override:
        return Path(override).expanduser().resolve()
    machine = platform.machine().lower()
    platform_name = {
        "aarch64": "linux-aarch64",
        "arm64": "linux-aarch64",
        "armv7l": "linux-armv7l",
        "armv6l": "linux-armv7l",
        "x86_64": "linux-x86_64-ubuntu22",
        "amd64": "linux-x86_64-ubuntu22",
    }.get(machine)
    if platform_name is None:
        return None
    candidate = o30i_sdk_root() / "lib" / platform_name / "libcanbus.so"
    return candidate if candidate.is_file() else None


def load_o30i_sdk() -> ModuleType:
    return importlib.import_module(
        "litchi_hardware.hardware.linker._vendor.o30i.linker_hand_o30i_control"
    )


def load_g20_controller_class() -> type[Any]:
    module = importlib.import_module(
        "litchi_hardware.hardware.linker._vendor.legacy.core.can.linker_hand_g20_can"
    )
    return module.LinkerHandG20Can
