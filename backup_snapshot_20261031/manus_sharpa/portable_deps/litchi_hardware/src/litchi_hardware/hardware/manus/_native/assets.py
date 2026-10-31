"""Locations and loading for the project-owned Manus bridge."""

from __future__ import annotations

import ctypes
from pathlib import Path

from litchi_hardware.core.errors import DependencyUnavailableError
from litchi_hardware.hardware.manus.vendor import load_manus_library

NATIVE_ROOT = Path(__file__).resolve().parent


def native_bridge_library() -> Path:
    return NATIVE_ROOT / "lib" / "liblitchi_manus_bridge.so"


def native_bridge_include_dir() -> Path:
    return NATIVE_ROOT


def load_native_bridge() -> ctypes.CDLL:
    """Load the project bridge after its bundled Manus SDK dependency."""

    bridge = native_bridge_library()
    if not bridge.is_file():
        raise DependencyUnavailableError(
            f"Manus native bridge is missing: {bridge}; "
            "rebuild the private platform wheel"
        )
    load_manus_library()
    try:
        return ctypes.CDLL(str(bridge))
    except OSError as exc:
        raise DependencyUnavailableError(
            f"failed to load Manus native bridge {bridge}: {exc}"
        ) from exc
