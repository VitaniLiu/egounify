"""Bundled MANUS Integrated SDK assets owned by the MANUS backend."""

from __future__ import annotations

import ctypes
from pathlib import Path

from litchi_hardware.core.errors import DependencyUnavailableError

VENDOR_ROOT = Path(__file__).resolve().parent / "_vendor"


def manus_sdk_root() -> Path:
    return VENDOR_ROOT


def manus_include_dir() -> Path:
    return VENDOR_ROOT / "include"


def manus_library_dir() -> Path:
    return VENDOR_ROOT / "lib"


def manus_client_executable() -> Path:
    return VENDOR_ROOT / "bin" / "SharpaManusClient"


def manus_calibration_dir() -> Path:
    return VENDOR_ROOT / "config"


def load_manus_library() -> ctypes.CDLL:
    """Load the bundled MANUS Integrated SDK."""

    library = manus_library_dir() / "libManusSDK_Integrated.so"
    if not library.is_file():
        raise DependencyUnavailableError(
            f"bundled MANUS Integrated SDK library is missing: {library}"
        )
    try:
        return ctypes.CDLL(str(library), mode=ctypes.RTLD_GLOBAL)
    except OSError as exc:
        raise DependencyUnavailableError(
            f"failed to load bundled MANUS Integrated SDK library {library}: {exc}"
        ) from exc
