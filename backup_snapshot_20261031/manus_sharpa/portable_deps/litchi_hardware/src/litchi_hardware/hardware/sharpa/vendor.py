"""Bundled native SDK assets owned by the Sharpa backend."""

from __future__ import annotations

import ctypes
import importlib
import sys
from pathlib import Path
from types import ModuleType

from litchi_hardware.core.errors import DependencyUnavailableError

VENDOR_ROOT = Path(__file__).resolve().parent / "_vendor"


def sharpa_sdk_root() -> Path:
    return VENDOR_ROOT


def sharpa_include_dir() -> Path:
    return VENDOR_ROOT / "include"


def sharpa_library_dir() -> Path:
    return VENDOR_ROOT / "lib"


def load_sharpa_sdk() -> ModuleType:
    """Load and return the bundled native ``sharpa`` Python module."""

    module_root = VENDOR_ROOT / "python"
    library = sharpa_library_dir() / "libsharpa-wave-sdk.so"
    if not library.is_file():
        raise DependencyUnavailableError(
            "bundled Sharpa SDK is missing; rebuild the private platform wheel"
        )
    try:
        ctypes.CDLL(str(library), mode=ctypes.RTLD_GLOBAL)
        module_path = str(module_root)
        if module_path not in sys.path:
            sys.path.insert(0, module_path)
        return importlib.import_module("sharpa")
    except (ImportError, OSError) as exc:
        raise DependencyUnavailableError(
            f"failed to load bundled Sharpa SDK from {VENDOR_ROOT}: {exc}"
        ) from exc
