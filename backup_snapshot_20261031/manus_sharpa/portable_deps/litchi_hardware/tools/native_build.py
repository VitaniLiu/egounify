"""Helpers for host-native builds that must not inherit Pixi/Conda paths."""

from __future__ import annotations

import os
import shutil
from pathlib import Path

_BUILD_ENVIRONMENT_VARIABLES = (
    "CC",
    "CXX",
    "CPPFLAGS",
    "CFLAGS",
    "CXXFLAGS",
    "LDFLAGS",
    "LIBRARY_PATH",
    "LD_LIBRARY_PATH",
    "CPATH",
    "C_INCLUDE_PATH",
    "CPLUS_INCLUDE_PATH",
    "GCC_EXEC_PREFIX",
    "PKG_CONFIG_PATH",
    "PKG_CONFIG_LIBDIR",
    "PKG_CONFIG_SYSROOT_DIR",
    "CMAKE_PREFIX_PATH",
    "CONDA_PREFIX",
)


def resolve_host_tool(name: str, override_name: str) -> Path:
    """Resolve a host tool without searching the active environment prefix."""

    requested = os.environ.get(override_name)
    if requested:
        discovered = shutil.which(requested)
        if discovered is None:
            candidate = Path(requested).expanduser().resolve()
            if not candidate.is_file():
                raise FileNotFoundError(candidate)
            return candidate
        return Path(discovered).resolve()

    discovered = shutil.which(name, path=os.defpath)
    if discovered is None:
        raise FileNotFoundError(
            f"host {name} is required; set {override_name} to override it"
        )
    return Path(discovered).resolve()


def clean_host_build_environment() -> dict[str, str]:
    """Return an environment without compiler/linker prefix injection."""

    environment = os.environ.copy()
    for name in _BUILD_ENVIRONMENT_VARIABLES:
        environment.pop(name, None)
    environment["PATH"] = os.defpath
    return environment
