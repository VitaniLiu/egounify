"""Compatibility import for the Xynova SDK 1.x module name."""

from litchi_hardware.hardware.xynova._ffi import (  # noqa: F401
    TACTILE_FORCE_NUM,
    TACTILE_NUM,
)

from .dexterous_hand import *  # noqa: F403
