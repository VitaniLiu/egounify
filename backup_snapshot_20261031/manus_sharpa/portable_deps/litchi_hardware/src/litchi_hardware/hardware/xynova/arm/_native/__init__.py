"""Private ctypes boundary for the project-owned Xynova Arm C ABI bridge."""

from .ffi import NativeArm, NativeArmInfo
from .vendor import (
    arm_bridge_library,
    arm_include_dir,
    arm_library_dir,
    arm_sdk_library,
    load_arm_bridge,
)

__all__ = [
    "NativeArm",
    "NativeArmInfo",
    "arm_bridge_library",
    "arm_include_dir",
    "arm_library_dir",
    "arm_sdk_library",
    "load_arm_bridge",
]
