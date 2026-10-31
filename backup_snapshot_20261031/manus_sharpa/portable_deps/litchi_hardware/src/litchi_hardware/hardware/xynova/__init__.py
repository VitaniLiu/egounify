"""Python bindings and HAL adapter for Xynova SDK 2.x Dexterous Hand.

Public re-exports so callers can write ``from litchi_hardware.hardware.xynova import ...``.
"""

from litchi_hardware.hardware.xynova._ffi import TACTILE_FORCE_NUM, TACTILE_NUM, sdk_build_info
from litchi_hardware.hardware.xynova._ffi import is_loaded as is_sdk_loaded
from litchi_hardware.hardware.xynova.arm import (
    COLLABORATIVE_MODELS,
    MODEL_JOINT_COUNTS,
    XYNOVA_ARM_JOINT_SCHEMAS,
    MockXynovaArm,
    NativeArm,
    NativeArmInfo,
    XynovaArmBase,
    XynovaArmConfig,
    XynovaArmModel,
    XynovaSdkArm,
    arm_bridge_library,
    arm_include_dir,
    arm_joint_schema,
    arm_library_dir,
    arm_sdk_library,
    create_arm,
)
from litchi_hardware.hardware.xynova.config import XynovaConfig
from litchi_hardware.hardware.xynova.errors import XynovaError, XynovaSDKNotLoadedError, check
from litchi_hardware.hardware.xynova.factory import create_hand
from litchi_hardware.hardware.xynova.hand.dexterous_hand import (
    JOINT_NAMES,
    TACTILE_NAMES,
    ControlMode,
    Device,
    HandErrorState,
    HandType,
    JointID,
    RangePoint,
    RangePointStatus,
    TactileID,
    TactileTaxel,
    TactileWrench,
    WrenchDimension,
)
from litchi_hardware.hardware.xynova.infra.log import LogLevel, LogManager
from litchi_hardware.hardware.xynova.manager import (
    CanConfig,
    CommConfig,
    CommType,
    DeviceConfig,
    Manager,
    MasterConfig,
    ResponseTimeSetting,
    SlaveConfig,
)
from litchi_hardware.hardware.xynova.mock import MockXynovaHand
from litchi_hardware.hardware.xynova.schema import (
    XYNOVA_DEXTEROUS_HAND_JOINT_SCHEMA,
    XYNOVA_JOINT_NAMES,
    XYNOVA_XZ_JOINT_SCHEMA,
)
from litchi_hardware.hardware.xynova.sdk_driver import XynovaSdkHand

__all__ = [
    "CanConfig",
    "COLLABORATIVE_MODELS",
    "CommConfig",
    "CommType",
    "ControlMode",
    "Device",
    "DeviceConfig",
    "HandErrorState",
    "HandType",
    "JOINT_NAMES",
    "JointID",
    "LogLevel",
    "LogManager",
    "Manager",
    "MasterConfig",
    "MODEL_JOINT_COUNTS",
    "MockXynovaArm",
    "NativeArm",
    "NativeArmInfo",
    "ResponseTimeSetting",
    "RangePoint",
    "RangePointStatus",
    "SlaveConfig",
    "TACTILE_FORCE_NUM",
    "TACTILE_NUM",
    "TACTILE_NAMES",
    "TactileID",
    "TactileTaxel",
    "TactileWrench",
    "MockXynovaHand",
    "XYNOVA_ARM_JOINT_SCHEMAS",
    "XYNOVA_JOINT_NAMES",
    "XYNOVA_DEXTEROUS_HAND_JOINT_SCHEMA",
    "XYNOVA_XZ_JOINT_SCHEMA",
    "XynovaConfig",
    "XynovaArmBase",
    "XynovaArmConfig",
    "XynovaArmModel",
    "XynovaError",
    "XynovaSdkHand",
    "XynovaSdkArm",
    "XynovaSDKNotLoadedError",
    "WrenchDimension",
    "arm_bridge_library",
    "arm_include_dir",
    "arm_joint_schema",
    "arm_library_dir",
    "arm_sdk_library",
    "check",
    "create_hand",
    "create_arm",
    "is_sdk_loaded",
    "sdk_build_info",
]

__version__ = "0.4.0"
