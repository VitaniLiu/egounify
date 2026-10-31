"""ROS-independent hardware abstraction for Litchi robots."""

from . import linker, manus, sharpa, xynova
from .core.config import DeviceConfig, DeviceInfo
from .core.interfaces import (
    ArmInterface,
    DeviceInterface,
    HandInterface,
    HandTrackerInterface,
    TactileCapable,
)
from .core.types import (
    DeviceAction,
    DeviceObservation,
    FeatureSpec,
    HandSide,
    HandSkeletonSchema,
    HandTrackingFrame,
    JointSchema,
    JointState,
    Pose,
    TactileFrame,
)

__all__ = [
    "ArmInterface",
    "DeviceAction",
    "DeviceConfig",
    "DeviceInfo",
    "DeviceInterface",
    "DeviceObservation",
    "FeatureSpec",
    "HandSkeletonSchema",
    "HandInterface",
    "HandSide",
    "HandTrackerInterface",
    "HandTrackingFrame",
    "JointSchema",
    "JointState",
    "linker",
    "manus",
    "Pose",
    "sharpa",
    "TactileCapable",
    "TactileFrame",
    "xynova",
]
