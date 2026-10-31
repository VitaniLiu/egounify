"""Pure domain contracts. This package must not import ROS or vendor SDKs."""

from .config import DeviceConfig, DeviceInfo
from .errors import (
    CommandRejectedError,
    ConnectionError,
    DependencyUnavailableError,
    DeviceError,
    InvalidStateError,
    ProtocolError,
)
from .interfaces import (
    ArmInterface,
    DeviceInterface,
    HandInterface,
    HandTrackerInterface,
    TactileCapable,
)
from .lifecycle import ConnectionState, LifecycleGuard
from .types import (
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
    "CommandRejectedError",
    "ConnectionError",
    "ConnectionState",
    "DependencyUnavailableError",
    "DeviceAction",
    "DeviceConfig",
    "DeviceError",
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
    "InvalidStateError",
    "JointState",
    "LifecycleGuard",
    "Pose",
    "ProtocolError",
    "TactileCapable",
    "TactileFrame",
]
