"""Abstract contracts for hardware and domain services."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Protocol, runtime_checkable

from .config import DeviceInfo
from .lifecycle import ConnectionState
from .types import (
    DeviceAction,
    DeviceObservation,
    FeatureSpec,
    HandSide,
    HandTrackingFrame,
    JointState,
    Pose,
    TactileFrame,
)


class ConnectableInterface(ABC):
    @property
    @abstractmethod
    def info(self) -> DeviceInfo: ...

    @property
    @abstractmethod
    def connection_state(self) -> ConnectionState: ...

    @property
    def is_connected(self) -> bool:
        return self.connection_state is ConnectionState.CONNECTED

    @abstractmethod
    def connect(self) -> None: ...

    @abstractmethod
    def disconnect(self) -> None: ...


class DeviceInterface(ConnectableInterface):
    @abstractmethod
    def read_joint_state(self) -> JointState: ...

    @property
    @abstractmethod
    def observation_features(self) -> tuple[FeatureSpec, ...]: ...

    @property
    @abstractmethod
    def action_features(self) -> tuple[FeatureSpec, ...]: ...

    @abstractmethod
    def get_observation(self) -> DeviceObservation: ...

    @abstractmethod
    def send_action(self, action: DeviceAction) -> None: ...

    @abstractmethod
    def stop(self) -> None: ...


class ArmInterface(DeviceInterface):
    @abstractmethod
    def read_tcp_pose(self) -> Pose: ...

    @abstractmethod
    def read_wrench(self) -> tuple[float, float, float, float, float, float]: ...


class HandInterface(DeviceInterface):
    """A robotic hand actuator. Grasp helpers can be added without arm semantics."""

    @abstractmethod
    def open(self) -> None: ...

    @abstractmethod
    def close(self) -> None: ...


class HandTrackerInterface(ConnectableInterface):
    """Read-only wearable or optical hand tracking device."""

    @abstractmethod
    def read_frame(
        self, side: HandSide, timeout_s: float | None = None
    ) -> HandTrackingFrame: ...

    @property
    @abstractmethod
    def available_sides(self) -> frozenset[HandSide]: ...


@runtime_checkable
class TactileCapable(Protocol):
    def read_tactile(self, timeout_s: float | None = None) -> TactileFrame: ...
