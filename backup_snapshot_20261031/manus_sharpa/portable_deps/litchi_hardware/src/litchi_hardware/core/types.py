"""Immutable domain values exchanged through HAL interfaces."""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from types import MappingProxyType
from typing import Any

from ._compat import StrEnum


def _finite_tuple(values: Sequence[float], field_name: str) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{field_name} must contain only finite values")
    return result


class HandSide(StrEnum):
    LEFT = "left"
    RIGHT = "right"


@dataclass(frozen=True, slots=True)
class HandSkeletonSchema:
    """Named, versioned ordering contract for hand keypoints."""

    id: str
    keypoint_names: tuple[str, ...]

    def __post_init__(self) -> None:
        names = tuple(self.keypoint_names)
        if not self.id:
            raise ValueError("hand skeleton schema id must not be empty")
        if not names or any(not name for name in names):
            raise ValueError("hand skeleton keypoint names must not be empty")
        if len(set(names)) != len(names):
            raise ValueError("hand skeleton keypoint names must be unique")
        object.__setattr__(self, "keypoint_names", names)

    @property
    def size(self) -> int:
        return len(self.keypoint_names)


@dataclass(frozen=True, slots=True)
class JointSchema:
    """Named joint ordering, unit, and optional limits for one device model."""

    id: str
    names: tuple[str, ...]
    unit: str = "rad"
    minimum: tuple[float, ...] = ()
    maximum: tuple[float, ...] = ()

    def __post_init__(self) -> None:
        names = tuple(self.names)
        minimum = _finite_tuple(self.minimum, "minimum")
        maximum = _finite_tuple(self.maximum, "maximum")
        if not self.id or not self.unit:
            raise ValueError("joint schema id and unit must not be empty")
        if not names or any(not name for name in names):
            raise ValueError("joint schema names must not be empty")
        if len(set(names)) != len(names):
            raise ValueError("joint schema names must be unique")
        if bool(minimum) is not bool(maximum):
            raise ValueError("joint schema minimum and maximum must be provided together")
        if minimum and (len(minimum) != len(names) or len(maximum) != len(names)):
            raise ValueError("joint schema limits must match joint names")
        if any(lower > upper for lower, upper in zip(minimum, maximum, strict=True)):
            raise ValueError("joint schema minimum must not exceed maximum")
        object.__setattr__(self, "names", names)
        object.__setattr__(self, "minimum", minimum)
        object.__setattr__(self, "maximum", maximum)

    @property
    def size(self) -> int:
        return len(self.names)

    @property
    def limits(self) -> tuple[tuple[float, float], ...]:
        if not self.minimum:
            return ()
        return tuple(zip(self.minimum, self.maximum, strict=True))


@dataclass(frozen=True, slots=True)
class Pose:
    """Cartesian pose with an xyzw quaternion."""

    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]

    def __post_init__(self) -> None:
        position = _finite_tuple(self.position, "position")
        orientation = _finite_tuple(self.orientation, "orientation")
        if len(position) != 3 or len(orientation) != 4:
            raise ValueError("pose requires 3 position and 4 orientation values")
        norm = math.sqrt(sum(value * value for value in orientation))
        if norm <= 1e-12:
            raise ValueError("orientation quaternion must have non-zero norm")
        object.__setattr__(self, "position", position)
        object.__setattr__(
            self, "orientation", tuple(value / norm for value in orientation)
        )

    @classmethod
    def identity(cls) -> Pose:
        return cls((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


@dataclass(frozen=True, slots=True)
class JointState:
    """Joint vectors in SI units."""

    names: tuple[str, ...]
    position: tuple[float, ...]
    velocity: tuple[float, ...] = ()
    effort: tuple[float, ...] = ()
    timestamp_s: float | None = None
    schema: JointSchema | None = None

    def __post_init__(self) -> None:
        names = tuple(self.names)
        if not names or len(set(names)) != len(names) or any(not name for name in names):
            raise ValueError("joint names must be non-empty and unique")
        position = _finite_tuple(self.position, "position")
        velocity = _finite_tuple(self.velocity, "velocity")
        effort = _finite_tuple(self.effort, "effort")
        if len(position) != len(names):
            raise ValueError("position length must match joint names")
        if velocity and len(velocity) != len(names):
            raise ValueError("velocity length must match joint names")
        if effort and len(effort) != len(names):
            raise ValueError("effort length must match joint names")
        if self.timestamp_s is not None and not math.isfinite(self.timestamp_s):
            raise ValueError("timestamp_s must be finite")
        if self.schema is not None and names != self.schema.names:
            raise ValueError("joint state names must match its joint schema")
        object.__setattr__(self, "names", names)
        object.__setattr__(self, "position", position)
        object.__setattr__(self, "velocity", velocity)
        object.__setattr__(self, "effort", effort)


@dataclass(frozen=True, slots=True)
class HandTrackingFrame:
    """A single hand skeleton sample in a named coordinate frame."""

    side: HandSide
    keypoints: tuple[Pose, ...]
    timestamp_s: float
    schema: HandSkeletonSchema
    frame_id: str = "hand_tracking"
    sequence: int | None = None

    def __post_init__(self) -> None:
        keypoints = tuple(self.keypoints)
        if len(keypoints) != self.schema.size:
            raise ValueError(
                f"tracking frame schema {self.schema.id!r} requires "
                f"{self.schema.size} keypoints, received {len(keypoints)}"
            )
        if not math.isfinite(self.timestamp_s):
            raise ValueError("timestamp_s must be finite")
        if not self.frame_id:
            raise ValueError("frame_id must not be empty")
        if self.sequence is not None and self.sequence < 0:
            raise ValueError("sequence must not be negative")
        object.__setattr__(self, "keypoints", keypoints)


@dataclass(frozen=True, slots=True)
class TactileFrame:
    """Flat taxel data plus its original shape."""

    values: tuple[float, ...]
    shape: tuple[int, ...]
    timestamp_s: float
    frame_id: str

    def __post_init__(self) -> None:
        values = _finite_tuple(self.values, "values")
        shape = tuple(int(value) for value in self.shape)
        expected = math.prod(shape)
        if not shape or any(value <= 0 for value in shape) or expected != len(values):
            raise ValueError("tactile shape must be positive and match values")
        object.__setattr__(self, "values", values)
        object.__setattr__(self, "shape", shape)


@dataclass(frozen=True, slots=True)
class FeatureSpec:
    """Schema for one observation or action feature."""

    name: str
    size: int
    unit: str = ""
    minimum: tuple[float, ...] = ()
    maximum: tuple[float, ...] = ()

    def __post_init__(self) -> None:
        if not self.name or self.size <= 0:
            raise ValueError("feature name must be non-empty and size positive")
        minimum = _finite_tuple(self.minimum, "minimum")
        maximum = _finite_tuple(self.maximum, "maximum")
        if minimum and len(minimum) not in (1, self.size):
            raise ValueError("minimum must be scalar or match feature size")
        if maximum and len(maximum) not in (1, self.size):
            raise ValueError("maximum must be scalar or match feature size")
        object.__setattr__(self, "minimum", minimum)
        object.__setattr__(self, "maximum", maximum)


@dataclass(frozen=True, slots=True)
class DeviceObservation:
    """Backend-neutral observation payload."""

    features: Mapping[str, Any]
    timestamp_s: float

    def __post_init__(self) -> None:
        if not math.isfinite(self.timestamp_s):
            raise ValueError("timestamp_s must be finite")
        object.__setattr__(self, "features", MappingProxyType(dict(self.features)))


@dataclass(frozen=True, slots=True)
class DeviceAction:
    """Backend-neutral action payload."""

    features: Mapping[str, Any]
    timestamp_s: float | None = None

    def __post_init__(self) -> None:
        if self.timestamp_s is not None and not math.isfinite(self.timestamp_s):
            raise ValueError("timestamp_s must be finite")
        object.__setattr__(self, "features", MappingProxyType(dict(self.features)))
