"""Python values exchanged across the Manus native bridge boundary."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from litchi_hardware.core.types import HandSide, Pose


@dataclass(frozen=True, slots=True)
class NativeSkeletonFrame:
    side: HandSide
    keypoints: tuple[Pose, ...]
    timestamp_s: float
    sequence: int


class NativeBridge(Protocol):
    def connect(
        self,
        discovery_wait_seconds: int,
        loopback_only: bool,
        calibration_directory: str,
    ) -> None: ...

    def disconnect(self) -> None: ...

    def read_frame(
        self, side: HandSide, timeout_s: float
    ) -> NativeSkeletonFrame | None: ...

    @property
    def available_sides(self) -> frozenset[HandSide]: ...
