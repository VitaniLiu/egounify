"""JointState ordering helpers for Linker Hand ROS commands."""

from __future__ import annotations

import math
from dataclasses import dataclass, field


class UnknownJointError(ValueError):
    """Raised when a command references a joint outside the active schema."""


@dataclass
class JointMapping:
    names: tuple[str, ...]
    side: str
    _index: dict[str, int] = field(init=False, repr=False)

    def __post_init__(self) -> None:
        self._index = {name.lower(): index for index, name in enumerate(self.names)}

    def _normalize(self, name: str) -> str:
        normalized = name.lower()
        prefix = f"{self.side.lower()}_"
        if normalized.startswith(prefix):
            normalized = normalized[len(prefix) :]
        return normalized

    def ros_positions_to_core(self, names: list[str], positions: list[float]) -> tuple[float, ...]:
        if not names:
            if len(positions) != len(self.names):
                raise ValueError(
                    f"expected {len(self.names)} unnamed positions, got {len(positions)}"
                )
            result = tuple(float(value) for value in positions)
        else:
            if len(names) != len(positions):
                raise ValueError(
                    "JointState name/position length mismatch: "
                    f"names={len(names)} positions={len(positions)}"
                )
            normalized = [self._normalize(name) for name in names]
            if len(set(normalized)) != len(normalized):
                raise ValueError("JointState contains duplicate joint names")
            if len(normalized) != len(self.names):
                raise ValueError(
                    f"expected all {len(self.names)} named joints, got {len(normalized)}"
                )
            ordered = [math.nan] * len(self.names)
            for name, value in zip(normalized, positions, strict=True):
                try:
                    index = self._index[name]
                except KeyError as exc:
                    raise UnknownJointError(f"unknown Linker Hand joint {name!r}") from exc
                ordered[index] = float(value)
            if any(math.isnan(value) for value in ordered):
                raise ValueError("JointState does not cover the complete joint schema")
            result = tuple(ordered)
        if not all(math.isfinite(value) for value in result):
            raise ValueError("joint positions must be finite")
        return result
