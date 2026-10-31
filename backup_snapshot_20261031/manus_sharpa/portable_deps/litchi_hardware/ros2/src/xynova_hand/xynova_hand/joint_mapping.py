"""ROS JointState name mapping for the core-aligned Xynova driver."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from litchi_hardware.hardware.xynova import XYNOVA_JOINT_NAMES


class UnknownJointError(ValueError):
    """Raised when a JointState name is not in the Xynova schema."""


@dataclass(frozen=True)
class JointCalibration:
    """Linear HAL-to-SDK mapping: ``sdk = gain * radians + offset``."""

    gain: float = 1.0
    offset: float = 0.0

    def __post_init__(self) -> None:
        if not math.isfinite(self.gain) or self.gain == 0.0:
            raise ValueError("joint calibration gain must be finite and non-zero")
        if not math.isfinite(self.offset):
            raise ValueError("joint calibration offset must be finite")


@dataclass
class JointMapping:
    """Map named ROS commands into the fixed 23-joint core ordering."""

    names: tuple[str, ...] = XYNOVA_JOINT_NAMES
    _index: dict[str, int] = field(init=False, repr=False)

    def __post_init__(self) -> None:
        self._index = {name: index for index, name in enumerate(self.names)}

    def index_of(self, name: str) -> int:
        try:
            return self._index[name.upper()]
        except KeyError as exc:
            raise UnknownJointError(
                f"Unknown joint name {name!r}. Expected one of {list(self.names)}."
            ) from exc

    def ros_positions_to_core(
        self,
        names: list[str],
        positions: list[float],
    ) -> list[float]:
        """Return a partial core-order vector, using NaN for missing joints."""

        if not names:
            if len(positions) != len(self.names):
                raise ValueError(
                    f"expected {len(self.names)} unnamed positions, got {len(positions)}"
                )
            values = [float(value) for value in positions]
            if not all(math.isfinite(value) for value in values):
                raise ValueError("joint positions must be finite")
            return values
        if len(names) != len(positions):
            raise ValueError(
                "JointState name/position length mismatch: "
                f"names={len(names)} positions={len(positions)}"
            )
        normalized_names = [name.upper() for name in names]
        if len(set(normalized_names)) != len(normalized_names):
            raise ValueError("JointState contains duplicate joint names")

        result = [math.nan] * len(self.names)
        for name, value in zip(normalized_names, positions, strict=True):
            resolved = float(value)
            if not math.isfinite(resolved):
                raise ValueError("joint positions must be finite")
            result[self.index_of(name)] = resolved
        return result


def load_calibration_from_dict(
    cfg: dict,
) -> dict[str, JointCalibration]:
    """Parse the YAML calibration table used to configure the HAL adapter."""

    result: dict[str, JointCalibration] = {}
    for raw_name, entry in (cfg or {}).items():
        name = str(raw_name).upper()
        if name not in XYNOVA_JOINT_NAMES:
            raise UnknownJointError(
                f"joint_calibration references unknown joint {raw_name!r}; "
                f"expected one of {list(XYNOVA_JOINT_NAMES)}"
            )
        result[name] = JointCalibration(
            gain=float(entry.get("gain", 1.0)),
            offset=float(entry.get("offset", 0.0)),
        )
    return result
