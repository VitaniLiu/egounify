"""Analytic Manus 25-keypoint to Sharpa Wave 22-joint retargeting."""

from __future__ import annotations

import math
from dataclasses import dataclass

from litchi_hardware.core.types import DeviceAction, HandSide, HandTrackingFrame
from litchi_hardware.hardware.manus.schema import MANUS_SHARPA_25_SCHEMA
from litchi_hardware.hardware.sharpa.schema import SHARPA_WAVE_JOINT_SCHEMA

Vector3 = tuple[float, float, float]


def _sub(first: Vector3, second: Vector3) -> Vector3:
    return tuple(a - b for a, b in zip(first, second, strict=True))  # type: ignore[return-value]


def _dot(first: Vector3, second: Vector3) -> float:
    return sum(a * b for a, b in zip(first, second, strict=True))


def _cross(first: Vector3, second: Vector3) -> Vector3:
    return (
        first[1] * second[2] - first[2] * second[1],
        first[2] * second[0] - first[0] * second[2],
        first[0] * second[1] - first[1] * second[0],
    )


def _norm(vector: Vector3) -> float:
    return math.sqrt(_dot(vector, vector))


def _normalized(vector: Vector3, label: str) -> Vector3:
    length = _norm(vector)
    if length <= 1e-8:
        raise ValueError(f"degenerate Manus skeleton: {label} has zero length")
    return tuple(value / length for value in vector)  # type: ignore[return-value]


def _angle(first: Vector3, second: Vector3) -> float:
    first_unit = _normalized(first, "bone")
    second_unit = _normalized(second, "bone")
    return math.acos(max(-1.0, min(1.0, _dot(first_unit, second_unit))))


def _clamp(value: float, bounds: tuple[float, float]) -> float:
    return max(bounds[0], min(bounds[1], value))


@dataclass(frozen=True, slots=True)
class RetargetingConfig:
    """Tuning that remains independent from transports and ROS."""

    smoothing_alpha: float = 0.35
    flexion_scale: float = 1.0
    abduction_scale: float = 1.0

    def __post_init__(self) -> None:
        if not 0.0 < self.smoothing_alpha <= 1.0:
            raise ValueError("smoothing_alpha must be in (0, 1]")
        if self.flexion_scale <= 0 or self.abduction_scale <= 0:
            raise ValueError("retargeting scales must be positive")


class AnalyticRetargeter:
    """Fast geometric retargeter with no native runtime dependencies.

    The Manus vendor client publishes points as wrist, four thumb points, then
    five points each for index, middle, ring, and pinky. Angles are calculated
    relative to a palm frame derived from the wrist and MCP points.
    """

    def __init__(self, config: RetargetingConfig | None = None) -> None:
        self.config = config or RetargetingConfig()
        self._previous: dict[HandSide, tuple[float, ...]] = {}

    def reset(self, side: HandSide | None = None) -> None:
        if side is None:
            self._previous.clear()
        else:
            self._previous.pop(side, None)

    def retarget(self, frame: HandTrackingFrame) -> DeviceAction:
        if frame.schema != MANUS_SHARPA_25_SCHEMA:
            raise ValueError(
                "Manus-to-Sharpa retargeting requires skeleton schema "
                f"{MANUS_SHARPA_25_SCHEMA.id!r}, received {frame.schema.id!r}"
            )
        points = tuple(pose.position for pose in frame.keypoints)
        raw = self._calculate(points)
        limited = tuple(
            _clamp(value, bounds)
            for value, bounds in zip(
                raw, SHARPA_WAVE_JOINT_SCHEMA.limits, strict=True
            )
        )
        previous = self._previous.get(frame.side)
        if previous is None:
            smoothed = limited
        else:
            alpha = self.config.smoothing_alpha
            smoothed = tuple(
                old + alpha * (new - old)
                for old, new in zip(previous, limited, strict=True)
            )
        self._previous[frame.side] = smoothed
        return DeviceAction(
            {
                "joint_position": smoothed,
                "joint_names": SHARPA_WAVE_JOINT_SCHEMA.names,
                "side": frame.side,
                "source_sequence": frame.sequence,
            },
            timestamp_s=frame.timestamp_s,
        )

    def _calculate(self, points: tuple[Vector3, ...]) -> tuple[float, ...]:
        wrist = points[0]
        forward = _normalized(_sub(points[10], wrist), "palm forward axis")
        lateral = _normalized(
            _sub(points[5], points[15]), "palm lateral axis"
        )
        normal = _normalized(_cross(forward, lateral), "palm normal axis")

        thumb_segments = tuple(
            _sub(points[index + 1], points[index]) for index in range(1, 4)
        )
        thumb_first = _normalized(thumb_segments[0], "thumb proximal bone")
        thumb_cmc_fe = abs(
            math.atan2(_dot(thumb_first, normal), _dot(thumb_first, lateral))
        )
        thumb_cmc_aa = math.atan2(
            _dot(thumb_first, forward), abs(_dot(thumb_first, lateral)) + 1e-9
        )
        thumb_mcp_fe = _angle(thumb_segments[0], thumb_segments[1])
        thumb_mcp_aa = 0.0
        thumb_ip = _angle(thumb_segments[1], thumb_segments[2])

        finger_values: list[float] = []
        mcp_flexions: list[float] = []
        for start in (5, 10, 15, 20):
            segments = tuple(
                _sub(points[index + 1], points[index])
                for index in range(start, start + 4)
            )
            proximal = _normalized(segments[0], "finger proximal bone")
            mcp_fe = _angle(forward, proximal)
            mcp_aa = math.atan2(
                _dot(proximal, lateral), max(1e-9, _dot(proximal, forward))
            )
            pip = _angle(segments[0], segments[1])
            dip = 0.5 * (
                _angle(segments[1], segments[2])
                + _angle(segments[2], segments[3])
            )
            mcp_flexions.append(mcp_fe)
            finger_values.extend((mcp_fe, mcp_aa, pip, dip))

        flex = self.config.flexion_scale
        abduct = self.config.abduction_scale
        index = finger_values[0:4]
        middle = finger_values[4:8]
        ring = finger_values[8:12]
        pinky = finger_values[12:16]

        return (
            thumb_cmc_fe * flex,
            thumb_cmc_aa * abduct,
            thumb_mcp_fe * flex,
            thumb_mcp_aa,
            thumb_ip * flex,
            index[0] * flex,
            index[1] * abduct,
            index[2] * flex,
            index[3] * flex,
            middle[0] * flex,
            middle[1] * abduct,
            middle[2] * flex,
            middle[3] * flex,
            ring[0] * flex,
            ring[1] * abduct,
            ring[2] * flex,
            ring[3] * flex,
            min(
                mcp_flexions[-1] * 0.25,
                SHARPA_WAVE_JOINT_SCHEMA.maximum[17],
            ),
            pinky[0] * flex,
            pinky[1] * abduct,
            pinky[2] * flex,
            pinky[3] * flex,
        )
