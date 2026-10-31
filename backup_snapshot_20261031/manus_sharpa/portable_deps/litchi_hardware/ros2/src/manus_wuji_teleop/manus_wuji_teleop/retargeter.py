from __future__ import annotations

import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from typing import Protocol

import numpy as np

from litchi_hardware.core.types import HandSide, HandTrackingFrame
from litchi_hardware.hardware.manus import MANUS_SHARPA_25_SCHEMA

JOINT_COUNT = 20
MEDIAPIPE_KEYPOINT_COUNT = 21
MANUS_TO_MEDIAPIPE_INDICES = (
    0,
    1,
    2,
    3,
    4,
    6,
    7,
    8,
    9,
    11,
    12,
    13,
    14,
    16,
    17,
    18,
    19,
    21,
    22,
    23,
    24,
)


class RetargetSession(Protocol):
    def step(self, keypoints: np.ndarray) -> Sequence[float]: ...


SessionFactory = Callable[[HandSide], RetargetSession]


@dataclass(frozen=True, slots=True)
class RetargetingFilterConfig:
    smoothing_alpha: float = 0.35
    max_joint_velocity_rad_s: float = 12.0
    pinky_max_joint_velocity_rad_s: float = 5.0
    nominal_input_rate_hz: float = 60.0
    max_filter_dt_s: float = 0.05

    def __post_init__(self) -> None:
        if not 0.0 < self.smoothing_alpha <= 1.0:
            raise ValueError("smoothing_alpha must be in (0, 1]")
        if self.max_joint_velocity_rad_s <= 0.0:
            raise ValueError("max_joint_velocity_rad_s must be positive")
        if self.pinky_max_joint_velocity_rad_s <= 0.0:
            raise ValueError("pinky_max_joint_velocity_rad_s must be positive")
        if self.nominal_input_rate_hz <= 0.0:
            raise ValueError("nominal_input_rate_hz must be positive")
        if self.max_filter_dt_s <= 0.0:
            raise ValueError("max_filter_dt_s must be positive")


def manus_25_to_mediapipe_21(frame: HandTrackingFrame) -> np.ndarray:
    if frame.schema != MANUS_SHARPA_25_SCHEMA:
        raise ValueError(
            "Wuji retargeting requires skeleton schema "
            f"{MANUS_SHARPA_25_SCHEMA.id!r}, received {frame.schema.id!r}"
        )
    positions = np.asarray(
        [frame.keypoints[index].position for index in MANUS_TO_MEDIAPIPE_INDICES],
        dtype=np.float32,
    )
    if positions.shape != (MEDIAPIPE_KEYPOINT_COUNT, 3):
        raise ValueError(
            "Wuji retargeting requires 21 three-dimensional keypoints, "
            f"received shape {positions.shape}"
        )
    if not np.isfinite(positions).all():
        raise ValueError("Wuji retargeting keypoints must be finite")
    return positions


def _official_session(side: HandSide) -> RetargetSession:
    from wuji_sdk import Handedness, HandModel
    from wuji_sdk import RetargetSession as OfficialRetargetSession

    handedness = (
        Handedness.Left if side is HandSide.LEFT else Handedness.Right
    )
    return OfficialRetargetSession.for_hand(
        HandModel.WujiHand2,
        side=handedness,
    )


class WujiSdkRetargeter:
    def __init__(
        self,
        sides: Sequence[HandSide],
        *,
        session_factory: SessionFactory | None = None,
        filter_config: RetargetingFilterConfig | None = None,
    ) -> None:
        unique_sides = tuple(dict.fromkeys(sides))
        if not unique_sides:
            raise ValueError("at least one hand side must be enabled")
        factory = session_factory or _official_session
        self._sessions = {side: factory(side) for side in unique_sides}
        self._filter_config = filter_config or RetargetingFilterConfig()
        self._previous: dict[HandSide, tuple[float, ...]] = {}
        self._previous_timestamp_s: dict[HandSide, float] = {}

    def retarget(self, frame: HandTrackingFrame) -> tuple[float, ...]:
        try:
            session = self._sessions[frame.side]
        except KeyError as exc:
            raise ValueError(
                f"no Wuji retargeting session for {frame.side.value} hand"
            ) from exc
        keypoints = manus_25_to_mediapipe_21(frame)
        raw_positions = tuple(float(value) for value in session.step(keypoints))
        if len(raw_positions) != JOINT_COUNT:
            raise ValueError(
                f"Wuji SDK retargeting returned {len(raw_positions)} joints; "
                f"expected {JOINT_COUNT}"
            )
        if any(not math.isfinite(value) for value in raw_positions):
            raise ValueError("Wuji SDK retargeting returned non-finite positions")
        return self._filter(frame.side, frame.timestamp_s, raw_positions)

    def reset(self, side: HandSide | None = None) -> None:
        if side is None:
            self._previous.clear()
            self._previous_timestamp_s.clear()
            for session in self._sessions.values():
                reset = getattr(session, "reset", None)
                if reset is not None:
                    reset()
            return
        self._previous.pop(side, None)
        self._previous_timestamp_s.pop(side, None)
        reset = getattr(self._sessions.get(side), "reset", None)
        if reset is not None:
            reset()

    def _filter(
        self,
        side: HandSide,
        timestamp_s: float,
        target: tuple[float, ...],
    ) -> tuple[float, ...]:
        previous = self._previous.get(side)
        if previous is None:
            self._previous[side] = target
            self._previous_timestamp_s[side] = timestamp_s
            return target

        config = self._filter_config
        dt_s = timestamp_s - self._previous_timestamp_s[side]
        if not math.isfinite(dt_s) or dt_s <= 0.0:
            dt_s = 1.0 / config.nominal_input_rate_hz
        dt_s = min(dt_s, config.max_filter_dt_s)

        limits = (config.max_joint_velocity_rad_s,) * 16 + (
            config.pinky_max_joint_velocity_rad_s,
        ) * 4
        filtered = tuple(
            old
            + max(
                -velocity_limit * dt_s,
                min(
                    velocity_limit * dt_s,
                    config.smoothing_alpha * (new - old),
                ),
            )
            for old, new, velocity_limit in zip(
                previous, target, limits, strict=True
            )
        )
        self._previous[side] = filtered
        self._previous_timestamp_s[side] = timestamp_s
        return filtered
