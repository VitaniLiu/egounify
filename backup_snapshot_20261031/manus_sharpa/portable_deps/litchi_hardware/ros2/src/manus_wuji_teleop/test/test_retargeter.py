from __future__ import annotations

import numpy as np
import pytest
from manus_wuji_teleop.retargeter import (
    MANUS_TO_MEDIAPIPE_INDICES,
    RetargetingFilterConfig,
    WujiSdkRetargeter,
    manus_25_to_mediapipe_21,
)

from litchi_hardware.core.types import HandSide, HandTrackingFrame, Pose
from litchi_hardware.hardware.manus import MANUS_SHARPA_25_SCHEMA


def _frame(
    side: HandSide = HandSide.RIGHT,
    *,
    timestamp_s: float = 1.0,
) -> HandTrackingFrame:
    return HandTrackingFrame(
        side=side,
        keypoints=tuple(
            Pose((float(index), float(index) + 0.1, 0.0), (0.0, 0.0, 0.0, 1.0))
            for index in range(25)
        ),
        timestamp_s=timestamp_s,
        schema=MANUS_SHARPA_25_SCHEMA,
    )


def test_manus_points_convert_to_mediapipe_order() -> None:
    converted = manus_25_to_mediapipe_21(_frame())

    assert converted.shape == (21, 3)
    assert converted.dtype == np.float32
    assert converted[:, 0].tolist() == list(MANUS_TO_MEDIAPIPE_INDICES)
    assert 5 not in converted[:, 0]
    assert 10 not in converted[:, 0]
    assert 15 not in converted[:, 0]
    assert 20 not in converted[:, 0]


def test_retargeter_uses_one_session_per_enabled_side() -> None:
    created = {}

    class FakeSession:
        def __init__(self, side: HandSide) -> None:
            self.side = side
            self.received = None

        def step(self, keypoints: np.ndarray) -> np.ndarray:
            self.received = keypoints.copy()
            offset = 100.0 if self.side is HandSide.RIGHT else 0.0
            return np.arange(20, dtype=np.float32) + offset

    def factory(side: HandSide) -> FakeSession:
        session = FakeSession(side)
        created[side] = session
        return session

    retargeter = WujiSdkRetargeter(
        (HandSide.LEFT, HandSide.RIGHT), session_factory=factory
    )

    right = retargeter.retarget(_frame(HandSide.RIGHT))

    assert set(created) == {HandSide.LEFT, HandSide.RIGHT}
    assert right == tuple(float(index) for index in range(100, 120))
    assert created[HandSide.RIGHT].received.shape == (21, 3)


def test_retargeter_rejects_disabled_side_and_bad_sdk_output() -> None:
    retargeter = WujiSdkRetargeter(
        (HandSide.RIGHT,),
        session_factory=lambda _side: type(
            "BadSession",
            (),
            {"step": lambda self, keypoints: (0.0,)},
        )(),
    )

    with pytest.raises(ValueError, match="no Wuji retargeting session"):
        retargeter.retarget(_frame(HandSide.LEFT))
    with pytest.raises(ValueError, match="returned 1 joints"):
        retargeter.retarget(_frame(HandSide.RIGHT))


def test_filter_applies_stricter_velocity_limit_to_pinky() -> None:
    class JumpSession:
        def __init__(self) -> None:
            self.calls = 0

        def step(self, keypoints: np.ndarray) -> np.ndarray:
            value = float(self.calls)
            self.calls += 1
            return np.full(20, value, dtype=np.float32)

    retargeter = WujiSdkRetargeter(
        (HandSide.RIGHT,),
        session_factory=lambda _side: JumpSession(),
        filter_config=RetargetingFilterConfig(
            smoothing_alpha=1.0,
            max_joint_velocity_rad_s=100.0,
            pinky_max_joint_velocity_rad_s=2.0,
            nominal_input_rate_hz=100.0,
            max_filter_dt_s=0.1,
        ),
    )

    initial = retargeter.retarget(_frame(timestamp_s=1.0))
    jumped = retargeter.retarget(_frame(timestamp_s=1.01))

    assert initial == (0.0,) * 20
    assert jumped[:16] == pytest.approx((1.0,) * 16)
    assert jumped[16:] == pytest.approx((0.02,) * 4)
