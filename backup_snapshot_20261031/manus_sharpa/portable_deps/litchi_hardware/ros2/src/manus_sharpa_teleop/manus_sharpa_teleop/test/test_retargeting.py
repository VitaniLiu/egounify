from __future__ import annotations

import time

import pytest
from manus_sharpa_teleop.analytic_retargeter import (
    AnalyticRetargeter,
    RetargetingConfig,
)
from manus_sharpa_teleop.v4_retargeter import _frame_to_v4_keypoints

from litchi_hardware.core.types import HandSide, HandTrackingFrame, Pose
from litchi_hardware.hardware.manus import MockManusTracker
from litchi_hardware.hardware.manus.schema import MANUS_SHARPA_25_SCHEMA
from litchi_hardware.hardware.sharpa.constants import SHARPA_JOINT_LIMITS


def _frame(closure: float, side: HandSide = HandSide.LEFT) -> HandTrackingFrame:
    return HandTrackingFrame(
        side,
        MockManusTracker._make_keypoints(side, closure),
        time.time(),
        MANUS_SHARPA_25_SCHEMA,
        sequence=1,
    )


def test_retargeter_produces_22_safe_joint_targets() -> None:
    retargeter = AnalyticRetargeter(
        RetargetingConfig(smoothing_alpha=1.0)
    )
    action = retargeter.retarget(_frame(1.0))
    positions = action.features["joint_position"]

    assert len(positions) == 22
    assert all(
        lower <= value <= upper
        for value, (lower, upper) in zip(
            positions, SHARPA_JOINT_LIMITS, strict=True
        )
    )
    assert action.features["side"] is HandSide.LEFT


def test_closed_fake_hand_has_more_flexion_than_open_hand() -> None:
    open_positions = AnalyticRetargeter(
        RetargetingConfig(smoothing_alpha=1.0)
    ).retarget(_frame(0.0)).features["joint_position"]
    closed_positions = AnalyticRetargeter(
        RetargetingConfig(smoothing_alpha=1.0)
    ).retarget(_frame(1.0)).features["joint_position"]
    flexion_indices = (0, 2, 4, 5, 7, 8, 9, 11, 12, 13, 15, 16, 18, 20, 21)

    assert sum(closed_positions[index] for index in flexion_indices) > sum(
        open_positions[index] for index in flexion_indices
    )


def test_retargeting_smooths_per_hand_and_reset_clears_history() -> None:
    retargeter = AnalyticRetargeter(RetargetingConfig(smoothing_alpha=0.25))
    initial = retargeter.retarget(_frame(0.0)).features["joint_position"]
    smoothed = retargeter.retarget(_frame(1.0)).features["joint_position"]
    direct = AnalyticRetargeter(
        RetargetingConfig(smoothing_alpha=1.0)
    ).retarget(_frame(1.0)).features["joint_position"]

    assert smoothed != direct
    assert smoothed != initial
    retargeter.reset(HandSide.LEFT)
    assert retargeter.retarget(_frame(1.0)).features["joint_position"] == direct


def test_retargeter_rejects_wrong_skeleton_schema() -> None:
    wrong_schema = type(MANUS_SHARPA_25_SCHEMA)(
        "other_25_v1", MANUS_SHARPA_25_SCHEMA.keypoint_names
    )
    frame = HandTrackingFrame(
        HandSide.LEFT,
        MockManusTracker._make_keypoints(HandSide.LEFT, 0.0),
        0.0,
        wrong_schema,
    )
    with pytest.raises(ValueError, match="skeleton schema"):
        AnalyticRetargeter().retarget(frame)


def test_v4_input_preserves_keypoint_order_and_uses_wxyz_quaternion() -> None:
    keypoints = list(MockManusTracker._make_keypoints(HandSide.LEFT, 0.0))
    keypoints[1] = Pose((1.0, 2.0, 3.0), (0.0, 0.0, 1.0, 0.0))
    frame = HandTrackingFrame(
        HandSide.LEFT,
        tuple(keypoints),
        1.0,
        MANUS_SHARPA_25_SCHEMA,
    )

    converted = _frame_to_v4_keypoints(frame)

    assert len(converted) == 25
    assert converted[1] == [1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0]
