from types import SimpleNamespace

import pytest
from wuji_hand_driver.driver import (
    JOINT_COUNT,
    feedback_values,
    joint_names,
    normalize_command,
)


def test_joint_names_follow_finger_major_firmware_order() -> None:
    names = joint_names("right")

    assert len(names) == JOINT_COUNT
    assert names[:5] == (
        "right_finger1_joint1",
        "right_finger1_joint2",
        "right_finger1_joint3",
        "right_finger1_joint4",
        "right_finger2_joint1",
    )
    assert names[-1] == "right_finger5_joint4"


def test_normalize_command_reorders_named_vectors_and_defaults_feedforward() -> None:
    expected = joint_names("left")
    names = tuple(reversed(expected))
    positions = tuple(float(index) for index in range(JOINT_COUNT))

    command = normalize_command(
        names=names,
        positions=positions,
        velocities=(),
        efforts=(),
        expected_names=expected,
    )

    assert command.positions == tuple(reversed(positions))
    assert command.velocities == (0.0,) * JOINT_COUNT
    assert command.efforts == (0.0,) * JOINT_COUNT


def test_normalize_command_rejects_partial_and_non_finite_commands() -> None:
    expected = joint_names("right")
    with pytest.raises(ValueError, match="requires 20 positions"):
        normalize_command(
            names=(),
            positions=(0.0,),
            velocities=(),
            efforts=(),
            expected_names=expected,
        )
    with pytest.raises(ValueError, match="must be finite"):
        normalize_command(
            names=(),
            positions=(0.0,) * 19 + (float("nan"),),
            velocities=(),
            efforts=(),
            expected_names=expected,
        )


def test_feedback_maps_firmware_node_ids() -> None:
    frame = SimpleNamespace(
        header=SimpleNamespace(frame_id="r_wrist"),
        num_joints=2,
        joints=[
            SimpleNamespace(nid=21, position=2.1, velocity=3.1, effort=4.1),
            SimpleNamespace(nid=4, position=0.4, velocity=1.4, effort=2.4),
        ],
    )

    feedback = feedback_values(frame, side="right")

    assert feedback.names == (
        "right_finger1_joint4",
        "right_finger5_joint1",
    )
    assert feedback.positions == (0.4, 2.1)
    assert feedback.velocities == (1.4, 3.1)
    assert feedback.efforts == (2.4, 4.1)


def test_feedback_rejects_tactile_node_ids() -> None:
    frame = SimpleNamespace(
        header=SimpleNamespace(frame_id=""),
        num_joints=1,
        joints=[
            SimpleNamespace(nid=5, position=0.0, velocity=0.0, effort=0.0),
        ],
    )

    with pytest.raises(ValueError, match="invalid joint id 5"):
        feedback_values(frame, side="right")
