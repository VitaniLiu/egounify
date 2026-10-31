from __future__ import annotations

import math

import pytest

from litchi_hardware.core.errors import InvalidStateError
from litchi_hardware.core.lifecycle import ConnectionState, LifecycleGuard
from litchi_hardware.core.types import (
    HandSide,
    HandSkeletonSchema,
    HandTrackingFrame,
    JointSchema,
    JointState,
    Pose,
)

_ONE_POINT_SCHEMA = HandSkeletonSchema("test_one_point_v1", ("wrist",))


def test_lifecycle_enforces_connecting_transition() -> None:
    lifecycle = LifecycleGuard()

    with pytest.raises(InvalidStateError):
        lifecycle.connected()

    lifecycle.begin_connect()
    lifecycle.connected()
    assert lifecycle.state is ConnectionState.CONNECTED
    assert lifecycle.is_connected

    lifecycle.disconnected()
    assert lifecycle.state is ConnectionState.DISCONNECTED


def test_failed_lifecycle_requires_reset_before_reconnect() -> None:
    lifecycle = LifecycleGuard()
    lifecycle.begin_connect()
    lifecycle.failed()

    with pytest.raises(InvalidStateError):
        lifecycle.begin_connect()

    lifecycle.reset()
    lifecycle.begin_connect()
    assert lifecycle.state is ConnectionState.CONNECTING


def test_string_enums_use_their_wire_values() -> None:
    assert str(HandSide.LEFT) == "left"
    assert str(ConnectionState.CONNECTED) == "connected"


def test_pose_normalizes_quaternion() -> None:
    pose = Pose((1, 2, 3), (0, 0, 0, 2))
    assert pose.orientation == (0.0, 0.0, 0.0, 1.0)


def test_joint_state_validates_vector_sizes() -> None:
    with pytest.raises(ValueError, match="position length"):
        JointState(("a", "b"), (0.0,))


def test_tracking_frame_is_immutable_and_validated() -> None:
    frame = HandTrackingFrame(
        side=HandSide.LEFT,
        keypoints=(Pose.identity(),),
        timestamp_s=1.5,
        schema=_ONE_POINT_SCHEMA,
        sequence=4,
    )
    assert frame.keypoints[0].orientation[-1] == 1.0

    with pytest.raises(ValueError, match="finite"):
        HandTrackingFrame(
            side=HandSide.RIGHT,
            keypoints=(Pose.identity(),),
            timestamp_s=math.nan,
            schema=_ONE_POINT_SCHEMA,
        )


def test_explicit_schemas_validate_names_limits_and_frames() -> None:
    schema = JointSchema(
        "test_joints_v1",
        ("a", "b"),
        minimum=(-1.0, 0.0),
        maximum=(1.0, 2.0),
    )
    state = JointState(schema.names, (0.0, 1.0), schema=schema)

    assert schema.size == 2
    assert schema.limits == ((-1.0, 1.0), (0.0, 2.0))
    assert state.schema is schema
    with pytest.raises(ValueError, match="match its joint schema"):
        JointState(("b", "a"), (0.0, 1.0), schema=schema)
