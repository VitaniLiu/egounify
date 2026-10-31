import pytest
from builtin_interfaces.msg import Time
from sharpa_driver.messages import (
    tactile_frame_id,
    tactile_image_message,
    tactile_topic_prefix,
    tactile_wrench_message,
    validate_tactile_route,
)
from sharpa_driver.node import normalize_command

from litchi_hardware.core.types import HandSide
from litchi_hardware.hardware.sharpa import (
    SHARPA_FINGERS,
    SHARPA_JOINT_NAMES,
    SharpaFinger,
    SharpaTactileFrame,
    TactileImage,
)
from litchi_hardware.hardware.sharpa.tactile import tactile_channel


def test_normalize_command_reorders_side_prefixed_names() -> None:
    names = [f"left_{name}" for name in reversed(SHARPA_JOINT_NAMES)]
    positions = list(range(22))
    result = normalize_command(names, positions, HandSide.LEFT)
    assert result == tuple(reversed(range(22)))


def test_normalize_command_rejects_partial_named_command() -> None:
    with pytest.raises(ValueError, match="missing"):
        normalize_command(("thumb_CMC_FE",), (0.0,), HandSide.RIGHT)


def test_tactile_standard_message_conversion() -> None:
    stamp = Time(sec=12, nanosec=34)
    image = tactile_image_message(
        TactileImage(2, 2, b"\x00\x01\x02\x03"),
        frame_id="left_thumb_fingertip",
        stamp=stamp,
    )
    frame = SharpaTactileFrame(
        side=HandSide.LEFT,
        finger=SharpaFinger.THUMB,
        channel=tactile_channel(HandSide.LEFT, SharpaFinger.THUMB),
        frame_index=1,
        timestamp_s=12.0,
        wrench=(1, 2, 3, 4, 5, 6),
    )
    wrench = tactile_wrench_message(frame, stamp=stamp)

    assert image.encoding == "mono8"
    assert (image.height, image.width, image.step) == (2, 2, 2)
    assert bytes(image.data) == b"\x00\x01\x02\x03"
    assert wrench.header.frame_id == "left_thumb_fingertip"
    assert wrench.wrench.force.z == 3.0
    assert wrench.wrench.torque.z == 6.0


@pytest.mark.parametrize("side", list(HandSide))
@pytest.mark.parametrize("finger", SHARPA_FINGERS)
def test_driver_topics_and_frames_use_physical_finger_identity(
    side: HandSide, finger: SharpaFinger
) -> None:
    # Topic and frame identity are the physical finger; channel selection
    # remains centralized in the HAL and is not reversed again in ROS.
    prefix = tactile_topic_prefix(side, finger)
    frame = SharpaTactileFrame(
        side=side,
        finger=finger,
        channel=tactile_channel(side, finger),
        frame_index=0,
        timestamp_s=0.0,
    )

    assert prefix == f"/sharpa/{side.value}/tactile/{finger.value}"
    assert tactile_frame_id(frame) == f"{side.value}_{finger.value}_fingertip"
    assert (
        validate_tactile_route(
            frame,
            expected_side=side,
            expected_finger=finger,
        )
        == f"{side.value}_{finger.value}_fingertip"
    )


@pytest.mark.parametrize(
    ("frame_side", "frame_finger"),
    [
        (HandSide.LEFT, SharpaFinger.INDEX),
        (HandSide.RIGHT, SharpaFinger.THUMB),
    ],
)
def test_driver_rejects_frame_for_another_physical_topic(
    frame_side: HandSide, frame_finger: SharpaFinger
) -> None:
    frame = SharpaTactileFrame(
        side=frame_side,
        finger=frame_finger,
        channel=tactile_channel(frame_side, frame_finger),
        frame_index=0,
        timestamp_s=0.0,
    )

    with pytest.raises(
        ValueError,
        match=r"expected left/thumb, received",
    ):
        validate_tactile_route(
            frame,
            expected_side=HandSide.LEFT,
            expected_finger=SharpaFinger.THUMB,
        )
