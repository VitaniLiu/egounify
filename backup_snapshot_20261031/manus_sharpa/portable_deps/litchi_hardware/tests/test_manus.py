from __future__ import annotations

from types import SimpleNamespace

import pytest

from litchi_hardware.core.types import HandSide, Pose
from litchi_hardware.hardware.manus import (
    ManusBackend,
    ManusConfig,
    ManusNativeConfig,
    ManusNativeTracker,
    ManusZmqConfig,
    ManusZmqTracker,
    create_tracker,
)
from litchi_hardware.hardware.manus._native import NativeSkeletonFrame
from litchi_hardware.hardware.manus.protocol import (
    _message_class,
    decode_mocap_keypoints,
)


def test_mock_manus_produces_teleoperation_25_point_order() -> None:
    tracker = create_tracker(
        ManusConfig(name="test_manus"), use_fake_hardware=True
    )
    tracker.connect()

    left = tracker.read_frame(HandSide.LEFT)
    right = tracker.read_frame(HandSide.RIGHT)

    assert len(left.keypoints) == 25
    assert len(right.keypoints) == 25
    assert left.side is HandSide.LEFT
    assert left.keypoints[0].position == (0.0, 0.0, 0.0)
    tracker.disconnect()


def test_decode_vendor_mocap_protobuf() -> None:
    message = _message_class()()
    message.header.stamp.sec = 12
    message.header.stamp.nanosec = 500_000_000
    message.header.frame_id = "42"
    for index in range(25):
        pose = message.left_mocap_pose.add()
        pose.position.x = index
        pose.orientation.w = 1.0

    packet = decode_mocap_keypoints(message.SerializeToString())

    assert packet.timestamp_s == 12.5
    assert packet.sequence == 42
    assert len(packet.left) == 25
    assert packet.left[-1].position[0] == 24.0


def test_factory_selects_native_and_zmq_backends() -> None:
    config = ManusConfig(name="real")

    assert isinstance(create_tracker(config), ManusNativeTracker)
    zmq_override = create_tracker(config, backend="zmq")
    assert isinstance(zmq_override, ManusZmqTracker)
    assert isinstance(zmq_override.config, ManusZmqConfig)
    assert isinstance(
        create_tracker(ManusZmqConfig(name="zmq")),
        ManusZmqTracker,
    )
    with pytest.raises(ValueError, match="conflicts"):
        create_tracker(
            ManusNativeConfig(name="native"),
            backend=ManusBackend.ZMQ,
        )


def test_concrete_trackers_reject_the_other_backends_config() -> None:
    with pytest.raises(TypeError, match="ManusNativeConfig"):
        ManusNativeTracker(ManusConfig(name="common"))  # type: ignore[arg-type]
    with pytest.raises(TypeError, match="ManusZmqConfig"):
        ManusZmqTracker(ManusConfig(name="common"))  # type: ignore[arg-type]


def test_native_tracker_converts_bridge_frame() -> None:
    calls: dict[str, object] = {}

    class _FakeBridge:
        @property
        def available_sides(self) -> frozenset[HandSide]:
            return frozenset({HandSide.LEFT})

        def connect(
            self,
            discovery_wait_seconds: int,
            loopback_only: bool,
            calibration_directory: str,
        ) -> None:
            calls["connect"] = (
                discovery_wait_seconds,
                loopback_only,
                calibration_directory,
            )

        def disconnect(self) -> None:
            calls["disconnect"] = True

        def read_frame(
            self, side: HandSide, timeout_s: float
        ) -> NativeSkeletonFrame | None:
            calls["read"] = (side, timeout_s)
            return NativeSkeletonFrame(
                side=HandSide.LEFT,
                keypoints=tuple(
                    Pose(
                        position=(float(index), 0.0, 0.0),
                        orientation=(0.0, 0.0, 0.0, 1.0),
                    )
                    for index in range(25)
                ),
                timestamp_s=12.5,
                sequence=42,
            )

    tracker = ManusNativeTracker(
        ManusNativeConfig(
            name="native",
            native_host_discovery_s=2,
            native_loopback_only=True,
            calibration_directory="/tmp/calibration",
        ),
        bridge=_FakeBridge(),
    )
    tracker.connect()
    frame = tracker.read_frame(HandSide.LEFT, timeout_s=0.1)

    assert tracker.info.metadata["transport"] == "native"
    assert tracker.available_sides == frozenset({HandSide.LEFT})
    assert calls["connect"] == (2, True, "/tmp/calibration")
    assert calls["read"] == (HandSide.LEFT, 0.1)
    assert frame.timestamp_s == 12.5
    assert frame.sequence == 42
    assert len(frame.keypoints) == 25
    assert frame.keypoints[-1].position == (24.0, 0.0, 0.0)

    tracker.disconnect()
    assert calls["disconnect"] is True


def test_manus_client_starts_in_vendor_directory(monkeypatch: pytest.MonkeyPatch) -> None:
    started = {}

    class _Socket:
        def setsockopt(self, *args: object) -> None:
            pass

        def setsockopt_string(self, *args: object) -> None:
            pass

        def connect(self, endpoint: str) -> None:
            pass

        def close(self, linger: int = 0) -> None:
            pass

    class _Context:
        def socket(self, socket_type: int) -> _Socket:
            return _Socket()

        def term(self) -> None:
            pass

    fake_zmq = SimpleNamespace(
        Context=_Context,
        SUB=1,
        RCVHWM=2,
        LINGER=3,
        SUBSCRIBE=4,
    )

    class _Process:
        def poll(self) -> int:
            return 0

    def fake_popen(command: list[str], **kwargs: object) -> _Process:
        started["command"] = command
        started.update(kwargs)
        return _Process()

    monkeypatch.setattr(ManusZmqTracker, "_load_zmq", staticmethod(lambda: fake_zmq))
    monkeypatch.setattr("subprocess.Popen", fake_popen)
    tracker = ManusZmqTracker(
        ManusZmqConfig(
            name="managed",
            client_executable="/opt/manus/client/SharpaManusClient.out",
        )
    )
    tracker.connect()
    assert started["cwd"] == "/opt/manus/client"
    tracker.disconnect()
