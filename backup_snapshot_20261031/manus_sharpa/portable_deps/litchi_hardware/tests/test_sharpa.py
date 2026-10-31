from __future__ import annotations

from dataclasses import dataclass
from types import SimpleNamespace

import pytest

from litchi_hardware.core.errors import CommandRejectedError, ConnectionError
from litchi_hardware.core.lifecycle import ConnectionState
from litchi_hardware.core.types import DeviceAction, HandSide
from litchi_hardware.hardware.sharpa import (
    SHARPA_FINGERS,
    SHARPA_JOINT_LIMITS,
    SharpaConfig,
    SharpaFinger,
    SharpaSdkHand,
    create_hand,
)
from litchi_hardware.hardware.sharpa.tactile import (
    SharpaTactileFrame,
    decode_tactile_frame,
    tactile_channel,
)


def test_fake_sharpa_accepts_valid_joint_action() -> None:
    hand = create_hand(SharpaConfig(name="fake", side=HandSide.LEFT), use_fake_hardware=True)
    hand.connect()
    targets = tuple((lower + upper) / 2 for lower, upper in SHARPA_JOINT_LIMITS)
    hand.send_action(DeviceAction({"joint_position": targets}))
    state = hand.read_joint_state()
    assert state.position == targets
    assert len(state.velocity) == 22
    assert state.effort == (0.0,) * 22


def test_sharpa_rejects_wrong_size_and_limits() -> None:
    hand = create_hand(SharpaConfig(name="fake"), use_fake_hardware=True)
    hand.connect()
    with pytest.raises(CommandRejectedError, match="22"):
        hand.send_action(DeviceAction({"joint_position": (0.0,)}))

    invalid = [0.0] * 22
    invalid[0] = 100.0
    with pytest.raises(CommandRejectedError, match="thumb_CMC_FE"):
        hand.send_action(DeviceAction({"joint_position": invalid}))


@dataclass
class _Error:
    code: int = 0
    message: str = ""


class _Wave:
    def __init__(
        self,
        calls: list[str],
        *,
        speed: float = 0.3,
        current: float = 0.6,
        source: object = "sdk",
        mode: object = "position",
        enabled: bool = True,
        enable_readbacks_before_true: int = 0,
        fail_at: str | None = None,
        ignore_write_at: str | None = None,
        start_result: bool = True,
    ) -> None:
        self.calls = calls
        self.fail_at = fail_at
        self.ignore_write_at = ignore_write_at
        self.start_result = start_result
        self.speed = speed
        self.current = current
        self.source = source
        self.mode = mode
        self.enabled = enabled
        self.enable_readbacks_before_true = enable_readbacks_before_true
        self.enable_requested = False
        self.enable_readbacks = 0
        self.position = [0.0] * 22

    def _ok(self, name: str) -> _Error:
        self.calls.append(name)
        if name == self.fail_at:
            return _Error(0x0D, "OPERATION_NOT_ALLOWED")
        return _Error()

    def set_control_mode(self, value: object) -> _Error:
        error = self._ok("set_mode")
        if error.code == 0 and self.ignore_write_at != "set_mode":
            self.mode = value
            self.enabled = True
        return error

    def set_enable_state(self, value: bool) -> _Error:
        error = self._ok("set_enable")
        if error.code == 0 and self.ignore_write_at != "set_enable":
            if value and self.enable_readbacks_before_true:
                self.enable_requested = True
            else:
                self.enabled = value
        return error

    def set_speed_coeff(self, value: float) -> _Error:
        error = self._ok("set_speed")
        if error.code == 0 and self.ignore_write_at != "set_speed":
            self.speed = value
        return error

    def set_current_coeff(self, value: float) -> _Error:
        error = self._ok("set_current")
        if error.code == 0 and self.ignore_write_at != "set_current":
            self.current = value
        return error

    def set_control_source(self, value: object) -> _Error:
        error = self._ok("set_source")
        if error.code == 0 and self.ignore_write_at != "set_source":
            self.source = value
        return error

    def get_control_mode(self) -> tuple[_Error, object]:
        return self._ok("get_mode"), self.mode

    def get_speed_coeff(self) -> tuple[_Error, float]:
        return self._ok("get_speed"), self.speed

    def get_current_coeff(self) -> tuple[_Error, float]:
        return self._ok("get_current"), self.current

    def get_control_source(self) -> tuple[_Error, object]:
        return self._ok("get_source"), self.source

    def get_enable_state(self) -> tuple[_Error, bool]:
        if self.enable_requested:
            self.enable_readbacks += 1
            if self.enable_readbacks > self.enable_readbacks_before_true:
                self.enabled = True
                self.enable_requested = False
        return self._ok("get_enable"), self.enabled

    def start(self) -> bool:
        self.calls.append("start")
        return self.start_result

    def stop(self) -> bool:
        self.calls.append("stop")
        return True

    def destroy(self) -> None:
        self.calls.append("destroy")

    def set_joint_position(self, values: list[float], interpolate: bool) -> _Error:
        self.position = values
        return self._ok("position")

    def get_joint_position_rad(self) -> tuple[_Error, list[float]]:
        return _Error(), self.position

    def get_states(self) -> object:
        return SimpleNamespace(
            angles=self.position,
            velocities=[0.1] * 22,
            torques=[0.2] * 22,
        )

    def fetch_tactile_frame(self, channel: int, timeout: float) -> dict[str, object]:
        self.calls.append(f"tactile:{channel}:{timeout}")
        return {
            "frame_id": 1,
            "channel": channel,
            "ts": 1.0,
            "content": {"F6": [1, 2, 3, 4, 5, 6]},
        }


def _sdk_hand(wave: _Wave, calls: list[str], config: SharpaConfig | None = None) -> SharpaSdkHand:
    device = SimpleNamespace(sn="WAVE-L-1", hand_side="left", device_type="hand")

    class _Manager:
        def get_all_devices(self) -> list[object]:
            return [device]

        def connect(self, serial: str, skip_tactile: bool) -> _Wave:
            calls.append(f"connect:{serial}:{skip_tactile}")
            return wave

        def disconnect(self, serial: str) -> None:
            calls.append(f"disconnect:{serial}")

    sdk = SimpleNamespace(
        SharpaWaveManager=SimpleNamespace(get_instance=lambda: _Manager()),
        DeviceType=SimpleNamespace(HAND="hand"),
        HandSide=SimpleNamespace(LEFT="left", RIGHT="right"),
        ControlMode=SimpleNamespace(POSITION="position"),
        ControlSource=SimpleNamespace(SDK="sdk"),
    )
    return SharpaSdkHand(config or SharpaConfig(name="real", side=HandSide.LEFT), sdk_module=sdk)


def test_sdk_adapter_skips_all_matching_persistent_settings() -> None:
    calls: list[str] = []
    wave = _Wave(calls)
    hand = _sdk_hand(wave, calls)
    hand.connect()

    assert calls == [
        "connect:WAVE-L-1:True",
        "get_speed",
        "get_current",
        "get_source",
        "get_mode",
        "get_enable",
        "start",
    ]
    hand.disconnect()
    assert calls[-3:] == ["stop", "destroy", "disconnect:WAVE-L-1"]


@pytest.mark.parametrize(
    ("setting", "initial", "setter", "getter"),
    [
        ("source", "app", "set_source", "get_source"),
        ("mode", "velocity", "set_mode", "get_mode"),
    ],
)
def test_sdk_adapter_sets_and_verifies_mismatched_enum_once(
    setting: str, initial: object, setter: str, getter: str
) -> None:
    calls: list[str] = []
    wave = _Wave(calls, **{setting: initial})
    hand = _sdk_hand(wave, calls)

    hand.connect()

    assert calls[1:5] == ["get_speed", "get_current", "get_source", "get_mode"]
    assert calls.count(setter) == 1
    assert calls.count(getter) == 2
    assert calls[-1] == "start"
    hand.disconnect()


def test_sdk_adapter_enables_disabled_running_mode_once() -> None:
    calls: list[str] = []
    wave = _Wave(calls, enabled=False)
    hand = _sdk_hand(wave, calls)

    hand.connect()

    assert calls.count("set_enable") == 1
    assert calls.count("get_enable") == 2
    assert calls[-1] == "start"
    hand.disconnect()


def test_sdk_adapter_polls_until_delayed_enable_is_observed() -> None:
    calls: list[str] = []
    wave = _Wave(calls, enabled=False, enable_readbacks_before_true=2)
    hand = _sdk_hand(wave, calls)

    hand.connect()

    assert calls.count("set_enable") == 1
    assert calls.count("get_enable") == 4
    assert calls[-1] == "start"
    hand.disconnect()


def test_sdk_adapter_mode_change_does_not_redundantly_enable() -> None:
    calls: list[str] = []
    wave = _Wave(calls, mode="velocity", enabled=False)
    hand = _sdk_hand(wave, calls)

    hand.connect()

    assert calls.count("set_mode") == 1
    assert calls.count("get_enable") == 1
    assert "set_enable" not in calls
    hand.disconnect()


def test_sdk_adapter_fails_closed_when_enable_is_rejected() -> None:
    calls: list[str] = []
    wave = _Wave(calls, enabled=False, fail_at="set_enable")
    hand = _sdk_hand(wave, calls)

    with pytest.raises(CommandRejectedError, match="set_enable_state"):
        hand.connect()

    assert calls.count("set_enable") == 1
    assert calls.count("get_enable") == 1
    assert "start" not in calls
    assert calls[-3:] == ["stop", "destroy", "disconnect:WAVE-L-1"]


def test_sdk_adapter_fails_closed_on_enable_readback_mismatch() -> None:
    calls: list[str] = []
    wave = _Wave(calls, enabled=False, ignore_write_at="set_enable")
    hand = _sdk_hand(wave, calls)

    with pytest.raises(CommandRejectedError, match="verification failed"):
        hand.connect()

    assert calls.count("set_enable") == 1
    assert calls.count("get_enable") >= 2
    assert "start" not in calls
    assert calls[-3:] == ["stop", "destroy", "disconnect:WAVE-L-1"]


@pytest.mark.parametrize(
    ("setting", "initial", "setter", "getter"),
    [
        ("source", "app", "set_source", "get_source"),
        ("mode", "velocity", "set_mode", "get_mode"),
    ],
)
def test_sdk_adapter_fails_closed_on_enum_readback_mismatch(
    setting: str, initial: object, setter: str, getter: str
) -> None:
    calls: list[str] = []
    wave = _Wave(
        calls,
        **{setting: initial},
        ignore_write_at=setter,
    )
    hand = _sdk_hand(wave, calls)

    with pytest.raises(CommandRejectedError, match="verification failed"):
        hand.connect()

    assert calls.count(setter) == 1
    assert calls.count(getter) == 2
    assert "start" not in calls
    assert calls[-3:] == ["stop", "destroy", "disconnect:WAVE-L-1"]


@pytest.mark.parametrize(
    ("setting", "initial", "setter", "getter"),
    [
        ("speed", 0.1, "set_speed", "get_speed"),
        ("current", 0.1, "set_current", "get_current"),
    ],
)
def test_sdk_adapter_sets_and_verifies_mismatched_coefficient_once(
    setting: str, initial: float, setter: str, getter: str
) -> None:
    calls: list[str] = []
    wave = _Wave(calls, **{setting: initial})
    hand = _sdk_hand(wave, calls)

    hand.connect()

    assert calls[1:5] == ["get_speed", "get_current", "get_source", "get_mode"]
    assert calls.count(setter) == 1
    assert calls.count(getter) == 2
    assert calls[-1] == "start"
    hand.disconnect()


def test_sdk_adapter_uses_coefficient_absolute_tolerance() -> None:
    calls: list[str] = []
    wave = _Wave(calls, speed=0.30009, current=0.59991)
    hand = _sdk_hand(wave, calls)

    hand.connect()

    assert "set_speed" not in calls
    assert "set_current" not in calls
    hand.disconnect()


@pytest.mark.parametrize(
    "getter", ["get_speed", "get_current", "get_source", "get_mode", "get_enable"]
)
def test_sdk_adapter_fails_closed_on_setting_getter_error(getter: str) -> None:
    calls: list[str] = []
    wave = _Wave(calls, fail_at=getter)
    hand = _sdk_hand(wave, calls)
    operation = {
        "get_speed": "get_speed_coeff",
        "get_current": "get_current_coeff",
        "get_source": "get_control_source",
        "get_mode": "get_control_mode",
        "get_enable": "get_enable_state",
    }[getter]

    with pytest.raises(CommandRejectedError, match=operation):
        hand.connect()

    assert not any(call.startswith("set_") for call in calls)
    assert "start" not in calls
    assert calls[-3:] == ["stop", "destroy", "disconnect:WAVE-L-1"]
    assert hand.connection_state is ConnectionState.FAILED


@pytest.mark.parametrize("failure", ["set_speed", "set_current"])
def test_sdk_adapter_fails_closed_when_coefficient_setter_is_rejected(
    failure: str,
) -> None:
    calls: list[str] = []
    wave = _Wave(calls, speed=0.1, current=0.1, fail_at=failure)
    hand = _sdk_hand(wave, calls)

    with pytest.raises(CommandRejectedError, match=failure):
        hand.connect()

    assert calls.count(failure) == 1
    assert "start" not in calls
    assert calls[-3:] == ["stop", "destroy", "disconnect:WAVE-L-1"]


def test_sdk_adapter_fails_closed_on_coefficient_readback_mismatch() -> None:
    calls: list[str] = []
    wave = _Wave(calls, speed=0.1, ignore_write_at="set_speed")
    hand = _sdk_hand(wave, calls)

    with pytest.raises(CommandRejectedError, match="verification failed"):
        hand.connect()

    assert calls.count("set_speed") == 1
    assert calls.count("get_speed") == 2
    assert "start" not in calls
    assert calls[-3:] == ["stop", "destroy", "disconnect:WAVE-L-1"]


def test_sdk_adapter_cleans_up_when_start_fails() -> None:
    calls: list[str] = []
    wave = _Wave(calls, start_result=False)
    hand = _sdk_hand(wave, calls)

    with pytest.raises(ConnectionError, match=r"start\(\) returned false"):
        hand.connect()

    assert calls == [
        "connect:WAVE-L-1:True",
        "get_speed",
        "get_current",
        "get_source",
        "get_mode",
        "get_enable",
        "start",
        "stop",
        "destroy",
        "disconnect:WAVE-L-1",
    ]
    assert hand.connection_state is ConnectionState.FAILED
    assert hand._wave is None
    assert hand._manager is None


def test_sdk_adapter_waits_past_non_hand_heartbeat() -> None:
    calls: list[str] = []
    wave = _Wave(calls)
    touch = SimpleNamespace(sn="TOUCH-1", hand_side="left", device_type="touch")
    left_hand = SimpleNamespace(sn="WAVE-L-1", hand_side="left", device_type="hand")

    class _Manager:
        polls = 0

        def get_all_devices(self) -> list[object]:
            self.polls += 1
            return [touch] if self.polls == 1 else [touch, left_hand]

        def connect(self, serial: str, skip_tactile: bool) -> _Wave:
            del serial, skip_tactile
            return wave

        def disconnect(self, serial: str) -> None:
            pass

    manager = _Manager()
    sdk = SimpleNamespace(
        SharpaWaveManager=SimpleNamespace(get_instance=lambda: manager),
        DeviceType=SimpleNamespace(HAND="hand"),
        HandSide=SimpleNamespace(LEFT="left", RIGHT="right"),
        ControlMode=SimpleNamespace(POSITION="position"),
        ControlSource=SimpleNamespace(SDK="sdk"),
    )
    hand = SharpaSdkHand(
        SharpaConfig(
            name="real",
            side=HandSide.LEFT,
            discovery_timeout_s=0.5,
        ),
        sdk_module=sdk,
    )
    hand.connect()
    assert manager.polls == 2
    assert hand.info.serial == "WAVE-L-1"
    hand.disconnect()


def test_sdk_joint_state_includes_velocity_and_effort() -> None:
    calls: list[str] = []
    wave = _Wave(calls)
    hand = SharpaSdkHand(SharpaConfig(name="real"), sdk_module=object())
    hand._wave = wave
    hand._lifecycle.begin_connect()
    hand._lifecycle.connected()

    state = hand.read_joint_state()

    assert state.position == (0.0,) * 22
    assert state.velocity == (0.1,) * 22
    assert state.effort == (0.2,) * 22


def test_sdk_adapter_reads_left_fingertip_tactile() -> None:
    calls: list[str] = []
    wave = _Wave(calls)
    device = SimpleNamespace(
        sn="WAVE-L-1",
        hand_side="left",
        device_type="hand",
        has_fingertip_tactile=lambda: True,
    )

    class _Manager:
        def get_all_devices(self) -> list[object]:
            return [device]

        def connect(self, serial: str, skip_tactile: bool) -> _Wave:
            calls.append(f"connect:{serial}:{skip_tactile}")
            return wave

        def disconnect(self, serial: str) -> None:
            calls.append(f"disconnect:{serial}")

    sdk = SimpleNamespace(
        SharpaWaveManager=SimpleNamespace(get_instance=lambda: _Manager()),
        DeviceType=SimpleNamespace(HAND="hand"),
        HandSide=SimpleNamespace(LEFT="left", RIGHT="right"),
        ControlMode=SimpleNamespace(POSITION="position"),
        ControlSource=SimpleNamespace(SDK="sdk"),
    )
    hand = SharpaSdkHand(SharpaConfig(name="real", side=HandSide.LEFT), sdk_module=sdk)
    hand.connect()

    frame = hand.read_tactile(SharpaFinger.THUMB, timeout_s=0.0)

    assert hand.tactile_available
    assert "connect:WAVE-L-1:False" in calls
    assert frame is not None
    # Field-verified: physical left thumb streams on SDK channel 9.
    assert frame.channel == 9
    assert "tactile:9:0.0" in calls
    assert frame.wrench == (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
    hand.disconnect()


def test_decode_sharpa_tactile_frame() -> None:
    payload = {
        "frame_id": 7,
        "channel": 9,
        "ts": 12.5,
        "content": {
            "RAW": bytes(240 * 320),
            "DEFORM": bytes(240 * 240),
            "F6": [1, 2, 3, 4, 5, 6],
            "CONTACT_POINT": [[10, 20, 0.9]],
        },
    }

    frame = decode_tactile_frame(payload, HandSide.LEFT, SharpaFinger.THUMB)

    assert frame.raw is not None
    assert (frame.raw.height, frame.raw.width) == (240, 320)
    assert frame.deformation is not None
    assert (frame.deformation.height, frame.deformation.width) == (240, 240)
    assert frame.wrench == (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
    assert frame.contacts == ((10.0, 20.0, 0.9),)


# --- Field-verified fingertip tactile channel mapping ---------------------
#
# Serial CD559038CD54 / Wave-L-01: the left hand's fingertip channels are
# wired in reverse. These intent-based tests pin the physical->channel
# contract so a future refactor cannot silently re-reverse it.

_LEFT_PHYSICAL_TO_CHANNEL = {
    SharpaFinger.THUMB: 9,
    SharpaFinger.INDEX: 8,
    SharpaFinger.MIDDLE: 7,
    SharpaFinger.RING: 6,
    SharpaFinger.PINKY: 5,
}
_RIGHT_PHYSICAL_TO_CHANNEL = {
    SharpaFinger.THUMB: 0,
    SharpaFinger.INDEX: 1,
    SharpaFinger.MIDDLE: 2,
    SharpaFinger.RING: 3,
    SharpaFinger.PINKY: 4,
}


@pytest.mark.parametrize(("finger", "channel"), list(_LEFT_PHYSICAL_TO_CHANNEL.items()))
def test_left_tactile_channel_is_reversed(finger: SharpaFinger, channel: int) -> None:
    assert tactile_channel(HandSide.LEFT, finger) == channel


@pytest.mark.parametrize(("finger", "channel"), list(_RIGHT_PHYSICAL_TO_CHANNEL.items()))
def test_right_tactile_channel_is_forward(finger: SharpaFinger, channel: int) -> None:
    assert tactile_channel(HandSide.RIGHT, finger) == channel


def test_left_tactile_channels_are_full_reverse_of_right() -> None:
    left = [tactile_channel(HandSide.LEFT, f) for f in SHARPA_FINGERS]
    right = [tactile_channel(HandSide.RIGHT, f) for f in SHARPA_FINGERS]
    assert left == list(reversed(range(5, 10)))
    assert right == list(range(5))
    # Same physical finger never shares a channel across hands.
    assert set(left).isdisjoint(right)


def test_frame_validation_binds_physical_finger_to_channel() -> None:
    # A left thumb frame must carry channel 9 (the corrected value); the old
    # reversed mapping put the thumb on channel 5.
    SharpaTactileFrame(
        side=HandSide.LEFT,
        finger=SharpaFinger.THUMB,
        channel=9,
        frame_index=0,
        timestamp_s=0.0,
    )
    with pytest.raises(ValueError, match="channel"):
        SharpaTactileFrame(
            side=HandSide.LEFT,
            finger=SharpaFinger.THUMB,
            channel=5,
            frame_index=0,
            timestamp_s=0.0,
        )


def test_mock_left_hand_reports_physical_channels() -> None:
    hand = create_hand(
        SharpaConfig(name="fake", side=HandSide.LEFT, enable_tactile=True),
        use_fake_hardware=True,
    )
    hand.connect()
    for finger, channel in _LEFT_PHYSICAL_TO_CHANNEL.items():
        frame = hand.read_tactile(finger, timeout_s=0.0)
        assert frame is not None
        assert frame.side is HandSide.LEFT
        assert frame.finger is finger
        assert frame.channel == channel
    hand.disconnect()


@pytest.mark.parametrize(
    ("side", "physical_to_channel"),
    [
        (HandSide.LEFT, _LEFT_PHYSICAL_TO_CHANNEL),
        (HandSide.RIGHT, _RIGHT_PHYSICAL_TO_CHANNEL),
    ],
)
def test_sdk_read_fetches_channel_for_each_physical_finger(
    side: HandSide,
    physical_to_channel: dict[SharpaFinger, int],
) -> None:
    calls: list[str] = []
    wave = _Wave(calls)
    hand = SharpaSdkHand(SharpaConfig(name="real"), sdk_module=object())
    hand._wave = wave
    hand._side = side
    hand._tactile_available = True
    hand._lifecycle.begin_connect()
    hand._lifecycle.connected()

    for finger, channel in physical_to_channel.items():
        frame = hand.read_tactile(finger, timeout_s=0.0)
        assert frame is not None
        assert frame.side is side
        assert frame.finger is finger
        assert frame.channel == channel
        assert f"tactile:{channel}:0.0" in calls
