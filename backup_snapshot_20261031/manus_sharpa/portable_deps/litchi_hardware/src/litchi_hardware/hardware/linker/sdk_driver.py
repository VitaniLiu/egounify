"""HAL adapter over the bundled Linker Hand G20 and O30I SDKs."""

from __future__ import annotations

import importlib
import time
from collections.abc import Callable, Sequence
from typing import Any

from litchi_hardware.core.errors import (
    CommandRejectedError,
    ConnectionError,
    DependencyUnavailableError,
)
from litchi_hardware.core.lifecycle import ConnectionState
from litchi_hardware.core.types import JointState, TactileFrame

from .base import LinkerHandBase
from .config import LinkerConfig, LinkerModel, LinkerTransport
from .transport import (
    CanInterfaceManager,
    make_libcanbus_communication,
    resolve_library_path,
)
from .vendor import bundled_libcanbus_path, load_g20_controller_class, load_o30i_sdk

ControllerFactory = Callable[[LinkerConfig], Any]
_FINGERS = ("thumb", "index", "middle", "ring", "little")


class LinkerSdkHand(LinkerHandBase):
    def __init__(
        self,
        config: LinkerConfig,
        *,
        controller_factory: ControllerFactory | None = None,
    ) -> None:
        super().__init__(config)
        self._controller_factory = controller_factory
        self._controller: Any | None = None
        self._last_position: tuple[float, ...] | None = None
        self._last_timestamp_s: float | None = None

    @property
    def transport_name(self) -> str:
        return self.config.transport.value

    def _create_g20_controller(self) -> Any:
        controller_class = load_g20_controller_class()
        module = importlib.import_module(controller_class.__module__)
        module.OpenCan = CanInterfaceManager
        can_id = 0x28 if self.config.side.value == "left" else 0x27
        return controller_class(
            can_channel=self.config.channel,
            baudrate=self.config.bitrate,
            can_id=can_id,
            yaml=self.config,
        )

    def _create_o30i_controller(self) -> Any:
        sdk = load_o30i_sdk()
        if self.config.transport is LinkerTransport.LIBCANBUS:
            library = resolve_library_path(self.config.libcanbus_path, bundled_libcanbus_path())
            sdk.CANFDCommunication = make_libcanbus_communication(sdk, library)
        channel: str | int
        if self.config.transport is LinkerTransport.SOCKETCAN:
            channel = self.config.channel
        else:
            channel = self.config.canfd_channel
        return sdk.LinkerHandO30IController(
            hand_type=self.config.side.value,
            canfd_device=self.config.canfd_device,
            frame_id=self.config.frame_id,
            comm_type=self.config.transport.value,
            channel=channel,
            bitrate=self.config.bitrate,
            dbitrate=self.config.data_bitrate,
            auto_setup=self.config.auto_setup_can,
        )

    def _create_controller(self) -> Any:
        if self._controller_factory is not None:
            return self._controller_factory(self.config)
        if self.config.model is LinkerModel.G20:
            return self._create_g20_controller()
        return self._create_o30i_controller()

    def connect(self) -> None:
        if self.is_connected:
            return
        if self.connection_state is ConnectionState.FAILED:
            self._lifecycle.reset()
        self._lifecycle.begin_connect()
        try:
            self._controller = self._create_controller()
            if self.config.model is LinkerModel.O30I:
                if not bool(getattr(self._controller, "is_connected", False)):
                    raise ConnectionError("O30I CAN-FD transport did not connect")
                info = dict(self._controller.get_device_info())
                self._serial = info.get("设备唯一标识") or None
                self._firmware_version = (
                    info.get("app版本") or info.get("控制板版本") or info.get("协议版本")
                )
                if not self._controller.setup():
                    raise ConnectionError("O30I joint setup was rejected")
            else:
                serial = self._controller.get_serial_number()
                self._serial = None if serial in (None, "", "-1") else str(serial)
                version = self._controller.get_version()
                self._firmware_version = str(version) if version else None
            if self.config.verify_identity and not self._serial:
                raise ConnectionError(f"{self.config.model.value} did not return a device identity")
            if self.config.initial_speed is not None:
                self._set_initial_speed(self.config.initial_speed)
            self._last_position = None
            self._last_timestamp_s = None
            self._stopped = False
            self._lifecycle.connected()
        except Exception as exc:
            self._cleanup()
            self._lifecycle.failed()
            if isinstance(
                exc,
                (ConnectionError, DependencyUnavailableError, CommandRejectedError),
            ):
                raise
            if isinstance(exc, (ImportError, ModuleNotFoundError)):
                raise DependencyUnavailableError(
                    "Linker Hand runtime dependencies are unavailable; install "
                    "litchi-hardware[linker]"
                ) from exc
            raise ConnectionError(f"failed to connect Linker Hand: {exc}") from exc

    def _set_initial_speed(self, speed: tuple[int, ...]) -> None:
        assert self._controller is not None
        values = list(speed)
        if self.config.model is LinkerModel.O30I:
            accepted = self._controller.set_target_velocity(values)
            if accepted is False:
                raise CommandRejectedError("O30I initial speed was rejected")
        else:
            self._controller.set_speed(values)

    def _cleanup(self) -> None:
        if self._controller is None:
            return
        try:
            if self.config.model is LinkerModel.O30I:
                self._controller.close()
            else:
                self._controller.close_can_interface()
        except Exception:
            pass
        self._controller = None

    def disconnect(self) -> None:
        if self.connection_state is ConnectionState.DISCONNECTED:
            return
        self._cleanup()
        self._stopped = True
        self._lifecycle.disconnected()

    def _radians_to_sdk(self, positions: Sequence[float]) -> list[int]:
        return [
            round(gain * value + offset)
            for value, gain, offset in zip(
                positions,
                self.config.position_gains,
                self.config.position_offsets,
                strict=True,
            )
        ]

    def _sdk_to_radians(self, positions: Sequence[float]) -> tuple[float, ...]:
        return tuple(
            (float(value) - offset) / gain
            for value, gain, offset in zip(
                positions,
                self.config.position_gains,
                self.config.position_offsets,
                strict=True,
            )
        )

    def _write_joint_position(self, positions: tuple[float, ...]) -> None:
        if self._controller is None:
            raise ConnectionError("Linker Hand controller is unavailable")
        values = self._radians_to_sdk(positions)
        if self.config.model is LinkerModel.O30I:
            accepted = self._controller.set_target_position(values)
        else:
            accepted = self._controller.set_joint_positions(values)
        if accepted is False:
            raise CommandRejectedError(
                f"{self.config.model.value} rejected the joint position command"
            )

    def read_joint_state(self) -> JointState:
        if not self.is_connected or self._controller is None:
            raise ConnectionError("Linker Hand is not connected")
        if self.config.model is LinkerModel.O30I:
            raw = self._controller.get_current_position()
        else:
            raw = self._controller.get_current_status()
        if raw is None or len(raw) != self.joint_schema.size:
            count = 0 if raw is None else len(raw)
            raise ConnectionError(
                f"{self.config.model.value} returned {count} joint positions; "
                f"expected {self.joint_schema.size}"
            )
        if any(float(value) < 0.0 or float(value) > 255.0 for value in raw):
            raise ConnectionError(f"{self.config.model.value} returned invalid joint feedback")
        positions = self._sdk_to_radians(raw)
        timestamp_s = time.time()
        velocity: tuple[float, ...] = ()
        if self._last_position is not None and self._last_timestamp_s is not None:
            elapsed = timestamp_s - self._last_timestamp_s
            if elapsed > 0.0:
                velocity = tuple(
                    (current - previous) / elapsed
                    for current, previous in zip(positions, self._last_position, strict=True)
                )
        self._last_position = positions
        self._last_timestamp_s = timestamp_s
        return JointState(
            self.joint_schema.names,
            positions,
            velocity,
            timestamp_s=timestamp_s,
            schema=self.joint_schema,
        )

    def read_tactile(self, finger: str = "thumb", timeout_s: float | None = 0.0) -> TactileFrame:
        del timeout_s
        if not self.is_connected or self._controller is None:
            raise ConnectionError("Linker Hand is not connected")
        if not self.config.enable_tactile:
            raise ConnectionError("Linker Hand tactile is disabled")
        if finger not in _FINGERS:
            raise ValueError(f"unsupported Linker Hand finger: {finger}")
        if self.config.model is LinkerModel.O30I:
            matrix = self._controller.read_tactile_matrix(_FINGERS.index(finger))
        else:
            reader = getattr(self._controller, f"get_{finger}_matrix_touch")
            matrix = reader()
        if matrix is None:
            raise ConnectionError(f"no tactile frame available for {finger}")
        rows = matrix.tolist() if hasattr(matrix, "tolist") else matrix
        shape = (len(rows), len(rows[0]) if rows else 0)
        values = tuple(float(value) for row in rows for value in row)
        return TactileFrame(
            values,
            shape,
            time.time(),
            f"linker/{self.config.side.value}/{finger}",
        )

    def stop(self) -> None:
        if self._controller is None or self._stopped:
            return
        try:
            if self.config.model is LinkerModel.O30I:
                accepted = self._controller.set_joint_enable([0] * self.joint_schema.size)
                if accepted is False:
                    raise CommandRejectedError("O30I disable command was rejected")
            else:
                self._controller.close_can_interface()
        finally:
            self._stopped = True
