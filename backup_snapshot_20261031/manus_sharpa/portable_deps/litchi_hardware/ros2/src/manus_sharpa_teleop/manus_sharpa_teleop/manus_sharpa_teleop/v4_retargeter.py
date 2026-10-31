"""Client for Sharpa's V4 Manus retargeting optimizer.

The vendor optimizer is only available as a CPython 3.10 extension.  It runs
in a small sidecar process so the ROS/Python ABI remains independent.
"""

from __future__ import annotations

import json
import os
import shutil
import socket
import subprocess
import tempfile
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import IO, Any

from ament_index_python.packages import get_package_share_directory

from litchi_hardware.core.types import DeviceAction, HandSide, HandTrackingFrame
from litchi_hardware.hardware.manus.schema import MANUS_SHARPA_25_SCHEMA
from litchi_hardware.hardware.sharpa.schema import SHARPA_WAVE_JOINT_SCHEMA


def _frame_to_v4_keypoints(frame: HandTrackingFrame) -> list[list[float]]:
    """Convert the domain xyzw pose to V4's [x, y, z, qw, qx, qy, qz]."""

    return [
        [
            *pose.position,
            pose.orientation[3],
            pose.orientation[0],
            pose.orientation[1],
            pose.orientation[2],
        ]
        for pose in frame.keypoints
    ]


def _default_worker_python() -> Path:
    override = os.environ.get("LITCHI_RETARGET_V4_PYTHON")
    if override:
        return Path(override).expanduser().resolve()
    project_root = os.environ.get("PIXI_PROJECT_ROOT")
    if project_root:
        candidate = (
            Path(project_root) / ".pixi" / "envs" / "retarget-v4" / "bin" / "python"
        )
        if candidate.is_file():
            return candidate
    # Editable/source checkout fallback for direct `ros2 run` after sourcing
    # the workspace, where PIXI_PROJECT_ROOT is not necessarily exported.
    for parent in Path(__file__).resolve().parents:
        if (parent / "pixi.toml").is_file():
            candidate = (
                parent / ".pixi" / "envs" / "retarget-v4" / "bin" / "python"
            )
            if candidate.is_file():
                return candidate
            break
    discovered = shutil.which("python3.10")
    if discovered:
        return Path(discovered).resolve()
    raise RuntimeError(
        "Sharpa V4 retargeting needs Python 3.10; run "
        "'pixi install -e retarget-v4' or set LITCHI_RETARGET_V4_PYTHON"
    )


@dataclass(frozen=True, slots=True)
class V4RetargetingConfig:
    """Process and filtering settings for the vendor V4 optimizer."""

    filter_alpha: float = 1.0
    worker_python: str | None = None
    startup_timeout_s: float = 30.0
    response_timeout_s: float = 2.0

    def __post_init__(self) -> None:
        if not 0.0 < self.filter_alpha <= 1.0:
            raise ValueError("filter_alpha must be in (0, 1]")
        if self.startup_timeout_s <= 0 or self.response_timeout_s <= 0:
            raise ValueError("V4 worker timeouts must be positive")


class SharpaV4Retargeter:
    """Run the exact V4 URDF/CasADi optimizer in its required Python ABI."""

    def __init__(self, config: V4RetargetingConfig | None = None) -> None:
        self.config = config or V4RetargetingConfig()
        self._temporary = tempfile.TemporaryDirectory(prefix="litchi-retarget-v4-")
        self._socket_path = Path(self._temporary.name) / "optimizer.sock"
        self._socket: socket.socket | None = None
        self._reader: IO[bytes] | None = None
        self._process: subprocess.Popen[bytes] | None = None
        self._lock = threading.Lock()
        try:
            self._start()
        except Exception:
            self.close()
            raise

    def _start(self) -> None:
        optimizer_root = (
            Path(get_package_share_directory("manus_sharpa_teleop"))
            / "retarget_v4"
        )
        urdf_root = (
            Path(get_package_share_directory("sharpa_wave_description"))
            / "urdf"
        )
        optimizer = optimizer_root / "include" / "hand_retargeting_optimizer.so"
        kinematics = optimizer_root / "include" / "hand_kinematic_casadi.so"
        if not optimizer.is_file() or not kinematics.is_file():
            raise RuntimeError(
                "Sharpa V4 optimizer is missing from the ROS package; run "
                "'pixi run stage-retarget-v4' and rebuild the ROS workspace"
            )
        for side in ("left", "right"):
            model = f"{side}_sharpa_wave"
            if not (urdf_root / model / f"{model}.urdf").is_file():
                raise RuntimeError(
                    f"Sharpa Wave description is missing the {side} URDF"
                )
        worker = Path(__file__).with_name("v4_worker.py")
        python = (
            Path(self.config.worker_python).expanduser().resolve()
            if self.config.worker_python
            else _default_worker_python()
        )
        if not python.is_file():
            raise RuntimeError(f"V4 worker Python does not exist: {python}")
        worker_environment = os.environ.copy()
        worker_environment.pop("PYTHONPATH", None)
        worker_environment.pop("PYTHONHOME", None)
        worker_environment["PYTHONNOUSERSITE"] = "1"
        self._process = subprocess.Popen(
            [
                str(python),
                str(worker),
                "--socket",
                str(self._socket_path),
                "--optimizer-dir",
                str(optimizer_root),
                "--urdf-root",
                str(urdf_root),
                "--filter-alpha",
                str(self.config.filter_alpha),
            ],
            env=worker_environment,
            start_new_session=True,
        )

        deadline = time.monotonic() + self.config.startup_timeout_s
        last_error: OSError | None = None
        while time.monotonic() < deadline:
            if self._process.poll() is not None:
                raise RuntimeError(
                    f"Sharpa V4 worker exited with code {self._process.returncode}"
                )
            connection = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            try:
                connection.connect(str(self._socket_path))
            except OSError as exc:
                last_error = exc
                connection.close()
                time.sleep(0.05)
                continue
            connection.settimeout(self.config.response_timeout_s)
            self._socket = connection
            self._reader = connection.makefile("rb")
            response = self._request({"operation": "ping"})
            if response.get("status") != "ok":
                raise RuntimeError(f"invalid V4 worker handshake: {response}")
            return
        raise RuntimeError(
            f"timed out starting Sharpa V4 worker: {last_error or 'socket unavailable'}"
        )

    def _request(self, request: dict[str, Any]) -> dict[str, Any]:
        if self._socket is None or self._reader is None:
            raise RuntimeError("Sharpa V4 worker is not connected")
        payload = json.dumps(request, separators=(",", ":")).encode() + b"\n"
        self._socket.sendall(payload)
        line = self._reader.readline()
        if not line:
            code = self._process.poll() if self._process is not None else None
            raise RuntimeError(f"Sharpa V4 worker disconnected (exit code {code})")
        response = json.loads(line)
        if response.get("status") == "error":
            raise RuntimeError(f"Sharpa V4 optimization failed: {response['message']}")
        return response

    def reset(self, side: HandSide | None = None) -> None:
        with self._lock:
            self._request(
                {
                    "operation": "reset",
                    "side": side.value if side is not None else None,
                }
            )

    def retarget(self, frame: HandTrackingFrame) -> DeviceAction:
        if frame.schema != MANUS_SHARPA_25_SCHEMA:
            raise ValueError(
                "Manus-to-Sharpa V4 retargeting requires skeleton schema "
                f"{MANUS_SHARPA_25_SCHEMA.id!r}, received {frame.schema.id!r}"
            )
        # V4's _protobuf_to_numpy contract is [x, y, z, qw, qx, qy, qz].
        keypoints = _frame_to_v4_keypoints(frame)
        with self._lock:
            response = self._request(
                {
                    "operation": "retarget",
                    "side": frame.side.value,
                    "sequence": frame.sequence,
                    "keypoints": keypoints,
                }
            )
        positions = tuple(float(value) for value in response["joint_position"])
        if len(positions) != SHARPA_WAVE_JOINT_SCHEMA.size:
            raise RuntimeError(
                f"V4 optimizer returned {len(positions)} joints instead of 22"
            )
        return DeviceAction(
            {
                "joint_position": positions,
                "joint_names": SHARPA_WAVE_JOINT_SCHEMA.names,
                "side": frame.side,
                "source_sequence": frame.sequence,
            },
            timestamp_s=frame.timestamp_s,
        )

    def close(self) -> None:
        if self._socket is not None and self._reader is not None:
            try:
                with self._lock:
                    self._request({"operation": "shutdown"})
            except (OSError, RuntimeError, ValueError):
                pass
        if self._reader is not None:
            self._reader.close()
            self._reader = None
        if self._socket is not None:
            self._socket.close()
            self._socket = None
        if self._process is not None:
            try:
                self._process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self._process.terminate()
                try:
                    self._process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    self._process.kill()
                    self._process.wait()
            self._process = None
        self._temporary.cleanup()

    def __enter__(self) -> SharpaV4Retargeter:
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
