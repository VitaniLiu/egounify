"""Python 3.10 sidecar for the proprietary Sharpa V4 optimizer."""

from __future__ import annotations

import argparse
import json
import os
import socket
import sys
import traceback
from pathlib import Path
from typing import Any


def _send(stream: Any, message: dict[str, Any]) -> None:
    stream.write(json.dumps(message, separators=(",", ":")).encode() + b"\n")
    stream.flush()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--socket", required=True)
    parser.add_argument("--optimizer-dir", required=True)
    parser.add_argument("--urdf-root", required=True)
    parser.add_argument("--filter-alpha", required=True, type=float)
    args = parser.parse_args()

    optimizer_dir = Path(args.optimizer_dir).resolve()
    urdf_root = Path(args.urdf_root).resolve()
    os.chdir(optimizer_dir)
    sys.path.insert(0, str(optimizer_dir / "include"))
    import numpy as np
    from hand_retargeting_optimizer import (  # type: ignore[import-not-found]
        HandKinematicCasadi,
        HandRetargetingOptimizer,
        JointAngleFilter,
        get_wave_l_config,
        get_wave_r_config,
    )

    models = {}
    for side, config_factory in (
        ("left", get_wave_l_config),
        ("right", get_wave_r_config),
    ):
        model_name = f"{side}_sharpa_wave"
        config = config_factory()
        config["urdf_path"] = str(
            urdf_root / model_name / f"{model_name}.urdf"
        )
        models[side] = HandKinematicCasadi(**config)
    optimizers = {
        side: HandRetargetingOptimizer(models[side], side)
        for side in ("left", "right")
    }
    filters = {
        side: JointAngleFilter(22, alpha=args.filter_alpha)
        for side in ("left", "right")
    }

    socket_path = Path(args.socket)
    socket_path.unlink(missing_ok=True)
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(str(socket_path))
    server.listen(1)
    try:
        connection, _ = server.accept()
        with connection, connection.makefile("rwb") as stream:
            for line in stream:
                try:
                    request = json.loads(line)
                    operation = request.get("operation")
                    if operation == "ping":
                        _send(stream, {"status": "ok", "implementation": "Sharpa V4.0"})
                    elif operation == "shutdown":
                        _send(stream, {"status": "ok"})
                        return
                    elif operation == "reset":
                        requested_side = request.get("side")
                        sides = (requested_side,) if requested_side else ("left", "right")
                        for side in sides:
                            optimizers[side].reset_optimizer()
                            filters[side] = JointAngleFilter(
                                22, alpha=args.filter_alpha
                            )
                        _send(stream, {"status": "ok"})
                    elif operation == "retarget":
                        side = request["side"]
                        keypoints = np.asarray(request["keypoints"], dtype=np.float64)
                        if keypoints.shape != (25, 7):
                            raise ValueError(
                                f"expected keypoints shape (25, 7), received {keypoints.shape}"
                            )
                        joint_angles = optimizers[side].optimize_with_casadi(
                            raw_keypoints=keypoints
                        )
                        filtered = filters[side].filter_joint_angles(joint_angles)
                        _send(
                            stream,
                            {
                                "status": "ok",
                                "sequence": request.get("sequence"),
                                "joint_position": np.asarray(filtered).tolist(),
                            },
                        )
                    else:
                        raise ValueError(f"unknown operation: {operation!r}")
                except Exception as exc:
                    traceback.print_exc()
                    _send(stream, {"status": "error", "message": str(exc)})
    finally:
        server.close()
        socket_path.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
