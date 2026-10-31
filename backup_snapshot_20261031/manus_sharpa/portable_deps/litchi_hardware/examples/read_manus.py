#!/usr/bin/env python3
"""Read MANUS hand-tracking frames through the litchi_hardware HAL.

Real hardware, using the calibration files created in this repository:

    pixi run python examples/read_manus.py --side both

Run without hardware:

    pixi run python examples/read_manus.py --fake --side both --count 5
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

from litchi_hardware import manus
from litchi_hardware.core import HandSide

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CALIBRATION_DIRECTORY = REPOSITORY_ROOT / "calibration" / "metaglove"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--backend",
        choices=("native", "zmq"),
        default="native",
        help="MANUS transport backend (default: native)",
    )
    parser.add_argument(
        "--side",
        choices=("left", "right", "both"),
        default="left",
        help="hand side to read (default: left)",
    )
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=10.0,
        help="maximum output rate (default: 10)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.5,
        help="frame read timeout in seconds (default: 0.5)",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=0,
        help="stop after this many frames; 0 runs until Ctrl+C",
    )
    parser.add_argument(
        "--calibration-directory",
        default=(
            str(DEFAULT_CALIBRATION_DIRECTORY)
            if DEFAULT_CALIBRATION_DIRECTORY.is_dir()
            else None
        ),
        help="directory containing Calibration_left.mcal and Calibration_right.mcal",
    )
    parser.add_argument(
        "--loopback-only",
        action="store_true",
        help="limit native MANUS host discovery to loopback",
    )
    parser.add_argument(
        "--endpoint",
        default="tcp://127.0.0.1:2044",
        help="ZMQ endpoint when --backend=zmq",
    )
    parser.add_argument(
        "--external-client",
        action="store_true",
        help="do not start the bundled ZMQ client",
    )
    parser.add_argument(
        "--fake",
        action="store_true",
        help="use the deterministic fake tracker",
    )
    args = parser.parse_args()
    if args.rate_hz <= 0:
        parser.error("--rate-hz must be positive")
    if args.timeout < 0:
        parser.error("--timeout must not be negative")
    if args.count < 0:
        parser.error("--count must not be negative")
    return args


def make_tracker(args: argparse.Namespace):
    if args.backend == "native":
        config = manus.ManusNativeConfig(
            name="manus-example",
            frame_timeout_s=args.timeout,
            calibration_directory=args.calibration_directory,
            native_loopback_only=args.loopback_only,
        )
    else:
        config = manus.ManusZmqConfig(
            name="manus-example",
            frame_timeout_s=args.timeout,
            endpoint=args.endpoint,
            manage_client=not args.external_client,
        )
    return manus.create_tracker(config, use_fake_hardware=args.fake)


def frame_payload(frame) -> dict[str, object]:
    return {
        "side": frame.side.value,
        "sequence": frame.sequence,
        "timestamp_s": frame.timestamp_s,
        "frame_id": frame.frame_id,
        "schema": frame.schema.id,
        "keypoint_names": frame.schema.keypoint_names,
        "positions_m": [pose.position for pose in frame.keypoints],
        "orientations_xyzw": [pose.orientation for pose in frame.keypoints],
    }


def main() -> int:
    args = parse_args()
    sides = (
        (HandSide.LEFT, HandSide.RIGHT)
        if args.side == "both"
        else (HandSide(args.side),)
    )
    tracker = make_tracker(args)
    emitted = 0
    period_s = 1.0 / args.rate_hz

    try:
        tracker.connect()
        info = tracker.info
        print(
            f"Connected to {info.vendor} {info.model} via "
            f"{info.metadata.get('transport', 'unknown')}",
            file=sys.stderr,
        )
        while args.count == 0 or emitted < args.count:
            started_at = time.monotonic()
            for side in sides:
                if args.count and emitted >= args.count:
                    break
                try:
                    frame = tracker.read_frame(side, timeout_s=args.timeout)
                except TimeoutError as exc:
                    print(exc, file=sys.stderr)
                    continue
                print(json.dumps(frame_payload(frame), separators=(",", ":")))
                emitted += 1
            remaining = period_s - (time.monotonic() - started_at)
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        print("\nStopping MANUS reader.", file=sys.stderr)
    finally:
        tracker.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
