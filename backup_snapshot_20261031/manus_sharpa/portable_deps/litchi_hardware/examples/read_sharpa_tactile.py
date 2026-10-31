#!/usr/bin/env python3
"""Read five-finger Sharpa tactile frames through the ROS-independent HAL.

Real left hand:

    pixi run python examples/read_sharpa_tactile.py \
      --serial CD559038CD54 --side left

The real SDK adapter selects POSITION control and enables the hand while
connected. Keep the hand clear even though this example sends no commands.
"""

from __future__ import annotations

import argparse
import json
import sys

from litchi_hardware import sharpa
from litchi_hardware.core import HandSide


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial", help="serial number of the hand")
    parser.add_argument(
        "--side", choices=("left", "right"), default="left"
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=1.0,
        help="per-finger frame timeout in seconds",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=0,
        help="stop after this many five-finger sweeps; 0 runs until Ctrl+C",
    )
    parser.add_argument("--fake", action="store_true")
    args = parser.parse_args()
    if args.timeout < 0:
        parser.error("--timeout must not be negative")
    if args.count < 0:
        parser.error("--count must not be negative")
    return args


def frame_summary(frame: sharpa.SharpaTactileFrame) -> dict[str, object]:
    return {
        "side": frame.side.value,
        "finger": frame.finger.value,
        "channel": frame.channel,
        "frame_index": frame.frame_index,
        "timestamp_s": frame.timestamp_s,
        "raw_shape": (
            [frame.raw.height, frame.raw.width] if frame.raw else None
        ),
        "deformation_shape": (
            [frame.deformation.height, frame.deformation.width]
            if frame.deformation
            else None
        ),
        "wrench": frame.wrench,
        "contacts": frame.contacts,
    }


def main() -> int:
    args = parse_args()
    config = sharpa.SharpaConfig(
        name="sharpa-tactile-example",
        serial=args.serial,
        side=HandSide(args.side),
        enable_tactile=True,
    )
    hand = sharpa.create_hand(config, use_fake_hardware=args.fake)
    if not args.fake:
        print(
            "Warning: connecting selects SDK POSITION control and enables the hand.",
            file=sys.stderr,
        )
    sweeps = 0
    try:
        hand.connect()
        if not getattr(hand, "tactile_available", False):
            raise RuntimeError("selected Sharpa hand has no fingertip tactile")
        while args.count == 0 or sweeps < args.count:
            for finger in sharpa.SHARPA_FINGERS:
                frame = hand.read_tactile(finger, timeout_s=args.timeout)
                if frame is not None:
                    print(
                        json.dumps(
                            frame_summary(frame), separators=(",", ":")
                        )
                    )
            sweeps += 1
    except KeyboardInterrupt:
        print("\nStopping Sharpa tactile reader.", file=sys.stderr)
    finally:
        hand.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
