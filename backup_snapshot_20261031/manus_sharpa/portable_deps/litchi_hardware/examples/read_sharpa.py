"""Read Sharpa Wave joint states through the litchi_hardware HAL.

Select one real hand by serial number:

    pixi run python examples/read_sharpa.py --serial WAVE-L-XXXX

Run without hardware:

    pixi run python examples/read_sharpa.py --fake --side left --count 5

The real SDK adapter selects SDK POSITION control and enables the hand while it
is connected. Keep the hand clear even though this example sends no commands.
"""

from __future__ import annotations

import argparse
import json
import sys
import time

from litchi_hardware import sharpa
from litchi_hardware.core import HandSide


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial", help="serial number of the hand to read")
    parser.add_argument(
        "--side",
        choices=("left", "right"),
        help="discover a hand on this side",
    )
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=10.0,
        help="maximum output rate (default: 10)",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=0,
        help="stop after this many samples; 0 runs until Ctrl+C",
    )
    parser.add_argument(
        "--discovery-timeout",
        type=float,
        default=10.0,
        help="device discovery timeout in seconds (default: 10)",
    )
    parser.add_argument(
        "--fake",
        action="store_true",
        help="use the deterministic fake hand",
    )
    args = parser.parse_args()
    if args.rate_hz <= 0:
        parser.error("--rate-hz must be positive")
    if args.count < 0:
        parser.error("--count must not be negative")
    if args.discovery_timeout <= 0:
        parser.error("--discovery-timeout must be positive")
    return args


def state_payload(state) -> dict[str, object]:
    payload: dict[str, object] = {
        "timestamp_s": state.timestamp_s,
        "schema": state.schema.id if state.schema else None,
        "unit": state.schema.unit if state.schema else "rad",
        "position": dict(zip(state.names, state.position, strict=True)),
    }
    if state.velocity:
        payload["velocity"] = dict(zip(state.names, state.velocity, strict=True))
    if state.effort:
        payload["effort"] = dict(zip(state.names, state.effort, strict=True))
    return payload


def main() -> int:
    args = parse_args()
    config = sharpa.SharpaConfig(
        name="sharpa-example",
        serial=args.serial,
        side=HandSide(args.side) if args.side else None,
        discovery_timeout_s=args.discovery_timeout,
    )
    hand = sharpa.create_hand(config, use_fake_hardware=args.fake)
    period_s = 1.0 / args.rate_hz
    emitted = 0

    if not args.fake:
        print(
            "Warning: connecting selects SDK POSITION control and enables the hand.",
            file=sys.stderr,
        )
    try:
        hand.connect()
        info = hand.info
        print(
            f"Connected to {info.vendor} {info.model}; "
            f"serial={info.serial or 'unknown'}, "
            f"side={info.metadata.get('side', 'unknown')}",
            file=sys.stderr,
        )
        while args.count == 0 or emitted < args.count:
            started_at = time.monotonic()
            state = hand.read_joint_state()
            print(json.dumps(state_payload(state), separators=(",", ":")))
            emitted += 1
            remaining = period_s - (time.monotonic() - started_at)
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        print("\nStopping Sharpa reader.", file=sys.stderr)
    finally:
        hand.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
