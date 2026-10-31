"""Read Linker Hand G20/O30I joint feedback without sending motion commands."""

from __future__ import annotations

import argparse
import time

from litchi_hardware import linker
from litchi_hardware.core.types import HandSide


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", choices=("G20", "O30I"), default="O30I")
    parser.add_argument("--side", choices=("left", "right"), default="right")
    parser.add_argument(
        "--transport",
        choices=("socketcan", "libcanbus"),
        default="socketcan",
    )
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--frame-id", type=lambda value: int(value, 0))
    parser.add_argument("--fake", action="store_true")
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument("--samples", type=int, default=0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.rate <= 0.0 or args.samples < 0:
        raise SystemExit("--rate must be positive and --samples non-negative")
    config = linker.LinkerConfig(
        name=f"{args.side}_{args.model.lower()}",
        model=args.model,
        side=HandSide(args.side),
        transport=args.transport,
        channel=args.channel,
        frame_id=args.frame_id,
        verify_identity=not args.fake,
    )
    hand = linker.create_hand(config, use_fake_hardware=args.fake)
    hand.connect()
    print(hand.info)
    count = 0
    try:
        while args.samples == 0 or count < args.samples:
            state = hand.read_joint_state()
            values = " ".join(
                f"{name}={value:.4f}"
                for name, value in zip(state.names, state.position, strict=True)
            )
            print(values)
            count += 1
            time.sleep(1.0 / args.rate)
    except KeyboardInterrupt:
        pass
    finally:
        hand.disconnect()


if __name__ == "__main__":
    main()
