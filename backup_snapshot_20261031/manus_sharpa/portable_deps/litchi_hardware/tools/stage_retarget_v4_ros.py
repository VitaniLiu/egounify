#!/usr/bin/env python3
"""Stage Sharpa's proprietary V4 optimizer into the ROS workspace."""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

from elf_runtime_paths import remove_runtime_paths, runtime_paths

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = (
    REPOSITORY_ROOT
    / "reference"
    / "sharpa-manus-sdk"
    / "retargeting_alg_release_V4.0"
)
ROS_ROOT = REPOSITORY_ROOT / "ros2" / "src" / "manus_sharpa_teleop"
OPTIMIZER_TARGET = (
    ROS_ROOT / "manus_sharpa_teleop" / "vendor" / "retarget_v4"
)
DESCRIPTION_TARGET = ROS_ROOT / "sharpa_wave_description" / "urdf"


def _copy_file(source: Path, destination: Path) -> None:
    if not source.is_file():
        raise FileNotFoundError(source)
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--clean",
        action="store_true",
        help="Replace only the staged V4 optimizer and URDF directories.",
    )
    args = parser.parse_args()
    if args.clean:
        for target in (OPTIMIZER_TARGET, DESCRIPTION_TARGET):
            if target.exists():
                shutil.rmtree(target)

    for name in ("hand_kinematic_casadi.so", "hand_retargeting_optimizer.so"):
        destination = OPTIMIZER_TARGET / "include" / name
        _copy_file(
            SOURCE_ROOT / "include" / name,
            destination,
        )
        # The vendor binaries contain an RPATH to the vendor build machine.
        # They only declare libc as a shared dependency, so no private search
        # path is required after staging.
        remove_runtime_paths(destination)
        if runtime_paths(destination):
            raise RuntimeError(f"failed to remove vendor RPATH from {destination}")
    _copy_file(SOURCE_ROOT / "VERSION.json", OPTIMIZER_TARGET / "VERSION.json")
    _copy_file(
        REPOSITORY_ROOT / "reference" / "sharpa-manus-sdk" / "License",
        OPTIMIZER_TARGET / "LICENSE.vendor",
    )
    _copy_file(
        REPOSITORY_ROOT / "reference" / "sharpa-manus-sdk" / "NOTICE.txt",
        OPTIMIZER_TARGET / "NOTICE.vendor",
    )
    for side in ("left", "right"):
        model = f"{side}_sharpa_wave"
        _copy_file(
            SOURCE_ROOT / "urdf" / model / f"{model}.urdf",
            DESCRIPTION_TARGET / model / f"{model}.urdf",
        )
    print(f"Staged V4 optimizer in {OPTIMIZER_TARGET}")
    print(f"Staged Wave URDFs in {DESCRIPTION_TARGET}")


if __name__ == "__main__":
    main()
