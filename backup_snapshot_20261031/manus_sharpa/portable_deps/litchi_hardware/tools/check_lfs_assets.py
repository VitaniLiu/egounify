#!/usr/bin/env python3
"""Fail when a checkout contains Git LFS pointer files instead of SDK assets."""

from __future__ import annotations

from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
ASSET_ROOTS = (
    REPOSITORY_ROOT / "src" / "litchi_hardware" / "hardware",
    REPOSITORY_ROOT / "ros2" / "src",
)
LFS_POINTER_HEADER = b"version https://git-lfs.github.com/spec/v1"


def find_lfs_pointers() -> tuple[Path, ...]:
    """Return asset paths that were not hydrated by Git LFS."""

    pointers = []
    for root in ASSET_ROOTS:
        for path in root.rglob("*"):
            if not path.is_file():
                continue
            with path.open("rb") as stream:
                if stream.readline().rstrip() == LFS_POINTER_HEADER:
                    pointers.append(path.relative_to(REPOSITORY_ROOT))
    return tuple(pointers)


def main() -> int:
    pointers = find_lfs_pointers()
    if not pointers:
        print("Git LFS assets are hydrated.")
        return 0

    print("Git LFS assets are missing; pointer files found:")
    for path in pointers:
        print(f"  {path}")
    print("Run `git lfs pull` before testing or building wheels.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())

