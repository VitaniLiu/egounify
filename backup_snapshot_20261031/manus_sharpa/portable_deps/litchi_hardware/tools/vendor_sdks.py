#!/usr/bin/env python3
"""Stage proprietary hardware SDK assets inside the Python package.

This is an internal distribution tool. The resulting files and wheels must
only be shared with recipients covered by the applicable vendor agreements.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import shlex
import shutil
import subprocess
import tarfile
import tempfile
from pathlib import Path

from elf_runtime_paths import assert_relocatable_runtime_paths, validate_elf_tree
from native_build import clean_host_build_environment, resolve_host_tool

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
REFERENCE_ROOT = REPOSITORY_ROOT / "reference"
HARDWARE_ROOT = REPOSITORY_ROOT / "src" / "litchi_hardware" / "hardware"
MANUS_VENDOR_ROOT = HARDWARE_ROOT / "manus" / "_vendor"
SHARPA_VENDOR_ROOT = HARDWARE_ROOT / "sharpa" / "_vendor"
LINKER_VENDOR_ROOT = HARDWARE_ROOT / "linker" / "_vendor"


def _copy_file(source: Path, destination: Path, *, executable: bool = False) -> None:
    if not source.is_file():
        raise FileNotFoundError(source)
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)
    if executable:
        destination.chmod(destination.stat().st_mode | 0o111)


def _copy_tree(source: Path, destination: Path) -> None:
    if not source.is_dir():
        raise FileNotFoundError(source)
    shutil.copytree(source, destination, dirs_exist_ok=True)


def _copy_python_tree(source: Path, destination: Path) -> None:
    """Copy vendor Python sources without interpreter-generated caches."""
    if not source.is_dir():
        raise FileNotFoundError(source)
    shutil.copytree(
        source,
        destination,
        dirs_exist_ok=True,
        ignore=shutil.ignore_patterns("__pycache__", "*.pyc", "*.pyo"),
    )


def _stage_sharpa() -> None:
    source = REFERENCE_ROOT / "sharpa_extracted" / "opt" / "sharpa-wave-sdk"
    target = SHARPA_VENDOR_ROOT
    _copy_tree(source / "include", target / "include")
    _copy_tree(source / "lib", target / "lib")
    _copy_tree(source / "config", target / "config")
    _copy_tree(source / "python" / "sharpa", target / "python" / "sharpa")
    _copy_file(source / "pilot_sdk", target / "pilot_sdk", executable=True)
    for name in ("README.md", "VERSION", "sdk-release-info.json"):
        _copy_file(source / name, target / name)


def _build_manus_client(target: Path) -> None:
    client = REFERENCE_ROOT / "sharpa-manus-sdk" / "client"
    protobuf = client / "third_party" / "protobuf"
    with tempfile.TemporaryDirectory(prefix="litchi-manus-client-") as temporary:
        build = Path(temporary)
        compiler = resolve_host_tool("g++", "LITCHI_VENDOR_CXX")
        strip = resolve_host_tool("strip", "LITCHI_VENDOR_STRIP")
        pkg_config = resolve_host_tool("pkg-config", "LITCHI_VENDOR_PKG_CONFIG")
        build_environment = clean_host_build_environment()
        dependency_cflags = shlex.split(
            subprocess.run(
                [str(pkg_config), "--cflags", "eigen3", "libzmq"],
                check=True,
                capture_output=True,
                text=True,
                env=build_environment,
            ).stdout
        )
        common_includes = [
            f"-I{client / 'ManusSDK' / 'include'}",
            f"-I{protobuf / 'src'}",
            f"-I{client}",
            *dependency_cflags,
            f"-ffile-prefix-map={REPOSITORY_ROOT}=.",
            f"-fmacro-prefix-map={REPOSITORY_ROOT}=.",
        ]
        commands = [
            [
                str(compiler),
                "-std=c++17",
                "-O2",
                *common_includes,
                "-c",
                str(client / "SharpaManusClient.cpp"),
                "-o",
                str(build / "SharpaManusClient.o"),
            ],
            [
                str(compiler),
                "-std=c++17",
                "-O2",
                *common_includes,
                "-c",
                str(client / "ClientPlatformSpecific.cpp"),
                "-o",
                str(build / "ClientPlatformSpecific.o"),
            ],
            [
                str(compiler),
                "-std=c++17",
                "-O2",
                *common_includes,
                "-c",
                str(client / "sharpa_hand.pb.cc"),
                "-o",
                str(build / "sharpa_hand.pb.o"),
            ],
            [
                str(compiler),
                "-std=c++17",
                "-O2",
                "-o",
                str(build / "SharpaManusClient"),
                str(build / "SharpaManusClient.o"),
                str(build / "ClientPlatformSpecific.o"),
                str(build / "sharpa_hand.pb.o"),
                f"-L{client / 'ManusSDK' / 'lib'}",
                "-lManusSDK_Integrated",
                "-lncurses",
                "-lzmq",
                "-Wl,-rpath,$ORIGIN/../lib",
                "-Wl,-Bstatic",
                str(protobuf / "src" / ".libs" / "libprotobuf.a"),
                "-Wl,-Bdynamic",
                "-pthread",
            ],
        ]
        for command in commands:
            subprocess.run(command, check=True, env=build_environment)
        subprocess.run(
            [str(strip), str(build / "SharpaManusClient")],
            check=True,
            env=build_environment,
        )
        assert_relocatable_runtime_paths(build / "SharpaManusClient")
        _copy_file(build / "SharpaManusClient", target, executable=True)


def _stage_manus() -> None:
    client = REFERENCE_ROOT / "sharpa-manus-sdk" / "client"
    source = client / "ManusSDK"
    target = MANUS_VENDOR_ROOT
    _copy_tree(source / "include", target / "include")
    _copy_file(
        source / "lib" / "libManusSDK_Integrated.so",
        target / "lib" / "libManusSDK_Integrated.so",
    )
    _copy_file(client / "Calibration_left.mcal", target / "config" / "Calibration_left.mcal")
    _copy_file(
        client / "Calibration_right.mcal",
        target / "config" / "Calibration_right.mcal",
    )
    _copy_file(
        REFERENCE_ROOT / "sharpa-manus-sdk" / "License",
        target / "LICENSE.vendor",
    )
    _copy_file(
        REFERENCE_ROOT / "sharpa-manus-sdk" / "NOTICE.txt",
        target / "NOTICE.vendor",
    )
    _build_manus_client(target / "bin" / "SharpaManusClient")


def _patch_legacy_linker_imports(root: Path) -> None:
    """Make the flat upstream SDK importable as a namespaced package."""
    replacements = {
        "from utils.": "from .utils.",
        "from core.": "from .core.",
    }
    for path in root.rglob("*.py"):
        text = path.read_text(encoding="utf-8")
        if path.parent.name in {"can", "rs485"}:
            text = text.replace("from utils.", "from ...utils.")
        elif path.parent.name == "utils":
            text = text.replace("from color_msg import", "from .color_msg import")
            text = text.replace(
                "from load_write_yaml import", "from .load_write_yaml import"
            )
        else:
            for old, new in replacements.items():
                text = text.replace(old, new)
        path.write_text(text, encoding="utf-8")
    for path in (*root.rglob("*.py"), *root.rglob("*.yaml")):
        text = path.read_text(encoding="utf-8")
        normalized = "\n".join(line.rstrip() for line in text.splitlines()) + "\n"
        path.write_text(normalized, encoding="utf-8")


def _extract_member(archive: Path, member_name: str, destination: Path) -> None:
    if not archive.is_file():
        raise FileNotFoundError(archive)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tarfile.open(archive) as bundle:
        member = bundle.getmember(member_name)
        source = bundle.extractfile(member)
        if source is None:
            raise FileNotFoundError(f"{member_name} in {archive}")
        with destination.open("wb") as stream:
            shutil.copyfileobj(source, stream)
    shutil.copymode(archive, destination)


def _stage_linker() -> None:
    target = LINKER_VENDOR_ROOT

    legacy_source = REFERENCE_ROOT / "linkerhand_sdk"
    legacy_target = target / "legacy"
    _copy_python_tree(legacy_source, legacy_target)
    _patch_legacy_linker_imports(legacy_target)

    o30_source = (
        REFERENCE_ROOT
        / "Linker_Hand_O30I_ROS2_SDK"
        / "src"
        / "linker_hand_o30i_ros2_sdk"
        / "linker_hand_o30i_ros2_sdk"
        / "core"
        / "canfd"
    )
    _copy_file(
        o30_source / "linker_hand_o30i_control.py",
        target / "o30i" / "linker_hand_o30i_control.py",
    )
    _copy_file(
        o30_source / "HandProtocol_v1.0.pdf",
        target / "o30i" / "HandProtocol_v1.0.pdf",
    )
    _copy_file(
        REFERENCE_ROOT / "Linker_Hand_O30I_ROS2_SDK" / "src" / "99-canfd.rules",
        target / "o30i" / "udev" / "99-canfd.rules",
    )
    _copy_file(
        REFERENCE_ROOT
        / "Linker_Hand_O30I_ROS2_SDK"
        / "src"
        / "99-double-canfd.rules",
        target / "o30i" / "udev" / "99-double-canfd.rules",
    )

    archive_root = REFERENCE_ROOT / "Linker_Hand_O30I_ROS2_SDK" / "src"
    runtimes = {
        "libcanbus(ubuntu20).tar": "linux-x86_64-ubuntu20",
        "libcanbus(ubuntu22).tar": "linux-x86_64-ubuntu22",
        "libcanbus_arm64.tar": "linux-aarch64",
        "libcanbus_arm.tar": "linux-armv7l",
    }
    for archive_name, platform_name in runtimes.items():
        _extract_member(
            archive_root / archive_name,
            "libcanbus.so",
            target / "o30i" / "lib" / platform_name / "libcanbus.so",
        )


def _write_manifest(vendor_root: Path) -> None:
    entries = {}
    for path in sorted(vendor_root.rglob("*")):
        if path.is_file() and path.name != "manifest.json":
            entries[str(path.relative_to(vendor_root))] = {
                "size": path.stat().st_size,
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            }
    (vendor_root / "manifest.json").write_text(
        json.dumps(entries, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--clean",
        action="store_true",
        help="Replace the exact per-device _vendor directories before copying.",
    )
    parser.add_argument(
        "--only",
        choices=("manus", "sharpa", "linker"),
        help="Stage only one vendor family instead of all SDKs.",
    )
    args = parser.parse_args()
    stages = {
        "manus": (MANUS_VENDOR_ROOT, _stage_manus),
        "sharpa": (SHARPA_VENDOR_ROOT, _stage_sharpa),
        "linker": (LINKER_VENDOR_ROOT, _stage_linker),
    }
    selected = {args.only: stages[args.only]} if args.only else stages
    vendor_roots = tuple(root for root, _stage in selected.values())
    if args.clean:
        for vendor_root in vendor_roots:
            if vendor_root.exists():
                shutil.rmtree(vendor_root)
    for vendor_root in vendor_roots:
        vendor_root.mkdir(parents=True, exist_ok=True)
    for name, (vendor_root, stage) in selected.items():
        _copy_file(
            HARDWARE_ROOT / name / "VENDOR_README.md",
            vendor_root / "README.md",
        )
        stage()
    for vendor_root in vendor_roots:
        validate_elf_tree(vendor_root)
        _write_manifest(vendor_root)
    for name, (vendor_root, _stage) in selected.items():
        print(f"Staged {name} SDK in {vendor_root}")


if __name__ == "__main__":
    main()
