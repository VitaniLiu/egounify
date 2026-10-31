#!/usr/bin/env python3
"""Build the project-owned Xynova Arm bridge beside its Python backend."""

from __future__ import annotations

import platform
import subprocess
from pathlib import Path

from elf_runtime_paths import assert_relocatable_runtime_paths
from native_build import clean_host_build_environment, resolve_host_tool

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
ARM_ROOT = (
    REPOSITORY_ROOT
    / "src"
    / "litchi_hardware"
    / "hardware"
    / "xynova"
    / "arm"
)
SOURCE_ROOT = ARM_ROOT / "_native"
SDK_ROOT = ARM_ROOT / "_vendor"
TARGET = SOURCE_ROOT / "lib" / "liblitchi_xynova_arm_bridge.so"

ARCHITECTURES = {
    "x86_64": "x86_64",
    "amd64": "x86_64",
    "aarch64": "aarch64",
    "arm64": "aarch64",
}


def main() -> None:
    machine = platform.machine().lower()
    try:
        architecture = ARCHITECTURES[machine]
    except KeyError as exc:
        raise RuntimeError(f"unsupported Xynova Arm architecture: {machine}") from exc
    sdk_library_dir = SDK_ROOT / "lib" / "Linux" / architecture
    sdk_library = sdk_library_dir / "libXynovaSDK.so.0"
    model_library = sdk_library_dir / "libxynovaModel.a"
    sdk_header = SDK_ROOT / "include" / "xynova" / "robot.h"
    if (
        not sdk_header.is_file()
        or not sdk_library.is_file()
        or not model_library.is_file()
    ):
        raise FileNotFoundError(
            "bundled Xynova Arm SDK is missing; unpack the 0.7.1 release first"
        )

    TARGET.parent.mkdir(parents=True, exist_ok=True)
    compiler = resolve_host_tool("g++", "LITCHI_NATIVE_CXX")
    strip = resolve_host_tool("strip", "LITCHI_NATIVE_STRIP")
    command = [
        str(compiler),
        "-std=c++17",
        "-O2",
        "-fPIC",
        "-fvisibility=hidden",
        f"-ffile-prefix-map={REPOSITORY_ROOT}=.",
        f"-fmacro-prefix-map={REPOSITORY_ROOT}=.",
        "-shared",
        f"-I{SOURCE_ROOT}",
        f"-I{SDK_ROOT / 'include'}",
        f"-I{SDK_ROOT / 'external'}",
        str(SOURCE_ROOT / "litchi_xynova_arm_bridge.cpp"),
        f"-L{sdk_library_dir}",
        "-Wl,--no-as-needed",
        "-Wl,-l:libXynovaSDK.so.0",
        str(model_library),
        "-Wl,--enable-new-dtags",
        f"-Wl,-rpath,$ORIGIN/../../_vendor/lib/Linux/{architecture}",
        "-Wl,--no-undefined",
        "-pthread",
        "-o",
        str(TARGET),
    ]
    environment = clean_host_build_environment()
    subprocess.run(command, check=True, env=environment)
    subprocess.run([str(strip), str(TARGET)], check=True, env=environment)
    assert_relocatable_runtime_paths(TARGET)
    print(f"Built project-owned Xynova Arm bridge at {TARGET}")


if __name__ == "__main__":
    main()
