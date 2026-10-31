#!/usr/bin/env python3
"""Build the project-owned Manus native bridge beside its Python backend."""

from __future__ import annotations

import subprocess
from pathlib import Path

from elf_runtime_paths import assert_relocatable_runtime_paths
from native_build import clean_host_build_environment, resolve_host_tool

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = REPOSITORY_ROOT / "src" / "litchi_hardware"
SOURCE_ROOT = PACKAGE_ROOT / "hardware" / "manus" / "_native"
MANUS_SDK_ROOT = PACKAGE_ROOT / "hardware" / "manus" / "_vendor"
TARGET = SOURCE_ROOT / "lib" / "liblitchi_manus_bridge.so"


def main() -> None:
    sdk_header = MANUS_SDK_ROOT / "include" / "ManusSDK.h"
    sdk_library = MANUS_SDK_ROOT / "lib" / "libManusSDK_Integrated.so"
    if not sdk_header.is_file() or not sdk_library.is_file():
        raise FileNotFoundError(
            "bundled Manus SDK is missing; run tools/vendor_sdks.py first"
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
        f"-I{MANUS_SDK_ROOT / 'include'}",
        str(SOURCE_ROOT / "litchi_manus_bridge.cpp"),
        f"-L{MANUS_SDK_ROOT / 'lib'}",
        "-lManusSDK_Integrated",
        "-Wl,--enable-new-dtags",
        "-Wl,-rpath,$ORIGIN/../../_vendor/lib",
        "-Wl,--no-undefined",
        "-pthread",
        "-o",
        str(TARGET),
    ]
    environment = clean_host_build_environment()
    subprocess.run(command, check=True, env=environment)
    subprocess.run([str(strip), str(TARGET)], check=True, env=environment)
    assert_relocatable_runtime_paths(TARGET)
    print(f"Built project-owned Manus bridge at {TARGET}")


if __name__ == "__main__":
    main()
