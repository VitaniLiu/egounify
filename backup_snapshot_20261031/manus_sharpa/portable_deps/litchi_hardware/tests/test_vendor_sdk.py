from __future__ import annotations

import json
import os
import shutil
import subprocess
from pathlib import Path

from litchi_hardware import linker, manus, sharpa
from litchi_hardware.hardware.manus import (
    ManusConfig,
    ManusNativeTracker,
    ManusZmqConfig,
    ManusZmqTracker,
    create_tracker,
)
from litchi_hardware.hardware.manus._native import CtypesManusBridge

READELF = shutil.which("readelf", path=os.defpath)


def _runtime_paths(path: Path) -> tuple[str, ...]:
    if READELF is None:
        raise RuntimeError("host readelf is required for ELF path tests")
    result = subprocess.run(
        [READELF, "-dW", str(path)],
        check=True,
        capture_output=True,
        text=True,
    )
    values = []
    for line in result.stdout.splitlines():
        if "(RPATH)" not in line and "(RUNPATH)" not in line:
            continue
        values.append(line.rsplit("[", 1)[1].split("]", 1)[0])
    return tuple(values)


def _assert_elf_tree_is_relocatable(root: Path) -> None:
    for path in root.rglob("*"):
        if not path.is_file():
            continue
        with path.open("rb") as stream:
            if stream.read(4) != b"\x7fELF":
                continue
        for value in _runtime_paths(path):
            assert all(
                entry == "$ORIGIN" or entry.startswith("$ORIGIN/")
                for entry in value.split(":")
            ), f"{path} has non-relocatable runtime path {value!r}"


def test_public_facades_expose_bundled_sdk_assets() -> None:
    assert (sharpa.include_dir() / "SharpaWaveSDK.h").is_file()
    assert (sharpa.library_dir() / "libsharpa-wave-sdk.so").is_file()
    assert (manus.include_dir() / "ManusSDK.h").is_file()
    assert not (manus.include_dir() / "litchi_manus_bridge.h").exists()
    assert (manus.library_dir() / "libManusSDK_Integrated.so").is_file()
    assert manus.bridge_library().is_file()
    assert (manus.bridge_include_dir() / "litchi_manus_bridge.h").is_file()
    assert manus.client_executable().is_file()
    assert tuple(finger.value for finger in sharpa.SHARPA_FINGERS) == (
        "thumb",
        "index",
        "middle",
        "ring",
        "pinky",
    )
    assert linker.protocol_manual().is_file()
    assert linker.bundled_libcanbus_path().is_file()
    assert (linker.legacy_sdk_root() / "linker_hand_api.py").is_file()


def test_vendor_manifest_excludes_project_owned_bridge() -> None:
    manifest = json.loads(
        (manus.sdk_root() / "manifest.json").read_text(encoding="utf-8")
    )
    assert not any("litchi_manus_bridge" in path for path in manifest)
    assert not tuple(manus.sdk_root().rglob("*litchi_manus_bridge*"))
    assert not (manus.library_dir() / "libManusSDK.so").exists()
    assert (sharpa.sdk_root() / "manifest.json").is_file()


def test_sharpa_facade_loads_native_sdk() -> None:
    sdk = sharpa.load_sdk()
    assert hasattr(sdk, "SharpaWaveManager")
    assert sharpa.SharpaWaveManager is sdk.SharpaWaveManager


def test_manus_facade_loads_integrated_library() -> None:
    library = manus.load_library()
    assert Path(library._name) == manus.library_dir() / "libManusSDK_Integrated.so"


def test_manus_facade_loads_native_bridge() -> None:
    library = manus.load_bridge()
    assert library.litchi_manus_bridge_abi_version() == 1
    CtypesManusBridge()


def test_real_manus_factory_defaults_to_native_bridge() -> None:
    tracker = create_tracker(ManusConfig(name="bundled"), use_fake_hardware=False)
    assert isinstance(tracker, ManusNativeTracker)


def test_zmq_factory_defaults_to_bundled_client() -> None:
    tracker = create_tracker(
        ManusZmqConfig(name="bundled"), use_fake_hardware=False
    )
    assert isinstance(tracker, ManusZmqTracker)
    assert tracker.config.client_executable == str(manus.client_executable())
    assert tracker.config.client_working_directory == str(manus.calibration_dir())


def test_external_zmq_mode_does_not_manage_bundled_client() -> None:
    tracker = create_tracker(
        ManusZmqConfig(name="external", manage_client=False),
        use_fake_hardware=False,
    )
    assert isinstance(tracker, ManusZmqTracker)
    assert tracker.config.client_executable is None


def test_bundled_elf_runtime_paths_are_relocatable() -> None:
    _assert_elf_tree_is_relocatable(manus.sdk_root())
    _assert_elf_tree_is_relocatable(sharpa.sdk_root())
    _assert_elf_tree_is_relocatable(linker.sdk_root())
    _assert_elf_tree_is_relocatable(manus.bridge_include_dir())
    assert _runtime_paths(manus.bridge_library()) == (
        "$ORIGIN/../../_vendor/lib",
    )
