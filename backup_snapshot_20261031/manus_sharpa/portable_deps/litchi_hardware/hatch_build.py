"""Hatch hook that marks wheels containing vendor ELF binaries as platform wheels."""

import importlib
import sys
from pathlib import Path

from hatchling.builders.hooks.plugin.interface import BuildHookInterface

REPOSITORY_ROOT = Path(__file__).resolve().parent
TOOLS_ROOT = REPOSITORY_ROOT / "tools"
if str(TOOLS_ROOT) not in sys.path:
    sys.path.insert(0, str(TOOLS_ROOT))
validate_elf_tree = importlib.import_module("elf_runtime_paths").validate_elf_tree


class CustomBuildHook(BuildHookInterface):
    def initialize(self, version: str, build_data: dict[str, object]) -> None:
        del version
        validate_elf_tree(REPOSITORY_ROOT / "src" / "litchi_hardware")
        build_data["infer_tag"] = True
        build_data["pure_python"] = False
