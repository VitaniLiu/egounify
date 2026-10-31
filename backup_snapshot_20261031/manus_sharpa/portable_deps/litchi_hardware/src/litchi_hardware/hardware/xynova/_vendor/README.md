# Bundled Xynova SDK runtime

This directory contains vendor SDK release `2.0.0 R3` (`1a073a9`) assets used
by the Python ctypes wrapper.

- `include/`: the vendor's public C and C++ headers (architecture independent).
- `lib/linux-x86_64/`: the Linux x86_64 runtime (`libxynova.so.2.0.0`) and its
  CAN-FD dependency (`libcontrolcanfd.so`), plus that build's `build_info.json`.
- `lib/linux-aarch64/`: the Linux aarch64 (ARM64) runtime and CAN-FD dependency,
  plus that build's `build_info.json`.
- `build_info.json`: the vendor's release metadata (x86_64 reference build; see
  each `lib/<arch>/build_info.json` for the per-architecture metadata).

Native runtimes for both Linux x86_64 and aarch64 are bundled; the loader picks
the directory matching the running machine automatically. On any other
architecture, install the matching vendor runtime and set `XYNOVA_LIB` to the
absolute path of its primary shared library. The Python sources themselves are
architecture independent.

These files remain subject to Xynova's proprietary SDK terms. Confirm internal
redistribution rights before sharing the built package outside your organization.
