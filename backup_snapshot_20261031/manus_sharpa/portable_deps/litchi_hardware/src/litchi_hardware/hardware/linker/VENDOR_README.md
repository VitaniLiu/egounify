# Bundled Linker Hand SDK assets

This directory is populated by `tools/vendor_sdks.py` from two vendor deliveries:

- `reference/linkerhand_sdk`: Linker Hand Python SDK 3.1.0, used by the G20
  SocketCAN backend.
- `reference/Linker_Hand_O30I_ROS2_SDK`: Linker Hand O30I ROS2 SDK 3.0.1.
  The ROS-independent HOP controller, protocol manual, udev rules, and
  architecture-specific `libcanbus.so` runtimes are extracted from that SDK.

The legacy SDK's flat imports are rewritten mechanically to package-relative
imports while staging. No protocol or control behavior is changed. Generated
Python caches and the vendor copies of `libusb` are intentionally excluded;
the host must provide `libusb-1.0`.

These files are proprietary vendor material supplied for internal hardware
integration. They must only be distributed to recipients authorized under the
applicable Linker Hand agreement. The delivery did not include a standalone
license file; confirm redistribution terms before sharing a wheel outside the
organization.
