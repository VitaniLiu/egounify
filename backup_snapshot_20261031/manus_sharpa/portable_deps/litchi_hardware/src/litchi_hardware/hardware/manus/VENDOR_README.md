# Bundled MANUS SDK assets

`tools/vendor_sdks.py` stages the proprietary MANUS Integrated SDK assets into
`_vendor/` for the organization-internal Linux x86_64 wheel.

This directory contains the Integrated SDK headers and library, calibration
files, license notices, and the stripped `SharpaManusClient`. The remote/Core
SDK and its legacy gRPC/Protobuf runtime are intentionally not distributed.

Project-owned bridge code remains in `_native/`.

Do not publish these assets to a public package index. Distribution is limited
to recipients authorized by the applicable vendor agreement.
