# Bundled Xynova Arm SDK runtime

This directory contains the Linux runtime subset of vendor release
`Xynova_Arm_SDK-V0.7.1-Release-20260814144335.zip`.

- `include/`: vendor public C++ headers.
- `external/Eigen/`: header dependency required by the public API.
- `lib/Linux/{x86_64,aarch64}/libXynovaSDK.so.0`: vendor runtimes, renamed
  from the release's `.so.0.7.1` filename to match its ELF SONAME.
- `lib/Linux/{x86_64,aarch64}/libxynovaModel.a`: link-time model dependency
  required to resolve the shared SDK's KDL symbols in the project bridge.
- `README.md` and `CHANGELOG.md`: upstream documentation.

Windows libraries, examples, generated documentation, controller-upgrade
libraries, and the full static SDK were intentionally omitted from the Python
wheel. These assets are proprietary and may only be distributed through the
organization's private repository and package index under the applicable
vendor agreement.
