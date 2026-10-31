# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project state

The Manus → Sharpa HAL, V4 ROS retargeting, ROS2 drivers, and
`manus_sharpa_teleop` launch package are implemented. The authoritative source
of truth remains **`docs/architecture.md`** (written in Chinese).

## Tooling & commands

- Environment managed with **pixi** (`pixi.toml`, conda-forge channel, `linux-64`). `pyproject.toml` is the **distribution/packaging description** (project metadata, wheel deps). Keep the two in sync: runtime/dev tooling goes in `pixi.toml`; what ships in the wheel goes in `pyproject.toml`.
- **Python version is not freely chosen — it is pinned by the ROS2 stack.** ROS2 (robostack-packaged **Humble**, installed via conda-forge/pixi) binds the Python version (Humble → Python 3.11). The `requires-python`/`.python-version` values are stale scaffolding defaults; treat robostack Humble's Python as authoritative and align `pixi.toml` (and any `requires-python`) to it rather than the reverse.
- `pixi install` — resolve the environment into `.pixi/` (creates `pixi.lock`).
- `pixi run <task>` — run a task defined under `[tasks]` in `pixi.toml`. `pixi shell` — enter the environment.
- `pixi run test` / `pixi run lint` validate the HAL without ROS.
- `pixi run wheel` builds the internal platform wheel with bundled SDK assets.
- `pixi run -e ros2 ros-build` / `ros-test` validate the ROS workspace.

## Architecture (from docs/architecture.md)

This is a reusable hardware abstraction layer (HAL) for robot arms and hands: Franka arm, Linker Hand G20/O30, Sharpa hand, Xynova hand. **The HAL abstracts "devices," not "controllers."**

Two-tree layout with a single, non-negotiable dependency direction:

- **`src/litchi_hardware/`** — pure Python HAL, **zero ROS dependency**.
  - `core/` — pure contract layer: interfaces, frozen value objects, lifecycle state machine, errors. **Must never import any SDK, `rclpy`, or ROS message.**
  - `hardware/<vendor>/` — vendor-coupled backends + mocks (franka, linker, sharpa, xynova). Depends on `core`, **never the reverse**. A device family = `base.py` + `config.py` + `factory.py` + `mock.py` + backend(s).
- **`ros2/`** — a single colcon workspace for the ROS ecosystem. Subdirectories under `ros2/src/` are organized by **device/function domain** (e.g. `linker_hand/`, `franka_bridge/`), not by language or type; colcon scans recursively so subdirs are organizational only.

### The four hard boundaries (do not violate)

1. `core/` never imports any SDK, `rclpy`, or ROS message.
2. `hardware/*` depends on `core`, never the reverse.
3. `src/` (HAL) never depends on `ros2/` — the direction is one-way.
4. Franka real-time control does **not** enter the HAL — it stays an autonomous C++ `ros2_control` package; the Python side gets a HAL view by *subscribing* to its topics, while high-frequency commands stay in C++.

### Two natures of "ROS2" — decide which before writing a ROS package

- **A. ROS2 as a transport backend**: implements a `core` interface (`HandInterface`), depends on `src/`, hides ROS from upper layers.
- **B. ROS2 as a native hardware driver** (e.g. Franka impedance/force controllers): does **not** implement HAL interfaces and does **not** depend on `src/`; its contract boundary is ROS topics/actions. 

### Interface hierarchy

`DeviceInterface` (lifecycle + joints + observation/action + e-stop) splits into `ArmInterface` (+ `read_tcp_pose`, `read_wrench`) and `HandInterface` (grasp semantics). Tactile sensing is an **optional capability protocol** (`TactileCapable`), mixed in only by hands that have it — it is not a required `HandInterface` method. Connection lifecycle is a `core`-owned state machine: `DISCONNECTED → CONNECTING → CONNECTED/FAILED`; invariants (no skipping `CONNECTING`, failures land in `FAILED`) belong to `core`, concrete transitions belong to each backend.

## references/

`reference/` is **read-only source material**. Runtime code must not depend on
its paths. Maintainers may run `tools/vendor_sdks.py` to copy authorized SDK
assets into each backend's `hardware/<vendor>/_vendor/` directory and compile
the third-party ZMQ client. `tools/stage_retarget_v4_ros.py` separately stages
the V4 optimizer and URDFs into ROS packages. Project-owned code must not be
placed in `_vendor`; the Manus C ABI bridge lives under
`hardware/manus/_native` and is built with `tools/build_manus_bridge.py`.
Never modify files under `reference/`.

The staged vendor assets are proprietary, managed with Git LFS, and intended
only for an organization-internal package index. Do not publish them publicly.
