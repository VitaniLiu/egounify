# Changelog

## 0.1.6

### Added

- Added the Xynova Arm SDK 0.7.1 public headers and Linux x86_64/aarch64
  runtimes, plus a versioned project-owned C ABI bridge over the vendor C++
  API.
- Added `XynovaArmConfig`, `XynovaArmModel`, `XynovaSdkArm`,
  `MockXynovaArm`, and `create_arm()`. The HAL supports all seven SDK robot
  classes (3–7 DoF), joint position/velocity/torque state, TCP pose,
  collaborative-arm wrench feedback, non-real-time absolute joint commands,
  power control, and stop.
- Arm connections are read-only by default: they do not power on the robot.
  Motion requires an explicit `power_on()` unless `auto_power_on=True` is
  configured.

## 0.1.5

### Changed

- Updated the bundled Xynova runtime and headers from SDK 1.0.0 R3 to SDK
  2.0.0 R3 on Linux x86_64 and aarch64. The ctypes/HAL and ROS 2 adapters now
  use the `XynovaDexterousHand_*` API, its 23-joint angle control schema, and
  15-region six-axis tactile wrench feedback. The removed 1.x joint torque and
  report-period APIs are no longer used.

## 0.1.4

### Fixed

- **Sharpa Wave firmware 3.0.6 initialization compatibility.** The SDK backend
  now initializes persistent settings idempotently. It reads speed/current
  coefficients, control source, and control mode first; matching values are
  left untouched, while each mismatch is written once and verified before
  `start()`. Coefficients use a `1e-4` absolute comparison tolerance. This
  avoids firmware rejections of redundant writes while still failing closed on
  getter, setter, or readback errors.
- Initialization also reads the independent motor enable state after source
  and mode are established. A disabled hand is enabled once and verified after
  bounded getter polling; an already enabled hand is left untouched. This
  covers reconnecting after SDK shutdown with persistent SDK/POSITION settings
  but disabled motors, including firmware 3.0.6 taking more than 100 ms to
  report the enabled state.
- Failed initialization stops, destroys, and disconnects the partially
  initialized SDK handle, leaving the HAL lifecycle in `FAILED` so the
  connection can be retried cleanly. Initialization and cleanup no longer
  unconditionally disable the hand or switch its control source to APP.

## 0.1.3

### Fixed

- **Sharpa ROS 2 tactile topic/frame identity guard.** The in-repo
  `sharpa_driver` now derives tactile topic prefixes from the physical hand
  side and finger through one runtime helper. Before publishing, it verifies
  that the frame returned by the HAL has the same physical side/finger as the
  route being polled; a mismatched frame is rejected instead of being silently
  published under another fingertip's topic.
- SDK channel selection remains centralized in the ROS-independent HAL. ROS
  does not apply a second reversal: Wave left physical thumb through pinky
  remain SDK channels `9, 8, 7, 6, 5`, and right remains `0, 1, 2, 3, 4`.

### Consumer migration

- The `0.1.2` physical-finger migration guidance below still applies. No topic
  names change in `0.1.3`; this release makes their physical identity an
  enforced runtime invariant.

## 0.1.2

### Fixed

- **Sharpa Wave left-hand fingertip tactile channel mapping.** The left hand's
  fingertip channels are physically wired in reverse relative to the right
  hand. The previous mapping (`thumb→5 … pinky→9`) was reversed, misattributing
  the thumb, index, ring, and pinky tactile streams to their mirror-image
  fingers. Middle stayed on channel 7 under both the old and corrected mapping
  (it is the fixed point of the reversal), so it was already correct.
  Field-verified on serial `CD559038CD54` / `Wave-L-01`, the correct mapping is:

  | Physical finger | SDK channel (left) | SDK channel (right) |
  | --------------- | ------------------ | ------------------- |
  | thumb           | 9                  | 0                   |
  | index           | 8                  | 1                   |
  | middle          | 7                  | 2                   |
  | ring            | 6                  | 3                   |
  | pinky           | 5                  | 4                   |

  The fix lives entirely in `tactile_channel()`
  (`litchi_hardware.hardware.sharpa.tactile`); the SDK reader, mock, frame
  validation, and the in-repo `sharpa_driver` ROS 2 node all route through it
  and already key topics/`frame_id`s by physical finger, so no second reverse
  mapping exists. Right-hand mapping is unchanged. Joint control (the
  independent 22-joint schema) is unaffected.

### Consumer migration

- Downstream consumers subscribing to `/sharpa/left/tactile/<finger>/…` topics
  (or reading `frame_id` `left_<finger>_fingertip`) were previously receiving
  the mirror-image finger's data for the thumb, index, ring, and pinky (middle
  was already correct). After upgrading, `left/tactile/thumb` now carries the
  physical left thumb (SDK channel 9). Any consumer that compensated by
  swapping finger names on the left hand must remove that workaround.
