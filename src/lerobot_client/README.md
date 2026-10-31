# Base-frame data and replay

HDF5 and LeRobot pose streams use base_link. Chassis and joint coordinates retain
their original semantics. The current conversion configuration exports 78D only;
uncollected hands/grippers remain zero with invalid masks. Raw files are unchanged.

The 78D action/state layout is unchanged. Separate action.ik_* columns preserve
all eleven original IK poses (hands, elbows, torso, head, pelvis, feet and knees),
the eleven availability flags and the nameless nineteen-position joint seed.
All reference columns are causally aligned to the same original IK message.
Original pose frame metadata is retained in cleaning_report.json. The converter
rejects unexpected nonempty seed names, velocity, effort or seed headers instead
of silently dropping fields. IKRequest header stamps are regenerated at replay;
source alignment uses the recorded receive timestamps.

The client validates source provenance, frame, layout, finite SE(3), timestamps,
and completeness. The v3 source contract retains all original IK references.
Its explicit execution_target_policy is robot_controller_fk_inverse_support_v1:
arm goals remain recorded final IK inputs; torso/head robot goals come from the
recorded controller joint actions through the installed model FK. The request
adds the filtered recorded foot origin before native QP subtracts that origin.
This conversion is verified only for stationary mode zero, replay source 3,
foot_midpoint support and disabled base closure. Original keypoints are retained.

The endpoint and isolated harness share goal construction and message encoding.
The harness copies live QP parameters and modes, checks readback and uses ideal
100Hz feedback in ROS domain 199. It holds the first target for three seconds
before starting the trajectory, retaining the native two-second output ramp.
An optional GDB probe instruments only its newly launched isolated controller
and reads actual task setters; it never attaches to the physical controller.
Probe timings are diagnostic and must not certify real-time performance.

Full-source validation is carried separately from a selected replay prefix.
The necessary reach screen removes the fixed tool transform and checks the
requested wrist distance against the sum of arm link lengths. Passing this
bound is not sufficient IK or collision certification. Truncation cannot hide
an infeasible frame in the source episode. Existing measured-velocity, feedback,
home, start pose, collision-event, ownership, heartbeat and stop checks remain.

Default invocation is read-only preflight:

```bash
./client_lerobot --task TASK --episode 2
```

--execute remains subject to existing checks. No safety tolerances or live QP
parameters were changed. Effective-target conversion is currently disabled for
physical execution because complete offline and isolated validation has not
passed. Read-only preflight evaluates the corrected goals and whole source.

On 2026-10-26 the corrected torso/head start errors were below 1e-6 m/rad.
Episode 2 still contains infeasible exact wrist targets at the recorded body
goals: at frame 22 the left/right requested distances are 0.754816/0.714248 m,
while the conservative link-length upper bound is 0.619516 m. The screen finds
60 left and 25 right frames beyond that bound. A stationary frame-21 native QP
test retains about 0.0911/0.0841 m errors after four seconds. Do not relabel this
as a timing-only issue or silently replace desired actions with measured motion.
A reviewed target adjustment or a distinct recorded-motion replay policy is
required before completing physical replay.

## Original-semantics LeRobot replay (78D, including chassis)

For `robot_import_20261024_egoav_v4/episode_00000002`, use:

```bash
./client_lerobot --task robot_import_20261024_egoav_v4 --episode 2 --native-recorded
./client_lerobot --task robot_import_20261024_egoav_v4 --episode 2 --native-recorded --test-duration 2 --execute
./client_lerobot --task robot_import_20261024_egoav_v4 --episode 2 --native-recorded --execute
```

The default command is read-only preflight. The training action/state remain
78D at 20 Hz. `meta/source_timestamps.json` also contains `native_replay`:
full-rate IK poses in base_link, original availability and joint references,
physical chassis velocities, and independent nanosecond timestamps. These
auxiliary LeRobot fields are required for this replay; the 78D vector alone
does not contain the complete native controller input. No raw/HDF5 is opened
by the client or robot replay endpoint; both install file-access guards.

This policy restores original IK reference semantics, including torso/head,
rather than the alternative controller-FK goal conversion described above.
The chassis source is `/recording/chassis/manual_move_cmd`, never the all-zero
base-controller command or gamepad intent. One scheduler sends reconstructed
IK on domain 29 and chassis commands on domain 0. Live feedback is never
replaced by recorded feedback.

Execution requires matching isolated-validation provenance, live guards,
and confirmed home feedback. The two-second prefix explicitly holds its last
IK reference while publishing chassis zero, allowing the native startup ramp
to settle for 2–4 seconds before the unchanged stop check. This hold is a test
stage, not part of the original recording. The complete replay has no added
hold stage. Failure leaves execution blocked; manual home reset can be needed
between a short test and complete replay.

Idle-entry handling: the native-recorded endpoint accepts initial IK source 0
(idle) or 1 for preflight. Read-only preflight never changes the source. With
--execute, after live guards and recorded/home proximity checks, source 0 may
transition to 1 for home initialization only after verifying the mux clears
cached targets, does not publish cached targets, and disables on switching.
Sources 2/3 remain rejected as occupied. Normal cleanup restores the actual
entry source, including idle 0. Safety checks during execution remain active.
Read-only robot validation passed with initial/final source 0 and zero commands.
