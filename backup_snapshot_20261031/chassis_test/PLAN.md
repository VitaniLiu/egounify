# 20 Hz chassis-only replay implementation plan

> Implementation: inline, test-first; independent final read-only review.

Goal: user-launched chassis replay of the existing LeRobot Parquet action.chassis.
Spec: approved in conversation 2026-10-31; code delivered only under test/.
Architecture: local dataset verifier/planner + SSH robot endpoint; reuse the
unchanged, hash-pinned CoordinatedChassis and its two-stage tracking protection.
Tech: existing Python 3.12 venv, pyarrow/numpy; robot's existing ROS environment.

## Constraints and decisions
- Default offline, explicit --execute; --check is live read-only. No motion by agent.
- 20 Hz integer source timeline; [vx,vy,wz], m/s,m/s,rad/s, base_link.
- Existing limit |vx|<=0.4, |wz|<=0.5 and vy==0 retained (float32 tolerance only).
- Keep every selected sample incl. idle. Final sample held for one 50ms slot,
  then append a separately labelled stop, never rewrite stored source action.
- Optional --seconds chooses a prefix on the 50ms grid, no time compression.
- Do not alter old scripts, configs, data, services or hand integration.
- Report source hashes, timing, sent values and feedback; no path accuracy claim.
- Ruling: stage in a fresh temporary directory, then add only user-requested
  test/ files. No shared branch commits or main-code edits are needed.

## Review focus
1. Wrong units/order/rate, NaN, nonzero vy, tampered episode: reject before SSH.
2. SSH EOF, heartbeat loss, Ctrl+C: stop locally on robot and preserve evidence.
3. Do not catch up stale commands in a burst; late >10ms aborts (bounded jitter).
4. Competing chassis publisher or armed gamepad: reject, do not disable.
5. Stop requires fresh vx/vy/wz feedback stable for 200ms; missing ACK is failure.

## Task 1: loader and scheduler
- [x] Add test/test_parquet_chassis_replay.py with explicit fixture values and failure cases.
- [x] Run RED for missing make_plan/run_schedule.
- [x] Add test/replay_chassis.py (CLI, hashes, Parquet, SSH, reports).
- [x] Add test/chassis_endpoint.py (pure validator/scheduler + ROS adapter).
- [x] Run new suite and old tests; verify defaults never open SSH.

## Task 2: verification and delivery
- [x] Add README.txt with offline/check/execute, expected motion, safeguards and limits.
- [x] Run three real episodes through offline loader; no robot motion.
- [x] Independent review, repair important findings with regressions.
- [x] Deploy new test/ only; run full repo suite and hashes; report untested hardware.

## Progress
- Existing backend copied read-only; SHA256 d87b268260101a9cb1e02573e7ecaf4de44b6fc1ac1544bf0c112271412dea61.
- Robot unreachable (No route to host), hardware validation excluded.
- Final deployed suite: 224 passed in 16.66s (196 existing + 28 new).
- Episode 2: offline PASS, 314 frames, 20Hz, 15.65s timestamp span,
  15.70s command duration including final hold, all vy=0.
- Episodes 0/1 refused: older exports omit chassis frame/units declaration;
  no guessed coordinate frame or silent bypass. Error covered by regression.
- Read-only --check: SSH 255 / No route to host; no robot commands sent.
- Independent review findings fixed: abort >10ms lateness rather than compress
  frames, persist robot report before stdout, serialize nonfinite diagnostics.
- New test basename changed to avoid collision with existing tests module.
- git diff --exit-code passed; only additions test/ plus pre-existing integrations/.
- Existing backend SHA256 unchanged from pre-change value above.
- No physical replay, ROS discovery/preflight, tracking/stop hardware behavior,
  or trajectory accuracy claim. User will initiate after connectivity restored.

## Follow-up: chassis-only dependency (2026-10-31)
- User explicitly requests removing upper-body checks; replay controls chassis only.
- Remove domain-29 node and joint/source/VR subscriptions and checks entirely.
- Retain domain-0 chassis feedback, motor/STM/IMU checks, tracking, heartbeat and stop.
- Explicit motion_started=false acknowledgement distinguishes a failed preflight
  from lost acknowledgement after a possible command; retain warning for unknowns.
- Regression tests cover missing upper-body ROS, dead chassis executor, stale
  chassis feedback, and preflight-vs-motion stop warning behavior.
- Deployed suite: 229 passed in 17.02s (196 existing + 33 replay tests).
- Live read-only preflight PASS: test/runs/20261031-184048-d25c1d5b/report.json;
  status=preflight_passed, sent_frames=0, motion_started=false. No physical replay.
- No upper-body/VR subscriptions or domain-29 imports remain in endpoint.
- Unchanged existing backend hash and clean tracked-file git diff confirmed.
- Pre-change test/ backup: /tmp/apex-chassis-before-upper-removal-20261031-8f371.tar.gz.
