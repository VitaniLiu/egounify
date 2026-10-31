# Sharpa V4 retargeting assets

`manus_sharpa_teleop/vendor/retarget_v4` contains the upstream CPython 3.10
optimizer extensions. `sharpa_wave_description/urdf` contains the matching
left/right Wave URDFs as a separate ROS package.

Run `pixi run stage-retarget-v4` to refresh these files from
`reference/sharpa-manus-sdk/retargeting_alg_release_V4.0`. The source tree is
read-only input and is never a runtime dependency.

These assets are proprietary and must only be distributed internally to
recipients covered by the applicable vendor agreements.
