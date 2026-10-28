# Posture Mimic entry validation

This branch ports posture safety onto `feature-all-in-one` while retaining its
existing policy assets, selector IDs, group arbitration, localization,
and command interpolation. Mimic reference playback is unified as described below. No policy checkpoints, phase clocks, observation
layouts, or training settings are changed. The MuJoCo collision model remains
identical to `feature-all-in-one`.

## Configured motions

The K1 configuration uses the existing `mimic/umr_lie_down` and
`mimic/umr_stand_up` assets. Selector 201 requests `MimicLiedown`; selector 200
requests `MimicStandup`. Lying down completes to `Damping`
(`on_complete: Damping`). Damping, ReadyPose, and Velocity requests remain available during
lying playback; there is no direct transition to MimicStandup.
To request get-up after completion, use the existing ReadyPose route and select
MimicStandup; entry still requires the reference-matching orientation. Get-up
inherits the default completion state, `Velocity`.

The older branch's separate supine/prone asset names are not registered here:
the all-in-one workspace already has the UMR pair. The checked local CSVs have
a 6.9-degree gravity-direction difference between the lying endpoint and get-up
start. This is a reference-data check, not a simulation or hardware result.

## Entry checks

Every requested Mimic entry compares the current root IMU orientation with the
reference root orientation at the configured `time_start`. Projected gravity
removes world heading differences. A tilt difference above 35 degrees, invalid
orientation, unavailable IMU, or missing reference rejects the transition and
leaves the current state active. This is an orientation check, not a joint-pose
or contact-state check. It runs on entry, not continuously throughout playback.

A rejected manual policy request requires a valid release of the input code
before retrying. Changing only the selector or losing input does not rearm it.
Service and completion requests also pass through the entry validator.
Orientation matching applies only to Mimic entry. Non-Mimic policy entry,
including Velocity from ReadyPose while lying, does not check orientation.
IMU usability and the existing state-transition rules still apply.

Before the first valid IMU sample, policy entry is blocked. After valid reception,
invalid or stale IMU feedback latches the existing Damping failsafe and requires
restart. `SensorData::is_imu_usable` is the single shared validity flag used by policy
entry and localized policies. `ImuData::is_valid` describes the received sample;
the shared flag additionally accounts for timeout and latched sensor faults.

## Unified motion reference playback

Motion data is parsed once per file/FPS and shared through independent playback
cursors. There is no MJLab/legacy format switch or observation-based format
inference; `motion_format` entries have been removed from the K1 configuration.
All CSV motions use central joint-velocity differences internally and one-sided
differences at the endpoints. A single-frame motion has zero velocity. Root
quaternions are checked and normalized, and non-finite CSV values are rejected.

The last reference sample is at `(N - 1) / FPS`. Sampling uses `time * FPS`,
floor-based interpolation, and snapping near integer frame boundaries. Playback
uses the existing `episode_time` accumulated by `PolicyRuntime::advance_clocks()`.
There is no separate episode or playback step counter. `MotionPlayback` clamps
the first sample reaching/passing the inclusive endpoint to that exact time,
then reports completion on the next call. A small policy-period-relative
tolerance handles float-configured endpoints and accumulated-time roundoff.
Endpoint consumption is reset on policy entry. With 300 frames at 50 Hz, frame
299 runs at 5.98 seconds and completion is requested at 6.00 seconds. An endpoint
between policy ticks is sampled on the next tick, then completion follows one
tick later. `stay` retains the final output, and configured state handoffs still
use the entry validator. The common runtime update order, episode-time
accumulation, and gait/Any2Track advancement remain unchanged.

If motion FPS differs from policy frequency, interpolation follows the policy
ticks; steering still requires matching frequencies. This change does not add
resampling during export or replace asset data with training NPZ values.
Per-cursor runtime joint targets remain independent. World-Z heading alignment
remains defined for lying starting poses; localized Mimic retains its frame
alignment. Any2Track retains its separate adapter playback while using the same
CSV loader and central derivatives.

## Validation

Build and tests run in the existing `ai_sapiens_pkw` development container using
isolated `/tmp/posture-mimic-build` and `/tmp/posture-mimic-install` directories.
Focused tests cover entry tilt and yaw invariance, invalid IMU feedback, manual
rearming, configured start time, independent motion cursors, heading alignment,
state routing, group arbitration, gait clocks, and action transitions. Playback
regressions cover nonlinear frame values and central velocities, 1/2/3/299/300/2659
frames, nonzero windows, float-configured endpoints, and off-grid endpoints.

The full test run also reports an unchanged RadioMaster selector expectation
(`204` actual versus `201` expected) and repository lint failures. ROS tests use
Fast DDS, domain 217, and localhost discovery for isolation; the container's
Zenoh middleware exhibited an intermittent process-exit panic after passing
assertions. No robot service was started and no hardware motion was executed.
