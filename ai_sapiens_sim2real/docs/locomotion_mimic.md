# Locomotion Mimic module

This module runs walking and selected motions through one exported policy.
It is separate from ordinary Mimic playback, Any2Track and gait-phase policies.
Existing behaviors do not instantiate it unless their policy configuration
contains `observations.mimic_gate`.

## Boundaries and removal

Implementation lives entirely in:

- `include/ai_sapiens_sim2real/modules/locomotion_mimic/`
- `src/modules/locomotion_mimic/`

`config.hpp` / `config.cpp` resolve the export settings and motion assets.
`gate.hpp` owns the motion selection, pause, branch and loop state machine.
`runtime.hpp` / `runtime.cpp` adapt RC input, encode observations and compose
walking/upper-body actions. Gate state is private; diagnostics are read-only.
The module factory owns activation, compatibility checks and `mimic_gate`
observation registration. Constructing a standalone gate adapter does not mutate
the global observation registry. Observation overrides and walking action history
belong to the runtime instance and never reside in `SharedControlData`.

The common runtime has only module construction, observation binding, entry
reset, tick update and inference dispatch. Normal inference failure handling,
action validation, command publication and policy transitions remain shared.
The `locomotion_mimic` object-library target collects the module's build inputs.
The marked Locomotion Mimic block in `CMakeLists.txt` groups its target, links
and dedicated tests after the common targets. No separate CMake directory is needed.
`ObservationManager` applies optional raw-value corrections through one evaluation
path for initialization, reset and tick updates, before history, scale and clip.
Common observation functions contain no correction or module-specific branches.

To remove the feature:

1. Remove `loco_mimic`, `SubVelocityLocoMimic` and selector 269 from root config.
2. Remove the module pointer/include and its five lifecycle connections from
   `PolicyRuntime`; use its original single inference dispatch.
3. Remove the Locomotion Mimic block in `CMakeLists.txt`, the module
   directories and the dedicated `test_locomotion_mimic*` test files.
4. Restore the desired CH8 API-mode binding if that switch is no longer used
   for motion triggering, and remove the local policy asset folder.

### Shared support introduced for this feature

The following shared-code additions were introduced for Locomotion Mimic. They
are removable support, not a requirement to keep a permanent extension framework.
If no other feature uses them at removal time, remove them with the module:

- Observation corrections: `PolicyObservationOverrides`, its pointer in
  `ObservationContext` and `ObservationManager`, and the manager's correction
  hook. Restore direct raw-term evaluation and remove the dedicated correction
  test. Preserve the existing observation history, scaling and clipping behavior.
- Raw RC transport: `rc_us` and `rc_valid` in `TeleopInputCommand` and
  `TeleopInput`, collection in the RadioMaster plugin, copying in
  `TeleopInputHandle`, and clearing on group ownership in `GroupTeleopHandle`.
  Remove only their corresponding test assertions; retain the existing teleop
  and group tests. Rebuild the plugins together with the node after changing
  the command structure.
- Motion loading options: the `MotionReference::Derivative` argument and the
  orientation-normalization argument. Restore unconditional central differences
  and normalization. Keep the pre-existing derivative helpers.

The module, deployment wiring and shared support are delivered in one feature
commit. The list above identifies the shared additions within that commit.
Before removal, check current consumers so later features remain intact; remove
unused additions rather than retaining them by default. The common two-joint test fixture remains because existing mirror tests
also use it. No module-specific branch is added to `MimicPolicyRuntime` or
`PolicyController`.

## Deployed contract

Source: K1-26 `locomimic` stash
`6ec719ea45a952da851923f1a245b71fd52f8ecc`; its untracked gate/test tree is
`e8dc831a9976ad2f502c7cee733a2ca9f3f931ad`. The stash was read without applying
or deleting it. The archived apply notes describe an older export; the actual
robot export matches the supplied v17b model:

- ONNX SHA-256: `1d7615b70af831f96ec2d421244d469a1ac3259b9f2e07220371995b4c61eb18`
- 539 observations: 390 walking-history values + 149 motion values.
- 23 actions, 24 clips, 50 Hz, no gait-phase observation.
- Motion weight at ONNX input index 514.
- CH8 trigger, CH10 interaction choice, CH8 confirmation.

Assets are installed locally at
`assets/k1/locomotion/velocity/mimic_v17b_sd_20261029/`. The model and clips
are unchanged; the supplied YAML's trigger channel is changed from 5 to 8,
matching the actual K1-26 configuration. `deployment.json` records source and
file hashes. Locomotion assets are ignored by Git; a source-only checkout does
not include them. Rebuild the entire package, including teleop plugins, because
raw-channel transport extends the plugin command structure.

Selector 269 enters `SubVelocityLocoMimic`. Inside that state, selectors 200–223
choose the configured clip or loop slot. Damping, ReadyPose and Velocity retain
explicit exits. CH8 no longer doubles as API-mode selection in the local
RadioMaster configuration. Individual raw channels are withheld while group
commands own the input route, matching the original stash.

## Numerical compatibility and timing

All CSV parsing now uses `MotionReference`. This export explicitly requests
forward joint differences (last derivative repeated) and its original root
quaternion values. Ordinary Mimic keeps central differences and normalized
quaternions by default. This is a numerical export contract, not a file-format
auto-detection branch.

The gate advances once per accepted 0.02-second policy tick. Frame progression,
held frames, float timers, minimum-jerk ramps and transition ordering retain
the original behavior. Policy episode clocks and ordinary Mimic playback are
unchanged. Walking during an eligible clip performs a second inference with
motion weight zero and retains that inference's arm history independently.

Locomotion Mimic intentionally owns its frame cursor and float timers. Keep this
export's timing inside the module; do not migrate it to ordinary Mimic's playback
or episode clock. This separation is the deployment decision, not a temporary
compatibility layer. Clip progression pauses during holds and ramps, jumps on
interactive branches and resets between loop clips. The parity tests protect
endpoint dwell, hold/branch frame indices and same-tick transition ordering.
Forward reference velocities also remain part of this export's numerical contract.

## Verification

`test_locomotion_mimic` compares the refactored gate against the frozen original
header under `test/data/locomotion_mimic/`, which is test-only and never linked
into deployment. It checks 80,000 ticks covering all seven states, loops,
release/second-edge abort, holds, interactive branches and walking handoff.
Additional tests cover private observation masks, dual inference and hold rules.
`test_policy_observation_overrides` independently verifies aliases, both history
layouts, both clip/scale orders, initialization, reset and removal of corrections.

`test_locomotion_mimic_runtime` contains optional real-export validation. Set
`LOCOMIMIC_ASSET_DIR` to the asset directory above to check every frame of all
24 clips against the original loader, load the 539-dimensional ONNX, exercise
walking-plus-motion inference, release, switch to a normal policy and re-enter.
The ordinary-policy ONNX fixture lives in `test/data/common/two_joint_policy/`
and is independent of mirror-specific motion data.
RC tests verify channel delivery and group ownership. When testing an out-of-tree
build, put its library directory first in `LD_LIBRARY_PATH` to avoid loading an
older installed teleop plugin with a different command ABI.

These are software parity and inference checks. They do not constitute MuJoCo
tracking evaluation or validation on a physical robot.
