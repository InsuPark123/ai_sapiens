# Phase clock deployment

Deployment default confirmed on 2026-10-27: **command-only**. Adaptive phase
policies do not require an explicit schema or command-only option. Body-height,
uprightness and recovery masks require an explicit request and separate runtime
support. Training settings remain independent of this deployment default.
Record the actual phase mode in deployment metadata, history and configuration comments.

The design follows the rule injection structure in `main` (`937c67d`) and the
revision immediately before DWAQ support (`c6ce390`). Keep the existing
`gait_clock.hpp/cpp` filenames and `GaitClock` class. Name clock implementations
by behavior rather than by the learning algorithm.

- `make_free_running_phase_clock`: advance continuously at a fixed period.
- `make_command_reset_phase_clock`: reset phase on a standing command; used by DWAQ.
- `make_adaptive_phase_clock`: smoothly adjust the period based on the command; used by the current AMP policy.
- `PhaseState` / `PhaseUpdate`: raw episode state and its update rule.

Retain the external `gait_phase` YAML key, supported legacy schema names, policy
keys and observation layouts. `obs_bilateral_phase_with_swing` encodes left/right
phase and swing ratios. `obs_fixed_phase` handles existing fixed-period
observations. Algorithm names describe compatibility or provenance only.

`GaitClock::advance()` only calls the injected pure update function. The DWAQ rule
owns its first-step hold; the adaptive rule owns period and swing smoothing.

The former scalar phase is extended to `PhaseState` containing `phase`, `period`,
`swing_ratio` and `first_step`. This is episode state owned by `PolicyState`, not
an observation vector. The clock stores configuration and initial state, without
mutable episode state, shared pointers, reset/observe hooks or observation buffers.
`reset()` copies the initial state; each rule returns the next state from its input.
`resolve_command_before_update` preserves command resolution before adaptive entry-history
seeding and clock updates. DWAQ and legacy policies retain their existing order:
update phase, then resolve the command.

Observation encoding remains in `observations.cpp`, including DWAQ's four values,
the adaptive policy's six values and standing masks. The runtime does not copy a
clock observation buffer. All clock factory rules are internal functions in
`gait_clock.cpp`; only `make_gait_clock` is exposed. No new inheritance hierarchy
or registry is introduced.

The DWAQ introduction commit `babc3fe` changed five files. Frame-major history and
replacement of historical commands with the current command are part of the
export ABI and remain supported. The existing command-history replacement flag is retained; no parallel per-term
selection array is needed.
The legacy two-dimensional standing output of `[0, 0]`, introduced by that commit,
is also preserved. This differs from main's unconditional sine/cosine output and
is a behavioral distinction separate from the structural refactor.

- Configuration source: `observations.gait_phase.params` in the policy's `sim2real.yaml`.
- `periods` selects adaptive timing; the existing `period` selects fixed/command-reset timing.
- The values below describe the current training export; the runtime reads its values from YAML.
- Adaptive observation order: sin L, cos L, sin R, cos R, swing L, swing R.
- Active when `hypot(vx, vy) + abs(yaw) > 0.1`; otherwise all six observations are zero.
- The internal clock continues running while observations are masked. Re-entry resets phase to 0, period to 1.16 seconds and swing ratio to 0.36.
- For `vx < 1.6`: period 1.16 seconds, swing ratio 0.36. For `1.6 <= vx < 2.4`: 0.80 seconds, 0.60. For `vx >= 2.4`: 0.65 seconds, 0.60.
- Period/swing smoothing uses a 0.25-second time constant; the left/right offset is 0.5 cycles. The control interval remains the exported `step_dt`.
- No height/recovery estimator, additional ROS state message or MuJoCo-specific connection is required.

DWAQ retains four observations in the order sin L, sin R, cos L, cos R.
At or below `command_threshold`, its clock resets to zero and its observations
are `[0, 0, 1, 1]`. These observation layouts are not interchangeable; use the
layout and dimensions expected by the export.

The legacy names `k1_settle_march_phase_v1_command_only` and
`k1_settle_march_phase_v1` are accepted for compatibility but are optional.
Height/recovery masking fields are unsupported and rejected. `periods` and
`swing_ratios` each contain three values; `forward_thresholds` and `offsets` each
contain two. Periods, swing ratios, thresholds, smoothing time, reset phase,
offsets and the standing threshold are read from YAML. Validate finite values,
ranges, array lengths and threshold ordering. Validate the update method and
observation order against the supported ABI. Set actual periods in `periods`
instead of using a separate period multiplier. See
`test/data/command_only_phase/phase_contract.yaml` for an example.

These changes apply to runtime source only. They do not copy assets, change the
selected Velocity policy, replace the operational build/install or launch a
runtime. Using the command-only export validated in the public repository still
requires deploying the asset and policy configuration and rebuilding this package.
Omitting the training height/recovery masks changes observations during falls or
crouching compared with training.

Boundary update on 2026-10-27: every speed threshold is the inclusive lower bound
of the next range. The current training code uses `>` at the first threshold, so
its target period differs exactly at `vx = 1.6`: 1.16 seconds in training versus
0.80 seconds in this deployment runtime. Smoothing remains enabled; the running
training process is unchanged.

Adaptive implementation responsibilities remain internal to `gait_clock.cpp`:
`parse_adaptive_phase_config` validates the YAML contract and produces a typed
`AdaptivePhaseConfig`; `advance_adaptive_phase` performs the state transition
without YAML or robot-state dependencies; `make_adaptive_phase_clock` assembles
the initial state and injected rule. Fixed-size schedules use `std::array`.
Configuration errors identify the field and the expected constraint.
