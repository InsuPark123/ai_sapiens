# ai_sapiens_sim2real

`ai_sapiens_sim2real` runs ONNX control policies on an AI Sapiens robot. It
loads and validates the mode configuration and all policy assets at startup,
selects control authority and behavior at runtime, performs policy inference in
a realtime-oriented loop, and publishes joint position/gain commands.

> [!CAUTION]
> Joint command publishing is enabled by default. Before running on hardware,
> verify the controller joint order, policy assets, ROS topics, and that the
> operator can select the configured Damping state. Use
> `command_publisher_enabled:=false` for a non-actuating startup check.

## Any2Track adapters

The runtime supports `opentrack_anyadapter` (IMU and joint feedback) and
`any2track_gt_xy_adapter` (also requires pelvis odometry). Both load the exported
ONNX policy, matching `sim2real.yaml`, and a motion CSV.

The checked-in `k1_config.yaml` does not register an Any2Track behavior or selector.
To use an adapter, supply its assets and add a mimic behavior, state, and selector
to your root configuration, then use `ai_sapiens_sim2real.launch.py`.
See [Any2Track deployment](docs/any2track.md) for the configuration contract and
the available offline reference check.

## Runtime at a glance

The package deliberately separates mode selection from policy execution:

```text
ROS callbacks
  IMU / joint states / teleop / API heartbeat / cmd_vel
                         |
                         v
                  SharedControlData
                         |
       realtime loop: sense -> decide -> act -> command
                         |        |       |         |
                         |        |       |         +-> joint command publisher
                         |        |       +-----------> PolicyController
                         |        +-------------------> ModeController
                         +----------------------------> sensor handles
```

Each control tick follows this order:

1. `sense`: copy callback data into the realtime state.
2. `decide`: select failsafe, authority, velocity source, and active behavior.
3. `act`: enter or update the selected policy runtime.
4. `command`: publish position, feedforward, stiffness, and damping commands.

A transition into a policy is applied before `act`, so the new policy enters
and infers on the same tick.

## Startup sequence

Startup intentionally validates configuration before waiting for live sensors:

1. Load and validate the root YAML.
2. Resolve every policy asset and parse every policy `sim2real.yaml`.
3. Load all ONNX models and initialize command output.
4. Emit the `[startup_preflight]` log.
5. Wait for one valid teleop plugin sample.
6. Start the realtime thread and wait for required IMU, joint-state, and
   teleop inputs.
7. Reset the controllers and begin the control loop.

This ordering makes a malformed root config, missing asset, invalid policy
config, or ONNX loading failure visible before sensor startup waits.

The mode runtime starts in the configured initial state, normally `Damping`.
Even after inputs are ready, the startup gate holds that state until the
teleop input matches either `DampingRequested` or `ReadyPoseRequested`.

## Mode model

The active mode has two independent dimensions.

### Terminology

| Term | Example | Meaning |
| --- | --- | --- |
| State | `Velocity` | A named FSM node under `state_machine.states`; owns transitions and references a behavior through `run`. |
| Behavior | `velocity_policy` | A named execution configuration under `state_behaviors`. |
| Behavior kind | `policy` | The execution category: `damping`, `posture`, `policy`, or `mimic`. |
| Authority | `MANUAL` | Who owns commands and mode selection: `MANUAL`, `API_WARMUP`, or `API`. |

### Behavior

| Kind | Purpose |
| --- | --- |
| `damping` | Hold measured joint positions with configured damping and zero stiffness. |
| `posture` | Interpolate from measured positions to a configured target posture. |
| `policy` | Run an ONNX policy using its own `sim2real.yaml`. |
| `mimic` | Run a policy with reference-motion playback; internally a policy behavior. |

States and transitions are defined under `state_machine`. A concrete state
references one entry under `state_behaviors` through its `run` field. Abstract
states can provide inherited transitions but cannot become active.

### Authority

| Authority | Velocity owner | Mode selection |
| --- | --- | --- |
| `MANUAL` | Teleop input | Teleop FSM transitions |
| `API_WARMUP` | Zero velocity | Holds the current state |
| `API` | `/cmd_vel` | Mode request service |

API entry requires all of the following:

- a fresh rising edge of the configured API switch;
- a valid, advancing API heartbeat;
- an active state listed in `authority.api_entry.allowed_from_states`;
- teleop velocity within `velocity_neutral_threshold`;
- commanded API velocity within `velocity_neutral_threshold`;
- completion of the configured warmup duration.

Holding the switch high after a rejected entry does not retry. Toggle it off
and on after correcting the rejection. During an API session, heartbeat loss
takes priority over a normal switch release and performs the conservative
manual handoff.

`ApiHeartbeat.sequence` must keep advancing. Repeated messages with an
unchanged sequence become stale even if they continue arriving.

## Decision and safety priority

`ModeController::decide()` applies the system-wide priority below; the failsafe
checks themselves live in `failsafe_reason()` and the authority arbitration in
`compute_authority_change()`. The first matching condition wins:

1. teleop input timeout;
2. teleop Damping input;
3. internal Damping request;
4. policy action-limit violation;
5. bad orientation;
6. startup gate;
7. authority transition;
8. velocity source and behavior-state request.

Every failsafe in the first five entries selects the configured initial state,
which is expected to be Damping. Action-limit and bad-orientation checks apply
only while a policy behavior is active.

Finite action targets beyond `2*pi` radians are rejected without changing the
last accepted action or published command. The runtime retries at the next
normal policy tick, up to five retries after the initial failure; six consecutive
unusable actions trigger Damping. Non-finite processed actions (NaN/Inf) and
inference exceptions share this budget. Only an accepted action resets it.
Policy clocks retain their normal per-tick behavior during
retries. In the common ObservationManager path (velocity and ordinary Mimic), a
rejected action or inference exception replaces the latest history slot with the
preceding retained observation. Older frames and the per-tick history cadence
are preserved; repeated failures keep using the last retained sample. On entry,
the seeded observation supplies the fallback. The failed input remains available
in diagnostic output. This does not reset an entire history or guarantee that
the next action is valid.

Each action-limit rejection queues a separate diagnostic file containing all
processed actions and physical sensor values: IMU ZYX roll/pitch/yaw, body-axis gyro rates (not Euler derivatives), and
all joint positions/velocities. Angles use degrees and velocities use degrees
per second. Values and deltas are compared with the previous policy observation,
including a rejected attempt; the first observation has no comparison baseline.
`[JUMP!]` highlights changes exceeding 1000 deg/s for angles or 25000 deg/s^2
for velocities, scaled by the elapsed control periods between observations
(equivalent to 20 deg or 500 deg/s over 20 ms). Euler angle deltas wrap at 180
degrees; near Euler singularities they can be misleading. These are diagnostic
heuristics, not calibrated sensor validity limits or additional failsafe gates.
`[NONFINITE]` and `[PREV_NONFINITE]` identify non-finite measurements. Sensor
diagnostics use physical coordinates even for mirrored policies. The console
prints the attempt/damping status, only invalid actions (limit violations or
NaN/Inf), only `[JUMP!]` sensor rows, and the queued file path.

### Inference fault files

One background writer is shared by all policy runtimes. It writes readable YAML
starting with a `timestamp` in ISO 8601 format with milliseconds and the explicit
Korea time offset (`+09:00`), independent of the container timezone. The next line,
`timestamp_unix`, uses Unix seconds with nine fractional digits, matching the
terminal log time format. Both use the same capture from
`rclcpp::Clock(RCL_SYSTEM_TIME).now()`, independent of `use_sim_time`. This is the
inference capture time, not the later terminal-log or file-write time; each
snapshot also retains its precise `unix_time_ns`. The full sensor/action report follows as a literal
block, followed by the
actual inference observations grouped by term, joint/channel, and history
(oldest to newest). These are the final scaled/clipped inputs in policy
coordinates, including command history replacement and mirror transforms.
Every ONNX call retains its raw input tensors, normalized shapes, input order,
and raw output with round-trip float32 precision. Loco Mimic's extra inference
is included; Any2Track's three inputs and both history layouts are grouped
separately. YAML `.nan`, `.inf`, and `-.inf` preserve non-finite values.
The model path, startup sim2real configuration and joint order are also saved;
the ONNX model itself is not copied.

Each execution creates a unique `run-<UTC timestamp>-<random suffix>` directory
under `ai_sapiens_sim2real/logs/action_faults/` in the source checkout used at
build time, independently of the launch working directory. The whole `logs/`
directory is ignored by Git. The startup-only ROS parameter
`action_fault_directory` overrides this parent directory; set it when deploying
a binary to a different filesystem without its original source checkout.
Container builds use their source mount path, so a bind-mounted checkout makes
the files available in the same package directory on the host. Session directories
use mode `0755` and diagnostic files `0644`, allowing host users to read logs
created by a root container without granting them write access.
Files are named `incident-<id>-attempt-<n>.yaml`. The initial failure and all
five retries produce up to six separate files; a successful recovery gets its
own `incident-<id>-attempt-<n>-recovered.yaml` file instead of further retries. The first failure also contains the
preceding accepted inference, if one exists in that policy episode. Inference
exceptions use the same incident sequence and preserve the inputs available
before the exception. Entry/reset starts a fresh incident and baseline.

Both session directories and files use exclusive creation; existing files are
never truncated or automatically deleted. Serialization and disk writes run
only in the worker. The control thread copies snapshots and attempts a
nonblocking enqueue (16 records / 32 MiB maximum queued snapshot payload).
An unavailable/full queue drops that diagnostic record and reports it; it does
not delay or change control. The writer stops recording on an IO error or a
256 MiB storage quota, counting existing files under the parent at startup.
The quota is local to one writer, not coordinated across concurrent processes.
`(queued)` in the console is not a write-success confirmation: asynchronous
failures are reported separately. Orderly shutdown drains the queue and each
file is flushed; abrupt termination can lose queued records or leave a partial
file, which is preserved. Recording does not alter retry, clock, history repair,
or command acceptance semantics.

## Configuration

The launch file resolves `robot:=k1` to
`config/k1_config.yaml`. The root configuration connects the following
sections:

| Section | Responsibility |
| --- | --- |
| `robot_joint_order` | Command and feedback order shared by the runtime. |
| `policy_asset_roots` | Search roots for named policy assets. |
| `teleop_input` | Plugin, plugin YAML, velocity-command timeout, and watchdog timeout. |
| `teleop_conditions` | Semantic input conditions such as Damping or Velocity. |
| `selectors` | Selector-code to concrete-state mappings. |
| `state_machine` | Initial state, states, parents, and transitions. |
| `state_behaviors` | Damping, posture, policy, and mimic execution details. |
| `authority` | API entry checks, warmup, and default velocity state. |
| `mimic_defaults` | Optional shared defaults for mimic playback. |

The reference configuration is
[`config/k1_config.yaml`](config/k1_config.yaml), and its Radiomaster mapping
is [`config/teleop/radiomaster_pocket.yaml`](config/teleop/radiomaster_pocket.yaml).
`teleop_input.vel_command_timeout` zeros a stale manual velocity command without
marking the teleop input unavailable. `teleop_input.timeout` triggers the full
input-loss failsafe. The velocity timeout must not exceed the input timeout; when
it is omitted, it inherits the input timeout for backward compatibility.

### Policy assets

An `asset` entry is resolved below each `policy_asset_roots` directory:

```text
<asset>/
  exported/
    policy.onnx
  params/
    sim2real.yaml
    <mimic motion>.csv       # mimic only
```

For example:

```yaml
policy_asset_roots:
  - ../assets/k1

state_behaviors:
  velocity_policy:
    kind: policy
    asset: locomotion/velocity/walk_default
```

This resolves to:

```text
assets/k1/locomotion/velocity/walk_default/exported/policy.onnx
assets/k1/locomotion/velocity/walk_default/params/sim2real.yaml
```

Use `policy_path` and `sim2real_yaml_path` together when the standard asset
layout is not suitable.

For controller MoE exports (`dance_motion_reference_v2` and `dance_motion_reference_v3`), deployment uses
`exported/policy.onnx`, `params/sim2real.yaml` and the configured motion CSV.
Without an exported `training_reference_file`, reference data is derived from the CSV. The runtime derives foot poses
and sole clearance from the CSV and the installed K1 model, estimates contact
with a smooth 8–25 mm clearance transition, and extracts dance gestures in memory.
Live velocity commands drive the foot planner, leg IK and root shift each tick.
The planner reads the same `MotionReference` already loaded for Mimic playback.
Its joint targets go through the existing motion observation functions and joint
mapping; ONNX inference, action processing and command publishing remain shared
with the other Mimic policies. Seeking a new frame clears the previous targets.

V3 derives movement intent from the request after deadband and before velocity
smoothing. Releasing all movement axes prevents new supplemental placements
immediately, while an active gesture finishes at its latched endpoint. Planning
no longer shortens newly starting natural swings based on residual filtered speed
after release. Root steering, release rebasing, crouch and velocity observations
still use the filtered velocity. V2 retains its previous settling behavior.
The v3 YAML must declare `finish_active_gesture_no_new_steps` and
`deadbanded_request_before_smoothing`; renaming the version alone is insufficient.

The exported `phase_labels` and `phase_labels_sha256` describe the training input;
they are not deployment file requirements. CSV-derived contact differs from the
training labels, so this mode does not guarantee identical policy observations.
Copy the matching ONNX, deployment YAML, and motion CSV from the intended
training run into the asset layout above. Startup validates the ONNX joint and
observation metadata when present; localized Mimic requires both metadata fields.
Startup logs also include the ONNX `Training run` metadata when available.

Dedicated MoE import, smoke-test, and parity-probe tools are not included in this
checkout. The package tests described below do not establish training parity or
physical balance for a deployed model.

`dance_motion_reference_v4` additionally requires the exported training reference
file and its SHA256, plus the matching monotonic IK settings in `sim2real.yaml`.

A global-position policy also requires fresh `/state_estimator/odom` messages
with `odom` as the world frame and `pelvis` as the base frame. The existing
`run_k1_tmux.sh --sim --dualsense` command starts simulation, policy and teleop;
start the localization stack separately. Mimic entry is rejected while this
feedback is unavailable.

## Build

ROS 2 Jazzy, the workspace package dependencies (including `mujoco_vendor` and
OpenSSL Crypto), and
[ONNX Runtime](https://github.com/microsoft/onnxruntime/releases) are
required. Install ONNX Runtime under `/usr/local`, or pass
`--cmake-args -DONNXRUNTIME_ROOT=<extracted directory>` to `colcon build`.

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to ai_sapiens_sim2real
source install/setup.bash
```

## Run

Start with command output disabled when validating a new configuration:

```bash
ros2 launch ai_sapiens_sim2real ai_sapiens_sim2real.launch.py \
  robot:=k1 \
  command_publisher_enabled:=false
```

After hardware, controllers, inputs, and policies have been verified:

```bash
ros2 launch ai_sapiens_sim2real ai_sapiens_sim2real.launch.py robot:=k1
```

Useful launch arguments include:

| Argument | Default | Meaning |
| --- | --- | --- |
| `robot` | `k1` | Selects `config/<robot>_config.yaml`. |
| `imu_topic` | `/imu_sensor_broadcaster/imu` | Required IMU input. |
| `joint_states_topic` | `/joint_states` | Required joint feedback. |
| `localization_topic` | `/state_estimator/odom` | Pelvis odometry for policies requiring global XY. |
| `localization_world_frame` | `odom` | Expected odometry world frame. |
| `localization_base_frame` | `pelvis` | Expected odometry child frame. |
| `localization_timeout` | `0.5` | Maximum source and reception age in seconds. |
| `localization_align_on_entry` | `true` | Align localization coordinates on policy entry. |
| `control_rate` | `1000.0` | Command loop rate in Hz. |
| `wait_for_ready_timeout` | `30.0` | Input startup timeout; `0` disables it. |
| `joint_command_topic` | `/joint_group_impedance_controller/commands` | Command output. |
| `command_publisher_enabled` | `true` | Enables hardware command publishing. |
| `enforce_position_limits` | `false` | Applies active-policy position limits. |
| `thread_priority` | `50` | Requested FIFO realtime priority. |
| `lock_memory` | `true` | Requests memory locking for the RT thread. |
| `api_heartbeat_topic` | `/ai_sapiens/api_heartbeat` | API heartbeat input. |
| `api_heartbeat_timeout` | `0.2` | Advancing-sequence watchdog timeout. |
| `cmd_vel_topic` | `/cmd_vel` | API velocity command input. |
| `set_mode_by_name_service` | `/ai_sapiens/set_mode_by_name` | Set mode service. |
| `status_log_enabled` | `false` | Enables periodic control-loop status logs. |
| `detailed_status_log` | `false` | Adds sensor/action previews to status logs. |
| `debug_publish_enabled` | `false` | Enables observation and action debug topics. |

Run the following to see the complete launch interface:

```bash
ros2 launch ai_sapiens_sim2real ai_sapiens_sim2real.launch.py --show-args
```

## ROS interfaces

Default runtime interfaces are:

| Direction | Name | Type |
| --- | --- | --- |
| Input | `/imu_sensor_broadcaster/imu` | `sensor_msgs/msg/Imu` |
| Input | `/joint_states` | `sensor_msgs/msg/JointState` |
| Input | `/state_estimator/odom` | `nav_msgs/msg/Odometry` (required by global-XY policies) |
| Input | `/ai_sapiens_rc/status` | `ai_sapiens_interfaces/msg/RcStatus` |
| Input | `/ai_sapiens/api_heartbeat` | `ai_sapiens_interfaces/msg/ApiHeartbeat` |
| Input | `/cmd_vel` | `geometry_msgs/msg/Twist` |
| Output | `/joint_group_impedance_controller/commands` | `ai_sapiens_interfaces/msg/JointImpedanceCommand` |
| Output | `/ai_sapiens/mode_status` | `ai_sapiens_interfaces/msg/ModeStatus` |
| Service | `/ai_sapiens/set_mode_by_name` | `ai_sapiens_interfaces/srv/SetModeByName` |
| Service | `/ai_sapiens/list_modes` | `ai_sapiens_interfaces/srv/ListModes` |

The mode request service accepts requests only under full `API` authority while
the API switch is held and the heartbeat is valid. `ListModes` reports all
concrete states and the subset currently available to the service.

AI Sapiens-specific messages and services are defined in the shared
`ai_sapiens_interfaces` package rather than this runtime package.

When `debug_publish_enabled` is true, the node also publishes:

- `/policy_input/raw_observation`;
- `/policy_output/processed_action`.

## Testing

Build with tests enabled and run the package suite:

```bash
colcon build --packages-up-to ai_sapiens_sim2real \
  --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select ai_sapiens_sim2real
colcon test-result --verbose
```

The suite covers configuration validation, state-machine and startup gates,
failsafe and authority priority, API/mimic handoffs, request-gate concurrency,
action helpers, gait timing, and motion playback.

### Mode transition smoke tests

[`scripts/run_mode_smoke_tests.py`](scripts/run_mode_smoke_tests.py) runs
full-stack smoke scenarios. Each scenario starts a Zenoh RMW router, the
`ai_sapiens_bringup` stack with `use_mock_hardware:=true`, and the sim2real
node, then drives fake RC input and API heartbeats and verifies the resulting
`ModeStatus` sequence: active state, authority, and transition reason.
Scenarios cover the manual FSM path, teleop watchdog failsafes, and API
warmup, release, and heartbeat-loss handoffs.

Run inside the ROS container after sourcing the workspace (requires
`ai_sapiens_bringup` and `rmw_zenoh_cpp`):

```bash
python3 scripts/run_mode_smoke_tests.py                  # all scenarios
python3 scripts/run_mode_smoke_tests.py \
  --scenario manual_sequence --scenario api_teleop_watchdog_ready
```

The script runs every selected scenario, exits non-zero if any expected
sequence did not match, and writes per-scenario `zenoh.log`, `bringup.log`,
`driver.log`, and `sim2real.log` under `/tmp/ai_sapiens_mode_smoke/<scenario>/`.

## Code map

Recommended reading order:

1. [`src/sim2real_node.cpp`](src/sim2real_node.cpp): startup and component wiring.
2. [`src/control_loop.cpp`](src/control_loop.cpp): realtime loop and stage order.
3. [`src/controllers/mode_controller.cpp`](src/controllers/mode_controller.cpp):
   safety, authority, velocity, and state decision priority.
4. [`src/controllers/policy_controller.cpp`](src/controllers/policy_controller.cpp):
   policy loading, entry, per-tick execution, and command transition management.
5. [`src/policy/policy_runtime.cpp`](src/policy/policy_runtime.cpp): observation,
   ONNX inference, and validated target command generation.
6. [`src/config/root_config.cpp`](src/config/root_config.cpp): root schema and
   asset resolution.
7. [`src/config/sim2real_config.cpp`](src/config/sim2real_config.cpp): per-policy
   joint, observation, and action configuration.

The `mode_runtime` directory contains focused state-machine, authority runtime,
and startup helpers.
`SharedControlData` is the explicit data boundary between callbacks, mode
selection, policy execution, and command publication.

## Optional dual RadioMaster / group teleoperation

Use `k1_config.yaml` (`robot:=k1`) for both single and dual RadioMaster operation.
Set `group.enabled: true` for dual operation (currently enabled), and enable
both USB receivers in the MuJoCo bringup. Set it to `false` to restore the legacy
single-input path, including CH8/API control.
Use `--group-device=/dev/input/js1` with `run_k1_tmux.sh` to enable the second USB input.
CH5 on the individual controller selects group participation; `/ai_sapiens/group_status`
reports the actual participation and group input availability.
