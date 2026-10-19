# Any2Track adapter deployment

## Supported runtimes

| `runtime_type` | Observation size | Live inputs |
| --- | --- | --- |
| `opentrack_anyadapter` | 132 | Pelvis IMU attitude/gyro and joint positions/velocities |
| `any2track_gt_xy_adapter` | 134 | The same inputs plus fresh pelvis odometry XY |

Both use `PolicyRuntime`, `Any2TrackPolicy`, and `OnnxInference`. The runtime
validates the exported schema and ONNX ABI at startup.

## Configure and launch

This checkout provides the runtime and the general
[`ai_sapiens_sim2real.launch.py`](../launch/ai_sapiens_sim2real.launch.py).
Dedicated BTS/Redred launch files, root configurations, and selectors are not
included. The default [`k1_config.yaml`](../config/k1_config.yaml) does not select
an Any2Track adapter.

1. Supply a compatible asset bundle with `exported/policy.onnx`,
   `params/sim2real.yaml`, and the motion CSV under `params/`.
2. Add a `kind: mimic` behavior referencing that asset and motion. Use 50 Hz,
   `motion_format: mjlab`, and a finite frame-aligned playback window. Set
   `on_complete` to a configured fallback state such as `Velocity`; `stay` is
   unsupported for Any2Track.
3. Register a concrete state using that behavior and configure the transitions
   and selector needed to enter it. Choose a ReadyPose appropriate to the motion.
4. Build and source the workspace. Start robot or simulator bringup separately,
   then launch the policy with your configured root YAML.

For a custom `config/<robot>_config.yaml`, rebuild/install the package and pass
`robot:=<robot>` to the general launch. For example, after adding the behavior to
`k1_config.yaml`, check startup with:

```bash
ros2 launch ai_sapiens_sim2real ai_sapiens_sim2real.launch.py \
  robot:=k1 command_publisher_enabled:=false
```

The general launch starts the policy node; it does not start a simulator or
localization stack. Model assets are supplied separately and are ignored by Git.

## Odometry and attitude

The XY runtime subscribes to `/state_estimator/odom` by default, with
`header.frame_id=odom` and `child_frame_id=pelvis`. Configure
`localization_topic`, `localization_world_frame`, `localization_base_frame`,
`localization_timeout`, and `localization_align_on_entry` through the general
launch. Use the same ROS domain and middleware as the producer.

At entry, the runtime aligns IMU yaw to the configured start frame and applies
that fixed rotation to odometry XY displacement from the entry position. IMU
and odometry XY axes must share the same world heading. Roll/pitch and gravity
come from the IMU. Invalid attitude or missing/stale required odometry requests
`authority.default_velocity_state`. Motion completion requests the behavior's
`on_complete` state. The orientation-only adapter does not require odometry.

The XY export describes ground-truth, simulation-only training. ROS execution
uses the configured odometry source; it does not substitute simulator ground
truth. Deployment with measured odometry needs separate validation.

## Deployment contract

| ONNX input | Shape | Contents |
| --- | --- | --- |
| `obs` | `[1,132]` or `[1,134]` | Ordered terms from the YAML; the 134-element variant includes XY error |
| `velocity_history` | `[1,20,75]` | Oldest → newest; gravity, gyro×0.05, joint offset, joint velocity×0.05, previous motor targets |
| `history` | `[1,75,79]` | Channels × oldest → newest; gyro×0.05, gravity, joint offset, joint velocity×0.05, resulting motor targets |

The adapter history pairs **pre-action sensors with that action's motor targets**.
At reset, both histories are filled with measured initial sensor/target values;
there is no history leakage across policy transitions. Velocity history includes
the current observation. The adapter history includes only preceding transitions.

The first inference uses reference frame **start + 1**, matching training reset.
`continuous_actions` is a 23-element tanh residual; the command is
`reference_joint_position + scale * residual`. `last_motor_targets` stores that
absolute target, not the residual or the command blended by the controller.
`estimated_velocity` is the model's auxiliary output; the graph already feeds it
into the actor. The runtime requests `continuous_actions` only.

`path_position_error` is reference-minus-robot XY rotated by **robot yaw only**.
`motion_anchor_ori_b` is the first two columns of the relative rotation matrix,
flattened in row-major order. Reference foot heights follow the training site's
order: left foot, right foot, left foot top, right foot top.

## Offline reference check

The built `any2track_reference_check` executable uses MuJoCo ground-truth XY.
It exercises an offline rollout rather than ROS sensors or the pelvis estimator.
After building and sourcing the workspace, invoke it with your bundle and scene:

```bash
ros2 run ai_sapiens_sim2real any2track_reference_check \
  /path/to/bundle /path/to/scene.xml
```

Optional positional arguments are a numeric step count, an output CSV, and a
motion CSV filename relative to the bundle's `params/` directory:

```bash
ros2 run ai_sapiens_sim2real any2track_reference_check \
  /path/to/bundle /path/to/scene.xml 100 /tmp/rollout.csv motion.csv
```

Without these arguments, it runs the full clip using `params/redred_k1.csv`.
It uses 50 Hz inference, 200 Hz PD control, Euler integration, Newton 10
iterations, and the exported gains/torque limits. The scene must contain a
free-base K1 with 23 direct torque actuators.

## Reference loading

The runtime uses the existing `MotionReference` CSV reader. At startup it derives
reference velocities using central differences (one-sided at the ends), and
computes the four training foot-point heights from the installed
`ai_sapiens_description/mujoco/k1/k1.xml` model. This is offline FK of the CSV
reference, not a live MuJoCo position or an estimator input. No FK runs in the
control loop. CSV joint names are mapped to policy/model order.

The ONNX input/output shapes and exported observation/action schema are still
validated. There is no ONNX/CSV hash binding and no dependency on
`any2track_reference.bin`, `any2track_reference.yaml`, a training NPZ, or a separate
conversion command. The original deployment assets remain ignored by Git, so
copy ONNX, YAML, and the configured CSV to the target machine as usual.

## Validation scope

Dedicated Any2Track ROS smoke tests, training-parity generators/probes, and
rollout-viewer scripts are not included in this checkout. Historical results
from other branch configurations do not validate this merged tree. Use the
[package testing instructions](../README.md#testing) for the available suite;
the offline reference check does not establish odometry-based sim2sim or
physical sim2real performance.
