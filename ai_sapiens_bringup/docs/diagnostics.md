# K1 diagnostics and recovery deployment

This checkout contains the production packages and their normal K1 launch integration.
It does not require a robot-specific diagnostics workspace, setup script, web container,
or replacement service runner. Existing app and boot services can keep invoking
`ros2 launch ai_sapiens_bringup k1.launch.py` after building and selecting this checkout.

## Integration scope

The AIS changes are limited to:

| Component | Integration |
|---|---|
| `ai_sapiens_diagnostics` | Independent startup observation, health JSON and standard aggregation configuration |
| `ai_sapiens_recovery` | Optional hardware/controller recovery supervisor |
| `ai_sapiens_bringup` | Start the diagnostic processes from the normal launch; opt in to recovery with `restart:=true` |
| `ai_sapiens_description` | Pass diagnostics, response-evidence and recovery parameters to DHI through Xacro |

Container image creation and the installed DHI/SDK environment are managed by the operator.
This integration does not require a new Dockerfile, compose configuration, service runner,
fixed installation prefix or robot-specific setup script. Preserve each robot's existing
controller, model, policy and firmware configuration when applying these changes.
For example, an older robot with only `rc_broadcaster` must not be assigned an unavailable
`group_rc_broadcaster` just by copying another robot's spawner configuration.

## Dependencies and build

Use a matching diagnostics-capable DHI and SDK already installed in the robot environment.
The reference sources are DHI `f1707863a7a956ff0a4cf4de26d2d64311232935` and SDK
`d2049a3a67cadd7efc0f75c6df2f0c60288a3038`. These are compatibility references, not
image requirements or a substitute for actual binary/1 kHz verification.
The SDK must support `ScopedRxFailureCapture`; the DHI must support runtime diagnostics,
the startup observer, response evidence and recovery when enabled.
The public DHI `main` in `ai_sapiens_ci.repos` alone does not establish those capabilities.

After selecting the intended DHI/SDK environment, build the AIS integration in its normal
workspace. `ai_sapiens_interfaces` and the existing robot dependencies must already be built.
For the common layout, inside the robot container:

```bash
cd /root/ros2_ws
rosdep install --from-paths src/ai_sapiens/ai_sapiens_diagnostics \
  src/ai_sapiens/ai_sapiens_recovery --ignore-src --rosdistro jazzy -y
colcon build --symlink-install --packages-select \
  ai_sapiens_description ai_sapiens_diagnostics ai_sapiens_recovery ai_sapiens_bringup
source install/local_setup.bash
# If DHI/SDK are in tmp_ws, select that intended install after AIS:
source /workspace/tmp_ws/install/local_setup.bash
```

The last source command is an example for robots using `tmp_ws`; use the actual installed
prefix in your image. It does not require cloning or rebuilding DHI/SDK as part of this AIS change.
`diagnostic_aggregator` is a declared runtime dependency (`ros-jazzy-diagnostic-aggregator`).
Do not rebuild/restart a robot that is in operation.

Check installed packages and dynamic linkage, not just source directories:

```bash
ros2 pkg prefix ai_sapiens_diagnostics
ros2 pkg prefix ai_sapiens_recovery
ros2 pkg prefix dynamixel_hardware_interface
ros2 pkg prefix dynamixel_sdk
ldd "$(ros2 pkg prefix dynamixel_hardware_interface)/lib/libdynamixel_hardware_interface.so"
ros2 launch ai_sapiens_bringup k1.launch.py --show-args
```

App/boot execution must select the same DHI/SDK and updated AIS install as the terminal.
An unchanged command `ros2 launch ai_sapiens_bringup k1.launch.py` starts the integrated
processes once those packages are built and selected. Rebuilding alone does not replace
an already running process or override a later-sourced old overlay.
Check `/proc/<controller_manager_pid>/maps` to confirm what the running process loaded.

## Robot identity and startup history

Pass `robot_id` and a writable persistent startup directory as launch arguments, or set
`ROBOT_ID` and `DHI_DIAGNOSTICS_STARTUP_DIRECTORY` in the existing app/terminal environment.
Explicit launch arguments take precedence over the environment defaults.
For the common `/workspace` host bind mount:

```bash
ros2 launch ai_sapiens_bringup k1.launch.py robot_id:=k1-23 \
  diagnostics_startup_directory:=/workspace/dhi-diagnostics/startup
```

Use the same values for app/boot launches. Match the web's journal host path to this directory.
Without these settings, the defaults are `unassigned` and `~/.ros/dhi-diagnostics/startup`.
Creating/exporting a container image is outside this source integration. In particular,
`docker commit` does not include a host bind-mounted `tmp_ws`; the operator must include
or provision the actual DHI/SDK install separately.

## Start

Normal bringup enables diagnostics, failed-response capture, standard aggregation,
and the read-only HAT/BMS broadcaster. Automatic recovery remains disabled:

```bash
ros2 launch ai_sapiens_bringup k1.launch.py
```

Use the actual robot ID. The generic default is `unassigned`; it labels app JSON
and does not choose motor IDs, addresses, ROS domain or control settings.
Enable E-stop recovery explicitly:

```bash
ros2 launch ai_sapiens_bringup k1.launch.py restart:=true
```

Recovery restores motor hardware and controllers to the startup state. It does
not resume the policy process or bypass HAT/RC freshness, E-stop, control-input
permission, communication or device-error checks. It requires normal baseline
observation first; it does not repair an initial bringup failure. Run only one
supervisor on the same host/monotonic clock namespace as DHI.

To disable the diagnostics additions (with recovery off):

```bash
ros2 launch ai_sapiens_bringup k1.launch.py \
  diagnostics_enabled:=false diagnostics_response_evidence_enabled:=false
```

HAT telemetry follows `diagnostics_enabled` by default and can be separately
disabled with `diagnostics_hat_telemetry:=false`. No H5 evidence polling or
firmware collector starts automatically.

## App data and startup records

| Output | Publisher | Purpose |
|---|---|---|
| `/diagnostics` | DHI reporters, independent startup observer, controller manager, optional recovery supervisor | Standard `diagnostic_msgs/msg/DiagnosticArray`: device faults, bus failures, initialization and timing |
| `/diagnostics_agg` | Standard `diagnostic_aggregator` | K1 body-part diagnostic tree |
| `/ai_sapiens/health` | `ai_sapiens_diagnostics` health node | `std_msgs/msg/String` JSON: validated device/bus state, freshness, startup and recent events |
| `/ai_sapiens/health/events` | Health node | State-change JSON events; consumers must persist incident history themselves |
| `/hat_state_broadcaster/dynamic_joint_states` | Standard joint-state broadcaster at 5 Hz | Already-read HAT/BMS memory; no new DXL requests |

```bash
ros2 topic echo /diagnostics diagnostic_msgs/msg/DiagnosticArray
ros2 topic echo /diagnostics_agg diagnostic_msgs/msg/DiagnosticArray
ros2 topic echo /ai_sapiens/health std_msgs/msg/String
```

The launch creates a new run UUID per launch. The startup directory defaults to
`~/.ros/dhi-diagnostics/startup` unless an environment default or launch argument is supplied. Set `diagnostics_startup_directory` to a writable persistent
volume for container replacement. The startup observer can replay a failed run;
it is independent of the controller manager process but exits with the parent
launch. See the diagnostics package README for a separately managed observer.
Runtime health history is bounded memory, not a permanent incident database.

Group RX failure does not identify a failed motor by itself. A validated response
boundary means which IDs were confirmed, not which motor or cable physically
failed. Brief failures can recover before 5 Hz publication and may leave counters
without ID evidence. IMU firmware without a configured error register can be fresh
yet report unknown fault coverage. No register is added to the IMU read block here.

## Validation before production use

This integration preserves the existing controller spawners (including
`group_rc_broadcaster`), motor IDs, read/write interfaces, control rate and Optimized
communication settings. Source tests and a local build are not a real-time approval.
For each robot/dependency deployment, compare diagnostics OFF/ON in the same
supported stationary setup: actual loop interval and execution mean/p99/max,
per-bus read/write, jitter, counts/amounts over 1 ms, communication failures and
mode fallback. Separate startup/recovery from steady state, verify loaded binaries,
and retain raw samples. Do not infer p99 from controller-manager cumulative stats.

English: The normal AIS launch owns this integration. Matching DHI/SDK binaries and
same-condition hardware timing tests remain deployment requirements.


## 모터 온도

일반 `ros2 launch ai_sapiens_bringup k1.launch.py`는 모터 온도를 기본 수집합니다.
기존 DHI/SDK의 모델 파일에 `Present Temperature`, `Present Temperature (Motor)`가
지원되어 있어야 합니다. 진단 OFF 비교에는 `motor_temperatures_enabled:=false`도 전달하세요.
데이터는 `/motor_temperature_broadcaster/dynamic_joint_states`를 거쳐 `/diagnostics`에
5 Hz로 발행하며, 기존 k1-status의 모터 온도 표에서 확인할 수 있습니다.
하드웨어 읽기는 1 kHz이므로 통신량이 증가합니다. 모델·baudrate가 같은 로봇이라도
배포 전후 1 kHz 실측이 필요하며, 이번 로컬 통합의 실기 성능 검증은 별도입니다.
[온도 데이터 계약과 통신량 계산](../../ai_sapiens_diagnostics/docs/motor_temperatures.md)을 참고하세요.
