# AI Sapiens diagnostics

This standalone ROS 2 node package converts K1 rev1 DHI device/bus diagnostics
and initialization failure records into app health JSON. It is not a ros2_control
controller and does not open DXL ports. The expected inventory is 23 motors,
one HAT and one IMU. Adjust the YAML for other robot configurations.

```text
DHI read/write → fixed-size snapshot → non-RT reporter (5 Hz) → /diagnostics
DHI initialization → startup journal → independent observer ──────┘
                                     ├─ diagnostic_aggregator → /diagnostics_agg
                                     └─ health_node (10 Hz) → /ai_sapiens/health
                                                          → /ai_sapiens/health/events
```

## Installation and launch

Build the package normally in this AIS repository; no separate per-robot workspace
is required. Follow the [bringup guide](../ai_sapiens_bringup/docs/diagnostics.md)
for dependencies, builds, app/boot integration, startup records and 1 kHz validation.
Use the standard entry point below, replacing robot_id with the actual robot name.

```bash
ros2 launch ai_sapiens_bringup k1.launch.py robot_id:=k1-18
```

Normal launch enables diagnostics, capture, standard aggregation, HAT telemetry,
and motor/IMU temperatures. It does not enable automatic recovery.
Motor temperatures add two bytes per motor to the hardware read data; disable
with `motor_temperatures_enabled:=false` when needed. IMU temperature reads another
two bytes and can be disabled with `imu_temperature_enabled:=false`. Older IMU
models may lack the temperature field. See the
[temperature usage, data contract and traffic calculations](docs/motor_temperatures.md).
Response capture requires matching feature-diagnostics DHI/SDK versions. Before
opening ports, launch checks capability markers in the selected DHI and resolved
SDK. This does not replace ABI or physical 1 kHz validation; verify the selected
DHI/SDK and performance separately. Diagnostics default to OFF in the underlying
Xacro files, preserving the behavior of other launches.

## App integration

- `/diagnostics`: standard `diagnostic_msgs/msg/DiagnosticArray`, with original DHI/CM status.
- `/diagnostics_agg`: standard `diagnostic_aggregator` output grouped by body part; faults are not hidden.
- `/ai_sapiens/health`: `std_msgs/msg/String`, JSON schema 1, reliable/transient-local depth 1.
- `/ai_sapiens/health/events`: state-change JSON, reliable depth 50.
- Recent events use bounded memory; the app/backend must persist permanent incident history.
- Current state, data freshness, past faults and startup records are distinct. Healthy diagnostics do not authorize a restart.

See the [JSON, failure-origin and topic contract](docs/app_contract.md) and
[standard body-part aggregation YAML](config/k1_aggregator.yaml).
CRC-valid response boundaries indicate which IDs have confirmed responses, not
a confirmed physical fault location. A brief fault that clears before the 5 Hz
publication may leave no published boundary evidence.

## Starting diagnostics before bringup

For independently managed services, pass the same startup directory and run ID
to both processes. The run ID below is an example; use a new value for each bringup.

```bash
ros2 launch ai_sapiens_diagnostics health.launch.py \
  robot_id:=k1-18 start_startup_observer:=true start_standard_aggregator:=true \
  startup_directory:=/data/dhi-diagnostics/startup startup_run_id:=example-run-001
```

In a separate terminal:

```bash
ros2 launch ai_sapiens_bringup k1.launch.py robot_id:=k1-18 \
  start_diagnostics_nodes:=false \
  diagnostics_startup_directory:=/data/dhi-diagnostics/startup \
  diagnostics_run_id:=example-run-001
```

Do not start duplicate observer/health/aggregator processes. Separate containers
need a shared volume for the journal files and matching ROS domain/RMW settings.

## Tests and scope

`test/` covers adapter behavior, freshness, device faults, launch, description and
the H5 protocol. These functional tests do not replace physical 1 kHz comparisons.
The HAT/BMS broadcaster publishes only memory that has already been read.
The [H5 collector](docs/hat_collector.md) is opt-in and does not start in default bringup.
If the existing IMU read block lacks a fault register,
`FAULT_REGISTER_NOT_CONFIGURED` means limited observation coverage, not a confirmed IMU fault.

Standard ROS diagnostics feed an independent app health node. The normal AIS
launch integrates these packages; hardware timing approval remains separate.
