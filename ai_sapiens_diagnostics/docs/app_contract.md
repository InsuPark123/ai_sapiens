# App health data contract

## Outputs and display semantics

```bash
ros2 topic echo /diagnostics diagnostic_msgs/msg/DiagnosticArray
ros2 topic echo /ai_sapiens/health std_msgs/msg/String
ros2 topic echo /ai_sapiens/health/events std_msgs/msg/String
```

`/ai_sapiens/health`: reliable + transient-local KeepLast(1), JSON `schema_version=1`.
`/ai_sapiens/health/events`: reliable depth 50, change events. The latest 64 events
also appear in the snapshot's `recent_events`. History is stored in memory and
resets when the aggregator restarts. The separate startup journal is persistent.

| Section | Contents | Display semantics |
|---|---|---|
| `components` | `dhi/<component>/dxl/<comm_id>/<id>`, joint, reason, measurements, evidence | Currently observed state per ID |
| `buses` | DHI phase/stage/target, read/write DHI and SDK results | Group failure does not prove that a particular ID failed to respond |
| `startup.records` | run/attempt, phase, stage, target ID, failure_detail | **Historical** initialization records; READY does not establish current health |
| `known_faults`, `unknown_required` | Observed faults and required devices whose state is unknown | Distinguish historical faults from current state during communication loss |
| `coverage` | Observation scope, `controllers=not_collected`, `restart_authorized=false` | Does not indicate full system readiness or restart authorization |

Key `reason` values: `OBSERVED_OK`, `DEVICE_REPORTED_FAULT`, `GROUP_READ_FAILED`, `GROUP_WRITE_FAILED`,
`ESTOP_ACTIVE`, `NO_CONFIRMED_SAMPLE`, `SAMPLE_EXPIRED`, `PRODUCER_EXPIRED`, `INITIALIZATION_FAILED`.
Use `bus/comm_id/dxl_id/joint` to identify body parts and devices in the UI.

### Failure origin in the FAILED state

Do not interpret `phase=FAILED` alone as an initialization failure. Runtime
`on_error()` uses the same phase. The adapter links the origin using an explicit
device `failure_origin`, or a matching **bus, producer_session, publication_sequence
and FAILED phase within the same DiagnosticArray**. Device/bus array order does
not matter. Duplicate status names, duplicate KeyValue entries, rejected buses
and buses from earlier arrays cannot establish the origin of a new device diagnostic.

| Origin / communication result | Health `reason` | Device health |
|---|---|---|
| `startup` | `INITIALIZATION_FAILED` | ERROR only for devices whose original level is ERROR; otherwise UNKNOWN |
| `runtime` + read failure | `GROUP_READ_FAILED` | UNKNOWN |
| `runtime` + write failure | `GROUP_WRITE_FAILED` | UNKNOWN |
| `runtime` without read/write failure evidence | `RUNTIME_HARDWARE_FAILED` | UNKNOWN |
| Missing, mismatched or conflicting origin | `HARDWARE_FAILURE_ORIGIN_UNKNOWN` | UNKNOWN |

FAILED quality is STALE or MISSING. UNKNOWN is used because a group communication
failure does not establish a physical fault in a particular DXL. The original
device level is retained in `components[].evidence.diagnostic_level`, and the
linking evidence in `failure_origin_source`. This changes the interpretation in
`/ai_sapiens/health`; **the original `/diagnostics` and standard aggregation
level/message remain unchanged.** `restart_authorized=false` is also preserved.

The following test validates only the ROS message path: synthetic DiagnosticArray
→ installed health_node → actual health topic, on localhost domain 188. It does
not run controller manager, DHI/SDK, port access or motor commands.

```bash
python3 src/ai_sapiens/ai_sapiens_diagnostics/test/failure_origin_smoke.py \
  --output /tmp/failure-origin-smoke-new-run
```

The output directory must be a new path. Functional testing does not replace
physical 1 kHz performance validation.

### Installed compatibility checks for response evidence

The standard K1 launch enables `diagnostics_response_evidence_enabled` by default.
ON requires `diagnostics_enabled:=true` and DHI/SDK support for response-evidence
capture. Before starting hardware processes, launch checks capability markers
in the selected DHI and the SDK actually resolved by `ldd`. This prevents older
DHI versions from silently ignoring the option. Physical SDK checks do not apply
to mock hardware.

The preflight check does not open ports or instantiate DHI. Capability markers
do not establish ABI/firmware compatibility or approve 1 kHz performance. Verify
the actual loaded paths, bus diagnostics confirming that the option is enabled,
and physical A/B measurements under identical conditions separately. OFF preserves
the existing startup path and does not change original ROS levels or automatic
recovery policy.

Robot-specific read blocks may contain additional Hall/raw velocity/BMS fields.
Preserve the existing inventory, state/command interfaces and communication
settings during integration. Default ON requires matching DHI/SDK versions and
does not imply physical performance approval for each robot.

- SDK results: `-3001=RX_TIMEOUT`, `-3002=RX_CORRUPT`. DHI `-7` and SDK `-3001` are different codes, also distinct from the DXL `Error Code` register.
- `last_read_error_sdk`, `last_write_error_sdk`, failure counts and last error register/E-stop values survive recovery. They do not record the complete sequence of every transient fault.
- `E-stop Active=1` is a device observation. Observing its release alone does not restart controllers or torque.
- The IMU remains in the inventory. Since the current Xacro has no fault register, even fresh reception yields `FAULT_REGISTER_NOT_CONFIGURED/UNKNOWN`; this does not confirm an IMU fault.
- The DHI health path interprets the HAT error register and E-stop. A separate standard broadcaster publishes existing HAT/BMS memory, distinct from health JSON. Do not apply unsupported unit conversions.
- `sample_count` counts successful group receptions observed by DHI. It does not guarantee freshness of the original sensors behind the HAT cache or completeness of every individual packet (`source_freshness=unknown`).
- `command_applied=unknown`: successful sync write is not an acknowledgement that the motor applied the command.

Unchanging values can still be fresh when actual reception counters advance.
A live reporter with stopped reception counters must not extend healthy status.
External apps/WS gateways must also detect aggregator disconnection using
**time since their own last reception**. A transient-local last-good snapshot
alone does not establish liveness. Cyclo HTTP/WS gateway/UI integration and CM
controller status collection are outside this health node's responsibilities.
This diagnostics integration only observes state; it does not perform automatic
hardware recovery.
