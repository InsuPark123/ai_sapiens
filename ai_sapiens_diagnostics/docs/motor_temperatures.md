# K1 motor and IMU temperature telemetry

Motor and IMU temperature collection are **enabled by default** in normal
`ros2 launch ai_sapiens_bringup k1.launch.py`. It requires `diagnostics_enabled:=true`.
Disable independently with `motor_temperatures_enabled:=false` and
`imu_temperature_enabled:=false`. When disabling diagnostics for an A/B comparison,
disable both temperature paths too. Direct Xacro expansion defaults to OFF;
the normal launch explicitly passes its enabled setting.

Data path:

1. Existing DHI reads `Present Temperature` (QC inverter °C, address 570) and
   `Present Temperature (Motor)` (QC motor °C, address 571) for the 23 K1 motors.
2. Standard `motor_temperature_broadcaster` exposes cached GPIO states through
   `/motor_temperature_broadcaster/dynamic_joint_states` at 5 Hz.
3. `ai_sapiens_diagnostics/temperature_node` checks the incoming values and DHI
   sample progression/freshness, then publishes `diagnostic_msgs/msg/DiagnosticArray`
   on `/diagnostics` at 5 Hz. It never opens a DXL port or calls hardware services.
4. k1-status consumes `temperature/<component>/dxl/<id>` with `robot_id`, `dxl_id`,
   `unit=degC`, `quality=FRESH|STALE`, `motor_temperature_c`, `inverter_temperature_c`.

Temperature values require a recent pair of finite byte-range values, advancing
DHI sample counters in the same producer session, `sample_received/sample_fresh=true`,
and `read_dhi_result=0`. Both streams expire after 0.6 seconds. A new session or
read error requires a subsequent temperature frame before becoming live again.
Repeated cached broadcaster messages cannot keep a stopped DHI source live.
These are receipt/source-age checks, not exact per-packet sample synchronization.

No thermal thresholds, torque policy, recovery permission, or shutdown action are
added. Existing motor fault diagnostics remain separate. In k1-status, missing or
STALE data preserves last-good values as historical rather than live.

The IMU uses a separate standard `imu_temperature_broadcaster` at 5 Hz and the
same temperature node. Its source identity is `k1_imu/dxl/202/202`; diagnostics
are named `temperature/k1_imu/dxl/202` with `imu_temperature_c`, `unit=degC`,
`robot_id`, `dxl_id=202`, and `quality=FRESH|STALE`. Motor and IMU caches, stamps
and source progression are independent; either stream can be disabled.

**5 Hz is the publication rate, not the hardware read rate.** The two read fields
are added to every motor's existing 1 kHz group transaction: 46 extra payload bytes
across the 23 motors per cycle, plus protocol/framing/indirect mapping effects.
The existing all-interface joint-state broadcaster also sees the extra states.
No claim of zero execution/network cost is made. Before deployment to another
robot, verify model/register compatibility and perform a matched OFF/ON 1 kHz
comparison with actual loaded DHI/SDK versions, startup/steady-state separation,
raw loop and worker timing, failures, and fallback modes.

## Build and run

In a workspace with the existing DHI/SDK and ROS environments sourced:

```bash
colcon build --symlink-install --packages-select \
  ai_sapiens_description ai_sapiens_diagnostics ai_sapiens_bringup
source install/setup.bash
ros2 launch ai_sapiens_bringup k1.launch.py robot_id:=k1-19
```

Set the robot ID to match the actual device. If the app uses this workspace's normal
launch without passing `motor_temperatures_enabled:=false`, collection is enabled
by default on the next bringup. This feature is not added to `k1_mujoco.launch.py`.
Do not use this K1 rev1 configuration unchanged on robots whose DHI model files lack
the two temperature fields or whose GPIO names/IDs differ.

## Communication overhead from additional state interfaces

Although ROS state values are doubles, DXL packets use **the register byte sizes in
the model file**. The QC registers `Present Temperature` (570) and
`Present Temperature (Motor)` (571) are each 1 byte. These fields are packed into
DHI indirect reads; the entire address range from error code 153 to 571 is not read.
Distinguish adding a ROS alias for an existing register from adding a new register
to the hardware read list.

Variables:

- `N`: DXL count on one physical bus. K1 rev1 has 5 on the right arm, 5 on the left arm, 6 on the left leg, and 7 on the right leg.
- `B`: Additional read bytes per DXL. One temperature field adds 1; the two fields add 2.
- `R`: UART baud rate for that bus, currently 6,000,000 bit/s.
- `f`: Hardware read frequency, currently 1,000 Hz, distinct from the 5 Hz web/ROS publication rate.

When adding fields to the existing group transaction without changing the ID count,
framing, or transfer mode:

```text
Additional response data/cycle     ΔP = N × B                       [byte]
Additional response data/second    ΔV = N × B × f                   [byte/s]
Additional UART wire time          ΔT = N × B × 10 / R × 1,000,000 [µs/cycle]
Additional UART utilization        ΔU = N × B × 10 × f / R × 100   [% point]
```

8N1 uses 1 start + 8 data + 1 stop bit, for 10 bits per byte. Do not add the fixed
protocol header again for every field. If new requests/responses are introduced or
padding, segmentation, or byte stuffing changes, include the additional bytes and
turnaround separately.

| Bus | N | One temperature field: extra bytes / µs | Two fields: extra bytes / µs | Additional utilization for two fields |
|---|---:|---:|---:|---:|
| Right arm | 5 | 5 / 8.33 | 10 / 16.67 | 1.67%p |
| Left arm | 5 | 5 / 8.33 | 10 / 16.67 | 1.67%p |
| Left leg | 6 | 6 / 10.00 | 12 / 20.00 | 2.00%p |
| Right leg | 7 | 7 / 11.67 | 14 / 23.33 | 2.33%p |

Across all 23 motors, the additional response payload is 46 bytes/cycle =
**46,000 bytes/s**. The four buses use separate UARTs, so their summed occupancy of
76.67 µs must not be interpreted as CM loop latency. This is not total Ethernet
traffic either: Ethernet/UDP framing, HAT segmentation/buffering, and ROS message
serialization/publication costs are separate.

For the current private SDK's Optimized responses, DHI's own calculation and parser give:

```text
RX bytes = 8 + N × (Lread + 4)
TX bytes = 13 + N × Lwrite  # Normal READ_AND_WRITE instruction, estimated before stuffing
```

The existing motor GPIO read uses position 4 + velocity 4 + current 2 + error 1 =
11 bytes/DXL. Adding both temperature fields raises this to 13 bytes/DXL. For the
right leg with N=7, the response therefore grows from `8 + 7×(11+4) = 113` to
`8 + 7×(13+4) = 127` bytes, an increase of 14 bytes. This is the increase in the
normal cyclic read response; no command data is added. Initial indirect address
configuration and Optimized reconfiguration traffic are separate startup costs.

### Wire time and read execution time differ

DHI currently recalculates the dynamic return delay using the response length.
If the response schedule has enough slack, the response can start ΔT earlier while
preserving its completion time. This cannot absorb the delay if the delay is already
zero, a fixed delay is used, or the HAT/OS runs late. Thus ΔT above is the increase
in wire time, not a guaranteed increase in read(), write(), or total CM execution
time. Additional copying, conversion, interface lookup, existing full-state JSB
publication, and the temperature broadcaster/node also have CPU costs.

A previous stationary hardware comparison on K1-22 (2026-10-20, tmp DHI eeb157d /
SDK d2049a3, identical controller activation order, OFF/ON/OFF for 60 seconds each)
showed an average CM body cost increase of 8.854–12.698 µs, zero communication
failures, and zero CM body samples over 1 ms. Existing IMU worker samples over 1 ms
remained. These results do not validate the performance of this local change or
another robot. This local integration is **implemented / not verified at 1 kHz on hardware**.

## IMU chip temperature

The normal launch defaults `imu_temperature_enabled` to true. Disable it with
`imu_temperature_enabled:=false` for older IMUs without temperature support.
This option operates independently of motor temperature collection.

- Current ICM42688 model: `Temperature`, address 136, 2-byte signed value, scale 0.01 °C.
  For example, DHI already converts raw -1234 to -12.34 °C. The diagnostics node must not multiply it by 0.01 again.
- The legacy `k1_imu.model` has no `Temperature` field. Do not add address 136 without
  checking the model/firmware. DHI initialization may fail if the model does not support the field.
- The Xacro GPIO state defines the hardware read list; the sensor state defines the ROS export.
  Although declared in both places, the DXL register is read only once, adding **2 bytes/cycle = 2,000 bytes/s**.
- At 6 Mbps with 8N1 and the existing transaction unchanged, the additional wire time is
  `2×10/6,000,000×1e6 = 3.33 µs`. The combined additional response payload for the 23 motors
  and IMU is 48,000 bytes/s. This is neither total Ethernet traffic nor the increase in CM
  wall time, and it excludes initial configuration traffic.
- This is the internal chip temperature, not ambient, battery, or motor temperature.
  The firmware temperature validity bit is not read separately. `quality=FRESH` indicates
  freshness of DHI/read/cache delivery. The status includes
  `firmware_temperature_validity_evaluated=false` to distinguish this from internal sensor validity.
- k1-status displays it through the `imu` object in `/api/temperatures`. The existing
  `samples` and `total=23` remain motor-only.
- Startup and steady-state performance impact is **implemented / not verified at 1 kHz on hardware**.
  The earlier K1-22 motor temperature measurements did not include this IMU path.
  No DHI/SDK source changes or firmware flashing were performed.
