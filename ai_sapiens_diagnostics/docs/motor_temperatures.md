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


## 빌드·실행 / Build and run

기존 DHI/SDK와 ROS 환경을 source한 workspace에서:

```bash
colcon build --symlink-install --packages-select \
  ai_sapiens_description ai_sapiens_diagnostics ai_sapiens_bringup
source install/setup.bash
ros2 launch ai_sapiens_bringup k1.launch.py robot_id:=k1-19
```

로봇 ID는 실제 기기에 맞춘다. 앱이 이 workspace의 일반 launch를 사용하고 별도로
`motor_temperatures_enabled:=false`를 전달하지 않으면 다음 bringup부터 기본 ON이다.
이 기능은 `k1_mujoco.launch.py`에는 추가하지 않는다. DHI 모델 파일에 두 온도 항목이
없거나 GPIO 이름/ID가 다른 로봇에는 이 K1 rev1 설정을 그대로 쓰지 않는다.

## state interface 증가에 따른 통신량 계산

ROS의 state 값은 double이어도 DXL 패킷은 **모델 파일의 register byte 크기**로 전송한다.
QC 계열의 `Present Temperature`(570), `Present Temperature (Motor)`(571)는 각각 1 byte다.
이 두 필드는 DHI indirect read에 조밀하게 매핑되므로 153번 오류 코드부터 571번까지의
주소 간격을 모두 읽지 않는다. 이미 읽던 register의 ROS alias만 추가하는 경우와,
실제 read 목록에 새 register를 넣는 경우를 구분해야 한다.

변수:

- `N`: 한 물리 bus의 DXL 수. K1 rev1은 오른팔 5, 왼팔 5, 왼다리 6, 오른다리 7.
- `B`: 한 DXL에 새로 추가되는 read 데이터 byte 수. 온도 하나는 1, 이번 두 항목은 2.
- `R`: 해당 bus의 UART baudrate, 현재 6,000,000 bit/s.
- `f`: 하드웨어 read 주파수, 현재 1,000 Hz. 웹/ROS 발행 주파수 5 Hz와 다름.

기존 group transaction에 필드만 추가하며 ID 수·프레이밍·전송 모드가 유지될 때:

```text
추가 응답 데이터/cycle       ΔP = N × B                       [byte]
추가 응답 데이터/second      ΔV = N × B × f                   [byte/s]
추가 UART 선로 점유 시간     ΔT = N × B × 10 / R × 1,000,000 [µs/cycle]
추가 UART 점유율             ΔU = N × B × 10 × f / R × 100   [% point]
```

8N1은 1 start + 8 data + 1 stop이므로 byte당 10 bit다. 프로토콜 고정 헤더를 매 필드마다
다시 더하지 않는다. 새 요청/응답이 생기거나 padding/분할/byte stuffing이 바뀌면 해당
추가 byte와 turnaround도 별도로 포함해야 한다.

| bus | N | 온도 1항목: 추가 byte / µs | 이번 2항목: 추가 byte / µs | 2항목 추가 점유율 |
|---|---:|---:|---:|---:|
| 오른팔 | 5 | 5 / 8.33 | 10 / 16.67 | 1.67%p |
| 왼팔 | 5 | 5 / 8.33 | 10 / 16.67 | 1.67%p |
| 왼다리 | 6 | 6 / 10.00 | 12 / 20.00 | 2.00%p |
| 오른다리 | 7 | 7 / 11.67 | 14 / 23.33 | 2.33%p |

23개 전체의 추가 응답 payload는 46 byte/cycle = **46,000 byte/s**다.
네 bus는 별도 UART로 동작하므로 각 점유 시간을 합한 76.67 µs를 CM 루프 지연으로
해석하지 않는다. 이것은 Ethernet 전체 트래픽도 아니다: Ethernet/UDP framing과 HAT의
분할·buffering, ROS 메시지 직렬화·발행 비용은 별도다.

현재 private SDK의 Optimized 응답은 DHI 자체 계산과 parser에 따르면:

```text
RX bytes = 8 + N × (Lread + 4)
TX bytes = 13 + N × Lwrite  # 정상 READ_AND_WRITE instruction, stuffing 전 추정
```

모터 GPIO의 기존 read는 position 4 + velocity 4 + current 2 + error 1 = 11 byte/DXL.
온도 두 항목을 넣으면 13 byte/DXL이다. 따라서 N=7 오른다리 응답은
`8 + 7×(11+4) = 113` → `8 + 7×(13+4) = 127` byte로, 14 byte 증가한다.
이는 정상 주기의 read 응답 증가분이며 command 데이터는 추가하지 않는다.
초기 indirect address 설정·Optimized 재설정 통신은 별도 기동 비용이다.

### 선로 점유 시간과 read 실행시간은 다름

현재 DHI는 응답 길이를 사용해 dynamic return delay를 다시 계산한다.
응답 예약에 여유가 있으면 응답 시작을 ΔT만큼 앞당기고 완료 시점을 유지할 수 있다.
반대로 delay가 이미 0이거나 고정 delay를 쓰거나 HAT/OS가 늦으면 지연을 흡수하지 못한다.
따라서 위 ΔT는 wire-time 증가분이지 read(), write(), CM 전체 실행시간 증가의 보장값이 아니다.
추가 복사·변환·interface 조회·기존 전체 상태 JSB 발행·온도 broadcaster/노드의 CPU 비용도 있다.

22호기의 과거 정지 실기 비교(2026-10-20, tmp DHI eeb157d / SDK d2049a3,
같은 controller 활성 순서 OFF/ON/OFF, 각 60초)에서 CM body 평균 비용은 +8.854~12.698 µs,
통신 실패 0, CM body >1ms 0이었다. 기존 IMU worker 1ms 초과 표본은 남아 있었다.
이 수치는 이번 로컬 변경이나 다른 로봇의 성능 통과를 대신하지 않는다.
이번 로컬 통합은 **구현 완료 / 실기 1 kHz 미검증**이다.


## IMU 온도 / IMU chip temperature

일반 launch의 `imu_temperature_enabled` 기본은 true다. 온도를 지원하지 않는 구형 IMU는
`imu_temperature_enabled:=false`로 끈다. 이 옵션은 모터 온도와 독립적으로 동작한다.

- 현재 ICM42688 모델: `Temperature`, address 136, 2 byte signed, scale 0.01 °C.
  예를 들어 raw -1234는 DHI에서 이미 -12.34 °C로 변환된다. 진단 노드에서 다시 0.01을 곱하지 않는다.
- 레거시 `k1_imu.model`에는 `Temperature` 항목이 없다. 모델/펌웨어를 확인하지 않고 136번 주소를
  임의로 추가해서는 안 된다. 모델이 항목을 지원하지 않으면 DHI 초기화에 실패할 수 있다.
- Xacro의 GPIO state는 하드웨어 읽기 목록이고 sensor state는 ROS export다. 두 곳에 선언하지만
  DXL register는 한 번만 읽으므로 추가 payload는 **2 byte/cycle = 2,000 byte/s**다.
- 6 Mbps, 8N1, 기존 transaction 유지 시 추가 wire time은 `2×10/6,000,000×1e6 = 3.33 µs`.
  모터 23개와 IMU를 합하면 추가 응답 payload는 48,000 byte/s다. 이 값은 총 Ethernet 전송량이나
  CM wall-time 증가량이 아니며, 초기 설정 통신도 제외한다.
- 칩 내부 온도이며 외기/배터리/모터 온도가 아니다. firmware의 temperature validity bit를 추가로
  읽지는 않는다. `quality=FRESH`는 DHI/read/cache의 전송 최신성 판정이다. 상태에
  `firmware_temperature_validity_evaluated=false`를 넣어 센서 내부 유효성 판정과 구분한다.
- k1-status는 `/api/temperatures`의 `imu` 객체로 표시한다. 기존 `samples`와 `total=23`은 모터 전용으로 유지한다.
- 기동 전후 성능 영향은 **구현 완료 / 실기 1 kHz 미검증**이다. 과거 22호기 모터 온도 실측에는
  이번 IMU 경로가 포함되지 않았다. DHI/SDK source 수정이나 firmware flash는 하지 않았다.
