# AI Sapiens diagnostics

K1 rev1의 DHI 장치/bus 진단과 초기화 실패 기록을 앱용 health JSON으로 전달하는
독립 ROS 2 노드 패키지다. ros2_control controller가 아니며 DXL 포트를 열지 않는다.
대상 inventory는 모터 23개·HAT 1개·IMU 1개다. 다른 로봇 구성에는 YAML을 맞춰야 한다.

```text
DHI read/write → fixed-size snapshot → non-RT reporter (5 Hz) → /diagnostics
DHI initialization → startup journal → independent observer ──────┘
                                     ├─ diagnostic_aggregator → /diagnostics_agg
                                     └─ health_node (10 Hz) → /ai_sapiens/health
                                                          → /ai_sapiens/health/events
```

## 설치와 실행

패키지는 이 AIS 레포에서 일반 빌드한다. 별도 호기별 workspace가 필요하지 않다.
전체 의존성·빌드·앱/자동실행·초기화 기록·1 kHz 검증 절차는
[bringup 사용법](../ai_sapiens_bringup/docs/diagnostics.md)을 따른다.
정식 진입점은 다음과 같다. robot_id에는 실제 호기 이름을 넣는다.

```bash
ros2 launch ai_sapiens_bringup k1.launch.py robot_id:=k1-18
```

일반 launch는 diagnostics/capture/표준 집계/HAT telemetry/모터·IMU 온도 ON, 자동복구 OFF다.
온도는 하드웨어 읽기 데이터를 모터당 2 byte 늘린다. 필요하면
`motor_temperatures_enabled:=false`로 끈다. IMU 온도는 별도로 2 byte를 읽으며
`imu_temperature_enabled:=false`로 끈다. 구형 IMU 모델에는 온도 항목이 없을 수 있다.
[온도 사용법·데이터 계약·통신량 계산](docs/motor_temperatures.md)을 참고한다.
응답 capture는 대응하는 feature-diagnostics DHI/SDK가 필요하며 launch가 포트 접근 없이
선택된 DHI와 resolved SDK의 기능 marker를 사전 검사한다. 이것은 ABI 또는 실기 1 kHz
검증을 대신하지 않는다. DHI/SDK 선택과 성능을 별도로 확인한다.
하위 xacro의 진단 기본값은 OFF라 다른 launch의 기존 동작은 유지된다.

## 앱 연결

- `/diagnostics`: 표준 `diagnostic_msgs/msg/DiagnosticArray`, DHI/CM 등의 원본 상태.
- `/diagnostics_agg`: 표준 `diagnostic_aggregator`가 부위별로 집계. 오류를 숨기지 않는다.
- `/ai_sapiens/health`: `std_msgs/msg/String`, JSON schema 1, reliable/transient-local depth 1.
- `/ai_sapiens/health/events`: 상태 변화 JSON, reliable depth 50.
- 최근 이벤트는 bounded memory이며 영구 장애 이력은 앱/백엔드가 저장한다.
- 현재 상태, 데이터 최신성, 과거 오류, 초기화 기록은 구분한다. 정상 진단은 재가동 허가가 아니다.

[상세 JSON/실패 출처/토픽 계약](docs/app_contract.md),
[표준 부위 집계 YAML](config/k1_aggregator.yaml)을 참고한다.
CRC 유효 응답 경계는 ID별 응답 확인 범위이며 물리 고장 위치 확정이 아니다.
짧은 오류가 5 Hz 발행 전에 정상 복귀하면 경계가 발행되지 않을 수 있다.

## Bringup보다 먼저 진단 프로세스 실행

독립 서비스로 관리하려면 같은 startup directory/run ID를 두 프로세스에 전달한다.
아래 run ID는 예시이며 매 bringup 새 값을 사용한다.

```bash
ros2 launch ai_sapiens_diagnostics health.launch.py \
  robot_id:=k1-18 start_startup_observer:=true start_standard_aggregator:=true \
  startup_directory:=/data/dhi-diagnostics/startup startup_run_id:=example-run-001
```

별도 터미널에서:

```bash
ros2 launch ai_sapiens_bringup k1.launch.py robot_id:=k1-18 \
  start_diagnostics_nodes:=false \
  diagnostics_startup_directory:=/data/dhi-diagnostics/startup \
  diagnostics_run_id:=example-run-001
```

같은 observer/health/aggregator를 중복 실행하지 않는다. 서로 다른 컨테이너는 같은
journal 파일을 볼 수 있는 공유 volume과 같은 ROS domain/RMW 통신 설정이 필요하다.

## 시험과 범위

`test/`는 adapter·freshness·device fault·launch·description·H5 protocol의 기능 시험이다.
실기 1 kHz 비교를 대체하지 않는다. HAT/BMS broadcaster는 이미 읽은 메모리만 발행한다.
[H5 collector](docs/hat_collector.md)는 opt-in 도구이며 기본 bringup에서 시작하지 않는다.
기존 IMU read block에 fault register가 없다면 `FAULT_REGISTER_NOT_CONFIGURED`는
관측 범위 부족을 의미하며 IMU 고장 확정이 아니다.

English: Standard ROS diagnostics feed an independent app health node. The normal
AIS launch integrates these packages; hardware timing approval remains separate.
