# App health data contract

## 출력과 표시 기준

```bash
ros2 topic echo /diagnostics diagnostic_msgs/msg/DiagnosticArray
ros2 topic echo /ai_sapiens/health std_msgs/msg/String
ros2 topic echo /ai_sapiens/health/events std_msgs/msg/String
```

`/ai_sapiens/health`: reliable + transient-local KeepLast(1), JSON `schema_version=1`.
`/ai_sapiens/health/events`: reliable depth 50, 변경 이벤트. 최근 64개는 snapshot의 `recent_events`에도 포함한다.
이력은 메모리 기반으로 집계기 재시작 시 초기화된다. 영구 보관되는 것은 별도의 startup journal이다.

| 영역 | 내용 | 표시 시 의미 |
|---|---|---|
| `components` | `dhi/<component>/dxl/<comm_id>/<id>`, joint, reason, measurements, evidence | 현재 관측한 ID별 상태 |
| `buses` | DHI phase/stage/target, read/write DHI 및 SDK 결과 | 그룹 실패는 특정 ID의 무응답을 증명하지 않음 |
| `startup.records` | run/attempt, phase, stage, target ID, failure_detail | 초기화 **과거 기록**. READY는 현재 정상 판정이 아님 |
| `known_faults`, `unknown_required` | 관측된 문제와 확인 불가인 필수 장치 목록 | 통신 단절 중의 과거 fault와 현재 상태를 구분 |
| `coverage` | 관측 범위, `controllers=not_collected`, `restart_authorized=false` | 전체 시스템 준비 완료/재가동 허가가 아님 |

주요 `reason`: `OBSERVED_OK`, `DEVICE_REPORTED_FAULT`, `GROUP_READ_FAILED`, `GROUP_WRITE_FAILED`,
`ESTOP_ACTIVE`, `NO_CONFIRMED_SAMPLE`, `SAMPLE_EXPIRED`, `PRODUCER_EXPIRED`, `INITIALIZATION_FAILED`.
`bus/comm_id/dxl_id/joint`로 UI에서 신체 부위와 장치를 표시할 수 있다.

### FAILED 상태의 실패 출처 (failure origin)

`phase=FAILED`만으로 초기화 실패라고 판단하지 않는다. 운전 중 `on_error()`도
같은 phase를 사용한다. Adapter는 장치의 명시적 `failure_origin` 또는 **같은
DiagnosticArray 안의 같은 bus·producer_session·publication_sequence·FAILED phase**를
확인하여 출처를 연결한다. 장치와 bus의 배열 순서는 상관없다. 중복 status name,
중복 KeyValue, 거부된 bus, 과거 배열의 bus는 새 장치 진단의 출처 근거로 쓰지 않는다.

| 출처/통신 결과 | health의 `reason` | 장치 health |
|---|---|---|
| `startup` | `INITIALIZATION_FAILED` | 원본 level ERROR인 장치만 ERROR, 그 외 UNKNOWN |
| `runtime` + read 실패 | `GROUP_READ_FAILED` | UNKNOWN |
| `runtime` + write 실패 | `GROUP_WRITE_FAILED` | UNKNOWN |
| `runtime` + read/write 실패 근거 없음 | `RUNTIME_HARDWARE_FAILED` | UNKNOWN |
| 출처 누락·불일치·충돌 | `HARDWARE_FAILURE_ORIGIN_UNKNOWN` | UNKNOWN |

FAILED의 quality는 STALE 또는 MISSING이다. 그룹 통신 실패가 특정 DXL의 물리적
고장을 증명하지 않으므로 UNKNOWN을 사용한다. 원본 장치 level은
`components[].evidence.diagnostic_level`, 연결 근거는 `failure_origin_source`에 남긴다.
이는 `/ai_sapiens/health` 해석의 수정이며 **원본 `/diagnostics`와 표준 집계의
level/message는 변경하지 않는다.** `restart_authorized=false`도 유지한다.

ROS 메시지 경로만 검증하려면 아래 시험을 실행한다. localhost domain 188에서
synthetic DiagnosticArray → 설치된 health_node → 실제 health 토픽을 검증한다.
controller manager, DHI/SDK, 포트 접근, 모터 명령은 실행하지 않는다.

```bash
python3 src/ai_sapiens/ai_sapiens_diagnostics/test/failure_origin_smoke.py \
  --output /tmp/failure-origin-smoke-new-run
```

출력 directory는 새 경로여야 한다. 기능 시험은 실제 1 kHz 성능 검증을 대체하지 않는다.

### 응답 근거 옵션의 설치 호환성 확인

정식 K1 launch의 `diagnostics_response_evidence_enabled`는 기본 ON이다.
ON은 `diagnostics_enabled:=true`와 응답 근거 수집을 지원하는 DHI/SDK를 필요로 한다.
새 launch는 hardware 프로세스를 시작하기 전에 선택된 DHI와 `ldd`가 실제로 해석한
SDK의 기능 marker를 확인한다. 구버전 DHI가 이 옵션을 조용히 무시하는 상황을 막는다.
mock hardware에는 실제 SDK 확인을 적용하지 않는다.

이 사전 확인(preflight check)은 포트를 열거나 DHI를 인스턴스화하지 않는다. 다만
기능 marker 확인은 ABI·firmware 호환성 또는 1 kHz 성능 승인이 아니다. 실제 로딩 경로,
옵션이 활성화됐다는 bus 진단, 동일 조건 실기 A/B를 별도로 확인해야 한다.
OFF는 기존 기동 경로를 유지하며 원본 ROS level이나 자동 복구 정책을 바꾸지 않는다.

로봇별 read block에는 추가 Hall/raw velocity/BMS 필드가 있을 수 있다.
기존 inventory·state/command interface·통신 설정을 보존하며 통합한다.
기본 ON 설정은 대응 DHI/SDK를 요구하며 로봇별 실기 성능 승인을 의미하지 않는다.

- SDK 결과 `-3001=RX_TIMEOUT`, `-3002=RX_CORRUPT`. DHI `-7`과 SDK `-3001`은 다른 코드다. DXL의 `Error Code` register와도 구분한다.
- `last_read_error_sdk`, `last_write_error_sdk`, 실패 횟수, last error register/E-stop은 복구 후에도 보존한다. 모든 순간 오류의 전체 순서를 저장하는 기능은 아니다.
- `E-stop Active=1`은 장치 관측값이다. 해제 관측만으로 controller/torque를 재시작하지 않는다.
- IMU는 inventory에서 제외하지 않는다. 현재 xacro에 오류 레지스터가 없으므로 수신이 fresh여도 `FAULT_REGISTER_NOT_CONFIGURED/UNKNOWN`이다. IMU 자체 고장이 확인됐다는 뜻은 아니다.
- DHI health 경로는 HAT의 error register와 E-stop을 해석한다. 별도 표준 broadcaster가 기존 HAT/BMS 메모리를 발행하며 health JSON과 구분한다. 근거 없는 단위 변환을 하지 않는다.
- `sample_count`는 DHI가 그룹 데이터 수신 성공을 관측한 횟수다. HAT 내부 캐시의 원본 센서 신선도나 모든 개별 패킷의 완전성까지 보증하지 않는다 (`source_freshness=unknown`).
- `command_applied=unknown`: sync write 성공은 모터 적용 완료 acknowledgement가 아니다.

같은 값이 계속 와도 실제 수신 counter가 증가하면 fresh일 수 있다. 반대로 reporter만 살아 있고 수신 counter가 멈추면 정상으로 연장하지 않는다.
외부 앱/WS gateway 역시 **자신의 마지막 수신 이후 시간**으로 집계기 자체의 연결 끊김을 판정해야 한다. Transient-local의 마지막 정상 snapshot만으로 살아 있다고 판단하지 않는다.
Cyclo HTTP/WS gateway와 UI 연결, CM controller 상태 수집은 이 health 노드의 역할이 아니다.
자동 복구는 별도 recovery 패키지의 opt-in 기능이다.
