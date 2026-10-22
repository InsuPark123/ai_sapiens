# AI Sapiens K1 recovery

E-stop/통신 오류 후 같은 controller manager 프로세스에서 모터 hardware와 controller를
재활성화하는 선택적 ROS 2 supervisor다. 진단 수신만으로 재가동하지 않으며 기본 OFF다.

## 사용

일치하는 feature-diagnostics DHI/SDK와 이 AIS 레포를 빌드한 뒤 정식 launch를 사용한다.
[의존성·생산 배포·진단 사용법](../ai_sapiens_bringup/docs/diagnostics.md)을 참고한다.

```bash
ros2 launch ai_sapiens_bringup k1.launch.py robot_id:=k1-18 restart:=true
```

`restart:=true`는 DHI recovery/auto_rearm과 supervisor를 함께 활성화하고 startup 모드를
선택한다. 기본 `restart:=false`에서는 supervisor를 시작하지 않는다. 기존
`k1_recovery_test.launch.py`도 같은 정식 launch에 위임하며 supervisor를 중복 생성하지 않는다.

## 복구 시퀀스

`RUNNING → STOPPING → WAIT_RELEASE → RESTORING → ARMING → RECOVERED_STARTUP`

1. 정상 hardware/controller/HAT 기준선 관측 이후의 장애를 처리한다. 초기 bringup 실패 복구는 대상이 아니다.
2. command controller와 joint-state broadcaster를 중지하고 모터 4개 hardware를 정리한다.
3. HAT/RC 수신 최신성, 실제 Realtime Tick 진행, hardware_ok, E-stop 해제 및
   `is_control_input_safe`가 안정적으로 만족되는지 확인한다. 조종기 OFF이면 대기할 수 있다.
4. DHI가 단독으로 포트를 소유하며 설정·통신·장치 오류·위치·허용 신호를 검증한다.
5. 현재 위치에 명령을 동기화하고 토크를 활성화한 뒤 broadcaster/controller를 재활성화한다.

startup은 bringup 직후 상태 복원이며 정책 프로세스나 조종기 모드를 재시작하지 않는다.
명시적 `recovery_rearm_mode:=damping`에서는 kp/feedforward=0, 지정 kd를 유지한다.
Damping은 자세 유지가 아니며 정책 복귀 API는 없다. 정상 운전으로 돌아가려면 bringup을 재시작한다.
HAT/IMU는 복구 대상이 아니지만 기존 동작과 진단에서 제거하지 않는다.

## 허용 신호와 실패 처리

`/ai_sapiens/recovery/permit` (`std_msgs/msg/UInt64`)은 같은 host의 CLOCK_MONOTONIC
허용 만료 시각(ns), 0은 거부다. DHI와 같은 host/monotonic clock namespace에서 실행해야 한다.
원격 앱에서 직접 permit을 발행하지 않는다. 50 ms마다 갱신하고 최대 250 ms까지 유효하다.
최신성/입력 허용이 사라지면 허용을 회수한다. 복수 publisher·만료·과도한 미래 시각은 거부한다.

서비스는 한 번에 하나만 요청한다. 실패 재시도는 bounded이며 timeout에서는 중복 요청을
보내지 않고 BLOCKED로 남는다. Torque OFF 확인에 실패하면 그 사실을 진단에 남긴다.
소프트웨어 permit 상실은 물리 전원 차단을 보장하지 않는다.

## 상태 확인

```bash
ros2 topic echo /diagnostics diagnostic_msgs/msg/DiagnosticArray
ros2 control list_hardware_components
ros2 control list_controllers
```

`ai_sapiens/recovery`의 phase `RECOVERED_STARTUP`, `startup_verified=true`, motor bus의
`recovery_startup_active=true`로 복구 결과를 확인한다. 장치 정상 진단은 재가동 허가가 아니다.
시험 범위는 정지·지지 고정 상태의 명시적 시험으로 관리한다. 실제 펌웨어/로봇별 E-stop 복구와
1 kHz 성능은 기능/mock 시험과 별도로 검증해야 한다. 기존 보행 정책은 이 패키지가 변경하지 않는다.

English: Recovery is opt-in and restores hardware/controllers only after fresh
HAT/RC permission and device checks; it does not resume the locomotion policy.
