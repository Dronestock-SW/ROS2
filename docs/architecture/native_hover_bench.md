# Native hover bench

현장 1회 이륙 시험의 명령 경계를 정의한다.
웹 START와 PX4 모드 연결을 검토할 때 읽는다.

목표는 PX4 기본 높이 이륙, 안정 후 2초 호버, 착륙이다.
기존 REPLAY 엔진의 승인 조건을 변경하지 않는다.
별도 C++ 실행기가 실제 MAVROS 서비스에 연결한다.
2026-10-08 현장 소스 9개를 보존하고 통합했다.
웹 FSM과 같은 PX4 명령 잠금을 획득한다.
`state_dir`가 달라도 실물 출력 실행은 하나만 허용한다.
관측·IPC 잠금은 기존 `state_dir/writer.lock`도 유지한다.
원래 현장 배포본은 이 수정이 적용되지 않은 상태다.

```text
loopback web -> private IPC -> C++ sequence
                               | SetMode / CommandBool
                               v
                       MAVROS -> PX4
                               |
             state / odometry / estimator / RC
                               v
                        C++ confirmation
```

- 부팅은 관측만 한다. START는 현재 실행기 세션에 묶는다.
- TAKEOFF 모드를 실제 확인한 뒤 ARM을 요청한다.
- 높이는 FC의 MIS_TAKEOFF_ALT를 읽는다. 기본값은 1.3m다.
- companion은 위치·고도·자세 setpoint를 발행하지 않는다.
- PX4 AUTO.LOITER, 높이 도달, 정지 상태를 확인한다.
- 안정 0.5초 뒤 연속 2초가 지나면 AUTO.LAND를 요청한다.
- 착륙 및 PX4 자동 disarm을 확인해야 완료다.
- 취소 중 ARM 응답 대기에는 완료를 표시하지 않는다.
- 지상 취소는 일반 disarm과 POSCTL 복귀를 확인한다.
- 실패·취소 뒤 착륙을 정상 성공과 구분한다.
- 명령 timeout은 효과 불명으로 표시한다.
- 중복·역순 입력은 유효 상태를 덮어쓰지 않는다.
- 기본 실행기는 MAVROS·화면·로그를 함께 준비한다.
- 실제 RC·배터리·자동 인계 bit를 조회한다.
- 모드 요청 접수는 실제 모드 전환 완료가 아니다.
- RC 모드·ARM·KILL 스위치 변화에는 명령을 중단한다.
- 스틱 개입에는 출력을 멈춘다. 인계는 PX4가 처리한다.
- 실제 POSCTL 전환을 별도로 확인한다.
- 인계 후 같은 실행기에서 재시작하지 않는다.
- COM_RC_OVERRIDE=2만으로 자동 모드 인계를 보장하지 않는다.
- 프로세스·링크 전체 소실은 PX4 자체 failsafe가 담당한다.
  현장 failsafe 설정과 RC 인계 실증은 별도로 필요하다.

FLIGHT 출력은 기본 잠긴다. SITL 성공과 실물 성공을 구분한다.
UWB 정렬·장착·시각을 이 시험으로 확인 처리하지 않는다.
이 시험은 PX4의 유효한 위치 추정을 사용한다.
UWB 기반 XY 이동과 라벨 정렬은 후속 단계다.
[실행 절차](../runbooks/native_hover_test.md)를 따른다.
