# RC 비상 수동 인계·기존 스위치 유지 명세
이 문서는 자율비행에서 RC로 제어권을 넘기는 조건과 Jetson의 출력 차단을 정의한다.
RC 설정 조회·PX4 어댑터 구현·부팅 점검·비상 인계 시험 전에 읽는다.

상태: 사용자 정책 확정 / 실제 RC 채널·현재 PX4 설정·기체 시험 미확인 / 2026-10-02.
관련: [전체 설계](DESIGN.md), [PX4 연결](PX4_INTERFACE.md), [행동 트리](BT_SPEC.md), [제공 로그](LOG_REVIEW.md).

## 1. 확정 요구사항

**수동으로 인계된 뒤에는 기존 RC/PX4 설정 그대로 조종한다.**
Jetson이 별도의 수동 제어기를 만들거나, 수동 인계 직후 임의의 모드·착륙 동작을 덮어쓰지 않는다.
현재 스위치 배치·방향·조종 감각을 보존하며 새 Land/Kill 스위치의 추가를 전제하지 않는다.

| ID | 요구사항 | 확인 방법 |
|---|---|---|
| RC-01 | RC 스틱·기존 모드 스위치에 의한 인계는 PX4가 직접 처리 | Jetson·웹 중단 상태에서도 기존 RC 동작 시험 |
| RC-02 | PX4가 수동/외부 제어권을 가져가면 Jetson의 위치·yaw·모드·arm·Land 요청을 중단 | 출력 감사 기록·PX4 실제 모드 대조 |
| RC-03 | 수동 인계된 같은 비행에서는 웹 START/RESUME 및 내부 재시도가 자율 제어권을 회수하지 못함 | 동일 비행의 명령 거부·재시작 시험 |
| RC-04 | 실제 착륙·disarm·새 점검 이후 새로운 웹 시작 명령으로만 새 자율 임무 허용 | 이전 명령 재전달·새 execution 구분 |
| RC-05 | RC 미연결 자체는 자동 이륙 차단 사유가 아님. 수동 인계 불가 경고는 항상 표시 | 부팅/웹 표시 시험 |
| RC-06 | 자율비행 중 RC 링크만 끊기면 저장 임무를 계속함. PX4 자체 비상 제어가 실행되면 그 제어권을 존중 | RC loss와 Offboard loss 분리 시험 |
| RC-07 | 예외 RC 이륙은 이륙점 위 수직 상승·호버만 지원. 웹 시작 전까지 조종자가 유지 | 기존 출발점·위치·방향 검증 |
| RC-08 | READY 알림은 시동·모터 회전·Kill·arm/disarm 명령을 발생시키지 않음 | 부팅부터 READY까지 출력 검사 |

RC가 연결되지 않아도 시작할 수 있다는 운용 결정과, **설치된 RC 인계 기능의 설정·시험 승인**은 별개다.
연결 유무는 경고지만, 승인된 설정과 현재 설정이 다르거나 인계 경로가 미검증이면 FLIGHT 준비를 완료했다고 표시하지 않는다.

## 2. 동작의 의미

| 이름 | 실제 의미 | 본 시스템의 역할 |
|---|---|---|
| RC 수동 인계 | PX4가 RC 입력을 받아 기존 규칙에 따라 수동 조종 가능한 모드로 전환 | 자율 출력을 반환하고 기존 RC 설정을 보존 |
| PX4 Land | PX4의 제어를 유지하며 하강·착륙하는 비행 모드 | 자율 제어권 보유 중 필요할 때만 요청. 수동 인계 뒤 강제 요청 없음 |
| Kill switch | 설정된 RC 스위치로 모터 출력을 즉시 중지하는 기능 | 기존 할당이 있으면 상태·사건을 기록. Jetson이 새로 할당하거나 대행하지 않음 |
| Disarm | 시동 해제. 허용 조건은 모드·PX4 설정에 따라 다름 | 착륙 완료와 별도 상태로 확인. 공중 긴급 착륙의 대체 명령으로 사용하지 않음 |
| 비상 정지 | 사용자가 쓰는 포괄 표현 | 웹·문서에서는 실제로 수동 인계/Land/Kill 중 무엇인지 명확히 표기 |

Kill은 호버링 정지가 아니며 공중에서는 추력 상실을 일으킨다.
기존 Kill 스위치를 해제했을 때 출력이 돌아오는 조건과 자동 disarm 시간도 기존 PX4 설정에 따른다.
Jetson은 Kill 해제를 자동 arm·Offboard 재시작 신호로 해석하지 않는다.
기존 Kill이 미할당이면 웹에 `미할당`을 표시하고, 존재하는 것처럼 `사용 가능`으로 표시하지 않는다.
근거: [PX4 비상 스위치 설명](https://docs.px4.io/v1.17/en/config/safety#emergency-switches), [Kill 채널·해제 조건](https://docs.px4.io/v1.17/en/advanced_config/parameter_reference#RC_MAP_KILL_SW).

## 3. 제어권 전환

```text
PX4 Offboard + Jetson 소유
  -> PX4의 RC 인계/외부 모드 전환 증거
  -> flight_guard: 기존 목표·대기 중 모드/arm/Land 요청 폐기
  -> BT: 행동 중단, execution을 수동 인계 종료로 기록
  -> PX4/RC 감시 전용, 같은 비행 자율 재획득 금지
  -> 실제 착륙 + disarm 확인
  -> 새 점검 + 새 웹 START로 새 execution
```

### 3.1 관측과 확인

1. PX4의 실제 `nav_state`/MAVROS `mode`, 시동 상태, 인계·failsafe 사건을 같은 PX4 부팅 문맥에서 대조한다.
2. `State.manual_input=true` 또는 RC 채널 수신만으로 수동 인계 완료라 판정하지 않는다.
3. 스틱 조작의 1차 판정·모드 전환은 PX4가 맡는다. Jetson은 RC PWM을 임의의 별도 임계값으로 재해석하여 조종하지 않는다.
4. 요청하지 않은 Offboard 이탈은 원인이 확정되지 않아도 출력 권한을 반환한다. 원인은 `EXTERNAL_CONTROL_UNKNOWN` 등으로 남긴다.
5. 단순 송신 중단은 수동 전환 성공 증거가 아니다. 실제 모드가 불명확하면 `CONTROL_AUTHORITY_UNKNOWN`을 보고하고 `RC_MANUAL`로 단정하지 않는다.
6. 명시적 Kill/출력 잠금 증거가 있으면 진행 목표를 폐기하고 출력 금지 상태를 유지한다. 해당 상태를 읽을 실제 토픽·필드는 PX4 버전·발행 설정에 맞춰 검증한다.

스틱 override는 PX4의 일부 failsafe 단계·위험 배터리 반응에서 제한될 수 있다.
이때 Jetson이 PX4의 안전 결정을 무력화하거나, 실제로 인계되지 않았는데 성공을 표시하지 않는다.
기존 RC 모드 스위치와 스틱 override는 각각 시험한다.

### 3.2 C++ flight_guard 필수 규칙

| 상황 | guard 행동 | 금지 사항 |
|---|---|---|
| RC 인계 확인 | owner를 RC로 변경, 현재 generation 무효화, 출력 큐 비움 | 새 위치 목표·Offboard·arm·Land 요청 |
| 요청하지 않은 PX4 모드 전환 | owner를 PX4/외부 또는 UNKNOWN으로 변경, 출력 중단 | 모드를 반복 설정하며 제어권 회수 |
| 인계와 BT 목표 만료가 동시에 발생 | 먼저 제어권 확인. RC/PX4가 소유하면 lease 실패 Land를 송신하지 않음 | 수동 비행에 watchdog Land를 덮어쓰기 |
| 인계 전에 보낸 명령의 응답이 늦게 도착 | 실행 세대·송신 당시 owner를 대조하고 폐기된 명령을 재시도하지 않음 | 오래된 성공 ACK로 ownership 복구 |
| Kill 상태 해제 | 출력 금지·같은 비행 재개 금지를 유지하고 PX4 상태만 관측 | 자동 재시동·임무 재개 |
| 수동 비행 중 기록 실패/배터리 경고 | 기존 수동 소유권 유지, 웹과 기록에 상태 보고 | 자율의 복귀·Land 정책으로 수동 조종을 덮어쓰기 |
| Jetson/Python 재시작 | 기록된 실행/RC 잠금과 PX4 공중 상태 확인, 비행 중이면 감시 전용 | 재부팅을 새 비행으로 취급해 자동 인수 |

flight_guard의 `RELEASE`는 로컬 출력 권한을 포기하는 동작이다.
새로운 PX4 비행 모드를 지정하는 명령으로 구현하지 않는다.
공중 재시작으로 잠금 기록을 복원할 수 없으면 자율 재개를 허용하지 않는다.
PX4가 수동으로 인계한 이후의 모드 선택·스틱·스위치·failsafe 처리는 기존 RC/PX4가 수행한다.
PX4 자체 안전장치는 Jetson의 소유권 모델과 독립적으로 계속 동작한다.

## 4. 현재 설정 조회와 승인

아래는 제공 로그와 해당 펌웨어 소스에 근거한 확인 목록이다.
**현재 기체의 읽기 결과가 아니며, 설정 자동 쓰기 목록도 아니다.**
기존 설정 백업 → 현재 값·실물 채널 조회 → 충돌 확인 → SITL/지상 시험 → 승인본 등록 순으로 진행한다.

| 항목 | 제공 로그/근거 | 확인·승인할 내용 |
|---|---|---|
| PX4 버전·해시 | ULog: `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` | 실제 기체 버전과 동일 여부 |
| `COM_RC_OVERRIDE` | 로그 1. bit 0=Auto, bit 1=Offboard | Offboard 스틱 인계 요구 충족 여부. 3은 두 비트를 켜는 시험 후보일 뿐 자동 적용하지 않음 |
| `COM_RC_STICK_OV` | 로그 30% | 실측 스틱 임계값·노이즈·전환 지연 |
| `COM_RC_IN_MODE` | 로그 3, 첫 유효 입력원 고정 | 실제 물리 RC가 선택되는지. 웹/가상 조이스틱이 입력원을 선점하지 않는지 |
| `COM_RCL_EXCEPT` | 로그 0, Offboard 예외는 bit 2 | RC loss 임무 계속 정책과 충돌 확인. 다른 모드의 기존 RC loss 처리는 유지 |
| `COM_OBL_RC_ACT`, `COM_OF_LOSS_T` | 로그 Position/1초, 설계 목표 Land/1초 시험 | RC loss와 Offboard 송신 loss의 독립 동작 |
| 기존 모드 스위치·채널 | 현재 할당 미확인 | 스위치 위치별 현재 PX4 모드, 수동 복귀 방법, 라벨·방향 기록 |
| 기존 Kill·arm/disarm 설정 | 현재 할당 미확인 | 할당 유무, 채널·극성, 해제/자동 disarm 조건. 변경 전후 일치 검증 |
| RC 수신기 failsafe | 현재 설정 미확인 | 수신기 단절이 실제 PX4에 어떻게 전달되는지. 고정 PWM을 정상 조작으로 오인하는지 |

해당 펌웨어의 스틱 override는 Position, 위치를 사용할 수 없으면 Altitude로 전환하도록 정의돼 있다.
Jetson이 기존 조종기를 보존한다는 요구는 특정 수동 모드를 새로 강제한다는 뜻이 아니다.
설정이 스틱 인계 요구와 충돌하면 부팅 점검은 이유를 보고한다. 몰래 값을 고쳐 READY로 만들지 않는다.
근거: [ULog 펌웨어의 RC·failsafe 매개변수 정의](https://raw.githubusercontent.com/PX4/PX4-Autopilot/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/commander/commander_params.c).

## 5. 웹 표시·자동 점검 인터페이스

상태 조회는 명령 실행과 분리한다. RC 비상 인계 경로는 웹 연결에 의존하지 않는다.
아래는 내부 필드 정의이며 외부 JSON 배치는 웹 담당자 회신을 반영한다.

| 필드 | 값·의미 |
|---|---|
| `control_owner` | `JETSON`, `RC`, `PX4_FAILSAFE`, `EXTERNAL`, `UNKNOWN` |
| `rc_link_state` | `CONNECTED`, `LOST`, `NOT_PRESENT`, `UNKNOWN` |
| `rc_input_source` | `PHYSICAL_RC`, `OTHER`, `UNKNOWN`. 실제 PX4 입력원 근거 필요 |
| `rc_takeover_capability` | `VERIFIED`, `UNVERIFIED`, `UNAVAILABLE`. 연결과 기능 검증을 구분 |
| `rc_profile_revision` | 승인된 RC/PX4 설정·채널 매핑 기록의 개정 |
| `manual_latch` | 같은 비행 자율 재획득 금지 여부 |
| `kill_state` | `ACTIVE`, `INACTIVE`, `UNMAPPED`, `UNKNOWN`. 읽기 근거가 없으면 UNKNOWN |
| `last_takeover` | 관측 시각·PX4 부팅 식별·실제 모드·근거·당시 execution |

| 웹 상황 | 표시·명령 결과 |
|---|---|
| RC 없이 READY | `준비됨 · RC 수동 인계 불가` 경고. RC 기능이 정상 연결됐다고 표시하지 않음 |
| RC 인계 성공 | `RC 수동 조종 중 · 자율 임무 중단` |
| 수동 인계 뒤 START/RESUME | `MANUAL_TAKEOVER_LATCHED`, 착륙·새 점검 후 새 임무 안내 |
| 수동 인계 뒤 웹 LAND/CANCEL 등 비행 변경 요청 | `CONTROL_NOT_OWNED`; 웹 요청으로 기존 수동 제어를 덮어쓰지 않음 |
| 기존 Kill 활성 관측 | `RC 모터 정지 활성`과 실제 armed/landed 상태를 각각 표시 |
| RC 링크 상실 중 자율 진행 | `RC 연결 끊김 · 저장 임무 수행 중` |
| 인계 원인/모드 확인 불가 | `제어권 확인 불가`와 마지막 확정 상태의 시각 |
| 설정 불일치/시험 승인 없음 | `RC_CONFIGURATION_MISMATCH` 또는 `RC_TAKEOVER_UNVERIFIED`, FLIGHT 준비 미완료 사유 |

READY는 현재 점검 결과이며 arm·비행 허가 요청 성공을 뜻하지 않는다.
RC 상태가 바뀌면 준비 결과를 재평가하며, 수동 비행 중에는 READY를 표시하지 않는다.
스캐너 LED 등 선택적 알림장치는 제어 가능성과 기존 스캔 기능 영향을 확인한 뒤 붙인다.
모터를 살짝 돌려 점검 완료를 알리는 방식은 사용하지 않는다.

## 6. 검수 시나리오

아래 시험은 계획이며 실제 통과한 결과가 아니다.
실물 RC 채널·출력 시험은 프로펠러를 분리한 지상 조건에서 수행한다.
공중 안전 동작의 의미와 순서는 SITL로 먼저 확인한다.

| ID | 입력·고장 주입 | 합격 기준 |
|---|---|---|
| RC-T01 | 프로펠러 분리, 기존 스틱·모드·arm/disarm·할당된 Kill 조작 | 승인 전후 채널·극성·PX4 기능 동일, 새 기능 강제 없음 |
| RC-T02 | SITL Offboard에서 임계값 아래/위 스틱 조작 | 노이즈 인계 없음, PX4 실제 전환과 Jetson 출력 차단 확인 |
| RC-T03 | 기존 모드 스위치로 수동 전환 | PX4 실제 모드 유지, Jetson 모드 재요청 0건 |
| RC-T04 | RC 인계와 BT lease 만료·배터리30%·기록 실패 동시 발생 | RC 인계 뒤 Jetson의 Land/복귀/arm 요청 0건 |
| RC-T05 | 인계 전 명령 ACK 지연·중복 응답 | 이전 세대로 제어권 복원·명령 재시도 없음 |
| RC-T06 | 수동 인계 뒤 웹 START/RESUME/LAND 및 서비스 재시작 | 같은 비행 자율 재개 없음, 웹 거부 결과 일치 |
| RC-T07 | Jetson·웹을 중단하고 기존 RC 조작 | RC 경로가 Jetson이나 Wi-Fi에 의존하지 않음 |
| RC-T08 | Offboard 정상, RC 링크만 끊기기/복구 | 임무 계속, 경고 표시, 링크 복구 자체로 인계·재시동 없음 |
| RC-T09 | RC 연결 여부를 바꾸고 Offboard 출력 상실 | PX4 자체 설정대로 Land. RC loss와 혼동 없음 |
| RC-T10 | 기존 Kill이 할당된 경우 활성·해제·disarm 타이밍 재현 | 기존 PX4 의미 유지, Jetson 자동 arm/Offboard 0건 |
| RC-T11 | RC 미연결 부팅, 이후 연결 | 미연결 경고, 연결 후 실제 입력원·capability 재평가 |
| RC-T12 | 예외 RC 수직 이륙·호버 후 웹 START | 안전 인계 검증 후에만 Offboard. 인계 전에는 RC 유지 |
| RC-T13 | 수동 인계 후 실제 착륙·disarm·새 START | 새 점검·새 execution에서만 잠금 해제 |
| RC-T14 | PX4 중대 failsafe 단계에서 스틱 조작 | firmware의 실제 제한 기록, 거짓 인계 성공 표시 없음 |
| RC-T15 | boot/READY 상태를 반복하고 선택 알림장치 장애 | 모터 명령·arm·Kill 출력 없음, READY 의미와 알림 실패 분리 |

각 결과에는 PX4 해시·전체 설정 백업 해시·RC 매핑 개정·시험 시각·입력/실제 모드/최종 Jetson 출력 시각·ULog 또는 SITL 기록을 남긴다.
소프트웨어 모의 시험만으로 RC 실기체 인계 기능을 `VERIFIED`로 올리지 않는다.

## 7. 구현 작업과 남은 확인

| 순서 | 구현 또는 확인 | 완료 조건 |
|---|---|---|
| 1 | 현재 RC/PX4 설정·채널 매핑 읽기 전용 수집 | 기존 수동·스위치 기능 표와 승인본 식별 가능 |
| 2 | PX4 상태 어댑터의 owner·RC 입력원·명시적 출력 잠금 매핑 | 실제 펌웨어 필드·시간·UNKNOWN 처리 명세 |
| 3 | C++ flight_guard release와 실행 세대 폐기 | RC-T02~T06 모의 검증 |
| 4 | 부팅 preflight의 설정 비교·capability 표시 | 자동 설정 변경 없이 차이·미검증 사유 반환 |
| 5 | 웹 소유권·수동 잠금·연결 경고 | 읽기·명령 거부 결과 일치 |
| 6 | SITL 및 프로펠러 분리 실물 시험 | 기존 RC 기능 보존과 Offboard 인계 모두 확인 |

현재 미확정은 **실물 채널/스위치 매핑, 현재 펌웨어·설정, 상태 관측 필드, 실측 인계 지연**이다.
운용 정책은 확정됐다. 이 항목들은 현장 조회·시험으로 채우며, 사용자의 RC 수동 설정을 임의로 재설계하지 않는다.
