# 스캔 실패 뒤 남은 작업 수행 기록
일부 스캔 실패 후 후속 목표·스캔을 수행하는 확인 기록이다.
2026-10-09 미션 구성과 시험 결과 확인 때 읽는다.

스캔 실패만으로 전체 미션을 끝내지 않는다.
대기점 복귀를 확인하고 남은 작업을 순서대로 수행한다.
모든 작업을 시도한 뒤 출발점 복귀·착륙한다.
브랜치는 `codex/mission-flight-flow-20261008`이다.
기준 commit은 `cadbae14d3ed909b1bd919dff8ada7dd602eb390`다.
main·어제 현장 수정·실물 FC 설정은 보존했다.

## 변경 결과

작업 성공과 전체 경로 수행을 별도로 표시한다.
실패한 스캔 기록은 후속 작업 성공으로 덮지 않는다.

```text
P1 도착
 -> S1 마커/판독 실패 저장
 -> S1 대기점 복귀 확인
 -> P2 도착
 -> S2 정렬·판독 성공 저장
 -> S2 대기점 복귀 확인
 -> P3 도착
 -> 방문 경로 역순 복귀
 -> LAND·새 ON_GROUND·disarm·END
```

| 표시 | 최종 기대값 |
|---|---|
| attempted_task_ids | P1, S1, P2, S2, P3 |
| remaining_task_ids | 빈 목록 |
| failed_scan_task_ids | S1 |
| route_complete | true |
| mission_complete | false |
| flight_outcome | SUCCEEDED |
| work_outcome | INCOMPLETE |

이전 재생의 S1은 마지막 작업이었다.
그 시행에는 실패 후 수행할 작업이 없었다.
이번에는 S1 뒤에 waypoint·scan·waypoint를 넣었다.
실물 스캔 adapter 미완성은 그대로다.
비행 중 판독 응답이 없어도 후속 작업을 시도한다.

## 고친 명령 경계

스캔 시간 제한으로 미확인 XY 명령을 덮지 않는다.
ALIGNING의 시간 제한을 새 보정 이동보다 먼저 검사한다.
기존 명령 응답을 받은 뒤 대기점 복귀를 요청한다.
응답 자체를 모르면 기존 UNCONFIRMED 정책을 따른다.
이유: 작업 실패가 명령 효과의 불명확함을 해소하지 않는다.

SCAN 상태에서 판독이 없으면 scanner_timeout을 기록한다.
정렬을 잃어도 전체 스캔 시간 제한은 연장하지 않는다.
실패한 작업은 무한 재시도하지 않는다.
이유: 다음 작업과 남은 비행 시간을 확보해야 한다.

조종자 중단·배터리 reserve·센서 장기 단절은 기존 정책이다.
스캔 실패 후 계속 수행은 정상 비행 상태에서 적용한다.

## 검증

작업 순서·대기점 복귀·실제 종료를 함께 검사했다.

| 검사 | 결과 |
|---|---|
| 미션 패키지 Linux 회귀시험 | 111개 통과 · 55.42초 |
| Platform Linux 회귀시험 | 19개 통과 · 2.88초 |
| 다중 작업 정책 사례 | 마커 timeout·판독 timeout·FAILED·성공 |
| 응답 대기 중 스캔 만료 | 기존 명령 유지·응답 후 대기점 복귀 |
| PX4 S1 마커 없음·S2 정상 | 통과 · 5개 작업 시도·END |
| PX4 S1 판독 없음·S2 정상 | 통과 · 5개 작업 시도·END |
| PX4 S1 FAILED·S2 정상 | 통과 · 5개 작업 시도·END |

실제 PX4 v1.17 SIH와 가상 RAW UWB·ToF·flow를 사용했다.
HTTP·ROS2·MAVROS·WebSocket을 실제로 연결했다.
3개 모두 S1 실패 뒤 S2 성공을 기록했다.
남은 작업은 빈 목록이고 전체경로수행은 true다.
실패 기록 때문에 임무완료는 false를 유지했다.
가상 IMU 잡음 배율은 0.1이다.
판독 응답은 SIMULATED-SCAN worker다.
실물 ARM·모드·파라미터 쓰기는 0회다.
이번 변경은 C++·UWB 처리기를 바꾸지 않았다.
그 이전 검증은 [전체 흐름 기록](mission_flight_flow_20261008.md)에 있다.

## 내일 사용

같은 브랜치의 최신 commit을 사용한다.
[현장 절차](../runbooks/mission_chain_field.md)를 따른다.
미션 route_tasks에 작업을 원하는 순서로 넣는다.
scan 뒤에도 남은 작업이 있으면 계속 수행한다.
웹의 남은작업·스캔실패작업·전체경로수행을 확인한다.

[모사 재현](../runbooks/mission_chain_virtual.md)의 후속 3개 시나리오를 쓴다.
시험 근거는 `scan_continuation_validation_20261008.tar.gz`다.
ULog·RAW·명령·scan 결과·최종 웹 상태를 보존한다.
실측 지도·좌표·RC·센서 확인 조건은 그대로다.
