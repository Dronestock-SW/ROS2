# B_TF 지상 시험 세션 초기화
사람이 기체를 새 시험점으로 옮긴 뒤의 절차다.
이전 위치 이력과 비행 중 점프를 구분할 때 읽는다.

## 사용 조건

이 초기화는 운영 배포 후 사용할 수 있다.
구현·격리 시험은 완료했다. 실기 적용은 미실시다.

| 조건 | 요구값 |
|---|---|
| FC 연결·ARM | 연결, DISARM |
| 착륙 상태 | ON_GROUND |
| 상태·착륙 메시지 나이 | 각각 0–1.5초 |
| 정지 판정 | 기존 속도 기준 통과, 0–0.2초 |
| 요청 | 운영자의 명시적 service 호출 |

ARM·공중·미확인·오래된 상태면 거부한다.
이유: 이력 삭제가 공중 위치 점프를 숨길 수 있다.
시험점 이동 자체를 이 서비스가 지시하지 않는다.

## 실행

1. 기체를 새 시험점에 고정한다.
2. 새 기준점 XY와 렌즈 높이를 기록한다.
3. 위 지상 조건을 확인하고 한 번 호출한다.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=2 ROS_LOCALHOST_ONLY=1
ros2 service call /uwb/reset_observation_history std_srvs/srv/Trigger '{}'
```

Tag A는 DOMAIN_ID=1이다. 기본 기체는 Tag B다.
성공 응답은 `ground_observation_history_reset`이다.
실패 응답은 `fresh_disarmed_stationary_ground_required`다.

4. 새 `observation_session` 번호를 기록한다.
5. 새 측정으로 창을 채우는 과정을 확인한다.
6. 기준점 대비 오차를 다시 평가한다.

초기화는 좌표 정합·시각 검증을 완료 처리하지 않는다.
이전 좌표를 새 시각으로 발행하지 않는다.
TDMA 세션·원본 시계·중복 방지 상태는 유지한다.
원시 점프 기준과 B_TF 기하 이력만 비운다.
지상에서 성공해도 이후 공중 점프 보호는 작동한다.

## 기록 확인

`/uwb/btf_decision`에 요청 결과가 남는다.
노드 입력 기록에도 `ground_session_reset`이 남는다.
`/uwb/btf_status`에서 세션 번호를 확인한다.
새 비행 수집기는 기존 토픽으로 이 이벤트를 보존한다.

`jump_check`는 기준 나이·변위·허용량이다.
`receiver_queue_expired`는 수신 후 처리 지연이다.
`pose_blocked_reason=output_expired`는 발행 나이 초과다.
실제 수신 단절과 거부 원인을 구분해서 확인한다.

구현 검증 결과는 [호버·공백 분석](../report/hover_gap_analysis_20261010.md)을 따른다.
