# 수동 비행 수집기 확인 — 2026-10-09

수집기 구현·시험 결과를 기록한다.
현장 수동 비행 로그를 준비할 때 읽는다.
실제 비행 또는 보정 확정 결과는 아직 없다.

## 구현

`uwb_manual_capture`는 관측만 구독한다.
원본 UWB와 B_TF 계산 결과를 함께 보존한다.
ROS 원본 메시지와 두 수신 시각을 저장한다.
MAVLink 양방향 패킷도 구독해 보존한다.
파라미터·ARM·모드 명령 경로는 없다.
절차는 [실행 문서](../runbooks/manual_flight_capture.md)에 있다.

## 단위 시험

Windows 순수 Python 시험 10개를 통과했다.
시각·원본 값·NaN·누락을 보존했다.
용량 제한과 디스크 오류를 불완전 기록으로 표시했다.
ARM 전 또는 상태 만료 때 자동 종료하지 않았다.
새 ARM 뒤 최신 지상·DISARM의 종료 조건을 시험했다.
기존 세션 덮어쓰기와 다른 부팅의 마커를 거부했다.

```bash
PYTHONPATH=src/drone_uwb python3 -m pytest -q \
  src/drone_uwb/test/integration/test_manual_capture.py
```

## 실기 사전 조사

현재 MAVROS allowlist에는 광류 plugin이 없다.
ROS graph에서도 광류 토픽은 발견되지 않았다.
MAVLink 원본 광류 수신 여부를 별도로 표시한다.
FC 내부 융합 확인에는 같은 시행의 ULog가 필요하다.
수집을 위해 MAVROS를 재시작하지 않는다.

Jetson 실물 수신 시험과 실제 비행은 아직 미실시다.
자동 보정·실기 START 활성도 수행하지 않았다.
