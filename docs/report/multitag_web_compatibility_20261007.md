# 멀티태그·웹 연결 검토 기록 — 2026-10-07
현재 태그와 ROS2의 호환성을 확인한 기록이다.
연결 문제의 근거와 미검증 범위를 찾을 때 읽는다.

현재 패치 결과는 [후속 검증](multitag_patch_validation_20261007.md)에 있다.
아래 내용은 수정 전 소스 검토 기록이다.

## 결론

웹 전송은 Wi-Fi/WebSocket 경로다.
LoRa 역송신을 필수 작업으로 본 설명을 정정했다.
사용자 확인과 실제 웹 송신 코드가 일치한다.
실행 코드·펌웨어·장치 설정은 이번에 바꾸지 않았다.

| 대상 | 검토한 버전 |
|---|---|
| ROS2 | `1cc9ba484d19cd3b41c4b0330394d8c30c029b20` |
| 멀티태그 펌웨어 | `0026d6414d1ac273f958003f42e939a7cce0471a` |
| 기존 ROS2 검토본 | `7065fcde`, 로컬 변경을 덮어쓰지 않고 별도 검토 |

## 확인 사실

| 항목 | 근거와 결과 |
|---|---|
| USB RAW | 921600 baud, schema 1, A1~A4 사선거리·시각 호환 |
| 새 LoRa 상태 | good/bad 1000ms와 추가 상태 필드를 두 ROS 처리기가 수용 |
| 기본 XY | `observations.py:130`에서 진단 오류가 정상 이력을 초기화 |
| B_TF | `measured_btf.py:125`는 진단을 오류로 세지만 이력 보존 |
| Tag B | 수신 YAML·B_TF JSON이 모두 ID 5로 기본 고정 |
| 웹 XY | `ros_monitor.py:25`가 `/uwb_pose`만 구독 |
| 웹 Z | `runtime.py`의 `current_z_m/current_z_source`가 null |
| 웹 전송 | WebSocket `/ws/drones/{id}/`, 기본 10Hz |
| ToF·자세 | 새 B_TF에 MAVROS 실제 입력 경로 구현 |
| 최종 XYZ | B_TF는 XY 전용. PX4 전달은 미연결/기본 비활성 |

참조 파일은 검토한 ROS2 커밋 기준이다.
현재 파일 위치는 [연결 기준](../reference/multitag_jetson_web_contract.md)에 있다.

## 합성 재현

실제 Python 처리기에 합성 메시지를 순서대로 넣었다.
각 Tag에 40Hz 간격의 RAW 160개를 제공했다.
ToF 1m·수평 자세·장착 간격 12cm를 사용했다.
원본 거리는 XYZ=(2,1.5,1.12)m에서 계산했다.
초기 시계/계산 준비 구간도 출력 횟수에 포함했다.

| 입력 | 기본 XY 수 | B_TF XY 수 |
|---|---:|---:|
| 올바른 Tag ID, RAW만 | 131 | 129 |
| 올바른 Tag ID, RAW+TDMA | 0 | 129 |
| Tag B를 ID 5 설정으로 입력 | 0 | 0 |

A/B 각각에 구형/신규 LoRa 상태를 대조했다.
총 9조건이며, 1Hz 상태로 바꿔도 결과는 같았다.
기본 출력 차단의 원인은 LoRa 주기가 아니다.
이 결과는 ROS 실행·RF·실제 위치 정확도 시험이 아니다.
[합성 결과 원본](evidence/multitag_web_20261007/replay-results.json)을 보관했다.

## 검사 기록

이 대화의 읽기 검토 단계에서 44개가 통과했다.
문서 커밋 단계에서 새 실측으로 재분류하지 않았다.
환경은 Python 3.11, NumPy, pytest, PyYAML이다.
처음에는 PyYAML이 없어 수집이 중단됐다.
검토용 환경에 설치한 뒤 아래 검사가 통과했다.

```bash
PYTHONPATH=src/drone_uwb:src/drone_demo \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
python3 -m pytest -q -p no:cacheprovider \
  src/drone_uwb/test/processing/test_observations.py \
  src/drone_uwb/test/processing/test_measured_btf.py \
  src/drone_uwb/test/processing/test_four_anchor_runtime.py \
  src/drone_uwb/test/processing/test_a123_observations.py \
  src/drone_uwb/test/processing/test_preimu_pipeline.py
```

[로그](evidence/multitag_web_20261007/pytest.log)와
[해시 목록](evidence/multitag_web_20261007/manifest.json)을 보관했다.
기존 `ros2-tdma-v02.patch` 적용 검사는 통과했다.
패치를 실제 적용하지 않았다.

## 과거 실측과 남은 확인

기존 B_TF 실측은 이번 TDMA 검토와 별개다.
두 60초 기록의 좌표 발행 수는 1974개·541개다.
최대 공백은 2.774초·17.267초였다.
조건이 달라 성능 향상/저하 비교에 쓰지 않는다.
[실물 B_TF 기록](uwb_btf_real_20261005.md)을 따른다.

| 구분 | 이번 검토에서 수행했는가 |
|---|---|
| Git 소스·설정 대조 | 수행 |
| Python 44개 검사·합성 9조건 | 수행 |
| 웹 서버·화면 코드 검토 | 미실시, 저장소 미확인 |
| 실제 companion 접속·ROS/MAVROS 실행 | 미실시 |
| colcon·펌웨어 새 빌드·업로드 | 미실시 |
| RF·위치 정확도·PX4 융합·비행 | 미실시 |

다음 작업은 [인수인계 절차](../runbooks/multitag_jetson_web_handoff.md)에 정리했다.
