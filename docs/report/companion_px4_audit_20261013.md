# 2026년 10월 13일 Companion PX4 연결 확인
설치·수신·융합 상태를 조회한 기록이다.
API 명세의 실물 적용 상태를 확인할 때 읽는다.

오늘 FC 설치·융합 상태는 재확인하지 못했다.
조회 프로세스에서는 LiDAR 점군만 수신했다.
FC 명령·파라미터 쓰기·펌웨어 업로드는 수행하지 않았다.

## 1. 이번 조회 결과

| 확인 항목 | 결과 |
|---|---|
| SSH | `100.110.163.94`, pgyxn 접속 성공. 중간 1회 timeout |
| MAVROS 프로세스 | arialhanho 계정, PID 2721 실행 |
| B_TF 프로세스 | arialhanho 계정, PID 2725 실행 |
| 새 LiDAR 속도 bridge | 해당 프로세스 없음 |
| UWB raw odometry bridge | 해당 프로세스 없음 |
| 고정 FC readback | heartbeat로 DISARM을 확인하지 못해 조회 전 중단 |
| 완료된 NSH 조회 | 0건, 결과 JSON `queries=[]` |
| 후속 ROS probe | 8초, `/point_cloud` 81개 |
| FC state·IMU·timesync | 이번 probe에서 0개 |
| 파라미터 응답 | 없음 |
| FC ARM 상태 | 확인 불가. readback 오류를 ARM 증거로 해석하지 않음 |

프로세스 존재는 FC 연결 성공을 뜻하지 않는다.
다른 계정 사이의 DDS 발견 문제일 수도 있다.
FC 또는 링크 상태 문제와 구분할 자료는 부족하다.
원인을 확정하지 않고 API 문서 작업을 진행했다.
원래 실행 중인 서비스는 재시작하지 않았다.

조회 원본은 Jetson에 보존했다.

```text
/home/pgyxn/lidar-physical-20261012/fc-readback-20261013-1302.json
/home/pgyxn/lidar-physical-20261012/status-20261013-1305/summary.json
/home/pgyxn/lidar-physical-20261012/status-20261013-1305/topics.jsonl
```

compact 결과와 원본 summary 해시는
[live check](evidence/companion_px4_api_20261013/live_check.json)에 있다.

## 2. 마지막으로 성공한 FC 확인

아래 값은 2026-10-12 기록이다.
오늘 현재값으로 대체해서 읽지 않는다.

| 항목 | 10월 12일 결과 |
|---|---|
| 실물 FC | FMUv6C, board 56 |
| 실행 중 PX4 | `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` |
| 전용 LiDAR 파라미터 | VER/EN/AGE 없음 |
| LiDAR 관측 노드 bench | 점군 399, IMU 3,984 |
| 해당 bench 발행 | 0, gate 차단 |
| 전용 펌웨어 | 로컬 빌드 성공, 실물 업로드 미실시 |
| 이후 정면·왼쪽 보정 | yaw 약 −91.5° |

기존 [실측 기록](lidar_physical_20261012.md)과
[firmware manifest](evidence/lidar_physical_20261012/firmware-manifest.json)를 보존한다.
저장 설정은 현재도 enabled=false다.
수평면·동적 취득 시각 근거는 비어 있다.

## 3. 설치 융합 검증의 정확한 의미

설치는 실제 FC가 새 펌웨어를 실행하는지 확인하는 일이다.
파일 생성이나 GitHub 업로드만으로 설치가 되지 않는다.

융합 검증은 아래 단계의 마지막까지 확인하는 일이다.

```text
companion published 증가
  -> FC lidar_velocity2d 수신
  -> lidar_velocity2d_status 검사 통과
  -> estimator_aid_src_aux_vel의 최신 융합 확인
```

FC 상태·센서 기록을 다시 받을 수 있어야 다음 확인이 가능하다.
설치 후에도 다른 수평 보조와 innovation 조건을 충족해야 한다.
자세한 계약은 [API 명세](../reference/companion_px4_api.md)에 있다.

## 4. 로직 개선을 위한 API 개정

반환값과 PX4 파라미터를 층별로 따라가도록 개정했다.
각 층은 다이어그램, 실제 필드, 적용값, 코드 추적 순서로 구성했다.
목적별 수정 함수와 값 변경이 미치는 영향을 추가했다.
장비 실행 절차 대신 소스·기존 로그에서 연결을 찾도록 정리했다.
최초 wire 사전과 위 실물 확인 결과는 보존했다.

| 항목 | 이번 개정 기준 |
|---|---|
| companion | FINAL `20bd603`, `99a4a9b`의 native/local 이륙 코드 포함 |
| path_heading 미커밋 작업 | 기준에서 제외 |
| 파라미터 수치 | 10/12 MAVROS cache 백업에서 발췌 |
| LiDAR2D 요구값 | companion gate와 전용 firmware 기본값을 분리 |
| 실물 변경 | 없음. 추가 설치·융합·비행 검증 미실시 |

10/12 파라미터는 오늘 FC에서 다시 읽은 값이 아니다.
원본 해시·발췌는 [파라미터 기록](evidence/companion_px4_api_20261013/parameter_snapshot_20261012.json)에 있다.
소스 범위는 [개정 manifest](evidence/companion_px4_api_20261013/source_manifest_v2.json)에 있다.

문서 검증: Mermaid 11개를 렌더링해 그림을 확인했다.
UTF-8, 예시 문법, 문서 링크, 기준 코드 링크도 검사했다.
파라미터 36개의 원본 수치를 보존하고, 표의 주요 값 27개를 대조했다.
알고리즘·설정 코드는 바꾸지 않아 실행 시험을 추가하지 않았다.
