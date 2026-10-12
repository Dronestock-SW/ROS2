# LiDAR 개선·PX4 위험 진단 — 다음 세션 프롬프트
2026-10-12 작업을 다른 세션에 넘기는 절차다.
아래 프롬프트를 복사해 후속 분석을 시작할 때 읽는다.

## 이어갈 때의 기준

현재 속도 입력 후보와 S14는 채택 보류다.
원본 −267° 재현과 전체 S01–S14는 미완료다.
구현·진단 기준 커밋은 `42421fd`다.
운영 브랜치가 다르면 기존 변경을 먼저 확인한다.
이번 오프라인 도구를 운영 LiDAR 코드와 구분한다.
두 브랜치를 강제 덮어써 같게 만들지 않는다.
이유: 운영 브랜치의 별도 시험 이력을 보존해야 한다.

| 브랜치 | 이어받는 범위 |
|---|---|
| `feature_uwb` | 기존 분석 기준에 이번 도구·근거·인계를 추가 |
| `FINAL-TEST1-SANGWON` | 기존 C++ 미션·관측 이력을 보존하며 같은 분석을 추가 |

이번 도구·결과·프롬프트는 두 브랜치에 동일하게 반영한다.
두 브랜치의 전체 코드가 같다는 뜻은 아니다.

## 복사할 프롬프트

```text
Dronestock-SW/ROS2의 이번 LiDAR 후속 커밋을 포함한
최신 작업 브랜치에서 이어서 작업해줘.
원격을 fetch하고 현재 브랜치·HEAD·미커밋 변경부터 확인해줘.
기존 변경을 보존하고, 강제 reset/clean/push는 하지 마.

AGENTS.md와 다음 문서를 먼저 읽어줘.
- docs/roadmap.md
- docs/altitude_policy.md
- docs/glossary.md
- docs/equipment_inventory.md
- docs/report/lidar_ground_analysis_20261012.md
- docs/report/lidar_keyframe_sitl_20261012.md
- docs/runbooks/lidar_uwb_sitl_handoff_20261012.md
- docs/runbooks/lidar_keyframe_validation_20261012.md
- docs/report/evidence/lidar_keyframe_sitl_20261012/scenario_status.json

이전 작업의 범위는 다음과 같아.
- 기준 스캔+IMU 각속도 기반 후보와 해시 검사 replay 도구 구현.
- Python 단위 테스트 17개 통과, 합성 장면 70회 평가.
- PX4 d6f12ad1c4f70ad3230afd7d86e971421e02fef4에서
  정지·DISARM SIH 진단 28조건×5 seed=140회 실행.
- 이는 실제 LiDAR→SITL 통합이나 S01–S14 전체 통과가 아니야.
- 원본 events.jsonl와 기존 analyze.py를 회수하지 못해서
  기존 누적 yaw −267.13°는 아직 재현하지 못했어.

우선 현재 후보의 수직 위험을 분리·조사해줘.
몸체 속도 vz=0에 큰 분산을 주어도 평균 리셋은 막히지 않았어.
기울기에서는 z 분산이 수평 속도 분산으로 투영됐어.
높이 관측을 제거한 정지 SIH 5회에서는
50–120초 z RMS가 55.98–100.76m였고,
수직 속도 리셋 687–700회와 고도 유효 플래그 유지를 확인했어.
위치 전용 대조군은 고도·수직 속도 유효를 해제했어.
이전 보고서·원시 ULog부터 대조해 중복 실험을 줄여줘.
EKF 내부 상태, output predictor, 수직 리셋,
최근 수직 융합 시각과 유효성 판정을 구분해 원인을 좁혀줘.
ExternalVisionVel::resetVerticalVelocity()의 non-virtual 호출과
EvVelBodyFrameFrd의 동명 함수 차이도 조사해줘.
단일 함수가 발산 원인이라고 미리 확정하지 마.
수평 2D 속도를 안전하게 표현할 대안을 검토하고,
운영 소스를 바꾸기 전 별도 실험에서 같은 조건으로 비교해줘.

원본 자료도 가능한 범위에서 회수해줘.
대상 capture는 capture-1791703092109574820-28f571f8이야.
events.jsonl 기대 SHA-256:
80f1c6c94ee0aa1d7ec392c8922a1676e5366c29d7b40642213c8f4ea620ac79
기존 analyze.py 기대 SHA-256:
654bc5471f63e0685b8811236fc1fa122d36a5ba5c8a1c96fad34454f7c4ffc5
연결된 companion의 기본 capture 후보 경로는
/home/arialhanho/.local/share/dronestock/manual-captures/야.
이전 pgyxn 계정으로는 읽지 못했어.
읽기 권한 없이 우회하거나 권한을 임의로 바꾸지 마.
fetch만으로 원본이나 접근 권한이 생기는 것은 아니야.
원본이 없으면 재현 완료라고 쓰지 말고 가능한 준비를 계속해줘.
scan↔IMU 장착 회전도 임의의 단위행렬로 가정하지 마.

대상은 360° 수평 YDLIDAR T-mini Pro야.
UWB 안테나 아래 약 10cm는 추정값이야.
하향 TFmini Plus나 FC 기준 레버암과 혼동하지 마.
PX4 EKF2가 비행 추정을 담당해.
companion z·자세 제어와 PX4 추정치의 독립 관측 되먹임은 금지야.
실기 설정·서비스·비행은 변경하지 마.
기존 WSL/PX4/Gazebo 실행과 원시 기록을 보존해줘.
격리 SITL은 별도 체크아웃·network namespace에서 실행해줘.

임무 허용 오차는 아직 정하지 않았어.
먼저 수치·분모·거절·복구·리셋을 기록하고 나중에 기준을 정할 거야.
S04/S05/S12/S13의 남은 검증과 위험을 정리한 뒤 S14를 판단해줘.
실행하지 않은 동적 시험·광류 비교·폐루프 결과는 만들지 마.
변경 코드, 실제 실행 결과, 미완료 항목과 필요한 자료를
UTF-8로 docs에 갱신해줘.
```

## 원시 기록 위치

Git에는 도구·요약·해시·그림을 넣었다.
1.42GB 원시 압축본은 Git에 올리지 않았다.
다른 PC라면 압축본을 별도로 옮겨야 한다.

| 기록 | 이전 세션에서 확인한 위치 |
|---|---|
| Windows 작업 루트 | `C:\Users\A\Documents\Codex\2026-10-12\dronestock-sw-ros2-feature-uwb-agents` |
| 원시 압축본 | 위 루트의 `outputs/lidar-sitl-raw-20261012.tar.gz` |
| 압축본 검증 | `docs/report/evidence/lidar_keyframe_sitl_20261012/raw_archive_verification.json` |
| WSL 원시 matrix | `/home/dronestock/uwb_sim/lidar-uwb-20261012/matrix-01` |
| WSL 격리 PX4 | `/home/dronestock/uwb_sim/lidar-uwb-20261012/PX4-Autopilot` |
| 실제 EKF probe | 같은 WSL 상위 폴더의 `reset-probe-03` |

압축본 SHA-256은 다음과 같다.

```text
ba3882a9a2198f634c18aea395ae0dbd2ecaabbfacaa275f9c49016ab6e106b5
```

140개 ULog와 840개 실행 기록의 해시를 확인했다.
다음 세션에서도 파일 존재·해시부터 다시 확인한다.
이유: 위 경로는 실행 환경이 달라지면 없을 수 있다.

## 참고

- [이전 결과와 S01–S14 상태](../report/lidar_keyframe_sitl_20261012.md)
- [격리 재실행 명령](lidar_keyframe_validation_20261012.md)
- [원래 S01–S14 명세](../report/evidence/lidar_ground_20261012/sitl_plan.json)
