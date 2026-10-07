# Gazebo UWB 비교 준비 결과 — 2026-09-27

가중 풀이와 Gazebo 경로 비교 도구의 검증 기록이다. 실제 연동 전에 구현 범위와 남은 작업을 확인한다.

## 완료 범위

WLS와 경로 기록·파일 비교 도구를 준비했다.
검증 입력은 합성 위치 자료다.
실제 Gazebo 수신과 UWB 기반 호버링은 미실시다.

| 모듈 | 이번 구현 |
|---|---|
| `processing/weighted_xy.py` | 4앵커 거리 가중 풀이·조건부 위치 공분산 |
| `integration/gazebo_capture.py` | Gazebo Python 연결로 최상위 모델 위치 기록 |
| `processing/experiments/gazebo_trial.py` | 한 경로에서 가상 거리 생성·A/B/C/D/WLS 비교 |
| `processing/experiments/gazebo_scenarios.py` | 동일 경로·seed로 여섯 조건 실행 |
| `config/gazebo_shadow.json` | 가상 배치·잡음·가중치·B/D 설정 |

A/B/C/D의 풀이 수식은 이번 추가에서 바꾸지 않았다.
WLS는 A와 별도 함수다.
비행 출력은 `external_output_allowed=false`다.
FC 파라미터 변경과 MAVLink 송신은 하지 않는다.

## 사용자 PC에서 확인한 환경

아래 값은 사용자 제공 출력이다.
작성자가 해당 WSL에 접속해 조회한 결과는 아니다.

| 항목 | 확인값 |
|---|---|
| 계정·호스트 | `dronestock@DESKTOP-0C8GRSK` |
| PX4 조회 경로 | `~/github/PX4-Autopilot` |
| `git describe --tags --always --dirty` | `v1.18.0-beta1-700-gc4e4ef98e9` |
| `gz sim --versions` | `8.15.0` |
| `ls /opt/ros` | `No such file or directory` |

기본 ROS 경로가 없는 상태로 확인했다.
다른 경로에 설치됐는지는 조회하지 않았다.
Gazebo 기록은 ROS 없이 Python 연결로 준비했다.
사용자 PC의 연결 모듈 설치 여부는 아직 미확인이다.

## WLS 산식과 조건

제공된 거리 공분산에 따라 거리별 영향을 조절한다.
품질을 자동으로 판단하는 기능은 아니다.

```text
r_cal_i = r_raw_i - bias_i
d_i(x,y) = sqrt((x-ax_i)^2 + (y-ay_i)^2 + (z_i-az_i)^2)
e_i = d_i - r_cal_i
(x,y) = argmin e^T * inverse(C_range) * e
G_i = [(x-ax_i)/d_i, (y-ay_i)/d_i]
P_xy = inverse(G^T * inverse(C_range) * G)
```

`C_range`는 대칭 양정치 4×4 행렬이다.
독립 거리의 대각 성분은 `sigma_i²`다.
계산은 Cholesky 분해와 SVD를 사용한다.
비용 감소를 확인하며 최대 40회 반복한다.
step 기준은 `1e-7m`, condition 상한은 `1e6`이다.
공분산은 제공된 거리 오차 모델에 대한 조건부 근사다.
Z·앵커 좌표 오차나 지속 편향을 자동 포함하지 않는다.

## 합성 검증 설정

Gazebo 연결과 별개로 같은 파일 계약을 검증했다.
출처를 `synthetic_pose_fixture`로 명시했다.

| 항목 | 적용값 |
|---|---|
| 입력 | 8초 구간의 위치 320개, 40Hz, seed 7 |
| 궤적 | `x=.7+.5*sin(t)`, `y=.3+.3*sin(t/2)`, `z=1.1+.02*t` |
| A1 / A2 | `(-3,-2.5,2.2)` / `(3,-2.5,2.2)` m |
| A3 / A4 | `(-3,2.5,2.2)` / `(3.1,2.3,2.2)` m |
| 가상 거리 편향 | `[-.14,+.20,-.17,-.06]` m. 생성 뒤 같은 값 차감 |
| Z 입력 | 같은 시각의 합성 높이. A/C/D/WLS에 제공 |
| B | 최근 0.8초, Huber 0.12m, 앵커별 최소 3개, 단절 0.15초 |
| D | 교점 부재 시 실패. 반지름 확장 없음 |
| 시각 | 네 거리 동시 생성, 시뮬레이션 시계 us |

가상 앵커와 편향은 이번 시험 설정이다.
실물 좌표·태그 교정값으로 채택하지 않았다.
태그 기준점은 모델 원점으로 가정한다.
ToF·장착·비동기 거리 오차 모사는 남아 있다.

## 합성 결과

같은 RAW에서 가중치의 효과를 분리했다.
표의 RMSE는 A와 WLS가 모두 출력한 시각 기준이다.

| 조건 | 공통 개수/전체 | A RMSE(cm) | WLS RMSE(cm) |
|---|---:|---:|---:|
| 네 거리 잡음 0.03m | 320/320 | 2.949 | 2.949 |
| A2 잡음 0.30m, 동일 가중 | 320/320 | 16.990 | 16.990 |
| 위 RAW, A2 불확실성 반영 | 320/320 | 16.990 | 3.799 |
| 위 RAW, 잘못 A1 불확실성 증가 | 320/320 | 16.990 | 18.516 |
| 2~6초 A2 편향 +0.35m, 동일 가중 | 320/320 | 14.743 | 14.743 |
| 3~3.25초 전체 단절 | 310/320 | 2.947 | 2.947 |

품질 정보가 맞을 때 가중 풀이가 유리했다.
틀린 품질 정보는 결과를 악화시켰다.
동일 가중에서는 미지의 편향을 자동 보정하지 못했다.
한 궤적·한 seed의 결과를 실측 성능으로 일반화하지 않는다.

전체 모델의 출력률·p95·평균 오차·표준편차는
[결과 폴더](../../data/processed/uwb/gazebo_shadow_fixture_20260927/)에 보존했다.
각 조건의 `summary.json`과 `results.jsonl`을 함께 읽는다.
누락 10개는 출력률 분모에 남겼다.
각 `manifest.json`에는 입력·설정·코드 해시가 있다.

## 확인 결과와 남은 작업

```bash
PYTHONPATH=src/drone_uwb python3 -m pytest src/drone_uwb/test -q
```

UWB 테스트 138개가 24.55초에 통과했다.
동일 가중 일치, 상관 공분산, 잘못된 가중치도 검사했다.
시각 역행·중복·출처 혼합 거부를 검사했다.
단절 후 B 재시작과 파일 재생 재현성도 검사했다.
여섯 조건의 합성 파일 실행을 완료했다.
두 CLI의 도움말과 문서 변경 형식을 확인했다.

| 남은 작업 | 상태 |
|---|---|
| 사용자 PC의 Gazebo 실행·통신 경로 확인 | 출력 대기 |
| Gazebo Python 구독과 실제 경로 수집 | 미실시 |
| 실제 경로에서 잡음·배치·속도별 비교 | 미실시 |
| ToF·장착·거리 시각차 모사 | 미구현 |
| PX4 관측 연결·좌표·시각·공분산 검증 | 미실시 |
| UWB 관측을 이용한 SITL 호버링 | 미실시 |
| 이번 변경의 colcon 빌드 | 미실시. 독립 Python 파일 실행만 확인 |

실행은 [경로 비교 절차](../uwb_gazebo_shadow_runbook.md)를 따른다.
Gazebo 시각·위치 필드는
[공식 SceneBroadcaster 소스](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/scene_broadcaster/SceneBroadcaster.cc)를 참고했다.
수신 대상은 월드의 최상위 모델로 제한한다.


## 기능별 커밋 전 통합 확인

2026-09-27 최종 작업본으로 UWB 테스트 142개를 통과했다.
전체 5개 패키지 빌드와 새 CLI 두 개도 확인했다.
이전 시험 수치와 당시 검증 기록은 보존했다.
이번 실기·Gazebo 실행 검증은 미실시다.
[통합 확인 기록](uwb_commit_review_20260927.md)을 따른다.
