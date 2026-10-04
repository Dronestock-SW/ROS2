# 드론 가상 고도 제거와 실측 ToF 표시

2026-10-04 고도 입력 변경·검증 기록이다.
데모 수정 또는 실측 연결을 이어갈 때 읽는다.

## 변경 내용

사용자는 드론의 가상 고도를 제거하도록 확인했다.
ToF는 실측 가능하다고 알렸다.
앵커 설치 높이 2.2m의 제거 요청은 아니다.

| 위치 | 변경 |
|---|---|
| `drone_demo/core.py` | 사인파 z·3D 정답·합성 사선거리 제거. XY 시험 관측만 생성 |
| `config/demo.json` | 높이 최소·최대·주기 제거. 거리 잡음을 XY 잡음으로 변경 |
| `drone_demo/node.py`, `launch/` | 가상 고도 인자·RAW 발행 제거. 실측 Range를 별도 표시 |
| `drone_demo/tof_readout.py` | 원본 거리·시각·frame과 나이 보관. 누락·오류·만료 시 null |
| `drone_demo/z_gate.py` | ToF 수신 시 데모를 종료하던 게이트 제거 |
| `export.py`, `cycle.py` | XY 파일과 임무 시험으로 변경 |
| 관련 기준·절차 문서 | 실측 가능 상태·XY 계약·제거된 옵션·장치 검증 경계 반영 |

위 코드는 `src/drone_demo/` 아래에 있다.
실제 UWB 수신·솔버와 PX4 비행 제어는 변경하지 않았다.
가상 고도 없이는 사선거리를 만들 수 없어 RAW 데모를 제거했다.
XY 시험 관측은 `source_mode=demo_xy`로 표시한다.
RAW 계산 회귀는 기존 UWB 테스트로 확인했다.
과거 원본·보고서와 독립 Gazebo·수식 시험은 보존했다.

## 적용값과 계약

| 항목 | 값 |
|---|---|
| 앵커 XY | A1=(0,0), A2=(6.3,0), A3=(0,4.6), A4=(6.3,4.6)m |
| 앵커 설치 z | 기존 2.2m 유지 |
| 데모 고도 | `z_m=null`, `z_source=unobserved` |
| Pose의 필수 z | 0 미관측 자리값. 고도 명령 아님 |
| 실측 ToF 토픽 기본값 | `/tof/range`, `sensor_msgs/Range` |
| 표시 만료 기본값 | `tof_timeout_s=0.2`초 |
| ToF 출력 | `/demo_status.tof.range_m`, 원본 시각·frame·나이·사유 |
| 실행 영역 | DOMAIN_ID 99, localhost |
| XY 출력 계약 | schema 2, `truth_xy_m`, `target_xy_m` |

`real_tof_topic`으로 실제 연결 토픽을 지정한다.
ToF 표본의 ROS 시각과 sensor frame이 필요하다.
중복 표본은 수신 나이를 갱신하지 않는다.
원시 ToF 거리에서 지도 높이를 임의로 만들지 않는다.
FC·안테나 높이 변환에는 자세·장착·바닥 기준이 필요하다.

기존 `--z-*`·`synthetic_z_enabled` 인자는 제거했다.
구 설정은 현재 `demo.json`을 기준으로 다시 작성한다.
새 내보내기는 `received.jsonl`을 만들지 않는다.
기존 파일은 당시 계약으로 재생한다.

## 검증

데모·임무와 기존 UWB 관측·파일 파이프라인 시험을 실행했다.

```bash
source /opt/ros/humble/setup.bash
export PYTHONPATH="$PWD/src/drone_uwb:$PWD/src/drone_demo${PYTHONPATH:+:$PYTHONPATH}"
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
python3 -m pytest -q -p no:cacheprovider \
  src/drone_demo/test \
  src/drone_uwb/test/processing/test_observations.py \
  src/drone_uwb/test/processing/test_preimu_pipeline.py
```

결과는 129개 통과, 2개 모듈 건너뜀이다.
건너뜀은 `pymavlink` 미설치 때문이다.
XY 재현·공백·도착과 ToF 누락·오류·만료·복구를 확인했다.
제거한 고도 설정은 오류로 거부한다.

| 추가 확인 | 결과 |
|---|---|
| `colcon build --symlink-install --packages-select drone_demo` | 1개 패키지 성공 |
| 설치 경로에서 설정 로딩·XY 내보내기 | 통과. 가상 고도 설정과 RAW 파일 없음 |
| 설치된 `demo_cycle` 세 시나리오 | 모두 최종 도착 판정 |
| 두 launch의 `--show-args` | ToF 토픽·만료 인자 확인. 가상 고도 인자 없음 |
| DOMAIN_ID 99 ROS 연결 시험 | XY 478개, 관측 438개, 공백 상태 40개 수신 |
| Range 시험 표본 연결 | 1.37m 표시 후 송신 단절 시 null 만료 |
| 임무 상태 | 공백 후 복구·접근·정착·도착 확인 |
| `git diff --check` | 통과 |
| 갱신 문서의 로컬 링크 | 255개 확인. 누락 없음 |

ROS 시험은 아래 명령으로 실행했다.

```bash
source install/setup.bash
ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1 \
  python3 src/drone_demo/test/check_ros_topics.py --mission
```

시험용 Range 표본은 실제 센서 측정이 아니다.
샌드박스의 네트워크 인터페이스·UDP 제한 로그가 있었다.
로컬 토픽 수신과 모든 단언은 통과했다. 종료 코드는 0이다.
실제 장치 간 네트워크 통신 검증으로 해석하지 않는다.
CLI 시험 산출물은 `/tmp/ros2_xy_tof_check__s0sy7ae`에 있다.
요약 증빙은 [검증 결과](evidence/demo_measured_tof_20261004/validation.json)에 보관했다.

## 남은 작업

실제 TFmini Plus 연결·수신 시험은 미실시다.
실측 토픽·시계·주기·장착 기준을 장치에서 확인해야 한다.
0.2초는 표시 기본값이며 실측 주기 검증값이 아니다.
동시 UWB·ToF·자세 정렬과 M02 연결 검증은 남아 있다.
PX4 융합·실제 비행·장치 배포는 미실시다.

[고도 원칙](../altitude_policy.md)과
[실행 절차](../runbooks/demo_procedure.md)를 따른다.
