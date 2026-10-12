# UWB·LiDAR 격리 실험 재현
2026-10-12 결과를 다시 만드는 명령이다.
RAW·펌웨어·환경이 일치하는지 확인한 뒤 읽는다.

## 경로와 경계

새 output 경로를 사용한다.
기존 RAW와 결과를 덮어쓰지 않는다.
이유: 이전 해시와 비교 기준을 보존해야 한다.

| 위치 | 값 |
|---|---|
| Windows 작업 루트 | `C:/Users/A/Documents/Codex/2026-10-12/new-chat` |
| ROS2 checkout | `work/ros2-uwb-lidar-integration` |
| RAW | `work/capture-archive-20261011/<capture>/events.jsonl` |
| 원본 manifest | `outputs/capture_transfer_manifest_20261012.json` |
| WSL 시험 루트 | `/home/dronestock/uwb_sim/uwb-lidar-integration-20261012` |
| Linux Python | `/home/dronestock/.venvs/px4/bin/python` |
| 검증 runtime | 시험 루트의 `runtime-lidar2d-v2` |

runtime의 artifact.json에 binary·patch·board 해시가 있다.
실행기는 이를 확인한 뒤 namespace 안에서만 실행한다.
PX4 instance 50–59 중 비어 있는 값을 사용한다.
기존 시뮬레이터와 실기 서비스를 종료하지 않는다.

## Python 회귀

Windows PowerShell, ROS2 checkout에서 실행한다.

```powershell
$env:PYTHONUTF8='1'
$env:PYTHONPATH="$PWD\src\drone_uwb;$PWD\src\drone_mission;$PWD\src\drone_demo"
python -m pytest src/drone_uwb/test src/drone_mission/test --ignore=src/drone_mission/test/test_local_flight_ros.py --ignore=src/drone_mission/test/test_local_web.py --ignore=src/drone_mission/test/test_mission_store.py --ignore=src/drone_mission/test/test_operator_arm.py --ignore=src/drone_mission/test/test_site.py -q
```

실행 결과는 235 passed, 9 skipped다.
제외한 5개 파일은 rclpy/websockets 의존성이 있다.
ROS 종단·web 검증은 이 명령의 통과 범위가 아니다.

## 원본 재생

checkout에서 실행한다.
출력은 기존 결과와 다른 경로를 지정한다.

```powershell
$env:PYTHONUTF8='1'
$env:PYTHONPATH="$PWD\src\drone_uwb"
python -m drone_uwb.processing.experiments.lidar_replay --events ../../work/capture-archive-20261011/capture-1791711935890071726-52ca3d2e/events.jsonl --expected-sha256 d6389183eabad4bea78bf0cad68d110ff6911005704c4712c62923f249ce13ec --imu-to-scan-rotation ../../outputs/scan_z_up_assumption.json --output ../../outputs/replay_1845_new_run
```

회전 파일은 명시적 가정값이며 정밀 장착값이 아니다.
원본 해시가 다르면 실행기가 중단한다.
출력에는 covariance 미보정·FC 출력 비활성을 기록한다.
다른 3개 RAW 경로와 해시는 전송 manifest에서 찾는다.

## H80 합성 응답

```powershell
python src/drone_uwb/tools/evaluate_h80_response.py --config src/drone_uwb/config/runtime/uwb_btf_real.json --output ../../outputs/h80_response_new_run.json
```

이 시험은 solver의 응답·출력 상관을 평가한다.
무선 전달·B_TF 전체·FC 융합 재현은 포함하지 않는다.

## PX4 패치와 runtime

현재 runtime은 기준 커밋에서 직접 빌드했다.
재빌드할 때도 별도 checkout을 사용한다.

- `src/drone_uwb/tools/prepare_px4_lidar2d.py --help`
- `src/drone_uwb/tools/package_px4_lidar2d.py --help`
- 변경분: `src/drone_uwb/tools/px4_lidar2d.patch`
- 근거: `docs/report/evidence/uwb_lidar_integration_20261012/px4_lidar2d_artifact.json`

기존 checkout이나 기체 펌웨어에 적용하지 않는다.
이유: 이 패치는 격리 실험용이다.

## 50cm 폐루프 반복

WSL Bash에서 실행한다.
아래 경로는 실제 시험에 사용한 checkout이다.

```bash
repo=/mnt/c/Users/A/Documents/Codex/2026-10-12/new-chat/work/ros2-uwb-lidar-integration
trial=/home/dronestock/uwb_sim/uwb-lidar-integration-20261012
export PYTHONPATH="$repo/src/drone_mission:$repo/src/drone_uwb:$repo/src/drone_demo"
/home/dronestock/.venvs/px4/bin/python "$repo/src/drone_mission/tools/run_obstacle_matrix.py" --runtime "$trial/runtime-lidar2d-v2" --output "$trial/matrix-50cm-new" --seeds 101 102 103
```

시행 25개를 순서대로 만들고 두 개씩 실행한다.
각 자식은 독립 네트워크 namespace를 생성한다.
instance 52·53이 비어 있는지 먼저 확인한다.
동일 runner 두 개를 동시에 시작하지 않는다.
이유: PX4 instance lock은 namespace 밖에서 공유된다.

새 원본은 각 시행의 `rootfs/log/`에 생성된다.
`mission.jsonl`은 명령·추정·truth를 기록한다.
`run_status.json`의 종료 상태와 오류를 확인한다.
반환 코드 0만으로 시험 통과를 선언하지 않는다.

## ULog 결과 분석

위 Bash 세션에서 실행한다.

```bash
/home/dronestock/.venvs/px4/bin/python "$repo/src/drone_mission/tools/analyze_obstacle_sitl.py" --matrix "$trial/matrix-50cm-new" --output "$trial/matrix-50cm-new-analysis.json"
```

계획한 시행 수와 완료 수가 다르면 분석을 중단한다.
개별 시행 분석은 `--runs`에 디렉터리를 지정한다.

50cm 최소 간격·실제 융합·정상 완료를 따로 읽는다.
합성 관측 결과를 실제 RAW 정확도로 확대하지 않는다.
이유: 실제 시각·장착·공분산 보정이 남아 있다.

## 호버 후속 검증 재현

다음 명령은 격리 SIH에서만 실행한다.
실기 파라미터를 설정하는 명령이 아니다.
2026-10-12 후속 후보는 기존 세 조건에서 3/3이다.
새 세 조건에서는 2/3을 완료했다.
[전체 결과와 남은 실패](../report/uwb_lidar_hover_followup_20261012.md)를 먼저 읽는다.

위 Bash의 `repo`·`trial`·`PYTHONPATH`를 사용한다.

```bash
/home/dronestock/.venvs/px4/bin/python "$repo/src/drone_mission/tools/run_obstacle_matrix.py" --runtime "$trial/runtime-lidar2d-v2" --output "$trial/hover-followup-new" --seeds 201 202 203 --fusion-horizon-ms 50 --zero-unused-airspeed-delay --velocity-output-tau-s .1
/home/dronestock/.venvs/px4/bin/python "$repo/src/drone_mission/tools/analyze_obstacle_sitl.py" --matrix "$trial/hover-followup-new" --output "$trial/hover-followup-new-analysis.json"
```

25회 시행을 만든다. 두 instance를 재사용한다.
다른 matrix 실행기가 끝난 뒤 시작한다.
이유: instance lock은 네트워크 namespace 밖에 있다.
모든 시행의 종료를 확인한 뒤 분석한다.
후보의 정상·벽 조건과 비교군을 분리한다.

| 선택값 | 의미 |
|---|---|
| `--dwell-control position` | 기본. 도착 뒤 고정 목표 유지 |
| `--dwell-control velocity` | 기존 호버 명령의 비교 경로 |
| `--fusion-horizon-ms 50` | SIH 최대 지연 설정 |
| `--zero-unused-airspeed-delay` | 미사용 airspeed의 100ms 강제 상향 제거 |
| `--velocity-output-tau-s .1` | SIH 속도 출력 보정 응답 |
| `--aiding-mode enabled` | 후보만 반복. 기본은 비교군 포함 |

지연·보정 옵션을 생략하면 기존 기본값을 쓴다.
50ms 요청값만 넣으면 PX4가 100ms로 바꿀 수 있다.
실행기는 실제 값이 요청과 다르면 ARM 전에 중단한다.
실험의 PX4 위치 게인도 ARM 전에 확인한다.
실기 센서 지연과 무관하게 이 값을 복사하지 않는다.
이유: 실제 관측은 실험보다 늦게 도착할 수 있다.

호버의 조건별 분석은 다음 도구를 사용한다.

```bash
/home/dronestock/.venvs/px4/bin/python "$repo/src/drone_mission/tools/analyze_hover_sitl.py" --runs "$trial/hover-followup-new/clear-s201-a1" "$trial/hover-followup-new/clear-s202-a1" "$trial/hover-followup-new/clear-s203-a1" --output "$trial/hover-followup-new-hover.json"
```

후속 Python 회귀 결과는 237 passed, 9 skipped다.
위의 ROS·web 의존 파일 5개 제외 조건을 유지했다.
시험 기준은 2초다. 실물 연속 10초 검증은 별도다.
