# 전체 변경의 기능별 커밋과 검증

2026-10-04 변경 분류와 통합 검증 기록이다.
Git 반영 범위나 다음 검증을 확인할 때 읽는다.

## 반영 범위

기능 변경을 9개 커밋으로 나눴다.
이 기록과 문서 링크 수정은 별도 커밋에 담는다.
검토 시작점은 `ab555ba`다.
반영 대상은 `feature_uwb`와 `main`이다.

| 커밋 | 기능 | 변경 내용 |
|---|---|---|
| `db86789` | SITL 관측 연결 | Gazebo·PX4 시계, TIMESYNC, NED 변환, 관측 송신 계약·검사 |
| `cda3300` | 센서·실시간 비교 | ToF·IMU 높이, 기록 RAW 재생, A/B/C/D/WLS 비교, Shadow 기록 |
| `6d164cb` | 수평 목표 연결 | 목표 명령, PX4 상태 감시, 도착·공백·중단 판정 |
| `2747729` | 이상 주입·독립 평가 | 거리 편향·단절, 실제 궤적 대조, 정상·이상·복구 구간 평가 |
| `777cb10` | 모델 비교 기록 | 시행별 설정·결과·경로, 원본 자료, 진행 근거·LiDAR 시험 명세 |
| `79d0498` | Windows/WSL 기록 | 재개·재시작 결과, 첫 센서 기록 ZIP, 증빙 JSON |
| `37f7b23` | 저장소 모듈화 | 구현·설정·테스트·문서 이동, 호환 alias·상대 링크, 설치 경로 |
| `f5edf38` | 앵커 배치 | 6.3×4.6m 직사각형을 현재 기본값에 반영 |
| `87a0c50` | 데모 ToF | 가상 고도·RAW 데모 제거, XY 데모와 실측 Range 표시 분리 |

이동 전 스냅샷의 파일 해시를 대조했다.
기능 변경과 경로 이동을 나눠 커밋했다.
최종 트리는 작업 시작 시점의 내용을 보존한다.
아래 링크 수정과 이번 검증 문서만 추가했다.
중간 커밋별 테스트 재실행은 미실시다.
통합 검증은 최종 작업트리에서 수행했다.

## 현재 적용값

| 항목 | 값·조건 |
|---|---|
| 앵커 XY | A1=(0,0), A2=(6.3,0), A3=(0,4.6), A4=(6.3,4.6)m |
| 앵커 설치 높이 | 기존 2.2m |
| 과거 재생 | 9월 6일 좌표·기록을 사용 |
| 기본 SITL 실행 | Shadow. 송신 확인 설정은 비활성 |
| 수평 목표 시험 | 기존 허용 구역·속도·타임아웃 조건 유지 |
| 데모 고도 | `z_m=null`, `z_source=unobserved` |
| ToF 표시 | `/tof/range`, `sensor_msgs/Range`, 기본 만료 0.2초 |
| 데모 실행 영역 | DOMAIN_ID 99, localhost |

상세 계약은 [앵커 배치](../reference/uwb_anchor_layout.md),
[관측 연결](../runbooks/uwb_gazebo_sitl_observer.md),
[목표 연결](../architecture/uwb_gazebo_target_adapter.md),
[실측 ToF 표시](demo_measured_tof_20261004.md)를 따른다.

## 통합 검증

422개 시험이 통과했다.
bringup의 기존 lint 검사 2개는 실패했다.
전체 테스트 통과로 보고하지 않는다.

| 검사 | 결과 |
|---|---|
| UWB pytest | 288 통과 |
| demo pytest | 116 통과 |
| platform pytest | 9 통과 |
| bringup pytest | 9 통과, lint 2 실패, copyright 1 skip |
| 전체 colcon 빌드 | 5개 패키지 성공 |
| 설치 환경 alias | 55쌍의 모듈 객체 동일 |
| 설치된 UWB 설정 | 새·기존 경로 41개 바이트 일치 |
| 설치 환경 CLI | /tmp에서 7개 모듈의 `--help` 성공 |
| 원본 보존 | 기록 추가 전 1,043개 일치. 최종 1,041개 일치·문서 2개 수정 |
| 상대 심볼릭 링크 | 끊어진 링크 없음 |
| 문서 링크 | 963개 확인. 누락 없음. 이동된 A 테스트 링크 1개 수정 |

ROS Humble 환경을 불러와 실행했다.
pytest 자동 플러그인과 캐시 쓰기를 비활성화했다.
선택 의존성은 임시 경로에 설치했다.
`pymavlink==2.4.50`, `websockets==13.1`을 사용했다.
설치 위치는 `/tmp/ros2_commit_audit_20261004/deps`다.

시험 명령의 기본 형식은 다음과 같다.

```bash
source /opt/ros/humble/setup.bash
export PYTHONPATH="/tmp/ros2_commit_audit_20261004/deps:$PWD/src/drone_uwb:$PWD/src/drone_demo:$PWD/src/drone_platform_link:$PWD/src/drone_bringup${PYTHONPATH:+:$PYTHONPATH}"
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export PYTHONDONTWRITEBYTECODE=1
python3 -m pytest -q -r a -p no:cacheprovider src/drone_uwb/test
python3 -m pytest -q -r a -p no:cacheprovider src/drone_demo/test
python3 -m pytest -q -r a -p no:cacheprovider src/drone_platform_link/test
colcon build --symlink-install
source install/setup.bash
```

bringup은 패키지 디렉터리에서 검사했다.
lint 도구가 현재 디렉터리를 검사하기 때문이다.

```bash
cd src/drone_bringup
python3 -m pytest -q -r a -p no:cacheprovider test
```

기존 lint 진단은 E128 3건, E501 1건이다.
문서 문자열의 D213도 8건 있다.
해당 코드가 검토 시작점과 같은지 Git으로 확인했다.
[모듈화 검증](repository_modularization_20261004.md)의 기존 실패와 같다.
copyright skip은 기존 헤더 미작성 조건이다.

Windows 기록의 CRLF와 설정의 끝 개행을 보존했다.
공백 검사는 해당 기존 형식을 허용해 실행한다.

```bash
git -c core.whitespace=blank-at-eol,space-before-tab,cr-at-eol,-blank-at-eof diff --check
```

기계 판독 결과와 명령 출력은
[검증 증빙](evidence/feature_uwb_commit_20261004/validation.json)에 있다.
작업 시작 파일 해시도 같은 증빙 폴더에 보관한다.

## Git 반영 방법

`feature_uwb`를 먼저 푸시한다.
이후 `main`을 같은 커밋으로 fast-forward하고 푸시한다.
확인 당시 `origin/main`은 `6a53473`이다.
`feature_uwb`의 조상임을 확인했다.
완료 판정은 두 원격 브랜치의 커밋 일치다.

## 남은 작업

- bringup의 기존 lint 오류 수정은 남아 있다.
- 실제 TFmini Plus 연결·주기·시계 확인은 미실시다.
- 현재 좌표의 독립 측량·다점 교정은 미실시다.
- 이번 작업의 Windows/WSL 배포는 미실시다.
- 실제 UART·Gazebo 센서·PX4 융합·비행 재검증은 미실시다.
- A/B/C/D 최종 선정과 동적 7cm 성능 검증은 남아 있다.
