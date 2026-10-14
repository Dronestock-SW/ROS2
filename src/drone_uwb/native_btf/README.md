# B_TF C++ 로컬 이식본
Jetson에서 독립 빌드하는 절차다.
압축을 풀거나 이 폴더를 복사한 뒤 읽는다.

**C++17 계산 라이브러리와 파일 재생기다.**
ROS·FC 송신기는 포함하지 않는다.
구조와 FREEZE 계약은 [DESIGN.md](DESIGN.md)에 있다.
Python 기준 소스는 `SOURCE_BASELINE.json`에 기록한다.

## Jetson 빌드

이 폴더에서 다음 명령을 실행한다.
CMake 3.22 이상을 사용한다.
필요한 개발 패키지가 있으면 설치 단계는 생략한다.

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake libeigen3-dev nlohmann-json3-dev
cmake -S . -B build-jetson -DCMAKE_BUILD_TYPE=Release
cmake --build build-jetson --parallel 2
ctest --test-dir build-jetson --output-on-failure
./build-jetson/btf_replay examples/config.json examples/freeze.jsonl > freeze-result.jsonl
```

| 결과물 | 용도 |
|---|---|
| `build-jetson/libbtf_core.a` | 계산·FREEZE 라이브러리 |
| `build-jetson/btf_replay` | 정규화 입력 파일 재생 |
| `include/btf/core.hpp` | 거리·높이 입력과 XY 판정 API |
| `include/btf/freeze.hpp` | 목표 XY 고정 API |

이 실행은 파일만 읽고 표준출력에 기록한다.
FC 연결·모드 전환·ARM·LAND는 수행하지 않는다.
기존 관측 서비스를 대체하는 명령도 아니다.
이유: 시계·ROS·송신 경계의 이식은 별도다.

## Python 대조

같은 저장소의 Python 기준 구현이 필요하다.
NumPy가 설치된 Python에서 실행한다.
아래는 이 폴더를 현재 디렉터리로 둔 예다.

```bash
python3 tests/parity.py \
  --binary build-jetson/btf_replay \
  --python-root .. \
  --output parity-jetson
```

합성 거리와 앵커별 시각을 동일하게 재생한다.
판정·거절 사유·후보 번호·XY·점수를 대조한다.
`summary.json`의 `passed`를 확인한다.
계산 시간은 이 PC의 수치 연산 시간이다.
센서부터 FC까지 지연을 뜻하지 않는다.

보존된 ROS 수집 파일도 대조할 수 있다.
`/uwb/btf_decision`에 정규화 입력이 있어야 한다.
해당 기록의 앵커·bias 설정을 지정한다.

```bash
python3 tests/compare_recording.py \
  --binary build-jetson/btf_replay \
  --python-root .. \
  --config ../config/runtime/uwb_btf_tag_b.json \
  --recording /path/to/events.jsonl \
  --output parity-recording
```

두 구현은 모두 빈 이력에서 재생을 시작한다.
로그 조각 앞부분의 운영 이력을 복원하지 않는다.
당시 실제 발행 결과와 일치한다는 시험은 아니다.

## 기존 실행기에 연결할 때

수신·계산·목표 송신을 독립적으로 실행한다.
수신 어댑터가 태그와 시계 유효성을 확인한다.
ToF·IMU로 각 앵커 시각의 높이를 준비한다.
`Processor::process()`에 정규화 입력을 준다.
승인 관측은 원본 측정시각으로 한 번 전달한다.
모든 승인·거절 결과는 `observe()`에 전달한다.
주기적인 `tick()`은 무수신 때도 계속 호출한다.

`pause_mission=true`이면 경로 진행을 멈춘다.
`target_xy`는 실행기의 같은 좌표계로 해석한다.
`observation_recovered_resume_next_tick`에서는
고정 목표를 미션의 재개 기준으로 보존한다.
다음 경로는 그 목표부터 기존 속도 제한으로 잇는다.

FC 보고 지연은 `Unknown`으로 분류한다.
기존 목표 송신은 유지하고 진행만 보류한다.
FC가 명시적으로 추정 무효를 보고하면 `Invalid`다.
이때 기존 FC 보호·조종자 인계 로직에 넘긴다.
배터리·RC·임무 시간 보호도 실행기에 남긴다.

아직 Jetson 빌드·ROS 연결·실비행은 미실시다.
실제 비행 적용 전에 위 연결 경계를 검증한다.
