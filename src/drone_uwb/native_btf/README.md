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
sudo apt-get install -y build-essential cmake libeigen3-dev nlohmann-json3-dev python3-numpy python3-pytest
cmake -S . -B build-jetson -DCMAKE_BUILD_TYPE=Release
cmake --build build-jetson --parallel 2
ctest --test-dir build-jetson --output-on-failure
./build-jetson/btf_replay examples/config.json examples/freeze.jsonl > freeze-result.jsonl
```

| 결과물 | 용도 |
|---|---|
| `build-jetson/libbtf_core.a` | 계산·FREEZE 라이브러리 |
| `build-jetson/libbtf_native.so` | 기존 Python 노드의 C++ 계산 연결 |
| `build-jetson/btf_replay` | 정규화 입력 파일 재생 |
| `include/btf/core.hpp` | 거리·높이 입력과 XY 판정 API |
| `include/btf/freeze.hpp` | 목표 XY 고정 API |

이 실행은 파일만 읽고 표준출력에 기록한다.
FC 연결·모드 전환·ARM·LAND는 수행하지 않는다.
기존 관측 서비스를 대체하는 명령도 아니다.
운영 연결은 [Jetson 연결 절차](../../../docs/runbooks/btf_cpp_jetson_20261014.md)를 따른다.
저장소 전체에서는 Python 어댑터 시험도 실행한다.
이 폴더만 복사하면 코어·파일 시험 2개 묶음만 실행한다.

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
계산 시간은 시험한 호스트의 수치 연산 시간이다.
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

## 기존 실행기 연결

`MeasuredBtf` 설정에서 `numerical_backend=cpp`를 선택한다.
`native_btf_library`에는 절대 경로를 지정한다.
기본값은 `python`이다. 실행 중 자동 대체하지 않는다.
라이브러리 로드 실패는 시작 실패로 보고한다.
원시 수신·시계·높이·발행 검사는 Python이 유지한다.
C++ 호출 동안 Python 인터프리터 잠금을 해제한다.
센서 전용 실행기는 계속 입력을 받는다.

`fresh_b4_recovery_enabled=true`는 새 실측 복구를 켠다.
기본값 false는 최초 Python 대조 동작을 보존한다.
긴 이력이 거절돼도 최근 150ms의 새 거리로 재계산한다.
4개 앵커·최소 3회·기존 6cm 잔차 기준을 만족해야 한다.
현재 거리 일치도와 기존 속도·연속성 기준도 확인한다.
통과한 뒤에만 오염된 이전 이력을 교체한다.
관측을 복사하거나 원본 시각을 갱신하지 않는다.
실측 재생 결과는 `RECOVERY_VALIDATION.json`에 있다.

운영 C++ 실행기는 기존 상태 머신의 목표 유지 기능을 쓴다.
`/uwb/btf_decision` 거절을 받으면 진행을 즉시 보류한다.
새 pose의 원본 시각이 거절 표본보다 최신이어야 해제한다.
지연된 과거 거절은 새 pose를 무효화하지 못한다.
`freeze.hpp`의 독립 API를 운영에 중복 설치하지 않는다.
이유: 목표 송신기와 미션 상태의 소유자는 하나여야 한다.

아래는 독립 C++ FREEZE API의 연결 계약이다.

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

Jetson 빌드와 ROS 연결 결과는 `JETSON_VALIDATION.json`에 있다.
`VALIDATION.json`은 최초 Windows 검증 기록이다.
실제 기체 비행은 아직 미실시다.
