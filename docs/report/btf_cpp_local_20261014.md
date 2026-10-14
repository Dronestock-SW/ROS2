# 로컬 B_TF C++와 목표 FREEZE
로컬 이식 범위와 검증 결과를 기록한다.
Jetson 빌드와 실행기 연결 전에 읽는다.

후속 [Jetson 연결 기록](btf_cpp_jetson_20261014.md)을 먼저 확인한다.
아래 표는 최초 로컬 이식 시점의 검증이다.

**계산 코어와 목표 FREEZE를 분리했다.**
통합 브랜치의 Python 계산을 기준으로 이식했다.
기준 커밋은 `5db9c0ddf7be164b03e097acffac0440256994c8`다.
운영 Python·ROS·FC 파라미터는 변경하지 않았다.

## 결과물

| 파일 | 역할 |
|---|---|
| `src/drone_uwb/native_btf/src/h80.cpp` | H80 수치 계산 |
| `src/drone_uwb/native_btf/src/btf.cpp` | 이력·B4/B3·연속성·재획득 |
| `src/drone_uwb/native_btf/src/freeze.cpp` | 목표 XY 고정과 복구 |
| `src/drone_uwb/native_btf/src/replay.cpp` | JSONL 파일 재생 |
| `src/drone_uwb/native_btf/tests/parity.py` | Python과 동일 입력 비교 |

[빌드 절차](../../src/drone_uwb/native_btf/README.md)를 따른다.
[구조·입력 계약](../../src/drone_uwb/native_btf/DESIGN.md)을 확인한다.

## 검증

Windows x64에서 Release 빌드를 통과했다.
검증 컴파일러는 Clang 18.1.6이다.
Zig 0.13.0의 C++ 도구를 사용했다.
Eigen 3.4.0·JSON 3.11.3을 사용했다.

| 검증 | 결과 |
|---|---|
| CTest | 2개 묶음 통과 |
| JSONL 실행 계약 | CTest 안의 5개 사례 통과 |
| 합성 동일 입력 대조 | 22개 시나리오·3,760건 통과 |
| 승인·거절 사유·선택 후보 | Python과 불일치 0건 |
| 합성 승인 XY 최대 차이 | 약 2.94e-14m |
| 보존 실측 입력 대조 | 113건 통과, 제외 0건 |
| 실측 입력의 승인 XY 최대 차이 | 약 5.33e-15m |
| 실측 거절 구간 FREEZE | 32건 모두 고정 목표 유지 |

정지·이동·가감속·거리 튐을 주입했다.
ToF 누락·높이 불일치·시계 재시작도 포함했다.
지상 초기화·근거리·이동 재획득을 대조했다.
FREEZE 중 바뀐 요청 목표도 반영하지 않았다.
관측 복구 뒤 RUNNING 전환을 확인했다.
오래된 관측을 새 시각으로 바꾸지 않았다.

실측 자료는 0.824초 공백 기록의 조각이다.
정규화한 113건을 빈 이력에서 대조했다.
기존 32회 `no_height_consistent_subset`을 재현했다.
FREEZE 시험의 host 시계는 시험용 대응값이다.
실물 시계 보정과 FC 호버를 검증한 것은 아니다.

PC 계산 시간의 마지막 대조 결과는 아래와 같다.
초기화·정상·B3 경로를 포함한 혼합 입력이다.

| 계산 시간 | Python | C++ |
|---|---:|---:|
| 중앙값 | 0.8165ms | 0.0482ms |
| p95 | 5.1700ms | 0.7450ms |

단일 PC 실행의 수치 연산 시간이다.
ROS·시계·센서 수신·FC 전달 시간을 포함하지 않는다.
Jetson 속도나 0.824초 공백 감소율로 환산하지 않는다.

검증 파일은 PC의 다음 경로에 있다.
`outputs/btf_cpp_20261014/parity-pc/summary.json`
`outputs/btf_cpp_20261014/recording-pc/summary.json`
`outputs/btf_cpp_20261014/recording-pc/freeze-summary.json`
이 경로는 이 채팅 작업공간 기준이다.
Jetson 빌드·ROS 연결·실비행은 미실시다.
0.824초 관측 공백 자체의 해소와 구분한다.

## 남은 작업

현재 이식은 시각·높이 보정 후 입력부터다.
원시 프로토콜·시계·ROS 어댑터는 기존 책임이다.
Jetson에서 동일 입력 대조와 계산 시간을 확인한다.
기존 실행기에 FREEZE API를 연결해 검증한다.
그 전에는 운영 관측 서비스를 교체하지 않는다.
이유: 계산 일치만으로 송신 경계를 검증할 수 없다.
이상 표본의 이력 격리는 별도 연구로 남는다.
