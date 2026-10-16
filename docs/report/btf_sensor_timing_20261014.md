# B_TF IMU 수신·계산 분리 검증

2026-10-14 통합 브랜치 수정과 13일 원본 재생 결과다.
실기 배포 여부를 검토할 때 읽는다.

## 결론

**IMU 수신 구조는 수정했다. 13일 비행 공백의 해결은 아직 아니다.**
DDS 큐를 5→100개로 확대했다.
IMU·ToF 수신을 계산과 다른 실행기에 배치했다.
기본 내부 기록과 고정 센서 스냅샷을 추가했다.
ToF 100ms·IMU 20ms 판정 기준은 유지했다.

13일 원본의 파일 재생을 완료했다.
수정 전후 100ms 결과는 같은 0.699959277초 공백을 보였다.
120·150ms로 늘려도 해당 27회에서 발행은 없었다.
그러므로 이번 수정만으로 비행 공백 해결을 선언하지 않는다.
현장 서비스·FC 펌웨어·센서 설정은 변경하지 않았다.

ToF 후속안은 [시나리오별 설계](../architecture/tof_rate_scenarios_20261014.md)에 있다.
우선 제안은 TFmini Plus I²C 50Hz 경로 검증이다.

## 원본과 수정 기준

다른 비행이나 재가공 파일을 원본으로 대체하지 않았다.
`arialhanho` SSH 계정에서 파일을 직접 읽었다.

| 항목 | 기준 |
|---|---|
| 통합 브랜치 시작점 | `19cdb8b975b2d56107d7e470289da6e7779bec2a` |
| 브랜치 | `이번에-상원-한거-통합-브랜치` |
| capture | `capture-1791884513951202644-a5b6e3e7/events.jsonl` |
| capture 크기 | 124,382,507 bytes |
| capture SHA256 | `829d1055c8d7e49b3c4e019560cb5bf4b278b885496b06c030412da5cc30c91e` |
| ULog | `20261013-1837-plus-y/ulog-flight-download/log-31.ulg` |
| ULog 크기 | 19,413,299 bytes |
| ULog SHA256 | `39b5a1c1073eb495e4931f205e7a0465486f060cc91e18ad2b3811bd32982166` |
| 재생 설정 | manifest의 `field-btf.json` 내용·해시 검증 |
| 실행 환경 | Jetson, ROS Humble, Python 3.10, `ROS_DOMAIN_ID=99`, localhost |

원본 위치는 다음과 같다.

```text
/home/arialhanho/.local/share/dronestock/manual-captures-20261013/
  capture-1791884513951202644-a5b6e3e7/
/home/arialhanho/ROS2-review-20261008-codex/.review/
  fc-owned-runtime-20261013/sessions/20261013-1837-plus-y/
```

시험 소스·결과는 별도 디렉터리에 둔다.
`/home/arialhanho/btf-timing-validation-20261014/`다.
기존 실행본과 다른 작업의 미커밋 파일은 수정하지 않았다.

## −2ms IMU가 빠진 원인에 대한 증거 수준

버퍼에 −2ms가 있으면 선택기는 −2ms를 고른다.
이 선택 규칙 자체를 바꾸는 패치는 필요하지 않았다.

기존 규칙은 `imu.stamp <= tof.stamp` 중 최신 값이다.
미래 IMU를 끌어오거나 시간차 절댓값만 최소화하지 않는다.
합성 입력 `−22, −12, −2, +8, +18ms`로 이를 검증했다.
최신 과거 샘플이 없으면 −22ms를 고르고 거부한다.

13일 기록에는 B_TF 노드의 개별 IMU 콜백 로그가 없다.
별도 수집기의 수신 기록만 있다.
seq 17497의 판단 수신보다 −2ms IMU 수신이 0.575ms 늦었다.
seq 17498 이후에는 수집기가 그 IMU를 이미 받았다.
그러나 수집기 수신은 B_TF 버퍼 반영 시각과 다르다.

| 증거 | 말할 수 있는 범위 |
|---|---|
| 실제 B_TF 선택 −22ms | 당시 선택 경로에서 더 최신의 유효한 과거 입력을 쓰지 못함 |
| 수집기의 −2ms 존재 | 동일 센서 데이터가 다른 구독자에는 전달됨 |
| DDS depth 5·단일 실행기 | 계산 중 도착 버스트가 유실될 수 있는 구조 |
| 36ms 지연·3ms 간격 버스트 실험 | depth 5에서 −12·−2ms 소실, depth 100에서 보존 가능 |
| 이번 전용 실행기 시험 | 계산 중 8개 IMU 수신, 다음 스냅샷에서 −2ms 선택 |
| 당시 내부 기록 부재 | 특정 메시지가 DDS에서 사라졌는지, 콜백이 늦었는지는 확정 불가 |

이전 조사에서 확인한 버퍼 역행 시 전체 삭제도 남겨 두었다.
이번에는 `backward_stamp_reset`으로 원인을 기록한다.
증거 없이 시각 역행을 정상 입력으로 허용하지 않는다.

## 최종 코드 구조

수신 실행기는 H80·B3 계산을 실행하지 않는다.
계산 실행기는 센서 버퍼 잠금을 잡은 채 적합을 수행하지 않는다.

```text
IMU/ToF DDS KEEP_LAST 100
          |
          v
uwb_btf_sensor_ingress / 전용 SingleThreadedExecutor
          |
          v
짧은 잠금: 시각·유효성 검사 → 버퍼 append → receive_id 기록
          |
          +---- 고정 스냅샷 ----> uwb_btf_node / 계산 실행기
                                  |
                                  +--> H80 / B_TF / 기존 발행 검사
                                  +--> 선택 IMU id·stamp·판단 기록

입력·스냅샷·판단 → bounded queue → 별도 JSON/파일 기록 스레드
```

| 변경 | 구현 | 유지한 조건 |
|---|---|---|
| 큐 확대 | ToF·IMU BEST_EFFORT KEEP_LAST 100 | 원본 stamp·500ms 입력 검사 |
| 수신·계산 분리 | 별도 노드·실행기·수신 스레드 | 계산·FC 상태·리셋 콜백의 직렬 처리 |
| 계산 스냅샷 | 한 주기의 네 앵커가 같은 버퍼 집합 사용 | 계산 중 도착은 다음 계산에 반영 |
| 내부 추적 | 수신 id·시각·버퍼 결과·선택 id·스냅샷·처리 시각 | 기존 거부·발행 기준 |
| 저장 부하 분리 | 큐 4096개, 기록 스레드에서 JSON 변환 | 저장 실패로 센서 유효성을 바꾸지 않음 |
| 용량 제한 | 32MiB×4 파일, 여유 공간 256MiB 기준 | 넘치면 오류·손실 카운터 표시 |

처음 비교한 공유 `MultiThreadedExecutor` 방식은 채택하지 않았다.
재생에서 UWB 대기가 증가했다.
최종 구조는 수신용·계산용 실행기를 나눈다.
Python GIL과 OS 스케줄은 여전히 공유한다.
따라서 하드 실시간 또는 무손실을 보장하는 구조는 아니다.

### 내부 기록 읽는 법

`record_directory=''`여도 기본 내부 기록은 켜진다.
기본 경로는 아래와 같다.

```text
${XDG_DATA_HOME:-~/.local/share}/dronestock/btf-trace/domain-${ROS_DOMAIN_ID:-0}/
  trace.jsonl       최신
  trace.1.jsonl
  trace.2.jsonl
  trace.3.jsonl     가장 오래된 보존 구간
```

`trace_directory`로 위치를 바꿀 수 있다.
빈 문자열은 명시적 비활성이다.
동일 경로의 중복 기록기는 파일 잠금으로 거부한다.
각 파일 첫 줄에 session·설정·소스 해시가 있다.
재시작과 회전으로 오래된 진단 파일은 순차 교체된다.
오류 구간을 보존하려면 재시작 전에 해당 session을 복사한다.

| 필드 | 뜻 |
|---|---|
| `received_monotonic_ns` | 센서 콜백이 행을 받은 로컬 시각 |
| `header_age_ns` | 콜백 ROS 시각 − 메시지 원본 stamp |
| `receive_id` | 이 프로세스의 ToF·IMU 공통 수신 번호 |
| `buffer_accepted`, `buffer_reason` | 삽입·중복·역행·시각 오류 구분 |
| `sensor_snapshot.receive_id` | 해당 계산이 볼 수 있는 마지막 수신 번호 |
| `sensor_snapshot.epoch` | 연결·시각 리셋에 따른 버퍼 세대 |
| `height_selection[].imu_receive_id` | 실제 선택된 IMU 수신 번호 |
| `callback_started_monotonic_ns` | UWB 계산 콜백 시작 |
| `processing_finished_monotonic_ns` | 판단 계산 종료 |
| `enqueued_monotonic_ns` | 진단 행을 기록 큐에 넣은 시각 |
| status `trace.error/dropped/pending` | 진단 기록의 누락·대기 여부 |

`received_monotonic_ns`는 DDS 도착 시각이 아니다.
발행자 기록과 대조해야 콜백 이전 누락을 더 좁힐 수 있다.
최대 1초 간격 flush는 정상 실행의 가시성 기준이다.
전원 차단 후 마지막 기록의 디스크 내구성을 보장하지 않는다.

## 재생 결과 해석

원본 기록, 파일 재생, ROS 재생을 서로 구분한다.
원본 수집 순서는 당시 B_TF 콜백 순서가 아니다.

| 비교 | 높이 불가 | 후보 부족 | 높이 잔차 불일치 | 27회 중 발행 | 앞뒤 공백 |
|---|---:|---:|---:|---:|---:|
| 실제 13일 판단 | 11 | 0 | 16 | 0 | 0.699959277s |
| 기존 코드 파일 재생, 100ms | 21 | 1 | 5 | 0 | 0.699959277s |
| 수정 코드 파일 재생, 100ms | 21 | 1 | 5 | 0 | 0.699959277s |
| 수정 코드 파일 재생, 120ms | 19 | 0 | 8 | 0 | 0.699959277s |
| 수정 코드 파일 재생, 150ms | 17 | 0 | 10 | 0 | 0.699959277s |

100ms의 수정 전후 계산 결과가 같은 것이 정상이다.
파일 재생에는 DDS 큐 유실이나 동시 실행이 없기 때문이다.
선택 규칙이나 기하 판정을 임의로 완화하지 않았다.
전체 3,369개 판단도 새 진단 필드를 제외하면 동일했다.

수집 순서를 그대로 재생하면 IMU 가용 시점도 달라진다.
seq 17497~17500의 IMU 시간차는 약 141·112·82·62ms다.
당시 판단에 기록된 22ms와 다르다.
따라서 이 재생을 당시 콜백의 정확한 복원으로 표현하지 않는다.

ROS 재생은 원본 입력 84.438421883초를 1배속으로 발행했다.
원본 ROS 시각에는 하나의 상수만 더했다.
간격·센서값·RAW monotonic 시각을 보존했다.
발행자와 B_TF는 다른 프로세스로 실행했다.
메시지 변환을 미리 끝내고 재생 중 큰 GC 지연을 피했다.
FC 연결이나 원래 ROS 도메인으로 발행하지 않았다.

재생 도구 초기 버전은 발행 지연이 300ms를 넘었다.
그 결과는 원인 입증이나 개선율 산정에 사용하지 않는다.
최종 결과와 잔여 제한은 아래 검증 표에 기록한다.

## 검증 상태

센서 수신 보존은 확인했다. 전체 처리 지연은 추가 개선이 필요하다.

| 시험 | 결과 |
|---|---|
| Jetson `colcon build --symlink-install --packages-select drone_uwb` | 성공 |
| 패키지 전체 pytest | 664 passed, 2 skipped, 기존 실패 1건 |
| 기존 실패 대조 | 기준 커밋에서도 `lidar_motion.py:14` 상대 import 검사 실패 |
| 계산 중 IMU 버스트 | 8개 수신, 고정 스냅샷 유지, 다음 계산 −2ms 선택 |
| 기록 회전·큐 포화·디스크 부족·중복 기록기 | 오류 격리 시험 통과 |
| 최종 원본 ROS 재생 | IMU 8,427/8,427, ToF 623/623 수신 |
| 최종 내부 기록 | error 없음, dropped 0, pending 0 |
| 최종 발행 도구 지연 | 최대 6.057989ms, 5ms 초과 12행 |
| 실물 ToF 50Hz·PX4 융합·비행 | 미실시 |

pytest는 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`로 실행했다.
기본 ROS pytest 플러그인 탐색은 패키지 전체를 skip했다.
실제 테스트 수를 확인하기 위해 자동 플러그인 로드를 제외했다.
필요한 ROS Python 환경은 Humble 설정으로 로드했다.
기존 실패를 숨기거나 이번 작업에서 관련 없는 LiDAR 코드를 고치지 않았다.

### 처리 성능의 미통과 항목

같은 1배속 입력의 최종 실행에서 발행 결과는 오히려 줄었다.
이 결과 때문에 실기 배포를 승인할 수 없다.

| 노드 관측값 | 기존 코드 비교 실행 | 최종 수정본 |
|---|---:|---:|
| IMU 수신 | 8,409 | 8,427 |
| ToF 수신 | 623 | 623 |
| 처리 시간 중앙값 | 7.17ms | 10.65ms |
| 처리 시간 p95 | 10.93ms | 20.95ms |
| RAW 큐 만료 | 77 | 117 |
| 출력 시각 만료 | 6 | 61 |
| 전체 pose 발행 | 2,912 | 2,427 |

개별 실행의 부하와 ROS 스케줄은 완전히 같지 않다.
그래도 증가한 지연을 무시하고 성능 개선으로 보고하지 않는다.
전체 수신·동시 실행·내부 기록의 CPU 비용을 더 줄여야 한다.
전용 실행기는 Python GIL까지 분리하지 않는다.
기록 직렬화의 프로세스 분리·계산 비용 축소가 후속 후보이다.
150ms RAW 제한이나 원본 stamp를 바꿔 감추지 않는다.

수정본 ROS 재생에서는 추가 큐 만료가 모델 이력을 리셋했다.
문제 구간에 `clock_unsynced` 등이 섞였다.
이후 일부 B4 발행이 있어도 원래 공백을 해결한 증거가 아니다.
그 구간 앞뒤가 모두 관측되지 않으면 공백 수치를 `null`로 남긴다.
정확한 원본 공백 비교에는 위의 파일 재생 결과를 사용한다.

### 저장 공간 보호 확인

여러 중간 재생 파일로 여유 공간이 256MiB 아래로 내려갔다.
기록기는 `trace_disk_reserve_low`로 중단했다.
관측 계산은 계속됐고 기록 누락을 status에 표시했다.
그 실행은 정상 기록 검증에서 제외했다.
이 작업의 중간 JSONL만 gzip 압축·해시 대조 후 교체했다.
약 270.6MiB를 확보한 뒤 최종 재생을 다시 수행했다.
최종 실행의 내부 기록 손실은 0이다.
사용자의 원본 로그나 다른 실행 파일은 삭제하지 않았다.

### 재실행 진입점

실물 서비스와 연결되지 않는 별도 작업 폴더에서 실행한다.
비교용 baseline 폴더는 `COLCON_IGNORE`로 빌드 탐색에서 제외한다.

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select drone_uwb
source install/setup.bash
export ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q src/drone_uwb/test
python3 src/drone_uwb/tools/replay_manual_btf.py "$CAPTURE" \
  --output "$RESULTS/offline-patched120.jsonl" --max-tof-age-s 0.12
python3 src/drone_uwb/tools/replay_btf_ros_capture.py "$CAPTURE" \
  --output "$RESULTS/new-ros-process-run"
python3 src/drone_uwb/tools/compare_btf_timing_replays.py "$CAPTURE" "$RESULTS" \
  --output "$RESULTS/timing-comparison.json"
```

`CAPTURE`는 위 원본 capture 디렉터리다.
`RESULTS`는 이 시험만의 새 결과 디렉터리다.
ROS 재생은 기존 출력 디렉터리를 덮어쓰지 않는다.
비교 도구는 압축 보존한 `.jsonl.gz`도 읽는다.

## 배포 전 남은 판단

센서 입력 누락을 줄이는 수정과 비행 공백 해소는 별도다.
이번 구조 변경만으로 비행을 재개할 근거는 부족하다.

1. 지상 부하에서 수신·계산 지연 분포를 확인한다.
   진단 기록을 켠 상태로 발행 마감 위반을 비교한다.
2. TFmini Plus 실물 설정과 FC 50Hz 읽기를 검증한다.
   결과를 보고 ToF 만료값을 결정한다.
3. A1의 22~25cm 거리 튐과 B3 기하 거부를 별도로 해결한다.
   잔차 기준을 넓혀 통과시키는 방식으로 대체하지 않는다.
4. 동일한 ULog 지표로 EV 공백·재개 리셋을 다시 확인한다.
   기존 약 850ms 공백과 물리적 밀림을 구분한다.

원본 비행과 ULog 연결은
[13일 공백 분석](takeoff_gap_root_cause_20261014.md)을 따른다.
첨부 API 문서는 수정 위치를 찾는 참고 자료로 사용했다.
문서의 설명용 수치를 새 실측 결과로 취급하지 않았다.
