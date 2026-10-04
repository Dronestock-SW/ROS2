# UWB 코드·자료 분류 기준

수신과 보정의 폴더·책임을 정한 기준이다.
파일을 추가하거나 API 경계를 바꿀 때 읽는다.

## 코드 경계

수신부는 RAW를 보존하고 계산부는 별도 결과를 만든다.

```text
UART bytes -> acquisition/serial_io.py -> acquisition/framing.py
                                            |
                                contracts/protocol.py
                                            |
                               acquisition/validation.py
                                            |
                   processing/clock.py + sensors.py
                                            |
                     processing/ranges.py -> 계산 후보
                                            |
                     integration/ -> ROS 관측 출력

기존 실시간 경로:
 integration/node.py -> processing/observations.py -> /uwb_pose

파일 비교 경로:
 processing/runner.py -> pipeline.py -> 진단 파일
 processing/experiments/ -> A/B 계산·평가 파일
```

두 실행 경로는 아직 별개다.
폴더 분리는 새 보정기의 실시간 연결을 뜻하지 않는다.

| 폴더 | 소유 책임 | 의존성 규칙 |
|---|---|---|
| `contracts/` | JSON·스칼라 공통 검사와 예외 | 수신·계산·ROS에 의존하지 않음 |
| `acquisition/` | 장치 읽기, 행 분리, RAW 계약 검사 | 공통 계약만 사용. 보정기 의존 금지 |
| `processing/` | 시각 대응, 거리 정제, 기하 계산 | 입력 검사·공통 계약 사용. ROS 의존 금지 |
| `processing/experiments/` | 파일 비교, 교정·평가 분리 | 계산 함수를 조합. 장치 출력 없음 |
| `integration/` | ROS·MAVROS 연결, 기록 파일 소유 | 수신과 계산을 연결 |
| 기존 최상위·`preimu/` | 이전 import·실행 경로 호환 | 새 구현을 연결. 계산 코드 추가 금지 |

의존성 제한은 수신기와 계산기를 따로 바꾸기 위함이다.
`test_module_layout.py`가 역방향 import를 검사한다.
호환 모듈은 같은 구현 객체를 반환한다.
기존 예외 클래스와 monkeypatch 대상도 유지한다.
전체 이전 경로는 [경로표](../../data/module_paths.json)에 있다.

## 자료 경계

활용 원본과 계산 결과는 물리적으로 나눈다.

| 경로 | 분류·추가 기준 |
|---|---|
| `data/raw/uwb/` | UWB UART 원본·수신 JSONL·측정 조건 |
| `data/raw/flight/20260926/`, `20260927/` | 날짜별 PX4 ULog 원본 |
| `data/processed/` | 추출값·보정값·그림·평가 결과 |
| `data/catalog.json` | 활용 파일의 용도·크기·SHA-256·이전 경로 |
| `src/drone_uwb/config/` | 실행 설정·앵커 배치·고정 입력 해시 |
| `docs/specs/uwb/` | 기능별 설계·아직 구현하지 않은 후보 |
| `docs/report/` | 날짜별 사람용 확인 기록 |
| `docs/report/evidence/` | 기존 증빙과 새 자료로 가는 호환 링크 |
| `docs/hw_handoff/` | HW 인계·외부 API 참고자료 |
| `src/drone_uwb/tools/` | 분석·그림·자료 감사 도구 |
| `build/`, `install/`, `log/` | 자동 생성물. 수작업 정리 대상 제외 |
| `/home/pgyxn/uwb_pipeline_runs/` | 이전 전체 실행 결과. 이번 이동 대상 제외 |

원본을 수정하지 않는다. 같은 입력 재생을 보장하기 위함이다.
보정 결과는 새 실행 폴더에 기록한다.
과거 증빙 경로에는 상대 심볼릭 링크를 남겼다.
이전 문서와 설정이 같은 바이트를 계속 읽는다.
링크를 지원하는 체크아웃에서 사용한다.
Windows에서는 WSL 체크아웃을 사용한다.
링크를 일반 텍스트 파일로 복사하지 않는다.

수신 당시 metadata와 수기 확인은 입력 근거다.
계산 summary와 구간 분석은 처리 결과다.
날짜별 `result.md`는 원래 문서 위치에 보존한다.
기존 과거 자료 전체를 최신 실측 자료로 재분류하지 않는다.

## 활용 데이터

교정·개발 평가·비행 센서 자료를 구분한다.

| 자료 | 용도 | 조건 |
|---|---|---|
| `uwb_static_20260906_161636` | 거리 편향 후보 산출 | 1차 정지 기록 |
| `uwb_static_20260906_170938` | 별도 개발 평가 | 2차 정지 기록. 임계값 조정 후 최종 평가용은 아님 |
| `raw/uwb/legacy/uwb_raw.txt` | 이전 수신 형식 참고 | 위 교정·평가 파일의 대체 입력 아님 |
| `raw/flight/20260926/*.ulg` | ToF·자세·비행 설정 분석 | 26일 활용 후보. UWB와 다른 세션 |

사용자는 26일 Position 자세제어 기록을 활용하도록 지정했다.
현재 확보한 파일은 기존 추출에서 `ALTCTL`로 확인됐다.
Position 성공 시험과 같은 파일인지는 미확인이다.
[기존 분석 근거](../reference/uwb_mavros_parameter_api.md)를 보존한다.
UWB와 ULog의 시간축을 임의로 합치지 않는다.
서로 다른 날짜의 측정을 동시 관측으로 만들기 때문이다.
자세와 고도 제어는 PX4가 담당한다.

## 관련 명세

[모듈 입출력](../reference/uwb_module_api.md)은 API 분리 시 읽는다.
[실행 절차](../runbooks/uwb_data_runbook.md)는 기록·재생 시 읽는다.
[정리 확인 기록](../report/uwb_data_reorganization_20260927.md)은 인계 시 읽는다.

## 27일 비행 자료 추가

원격에서 27일 ULog를 받아 같은 분류 기준으로 등록했다.
`data/raw/flight/20260927/`에 원본을 둔다.
`data/processed/flight_20260927/`에 추출물을 둔다.
Position 모드 표본 58개를 확인했다.
[등록 기록](../report/flight_log_20260927.md)에 근거를 남겼다.
