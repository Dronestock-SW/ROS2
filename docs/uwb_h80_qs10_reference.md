# UWB H80·Q_S10 참고 자료와 코드 상태

9월 20일 참고 ZIP과 현재 설정을 찾는 사전이다.
필터 구현을 재개하거나 자료를 인계할 때 읽는다.

확인일: 2026-09-21. 코드 기준: `feature_uwb`, `1ef3f03`.

## 2026-09-21 후속 확인

PDF v1.2 전체와 ZIP의 설명·설정·소스를 읽었다.
아래 표의 확인 범위는 최초 조사 시점 기록이다.
PDF는 `/home/pgyxn/0920_2_uwb_jetson_codex_handoff_v1_2.pdf`다.
소스의 실제 순서는 H80→조건부 Q 선택→위치 게이트다.
README 앞부분의 `fixed range bias is zero`는 오기다.
사용자가 작성자의 실수라고 확인했다.
시험 코드는 네 편향을 RAW에서 차감한다.

ToF 실측 자료는 아직 없다.
후속 개발은 시뮬레이션 입력 파이프라인부터 진행한다.
높이·좌표 계산과 외부 출력 게이트는 닫는다.
현재 파일 경로·설정은 [개발 기준](uwb_pipeline_design.md)에 있다.
빌드·검증 결과는 [작업 기록](report/uwb_pipeline_20260921.md)에 있다.

## 자료 위치와 날짜

찾던 자료는 PDF가 아닌 소스·펌웨어 ZIP이다.

| 항목 | 값 |
|---|---|
| 파일명 | `uwb_h80_qs10_reference_source_v1.zip` |
| 로컬 위치 | `/home/pgyxn/uwb_h80_qs10_reference_source_v1.zip` |
| 파일 수정일 | 2026-09-20 18:34:25, 한국 시간 |
| 내부 파일 날짜 | 모두 2026-09-20 |
| PDF 포함 여부 | 없음 |
| 직접 읽은 설명 | `JETSON_REFERENCE_README.md` |

ZIP은 저장소 밖에 있다.
저장소 복제만으로 이 파일이 함께 오지는 않는다.
수정일로 다운로드 시점을 확정할 수는 없다.
ZIP 내부 시각에는 시간대가 기록되어 있지 않다.

SHA256:

```text
1b49160e665c02a9f1e31c9093ae4693859d7053f4cd0507ed984c8348be3e55
```

파일 목록·크기·날짜는 [확인 근거](report/evidence/setup_status_20260921.json)에 있다.

| ZIP 내부 파일 | 수정 시각 | 설명 |
|---|---|---|
| `JETSON_REFERENCE_README.md` | 18:19:20 | 이식 목적과 시험 조건 |
| `firmware/uwb_tag_h80_qs10_test_reference.bin` | 17:18:54 | 시험용 실행 파일 |
| `README.md` | 17:16:16 | 프로젝트 설명. 이번 본문 확인 대상 아님 |
| `platformio.ini` | 17:16:16 | 빌드 환경 |
| `include/RawTagConfig.h` | 17:16:16 | 통신·출력 설정 |
| `src/ExperimentalPositionFilter.cpp` | 17:07:32 | 필터 구현 |
| `src/main.cpp` | 17:07:32 | RAW 수집·진단 연결 |
| `include/ExperimentalPositionFilter.h` | 17:06:50 | 좌표·보정·게이트 설정 |

## ZIP README의 처리 설명

새 ESP32 태그의 시험 로직을 companion에 옮기는 자료다.
기존 연결 태그는 RAW 거리와 메타데이터를 전송한다.
README는 기존 태그에 시험 펌웨어를 덮지 않아도 된다고 한다.

```text
RAW 사거리
    -> 고정 거리 편향 차감
    -> 거리 변화 게이트
    -> 최근 0.8초 H80 계산
    -> 위치 변화·적합 잔차 판별
    -> 150 ms 초과 단절 직후 Q_S10 복구 시도
    -> OLED·진단 JSON 출력
```

이 흐름은 README의 설명을 옮긴 것이다.
ZIP의 C++ 전체를 검토하거나 실행하지 않았다.

## ZIP 시험값과 저장소 기본값

시험 태그의 교정값을 현재 태그의 확정값으로 쓰지 않는다.
이유는 태그·앵커 조합과 시험 위치가 다르기 때문이다.

| 항목 | ZIP README의 시험값 | 현재 `PreimuSettings` 기본값 |
|---|---|---|
| 높이 | 1.13 m, 고정 시험값 | 1.2 m, 실측 전 임시값 |
| 거리 편향(m) | `[-0.08733, +0.41107, -0.11148, +0.15125]` | 전부 0, `range_bias_calibrated=False` |
| 거리 게이트 | `0.20 m + 1.20 m/s × Δt` | 같음 |
| 새 거리 확인 | 0.15 m 이내 3회 | 같음 |
| 위치 게이트 | `0.08 m + 0.80 m/s × Δt` | 같음 |
| 새 위치 확인 | 0.12 m 이내 3회 | 같음 |
| H80 창 | 0.8초 | 같음 |
| Huber 기준 | 0.12 m | 같음 |
| 적합 RMS 상한 | 0.10 m | 같음 |
| 복구 단절 기준 | 150 ms 초과 | `recovery_gap_s=0.15` |

현재 값의 근거는 [settings.py](../src/drone_uwb/drone_uwb/preimu/settings.py)다.
값의 일치는 전체 처리 동작의 일치를 증명하지 않는다.

ZIP의 시험 기준 태그 위치는 `(0.70, 1.80, 1.13) m`다.
앵커는 다음 좌표를 사용했다고 적혀 있다.

| 앵커 | x(m) | y(m) | z(m) |
|---|---:|---:|---:|
| A1 | 0.00 | 0.00 | 2.20 |
| A2 | 5.80 | 0.00 | 2.20 |
| A3 | 0.00 | 4.63 | 2.20 |
| A4 | 6.11 | 4.36 | 2.20 |

이 표로 기존 앵커 설정을 덮어쓰지 않았다.
실측 상태는 [앵커 문서](uwb_anchor_survey.md)를 따른다.

## 코드에서 확인한 상태

계산 모듈은 존재한다. 수신 노드 연결은 확인되지 않았다.

| 위치 | 확인한 내용 | 남은 확인 |
|---|---|---|
| [preimu/](../src/drone_uwb/drone_uwb/preimu/) | 설정·입력·시각·좌표·계산 소스 | 전체 처리 연결·검증 |
| [h80.py](../src/drone_uwb/drone_uwb/preimu/h80.py) | `fit_h80` 함수 | 재생·실측 결과 |
| [qs10.py](../src/drone_uwb/drone_uwb/preimu/qs10.py) | `solve_q_s10` 함수 | 복구 조건·결과 검증 |
| [node.py](../src/drone_uwb/drone_uwb/node.py) | 기존 RAW 수신 경로 | `preimu` 호출 연결 |
| [uwb.yaml](../src/drone_uwb/config/uwb.yaml) | 브리지 `enabled: false` | 검증 후 전달 조건 |

ZIP은 실시간 ToF·자세·장착 위치로 Z를 구하도록 한다.
현재 저장소의 기본값은 고정 높이 시험용이다.
두 조건은 아직 같지 않다.
실시간 Z가 없는 상태를 ZIP의 운용 조건 충족으로 보지 않는다.

README는 먼저 Shadow 모드로 기록하라고 한다.
RAW·후보값·최종값·판정 사유가 대상이다.
기체 고도 제어 책임은 [고도 원칙](altitude_policy.md)을 따른다.

## README에 보고된 시험 결과

아래 수치는 제공 자료의 보고값이다.
이번에 원시 로그로 재계산하거나 재시험하지 않았다.

| 시험 | 중앙 오차 | p95 | 최대 |
|---|---:|---:|---:|
| 고정 위치 20초 재시험 | 3.29 cm | 5.05 cm | 5.79 cm |
| 사람 차폐 5분, 게이트 적용 후 | 2.45 cm | 6.01 cm | 20.84 cm |

README는 다른 태그·위치·비행의 7 cm 보장을 부인한다.
현재 companion의 달성 성능으로 인용하지 않는다.
이유는 같은 입력과 조건에서 검증하지 않았기 때문이다.

## 함께 발견한 PDF

세 파일의 수정일도 모두 2026-09-20이다.
아래 시각은 파일시스템의 한국 시간이다.

| 파일 | 수정 시각 | 확인 범위 |
|---|---|---|
| [KCI_FI003284551.pdf](KCI_FI003284551.pdf) | 17:56:02 | 첫 페이지 제목·초록 |
| [uwb_imu_learning_guide_20260911.pdf](report/uwb_imu_learning_guide_20260911.pdf) | 17:56:03 | 파일 존재·수정일 |
| [uwb_field_request.pdf](report/uwb_field_request.pdf) | 17:59:10 | 파일 존재·수정일 |

첫 논문 제목은 다음과 같다.
「터널 내에서 UWB-IMU 센서융합 기반 열차 정밀 측위」.
세 PDF와 ZIP의 직접적인 연관성은 확인하지 않았다.
