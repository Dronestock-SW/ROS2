# UWB 파이프라인 구현·검증 기록

2026-09-21 파일 파이프라인의 확인 기록이다.
이번 구현 범위와 남은 작업을 인계할 때 읽는다.

## 확인 결과

**입력부터 진단 기록까지 연결했다.**
계산·외부 전달 게이트는 닫혀 있다.
설계는 [개발 기준](../uwb_pipeline_design.md)을 따른다.
실행은 [절차](../uwb_pipeline_runbook.md)를 따른다.

2026-09-21 후속 지시로 추가 제안을 아이디어로 분류했다.
대상은 후속 게이트와 H80·Q_S10 비교 시험안이다.
확정 개발 계획이나 성능 기준으로 사용하지 않는다.
첨부 대화의 제안도 같은 상태로 보관한다.
내용은 [대화 확인 기록](uwb_chat_review_20260921.md)에 있다.
아래 구현·시험 기록은 이번 문서 정리로 변경되지 않았다.

| 검증 | 결과 |
|---|---|
| UWB 패키지 pytest | 38개 통과 |
| 워크스페이스 colcon 빌드 | 5개 패키지 성공 |
| 설치된 `uwb_pipeline` 실행 | 성공 |
| 합성 10초 UWB cycle | 400개 모두 입력 검사 통과 |
| ToF·자세 표본 | 각각 400개 기록 |
| 상태·tick 포함 총 입력 | 1,202개 |
| 원본 재생 | 진단 로그 바이트 동일 |
| 최종 거리 이력 | 앵커별 32개 |
| 좌표·valid | 전 기록 x/y/z=null, valid=false |
| 입력 단절 tick | 0.6초에서 input_stale=true |

UWB clock은 30번째 cycle부터 준비 상태였다.
준비 상태의 cycle은 371개였다.
수신 지연의 하한을 이용한 근사 매핑이다.
하드웨어 동기화 성능을 검증한 결과는 아니다.

빌드에는 YDLIDAR의 미사용 인수 경고가 있었다.
해당 드라이버에 빌드 오류는 없었다.

## 구현한 모듈

기존 수식 모듈 앞에 입력·기록 경로를 붙였다.

| 변경 | 위치 |
|---|---|
| 합성 입력·재생 실행 | `preimu/simulator.py`, `preimu/runner.py` |
| 처리 단계 연결 | `preimu/pipeline.py` |
| 편향 차감·거리 게이트·이력 | `preimu/ranges.py` |
| ToF·자세 버퍼 | `preimu/sensors.py` |
| 보류 중인 높이 수식 | `preimu/height.py` |
| RAW 상태·duration 검사 보강 | `preimu/inputs.py` |
| 실행 진입점 | `setup.py`: `uwb_pipeline` |
| 명시적 편향·닫힌 게이트 설정 | `config/preimu_pipeline.json` |
| 파이프라인 검증 | `test/test_preimu_pipeline.py` |

RAW-only 소스의 상태에는 clock_domain이 없다.
알려진 펌웨어 식별자로만 그 생략을 수용한다.
알 수 없는 clock_domain을 추정해 통과시키지 않는다.

## 검증한 동작

계산 보류 상태에서 정보가 보존되는지 검사했다.

- RAW와 입력 객체가 변경되지 않음을 확인했다.
- 네 편향값을 RAW에서 차감하는 부호를 확인했다.
- 단발 거리 튐을 보류하고 정상값으로 복귀했다.
- 새 거리 군집을 3회 확인한 뒤 수용했다.
- 실패 앵커를 승인 이력에 추가하지 않았다.
- 중복 seq를 거부했다.
- source gap에서 이전 거리 창을 비웠다.
- boot에서 시간·거리·센서 상태를 초기화했다.
- 미래·오래된 센서 표본을 사용 가능으로 표시하지 않았다.
- 잘못된 JSON·UTF-8·비유한 값의 원문을 보존했다.
- 설정으로 계산·외부 전달을 켜면 거부했다.
- 계산 함수 호출 감시로 미호출을 확인했다.
- 높이 수식은 알려진 기울기·편향·장착 위치로 확인했다.

높이 수식의 단위시험은 파이프라인 활성화를 뜻하지 않는다.
기존 H80·Q 전체 수식의 재검증도 아니다.

## 원본과 확인 근거

실행 원문은 저장소 밖에 보존했다.
폴더를 제거하면 저장소 JSON만 남는다.

| 자료 | 경로 |
|---|---|
| 합성 입력 | `/home/pgyxn/uwb_pipeline_runs/20260921_simulation/input.jsonl` |
| 처리 진단 | `/home/pgyxn/uwb_pipeline_runs/20260921_simulation/diagnostics.jsonl` |
| 적용 설정·요약 | `/home/pgyxn/uwb_pipeline_runs/20260921_simulation/summary.json` |
| 재생 결과 | `/home/pgyxn/uwb_pipeline_runs/20260921_replay/` |
| 건수·동일성·SHA256 | [검증 JSON](evidence/uwb_pipeline_20260921.json) |

## 미실시·남은 작업

다음 항목은 이번 완료 범위에 포함하지 않는다.

| 항목 | 상태 |
|---|---|
| ToF 실측 자료 입력 | 자료 없음 |
| Gazebo·ULog 실시간 연결 | 미실시. 이번은 합성 생성기 |
| FC·ToF 장치 clock 어댑터 | 후속 구현 |
| SLERP·Z pending·정지/이동 전환 | 함수·상태 연결 보류 |
| raw XY·H80·Q·위치 게이트 실행 | 보류 |
| ZIP ↔ 기존 Python 수식 완전 일치 | 미검증 |
| PX4·LoRa 외부 위치 전달 | 미연결 |
| 500ms 비행용 failed·복귀 hold | 운용 정책 연결 보류 |
| 정지·이동·비행 정확도 | 미검증 |

현재 합성 자료로 센서 정확도를 주장하지 않는다.
거리 잡음·높이 궤적은 생성기의 설정이다.
계산 단계의 검증 시점과 시험안은 아직 확정하지 않았다.
