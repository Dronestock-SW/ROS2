# UWB 명세의 근거와 구현 상태

명세의 출처와 확인 범위를 보존하는 기록이다. 자료 주장·소스 사실·새 시험 설계를 구분할 때 읽는다.

## 근거 구분

명세의 제안 인터페이스는 새 설계다.
논문 또는 ZIP에 그대로 들어 있는 것으로 표시하지 않는다.
2026-09-26 작성 시 기존 소스와 관련 문서를 대조했다.

| 근거 | 확인한 내용 | 한계 |
|---|---|---|
| `/home/pgyxn/uwb_h80_qs10_reference_source_v1.zip` | README·C++ 거리 게이트·H80·Q·위치 판정 | 바이너리 실행·실측 재현 미실시 |
| 기존 `preimu/pipeline.py` | 입력·시간·편향·거리 이력·센서 버퍼 연결 | 좌표 계산·외부 출력은 닫힘 |
| 기존 `height.py`, `rawxy.py`, `h80.py`, `qs10.py`, `transform.py` | 독립 수식 함수와 인수 | 새 모델 명세와 동일 구현 아님 |
| [2026-09-21 시험 기록](../../report/uwb_pipeline_20260921.md) | 당시 파이프라인 테스트·빌드 결과 | 새 모델 시험의 통과 증거가 아님 |
| [대화·문헌 조사 기록](../../report/uwb_chat_review_20260921.md) | 앵커 조합·가중치·LiDAR·GNSS 아이디어 | 당시에는 미채택·보류 |

## 논문·공식 설명

읽은 범위에 맞춰 설계 근거를 제한한다.

| 자료 | 적용 범위 | 확인 범위 |
|---|---|---|
| [Li 등, 2017, Sensors 17(4), 795](https://www.mdpi.com/1424-8220/17/4/795) | M07의 세 앵커 조합·교점 선택·평균 | 이전 조사에서 PDF 3.4절 식 (19)·(20), 실험 구성 확인 |
| [陈静 등, 2023, 无线电工程 53(3), 669–677](https://wxdg.cbpt.cnki.net/portal/journal/portal/client/paper/476eac747bbba892fc7fabbbfce0d7a5) | M06·M08·M13의 조합/필터 아이디어 | 초록. 상세 가중치·UKF 수식 미확인 |
| [Zabalegui 등, 2021, Measurement 179, 109350](https://www.sciencedirect.com/science/article/pii/S0263224121003456) | M11의 GNSS 기법 UWB 적용 근거 | 출판사 초록·공개 본문 발췌 |
| [Zabalegui 등, 2023, Remote Sensing 15(1), 99](https://www.mdpi.com/2072-4292/15/1/99) | M11·M16의 관측 이상 검사·센서 결합 | 이전 조사에서 PDF 방법·시험·결과 절 확인 |
| [ESA 가중 최소제곱법](https://gssc.esa.int/navipedia/index.php/Weighted_Least_Square_Solution_%28WLS%29) | M09의 관측 가중 추정 원리 | 공식 설명 본문 |
| [ESA 위치 오차](https://gssc.esa.int/navipedia/index.php/Positioning_Error) | M10의 배치와 오차 구분 | 공식 설명 본문 |
| [ESA RAIM](https://gssc.esa.int/navipedia/index.php/RAIM_Fundamentals) | M11의 잔차 검출·제외 원리 | 공식 설명 본문 |
| [ESA 칼만 필터](https://gssc.esa.int/navipedia/index.php/Kalman_Filter) | M13의 예측·관측 결합 원리 | 공식 설명 본문 |
| [Julier·Uhlmann, Using covariance intersection for SLAM, 2007](https://www.sciencedirect.com/science/article/pii/S0921889006001436) | M16의 미상 상관관계 처리 참고 | 출판사 공개 설명. UWB 실험 논문이 아님 |

Li 논문은 일곱 앵커 실험이다.
네 묶음·12개 중간 후보는 그 일반식을 네 앵커에 적용한 설명이다.
2023년 GNSS 융합 논문은 UWB 앵커 여덟 개를 사용했다.
현재 네 앵커 환경의 정확도 보장으로 옮기지 않는다.

## 코드에서 확인한 차이

원본 재현과 새 시험 variant를 나눈다.

| 항목 | 확인한 차이 | 명세 처리 |
|---|---|---|
| H80 앵커 수 | ZIP은 네 앵커 각각 최소 3표본. Python은 세 앵커도 허용 | 네 앵커 reference를 명시 |
| H80 반복·잔차 | C++·Python의 반복과 최종 잔차 산출이 다름 | native 잔차와 공통 기하 잔차 분리 |
| 시간 계수 | 정규화 시간의 이동량과 m/s 속도는 다름 | window 길이로 환산 |
| Q 높이 인터페이스 | 기존 Python 인수는 한 값. 새 명세는 표본별 자료 | 별도 어댑터·variant 필요 |
| Q 규제·수렴 | 기존 두 구현의 규제·수렴 확인을 더 대조해야 함 | 실패·상한·수렴을 개별 출력 |
| source gap | 현재 Python 파이프라인은 거리 창만 초기화 | 게이트 초기화 정책도 variant에 기록 |
| 출력 | 기존 진단은 좌표 null·valid=false | 새 ModelResult는 별도 파일 시험 형식 |

## 현재 개발·자료 상태

현재는 명세와 제한된 입력·함수 확인 상태다.

| 항목 | 2026-09-26 상태 |
|---|---|
| 개별 기능 명세 | 17개 작성 |
| 공통 계약·비교 절차 | 작성 |
| 새 실험 실행기·모델 구현 | 미실시 |
| 입력 감사 도구 | 기존 로그 구조·ULog 필드·A/H80 합성 예시 검사 도구 추가 |
| ToF 기반 Z 자료 | 사용자 추후 제공 예정 |
| XY 독립 기준 위치 | 비교 자료별 확인 필요 |
| RF/CIR·LiDAR 앵커 대응·GNSS 자료 | 확보·지원 여부 미확인 |
| 기능 시험·성능 비교 | 기존 A/H80 합성 예시 수행. 실측 성능 비교 미실시 |
| 실운용 모델·계수 선정 | 미선정 |
| 외부 출력 | 닫힘 유지 |

문서 링크·형식 확인은 기능 시험과 분리해 보고한다.
[후속 확인 기록](../../report/uwb_h80_readiness_20260926.md)에 실행 범위와 결과를 보존했다.
