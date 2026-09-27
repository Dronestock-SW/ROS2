# 27일 비행 로그 수집·분류

원격 ULog를 수집하고 개발 자료로 등록한 기록이다.
Position·자세·거리 데이터를 선택할 때 읽는다.

## 확인 결과

27일 파일에는 Position 모드 구간이 있다.

후속 분석에서 두 구간의 추종 오차를 계산했다.
[위치 유지 분석](position_hold_20260927.md)을 함께 읽는다.
아래 미실시 표기는 최초 수집 당시의 범위다.

| 항목 | 확인값 |
|---|---|
| 원격 브랜치 | origin/feature_uwb |
| fetch·반영 | 30804d1 → cbfbb6b, fast-forward |
| 원본 | log_23_2026-9-27-13-34-50.ulg |
| 크기 | 2,839,356 bytes |
| 기록 길이 | 70.908608초 |
| vehicle_status | 146개: nav_state 1은 88개, 2는 58개 |
| vehicle_attitude | 1,417개 |
| distance_sensor | 71개 |
| vehicle_local_position | 709개 |
| EKF2_EV_CTRL | 0 |
| EKF2_OF_CTRL / EKF2_RNG_CTRL | 1 / 1 |
| EKF2_HGT_REF | 0 |
| UWB RAW 관련 필드 검색 | 일치 없음 |

nav_state 1은 ALTCTL, 2는 POSCTL로 해석했다.
근거는 [PX4 VehicleStatus 정의](https://docs.px4.io/main/en/msg_docs/VehicleStatus)다.
위 개수는 상태 메시지 수다. 비행 시간 비율은 아니다.
Position 모드 기록과 호버링 성능 검증을 구분한다.
외부 위치 융합 파라미터는 꺼져 있다.
UWB 활용 비행의 근거로 취급하지 않는다.
26일 로그와 27일 로그를 서로 다른 세션으로 유지한다.

## 저장 위치

원본과 추출물을 각각 별도 폴더에 둔다.

| 자료 | 위치 |
|---|---|
| 원본 | [27일 ULog](../../data/raw/flight/20260927/log_23_2026-9-27-13-34-50.ulg) |
| 센서·모드 요약 | [summary.json](../../data/processed/flight_20260927/summary.json) |
| 자세 | [vehicle_attitude.csv](../../data/processed/flight_20260927/vehicle_attitude.csv) |
| 거리 | [distance_sensor.csv](../../data/processed/flight_20260927/distance_sensor.csv) |
| 모드·시동 상태 | [vehicle_status.csv](../../data/processed/flight_20260927/vehicle_status.csv) |
| 해시·추출 코드 근거 | [manifest.json](../../data/processed/flight_20260927/manifest.json) |
| 전체 활용 파일 | [catalog.json](../../data/catalog.json) |

원본 SHA-256은 다음과 같다.
`896a3f021f0b2eb9fd943e7960e2add8dfff67bcfe15541c442fac175146690f`
저장소 루트의 이전 파일명은 상대 링크로 유지한다.
기존 감사 도구의 `audit_ulog`로 센서를 추출했다.
pyulog는 `/home/pgyxn/.cache/uwb-audit/pyulog`를 사용했다.
상태 CSV는 timestamp·nav_state·arming_state를 추출했다.
시각 단위는 PX4 부팅 기준 us다.
자세는 w, x, y, z 순서이며 거리는 m다.
장치 모델·장착·UWB와의 시계 대응은 미확인이다.

## 검증·남은 작업

원격 원본 바이트와 보관본의 일치를 검사한다.

| 확인 | 결과 |
|---|---|
| 원격 반영 | fast-forward 완료, 파일 충돌 없음 |
| 원본 | 이동 전후 SHA-256 일치 |
| 추출 파일 | manifest에 파일별 SHA-256 기록 |
| 활용 목록 | 기존 42개에 새 원본을 더해 43개 |
| 기능 검사 | 커밋 대상 스냅샷 pytest 70개 통과 |
| 빌드 | 격리 스냅샷 drone_uwb symlink 빌드·ROS import 통과 |
| 문서·형식 | 커밋 대상 문서 링크·git diff --cached --check 통과 |
| 실제 수신·새 비행 | 미실시 |
| Position 성능 분석 | 미실시 |
| UWB 동시 결합 | 미실시 |

H80 B의 진행 중 변경은 이번 커밋에서 분리한다.
해당 코드·설정·테스트는 로컬 작업에 보존한다.
이번 반영은 완료된 A·폴더 정리와 27일 자료 등록이다.

자료 바이트 보존을 위해 `.gitattributes`를 추가했다.
`data/** -text -whitespace`를 적용한다.
원본 CSV·UART의 줄바꿈을 Git이 변환하지 않게 한다.
진행 중인 공유 작업과 분리한 index 스냅샷을 검증했다.
원격 push는 일반 fast-forward 방식으로 수행한다.
