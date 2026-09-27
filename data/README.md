# 개발 활용 데이터

UWB 수신·보정과 26일 비행 분석에 쓸 자료 색인이다.
새 계산을 시작하기 전에 용도와 원본 해시를 확인한다.

| 경로 | 내용 |
|---|---|
| [raw/uwb](raw/uwb/) | UWB 정지 교정·평가 기록과 이전 RAW |
| [raw/flight/20260926](raw/flight/20260926/) | 사용자 지정 26일 비행 자료 후보 |
| [raw/flight/20260927](raw/flight/20260927/) | 27일 Position 모드 포함 비행 기록 |
| [processed](processed/) | 기존 계산·추출·그림 결과 |
| [Gazebo 앵커·장비 구성](processed/uwb/gazebo_equipment_20260927/) | 장착 배치·센서 SDF·실제 로그 추력 검토. 실제 Gazebo 실행 자료 아님 |
| [Gazebo 연결용 합성 입력 검증](processed/uwb/gazebo_shadow_fixture_20260927/) | 합성 위치 320개로 여섯 조건 비교. 실제 Gazebo 기록 아님 |
| [B H80 결과](processed/uwb/h80_b_20260927/) | A/B 실측 비교·합성 시험·검증·그림 |
| [C/D 비교 결과](processed/uwb/subsets_cd_20260927/) | A/B/C/D 정지 비교·12슬롯 진단·합성·재생·검증 |
| [C/D 시뮬레이션](processed/uwb/subsets_simulation_20260927/) | 여섯 합성 상황·경로·오차·출력률·실행 조건 |
| [27일 센서 입력 준비](processed/flight_inputs_20260927/) | PX4 원본 시계·좌표 유지. 안테나 높이·UWB 위치 계산 전 단계 |
| [catalog.json](catalog.json) | 파일별 역할·SHA-256·이전 경로 |
| [module_paths.json](module_paths.json) | 코드 모듈 이전 경로 |

원본은 수정하지 않고 새 결과 폴더로 계산한다.
1차 UWB는 편향 산출, 2차는 별도 개발 평가용이다.
26일 ULog와 두 UWB 기록은 동시 측정이 아니다.
26일 파일과 Position 성공 시험의 연결은 미확인이다.

[분류 기준](../docs/uwb_data_layout.md),
[모듈 계약](../docs/uwb_module_api.md),
[실행 절차](../docs/uwb_data_runbook.md)를 따른다.

27일 로그에는 Position 모드 표본이 있다.
[27일 로그 등록·추출 기록](../docs/report/flight_log_20260927.md)을 따른다.
