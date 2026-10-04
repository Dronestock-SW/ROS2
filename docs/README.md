# 문서 색인

작업에 필요한 기준·절차·확인 기록을 찾는 색인이다.
작업을 시작하거나 다른 채팅에 인계할 때 읽는다.

## 최근 확인 기록

현재 설정은 날짜와 계정을 함께 확인한다.

| 문서 | 확인 범위 |
|---|---|
| [Windows UWB 재시작·첫 실시간 기록](report/uwb_windows_restart_20261002.md) | 사용자 승인 재기동, RAW 812개·B 정지 진단·ULog 스냅샷. 실제 융합·동적 이동 미검증 |
| [UWB 새 Goal 프롬프트](uwb_goal_prompt_20261002.txt) | 기존 실행 보존, 실제 EKF2 융합, 독립 다점·동적 7cm 및 목표 이동의 완료 기준 |
| [Windows UWB 재개 점검](report/uwb_windows_resume_20261002_140858.md) | 실제 Windows·WSL 프로세스·메모리 고갈·누적본 54개 파일 반영. 새 센서·융합·비행 미실시 |
| [Gazebo 이상·복구 구간 평가](report/uwb_gazebo_fault_evaluation_20261002.md) | 구간별 CSV·복구 시간·누적본 68개 시험. v2는 Windows 인계·설치 문서 개정 |
| [Gazebo 이상 시험 갱신본](report/uwb_gazebo_fault_bundle_20261002.md) | 누적 ZIP·SHA-256·별도 폴더의 54개 시험. WSL 비행 미검증 |
| [Gazebo UWB 시행 기록](report/uwb_navigation_iteration_20260928.md) | 정지 RAW와 합성 이동·반전 비교, B 시간창 조정, M03 보완 기각 및 7cm 판정 범위 |
| [Gazebo UWB 시행 누적표](report/uwb_navigation_iteration_index_20260928.md) | 시행 0002~0016의 변경·오차·가용률·채택 여부 |
| [기능별 커밋 전 통합 확인](report/uwb_commit_review_20260927.md) | 8개 커밋 구분·142개 테스트·전체 빌드·남은 검증 |
| [구매 장비·Gazebo 대조](report/gazebo_equipment_audit_20260927.md) | 장비 문서와 실제 SDF 대조. 구매 엑셀 미발견, 제품별 미반영 항목·실행 확인 한계 |
| [UWB·장비 Gazebo 구성](report/uwb_gazebo_equipment_20260927.md) | 사용자 WSL 시작과 IMU·기압·하방 거리 표본 확인. 가상 UWB 기록·융합은 후속 검증 |
| [Gazebo UWB 비교 준비](report/uwb_gazebo_shadow_20260927.md) | WLS·경로 기록·여섯 조건 재생·WSL 버전 확인. 실제 Gazebo 연동은 대기 |
| [perfect_holdv2 설정 보관](report/perfect_holdv2_20260927.md) | 1,095개 파라미터. 27일 ULog와 공통 990개 중 차이 11개 |
| [9월 27일 Position 위치 유지](report/position_hold_20260927.md) | 두 구간 29.28초. 추종 오차·센서 융합·종료 사건 |
| [C/D 시뮬레이터 실행](report/uwb_cd_simulation_20260927.md) | 여섯 합성 상황·1,920주기·경로/오차/출력률 그림. Gazebo 미실시 |
| [C/D 구현·A/B 비교](report/uwb_subsets_cd_20260927.md) | 균등 조합·strict 교점 구현. 정지·합성·117개 테스트·재생 검증 |
| [27일 ULog의 B/C 입력 준비](report/uwb_flight_inputs_20260927.md) | 거리·자세 원본 추출·C 준비 프로파일·동시 UWB 없음 확인 |
| [B H80 구현·A 비교](report/uwb_h80_b_20260927.md) | 실측 정지 A/B 비교·동적 합성·86개 테스트·재생 검증 |
| [C 세 앵커 조합 명세](specs/uwb/06_anchor_triplets.md) | 네 후보·균등 결합 구현 계약과 후속 시험 범위 |
| [27일 비행 로그 등록](report/flight_log_20260927.md) | 원격 ULog 수집, Position 구간·센서 추출·원본 해시 |
| [UWB 자료 정리 확인](report/uwb_data_reorganization_20260927.md) | 코드·데이터 분리, 원본 해시, 회귀·빌드·재생 검증 |
| [현재 구현 현황·수동 호버링 보고](report/implementation_status_20260927.md) | Position 호버링 성공 보고, UWB 구현·검증·비행 연결 구분 |
| [실측 1분 로그 A 계산](report/uwb_baseline_a_static_20260926.md) | 수기 기준점으로 정지 기록 재계산. 편향 후보 산출·별도 평가 |
| [A 기준선 계산](report/uwb_baseline_a_20260926.md) | 파일 계산기·합성 371개 위치·재현성·9월 26일 실측 연결 조건 |
| [기능별 커밋 전 확인](report/feature_uwb_commit_check_20260926.md) | 커밋 범위, pytest 80개, 합성 재생·H80 검사 재실행 |
| [H80 입력·로그·샘플 확인](report/uwb_h80_readiness_20260926.md) | GitHub 병합, 기존 UWB·새 비행 로그, A/H80 합성 비교 |
| [UWB 비교 시험 명세 작성](report/uwb_specs_20260926.md) | 17개 기능의 입출력·아키텍처·비교 절차. 구현·시험 미실시 |
| [UWB 대화 발췌·확인](report/uwb_chat_review_20260921.md) | 대화·ZIP 대조와 GNSS 문헌 조사. 9월 21일 보류 기록과 후속 상태 |
| [UWB 파이프라인 구현](report/uwb_pipeline_20260921.md) | 합성 입력·편향·거리 게이트·로그 연결. 계산·전달 게이트 닫힘 |
| [기본 설정 확인](report/setup_status_20260921.md) | 2026-09-21 `pgyxn` 계정과 저장소 |
| [UWB 참고 자료와 코드 상태](uwb_h80_qs10_reference.md) | 9월 20일 ZIP과 필터 기본값 |
| [UWB 필드 매핑](uwb_preimu_field_mapping.md) | 구현 전 비교표와 9월 21일 보충 |
| [Gazebo·SITL 기본 시험](report/gazebo_sitl_20260921.md) | 9월 21일 연결·가상 이착륙 확인. UWB 정비 보류 |
| [Gazebo 출력 발췌](report/evidence/gazebo_sitl_20260921_user_excerpt.md) | 사용자 제공 출력과 수집 한계 |

## 기준과 절차

설계 판단은 로드맵을 먼저 따른다.

| 문서 | 읽는 때 |
|---|---|
| [Windows 로컬 작업 인계](uwb_windows_handoff.md) | Windows 터미널로 WSL 시험을 이어받고 Jetson 자료를 SSH로 읽을 때 |
| [Gazebo UWB 목표 이동 프롬프트](uwb_gazebo_navigation_prompt.md) | A/B/C/D 선정·보조기능 순차 보완·7cm 목표·시행별 기록과 목표 이동 검증을 이어갈 때 |
| [Gazebo UWB 구현 구조](uwb_gazebo_architecture.md) | 설정표·모델 생성·센서·PX4·가상 UWB의 역할을 이해할 때 |
| [Gazebo UWB의 PX4 입력 계약](uwb_gazebo_sitl_odometry.md) | SITL 외부 위치의 좌표·장착·시각·MAVLink 조건을 검토할 때 |
| [Gazebo UWB의 SITL 실행](uwb_gazebo_sitl_observer.md) | 실시간 계산→관측 후보→시각 응답·송신을 연결할 때. 기본 Shadow |
| [Gazebo 수평 목표 연결](uwb_gazebo_target_adapter.md) | 단일 MAVLink 연결의 관측·목표 송신, 도착·중단 처리와 미검증 범위를 확인할 때 |
| [Gazebo 목표 이동 실행 절차](uwb_gazebo_navigation_runbook.md) | WSL에 실행기를 반영하고 관측·이동 시험을 준비할 때 |
| [Gazebo 비행 기록의 독립 평가](uwb_gazebo_flight_evaluation.md) | UWB·PX4 측위와 목표 추종·실제 도착을 구분해 평가할 때 |
| [Gazebo UWB 이상 주입](uwb_gazebo_fault_trials.md) | 예약한 거리 편향·단절을 별도 시행으로 만들고 원본·발행 상태를 확인할 때 |
| [PX4 SITL MAVLink 읽기 점검](uwb_sitl_link_probe.md) | WSL 온보드 UDP와 외부 관측 파라미터를 변경 없이 읽을 때 |
| [PX4 SITL 부트 시계 조회](uwb_sitl_clock_probe.md) | WSL 시각과 PX4 부트 시각의 왕복 조회를 기록할 때 |
| [Gazebo UWB 실시간 Shadow 기록](uwb_gazebo_live_shadow.md) | Gazebo RAW·ToF·IMU 토픽을 함께 받아 A/B/C/D를 기록할 때 |
| [Gazebo 앵커·장비 적용 절차](uwb_gazebo_equipment_runbook.md) | 새 센서 기체와 앵커 월드 생성·설치·가상 RAW 수신 |
| [Gazebo UWB 경로 비교 절차](uwb_gazebo_shadow_runbook.md) | ROS 없이 경로 수집·잡음/가중치/단절 조건 비교 |
| [Position 로그 분석 절차](position_hold_audit_runbook.md) | 새 ULog의 위치 유지 성능 비교 |
| [UWB 코드·자료 분류](uwb_data_layout.md) | 수신·보정 책임과 활용 원본·결과 폴더 |
| [UWB 모듈 입출력 사전](uwb_module_api.md) | 파라미터·출력·단위·실패·기본값 |
| [UWB 자료 실행 절차](uwb_data_runbook.md) | 새 수신 기록·정지 계산·호환 경로 |
| [UWB·MAVROS 파라미터 API](uwb_mavros_parameter_api.md) | 위치 전달·설정 조회 경로와 9월 26일 ULog 설정 확인 |
| [UWB 기능별 프로젝트 명세](specs/uwb/README.md) | 아이디어별 구현 준비·동일 자료 비교·모델 선정 |
| [UWB 파이프라인 기준](uwb_pipeline_design.md) | 모듈 경계·입출력·닫힌 게이트 확인 |
| [UWB 파이프라인 실행](uwb_pipeline_runbook.md) | 합성 자료 생성·동일 입력 재생 |
| [로드맵](roadmap.md) | 기술 방향·Phase 판단 |
| [고도 처리 원칙](altitude_policy.md) | 고도 관측·제어 연결 |
| [용어 사전](glossary.md) | 문서·주석 작성 |
| [장비 문서](equipment_inventory.md) | 모델명·구성 확인 |
| [companion 설치](companion_setup.md) | 재설치·빌드 준비 |
| [Gazebo WSL 재실행](gazebo_wsl_runbook.md) | Windows·Ubuntu·PX4 창별 명령과 QGroundControl 연결 |
| [UWB 지상 시험](uwb_bench_procedure.md) | 실물 수신 시험 |
| [공용 데모 절차](demo_procedure.md) | 장비 도착 전 기능 시험 |

## 과거 기록

과거 결과는 해당 날짜의 근거로 사용한다.

| 문서 | 기준 시점·범위 |
|---|---|
| [SW 구현 현황](report/01_sw_status.md) | 최신 결과 안내 + 2026-08-16 상세 기록 |
| [SW 증빙 목록](report/04_evidence_index.md) | 후속 증빙 색인 + 2026-08-16 수집분 |
| [Gazebo 준비 논의](report/gazebo_preparation_20260920.md) | 2026-09-20 설치 전 검토 |
| [UWB 실기 결과](report/uwb_integration_20260906.md) | 2026-09-06 시험 |
| [공용 데모 검증](report/demo_cycle_20260913.md) | 2026-09-13 시험 |

현재 코드의 유무는 최근 기록과 함께 확인한다.
8월의 미구현 표시는 이후 구현을 반영하지 않는다.
