# 문서 색인

작업에 필요한 기준·절차·확인 기록을 찾는 색인이다.
작업을 시작하거나 다른 채팅에 인계할 때 읽는다.

## 최근 확인 기록

현재 설정은 날짜와 계정을 함께 확인한다.

| 문서 | 확인 범위 |
|---|---|
| [Gazebo UWB 비교 준비](report/uwb_gazebo_shadow_20260927.md) | WLS·경로 기록·여섯 조건 재생·WSL 버전 확인. 실제 Gazebo 연동은 대기 |
| [Gazebo UWB 경로 비교 절차](uwb_gazebo_shadow_runbook.md) | ROS 없이 경로 수집·잡음/가중치/단절 조건 비교 |
| [27일 ULog의 B/C 입력 준비](report/uwb_flight_inputs_20260927.md) | 거리·자세 원본 추출·C 준비 프로파일·동시 UWB 없음 확인 |
| [C/D 시뮬레이터 실행](report/uwb_cd_simulation_20260927.md) | 여섯 합성 상황·1,920주기·경로/오차/출력률 그림. Gazebo 미실시 |
| [C/D 구현·A/B 비교](report/uwb_subsets_cd_20260927.md) | 균등 조합·strict 교점 구현. 정지·합성·117개 테스트·재생 검증 |
| [C 세 앵커 조합 명세](specs/uwb/06_anchor_triplets.md) | 네 후보·균등 결합 구현 계약과 후속 시험 범위 |
| [B H80 구현·A 비교](report/uwb_h80_b_20260927.md) | 실측 정지 A/B 비교·동적 합성·86개 테스트·재생 검증 |
| [perfect_holdv2 설정 보관](report/perfect_holdv2_20260927.md) | 1,095개 파라미터. 27일 ULog와 공통 990개 중 차이 11개 |
| [9월 27일 Position 위치 유지](report/position_hold_20260927.md) | 두 구간 29.28초. 추종 오차·센서 융합·종료 사건 |
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
