# FINAL 연구 시작점

2026-10-13 후속 기준은 `이번에-상원-한거-통합-브랜치`다.
[통합 범위·전달 관측 0.7초](docs/report/sangwon_integration_20261013.md)를 먼저 읽는다.
[새 세션 절차](docs/runbooks/sangwon_integration.md)를 따른다.
아래 12일 기록은 당시 근거로 보존한다.

2026-10-11 현장 시험과 12일 코드 정리를 이어받는 문서다.
다음 연구 세션에서 이 파일부터 읽는다.

**당면 목표는 XY 이동 없는 수직 이륙·제자리 호버·착륙이다.**
테스트 웹도 [C++ AI 통합 경로](docs/report/cpp_field_mission_integration_20261012.md)를 사용한다.
후속 구현 브랜치는 `FINAL-TEST1-SANGWON`이다.
[C++ 실행 단계·20Hz 송신 경로](docs/report/cpp_executor_ground_20261012.md)를 읽는다.
[C++ 실행기 격리 시험](docs/runbooks/cpp_executor_ground.md)을 먼저 수행한다.
[상단 LiDAR 연결과 활성화 조건](docs/report/lidar_gap_support_20261012.md)을 읽는다.
[지상 시각 결함·수정](docs/report/lidar_ground_audit_20261012.md)을 읽는다.
[실측과 활성화 절차](docs/runbooks/lidar_ground_qualification.md)를 따른다.
[실행 8 자세 요구·추종](docs/report/run8_offboard_attitude_20261012.md)을 확인한다.
실제 기체 밀림과 위치 추정 오류의 선후관계를 먼저 밝힌다.
실행 8에서 이륙 중 밀림이 남았다.
정리본의 현장 비행 검증은 아직 수행하지 않았다.
START 활성화와 단위 테스트 통과는 비행 성공을 뜻하지 않는다.
12일 Python 1,105개·ROS 4개 패키지 빌드는 통과했다.
C++ 간헐 입력 만료·IPC 지연의 반복 안정성은 미확인이다.

1. [모든 브랜치 대조·FINAL 코드 정리](docs/report/final_cleanup_20261012.md)
2. [UWB 공백·현재 관측 경로](docs/report/uwb_gap_processing_20261012.md)
3. [실행 8 ULog·영상 분석](docs/report/hover_run8_analysis_20261012.md)
4. [실행 8 인계와 연구 과제](docs/report/hover_research_handoff_20261011.md)
5. [FINAL 재부팅·실행 절차](docs/runbooks/final_resume.md)
6. [이전 이륙 분석·수정 이력](docs/report/local_xy_takeoff_20261011.md)

12일에 원본과 같은 ULog를 FC에서 재회수했다.
고정 XY와 최종 공백 전 추정 변화가 확인됐다.
영상에도 실제 측면 이동이 보인다.
동적 관측 오류와 실제 이동량의 분리는 아직 남았다.
실행 8 분석 당시 속도·임계값·추력은 유지했다.
후속 정리에서는 guard 호출·기록·시작 계약을 수정했다.
새 보정 예제는 송신기 점프 보호를 켠다.
B4 높이 연구 검사는 명시적 선택으로 분리했다.
현장 코드·FC 설정에는 이 정리본을 배포하지 않았다.

FINAL에는 실행기·웹·브리지·설정 예제·복구 도구·테스트가 있다.
미션 높이는 1.7m, 상승 목표 제한은 0.5m/s다.
XY 목표 이동 제한은 0.25m/s다.
현재 연구 미션의 XY 경로 길이는 0m다.
지상에서 정한 출발 XY를 이륙·호버 동안 유지한다.
현재 위치·속도 관측은 이륙 전부터 계속 갱신한다.
목표 높이 진입 뒤 XY 안정화와 연속 10초 호버를 확인한다.
밀려간 위치로 출발점을 재설정하지 않는다.

실기 비밀정보와 로그 원본은 Git 외부에 보존한다.
복구 경로와 해시는 위 문서에 있다.
새 부팅에서는 기존 방위 보정 파일을 재사용하지 않는다.
C++가 XY·호버·복귀 판단을 담당한다.
`--cpp-executor-binary` 경로는 실행 단계·FC 송신도 C++다.
이 경로의 Python은 센서 라이브러리와 웹 계약을 유지한다.
해당 인자가 없는 기존 경로는 Python 실행기다.
전체 실물 스캔·회피·미션 검증은 아직 남아 있다.
2026-10-11 마지막 확인은 착륙·DISARM·RC 인계 상태였다.
다음 접속에서 실제 상태를 새로 확인한다.
