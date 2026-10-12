# FINAL 코드 검증 근거
2026-10-12 브랜치 정리의 검증 영수증이다.
시험 결과와 실제 비행의 한계를 확인할 때 읽는다.

Python 전체는 통과했다. CTest 반복 안정성은 미확인이다.
정리한 소스를 Jetson의 별도 폴더에서 검증했다.
운영 소스·서비스·FC 설정은 변경하지 않았다.

| 항목 | 근거 | 결과 |
|---|---|---|
| 원격 11개·보존 로컬 5개 | branch_inventory.json | 조상·미포함 수정 대조 |
| 변경 소스 42개 | verification.json의 source_files | Jetson과 PC 해시 일치 |
| 전체 Python | final-pytest.txt | 1,105 통과·0 실패·0 skip |
| ROS Humble 빌드 | colcon-build.txt | 4개 패키지 성공 |
| C++ 설정·빌드 | native-configure.txt·native-build.txt | 성공 |
| 마지막 CTest 묶음 | native-ctest-ram.txt | 31 통과·1 실패 |
| 실패 항목 재실행 | native-hover-recheck.txt | native_hover_ros 1/1 통과 |
| 이전 CTest 묶음 | native-ctest-final.txt | 30 통과·2 실패 |
| 웹 단독 재확인 | web-service-diagnostic.txt | 1/1 통과 |
| 새 실제 비행·SITL | 미실시 | 실비행 성공 미검증 |

## 검증 환경

Jetson aarch64·Python 3.10.12·ROS Humble을 사용했다.
소스는 아래의 전용 폴더다.
`~/ROS2-review-20261008-codex/.review/final-cleanup-20261012/source`.
drone_uwb·drone_demo·drone_mission·drone_platform_link를 빌드했다.
이 체크아웃의 install을 사용했다.
타 체크아웃의 임시 Python 소스를 사용하지 않았다.
pymavlink 2.4.49·websockets 15.0.1은 별도 폴더에 설치했다.
Python ROS 시험은 domain 99·localhost만 사용했다.
C++ ROS 시험은 173·181·182·183을 사용했다.
합성 비행·센서·서비스를 실제 운영 domain과 분리했다.

Python 수집 명령은 다음과 같다.
PYTHONPATH에는 이 소스와 격리 의존성이 포함된다.

```bash
python3 -m pytest -q --import-mode=importlib \
  src/drone_uwb/test src/drone_mission/test \
  src/drone_demo/test src/drone_platform_link/test src/sangwon_AI/tests
```

Windows의 CRLF는 Git LF 바이트로 정규화해 대조했다.
승인된 합성 JSON의 해시 목록은 수정하지 않았다.
원본 data·파라미터·펌웨어도 변경하지 않았다.
verification.json은 시험 출력의 SHA-256도 보존한다.
출력 해시도 Git LF 바이트로 정규화해 대조한다.
시험 소스 기준은 69c5510·733f448과 진단 출력 수정이다.
이후 문서·영수증 저장은 시험 코드 동작을 바꾸지 않는다.

## 남은 간헐 실패

마지막 전체 묶음은 native_hover_ros에서 실패했다.
global_loss 시나리오의 합성 입력 나이가 약 0.657초였다.
TELEMETRY_NOT_FRESH가 시작을 막았다. 명령 호출은 없었다.
원래 신선도 한계값을 유지한 단독 재실행은 통과했다.
이는 기존 C++ 2초 호버 시험이다.
FINAL LocalXY의 10초 호버는 Python 시험에 포함된다.

이전 묶음은 scan_sensor_transport·웹 planner가 실패했다.
합성 입력 만료·IPC 조회 timeout이 기록됐다.
마지막 묶음에서는 이 두 항목이 통과했다.
임시 합성 상태 파일을 /dev/shm에 둔 조건도 기록했다.
운영 로그·원장·DB의 저장 위치는 바꾸지 않았다.
원인이 저장장치나 스케줄링이라고 확정하지 않는다.
32개 개별 통과를 한 묶음 32/32 성공으로 표시하지 않는다.

브랜치 통합과 실행 8의 실제 밀림 원인 규명은 다르다.
이 기록은 실제 비행 준비 완료의 근거가 아니다.
