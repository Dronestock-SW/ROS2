# 저장소 단계별 모듈화 확인 기록

2026-10-04 디렉토리 정리와 검증 결과다. 변경을 검토하거나 다음 작업을 이어받을 때 읽는다.

## 완료 범위

파일 130개를 역할별 실제 경로로 옮겼다.
계산 결과·설정값·기존 실행 명령은 유지했다.
[단계별 위치표](../reference/repository_layout.md)에서 수정 위치를 찾는다.

| 분류 | 이동 수 | 실제 경로 | 이전 경로 처리 |
|---|---:|---|---|
| 계산 구현 | 14 | `processing/timing/`, `geometry/`, `solvers/` | 직접 Python alias |
| 외부 연결 구현 | 18 | `integration/ros/`, `gazebo/`, `sitl/` | 직접 Python alias·기존 main 호출 |
| UWB 설정 | 20 | `config/anchors/`, `runtime/`, `pipeline/`, `replay/`, `gazebo/`, `sitl/` | 상대 심볼릭 링크 |
| UWB 테스트 | 29 | `test/processing/`, `experiments/`, `integration/` | 활성 문서의 시험 경로 갱신 |
| 문서·참고 PDF | 46 | `docs/architecture/` 12, `reference/` 15, `runbooks/` 19 | 상대 심볼릭 링크 |
| 인쇄 마커·PX4 기록 설정 | 3 | `drone_bringup/assets/markers/`, `data/raw/flight/20260927/` | 루트 이름의 상대 링크 |

기존 호환 경로를 포함한 Python alias는 55쌍이다.
새 alias가 다른 alias를 거치지 않도록 갱신했다.
모듈 객체·예외 클래스·monkeypatch 대상을 유지한다.
이전 경로의 파일이 남아 있는 이유는 호환성 때문이다.
실제 구현을 수정할 때는 새 색인을 사용한다.

수신·공통 계약, 작은 거리 게이트·설정 파일은 유지했다.
다른 ROS 패키지에는 역할별 README를 추가했다.
제조사 서브모듈과 자동 생성물은 직접 편집하지 않았다.
빌드 도구가 생성하는 파일만 정상 갱신됐다.

## 처리 흐름과 현재 상태

각 실행 경로의 기존 계산·출력 조건을 유지했다.

```text
HW RAW JSONL
    |
acquisition/                  A: 수신·검사
    |
processing/timing/            B: 시각·센서 선택
    |                         C: config/·settings·좌표계
    +--> geometry/height.py    D: Z 수식
    +--> ranges.py            E: 거리 교정·게이트
              |
         solvers/rawxy.py     E: 진단 XY
         solvers/h80.py       F: 정상 구간 수식
         solvers/qs10.py      G: 복귀 수식
              |
         호출자 판정·변환       H: 출력 조건
              |
   integration/ros/ 또는 sitl/
              |
        기록·재생·독립 평가
```

이 그림은 책임 분류다.
공용 파일 파이프라인은 계산·외부 출력이 닫혀 있다.
실시간 Processor, 독립 비교기, SITL 실행기를 구분한다.
Q_S10 복귀 상태기계나 위치 게이트를 새로 구현하지 않았다.
companion EKF·z 제어·비행 출력 활성화도 추가하지 않았다.
인계서와 현재 기준의 차이는 [차이표](../reference/repository_layout.md#인계서와-현재-기준의-차이)에 있다.

## 보존한 변경과 원본

작업 시작 시 변경·미추적 상태 항목 98개가 있었다.
이동은 Git HEAD 대신 당시 작업트리 내용을 기준으로 했다.
이전 상태와 907개 파일의 목록·해시를 먼저 보관했다.
로컬 사본은 `/tmp/ros2_modularization_snapshot_20261004/`다.
인계에 필요한 목록·해시는 아래 저장소 증빙에 남겼다.

| 확인 | 결과 |
|---|---|
| 기존 RAW·가공 결과·과거 보고서 | 593개 바이트·SHA-256 일치 |
| 위 항목 중 기존 `docs/report/` | 197개 보존 |
| 이동한 설정·마커·기록 파라미터 | 23개 원본 바이트 일치 |
| 기존 data catalog | 43개 등록값 그대로 유지 |
| 추가 등록 | `perfect_holdv2.params` 1개. 총 44개 |
| Git 상태 | 사용자 변경을 유지하며 정리. 커밋·푸시 미실시 |

설정의 수치·기본값·필터 순서는 변경하지 않았다.
`setup.py`는 중첩 설정과 옛 파일명을 모두 설치한다.
Gazebo의 `__file__` 기반 경로 3곳을 보정했다.
원본 해시 manifest는 실제 하위 구현까지 포함한다.
`gazebo_rig`는 소스 설정과 설치된 share를 모두 찾는다.

## 검증 결과

변경 전후에 같은 환경과 명령으로 검사했다.
ROS pytest 자동 플러그인은 양쪽에서 비활성화했다.
`launch_testing`이 미설치 선택 의존성의 skip을
전체 수집 중단으로 확대하는 환경 문제가 있었기 때문이다.

| 검사 | 변경 전 | 변경 후 |
|---|---|---|
| UWB pytest | 275 통과, 1개 모듈 skip | 285 통과, 1개 모듈 skip |
| demo pytest | 89 통과, 2개 모듈 skip | 89 통과, 2개 모듈 skip |
| bringup pytest | 9 통과, 2 실패, 1 skip | 동일 |
| platform pytest | `websockets` 미설치 수집 오류 | 동일 |
| 합계 통과 | 373 | 383. 새 실패 0 |
| 전체 colcon 빌드 | 이번 작업의 변경 전 빌드는 미실시 | 5개 패키지 성공 |
| 설치 경로 설정 로딩 | 미실시 | 새·이전 설정 40개 바이트 확인 |
| 설치 환경 import | 미실시 | alias 55쌍 객체 동일 |
| 설치된 CLI 도움말 | 미실시 | 파일용 기존 실행 명령 6개 성공 |
| launch 인수 표시 | 미실시 | `uwb.launch.py --show-args` 성공 |

새 시험 10개는 CLI·예외·monkeypatch·설정 탐색 회귀를 확인한다.
기존 시험의 수치·실패 판정 단언은 유지했다.
manifest 경로 단언은 실제 이동 경로로 바꿨다.

bringup 실패는 기존 lint 검사 2개다.
진단은 E128 3건, E501 1건, D213 8건이다.
변경 전후 진단 위치와 내용도 동일했다.
MAVLink 시험 모듈 3개는 `pymavlink` 미설치로 skip했다.
platform 시험에는 `websockets==13.1`이 필요하다.

## 같은 입력 재생

결정론적 산출물 20개가 바이트 단위로 동일했다.
위치뿐 아니라 상태·reason·null·입력 기록도 비교했다.

| 실행 | 입력·계산 범위 | 결과 |
|---|---|---|
| pipeline 합성 | 10초, seed 7, 1,202행 | input·diagnostics·summary 동일 |
| pipeline 재생 | 위 입력 그대로 | input·diagnostics·summary 동일 |
| 정지 A | 저장된 교정·평가 입력, 2,540개 계산 | 결과·좌표·설정·요약 동일 |
| H80 B | 동일 정지 입력, 2,538개 유효 | 결과·좌표·설정·요약 동일 |

실행 시간 파일과 코드 해시 manifest는 동일성 비교에서 분리했다.
이동 후 코드 파일 124개는 manifest 해시와 각각 대조했다.
새 시간·기하·솔버 구현이 해시 목록에 포함됨을 확인했다.
허용오차를 넓혀 수치 차이를 통과시킨 항목은 없다.

## 증빙과 재실행

전체 이동과 명령은 다음 파일에서 확인한다.

| 증빙 | 내용 |
|---|---|
| [이동표 CSV](evidence/repository_modularization_20261004/moves.csv) | 이전·새 경로, 역할, 단계, 참조 영향, 이유, 해시 |
| [이동표 JSON](evidence/repository_modularization_20261004/moves.json) | 같은 이동표의 기계 판독 형식 |
| [작업 전 파일 목록](evidence/repository_modularization_20261004/inventory_before.json) | 용도 분류·해시·이동 여부 |
| [작업 전 Git 상태](evidence/repository_modularization_20261004/git_status_before.txt) | 기존 사용자 변경·미추적 항목 |
| [원본 보존 확인](evidence/repository_modularization_20261004/preservation_checks.json) | 기존 자료·설정·catalog 대조 |
| [pytest·재생 결과](evidence/repository_modularization_20261004/validation.json) | 실제 명령·환경·결과·20개 파일 해시 |
| [빌드 기록](evidence/repository_modularization_20261004/build.json) | 명령·패키지·결과 |
| [설치 확인](evidence/repository_modularization_20261004/install_checks.json) | 작업 디렉토리 /tmp에서 설정·CLI·import 확인 |
| [최종 링크·경로 검사](evidence/repository_modularization_20261004/final_checks.json) | 활성 문서·호환 링크·직접 import 경로 |

실행 절차는 [UWB 패키지 안내](../../src/drone_uwb/README.md)와
[자료 재생 절차](../runbooks/uwb_data_runbook.md)에 있다.
검증 산출물 전체는 `/tmp/ros2_modularization_after/`에 있다.
새 재생을 실행할 때는 다른 출력 폴더를 사용한다.

## 남은 사항

구조 정리는 완료했고 기존 검증 제한은 유지된다.

- `pymavlink`, `websockets`가 있는 환경의 시험은 미실시다.
- 기존 bringup lint 정리는 이번 변경에 포함하지 않았다.
- 실제 UART·ROS 토픽 송수신·Gazebo·SITL·기체 비행은 미실시다.
- 실제 배포와 Windows/WSL 복사 검증은 미실시다.
- 마커 1의 실제 사용 여부는 확인하지 못했다.
  관련 마커 폴더에 원본을 보존하고 미확인 상태를 표시했다.
- 저장소 밖의 PDF·ZIP·실행 폴더는 참조만 했다.

호환 심볼릭 링크는 WSL 등 링크를 지원하는 체크아웃에서 사용한다.
기존 문서·설정 경로를 지키기 위한 저장소의 기존 방식을 유지했다.
