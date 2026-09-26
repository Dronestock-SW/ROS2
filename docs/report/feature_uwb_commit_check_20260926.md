# feature_uwb 커밋 전 확인 기록

2026-09-26 누적 변경의 커밋 범위와 검증 기록이다.
이번 변경을 이어받거나 원격 반영 범위를 확인할 때 읽는다.

## 변경 범위

기존 변경을 아래 여섯 묶음으로 커밋한다.

| 구분 | 내용 |
|---|---|
| 환경·작업 규칙 | 개인 계정 설정 기록, 기능 완료 시 문서 갱신 규칙 |
| Gazebo·SITL | WSL 실행 절차, 사용자 제공 시험 결과와 근거 |
| UWB 파일 파이프라인 | 합성·재생, 거리 보정·게이트, 센서 버퍼와 테스트 |
| UWB 비교 명세 | 17개 기능 명세, 단계별 선택안, 참고자료·대화 기록 |
| H80 입력 검사 | RAW 구조·ULog 검사 도구, 합성 비교와 추출 근거 |
| 공통 문서 | README·색인·로드맵·용어·현황과 이번 검증 기록 |

대상 저장소는 `/home/pgyxn/github/ROS2`다.
원격은 `https://github.com/Dronestock-SW/ROS2.git`이다.
사용자 요청에 따라 `feature_uwb`에 반영한다.
fetch 후 로컬은 12 ahead / 0 behind였다.
기존 12개 커밋도 이번 push 범위에 포함한다.
이 문서는 push 전 확인 기록이다. 전송 성공 여부는 Git 원격으로 확인한다.

## 적용값과 확인 결과

계산·외부 전달 설정은 모두 false로 유지했다.
편향 네 값은 기존 참고값이며 실측 교정 완료로 표시하지 않는다.

| 검증 | 이번 결과 |
|---|---|
| UWB·데모 pytest | 80개 통과, 3.38초 |
| 합성·재생 실행 | 각각 1,202개 입력, 진단 파일 바이트 동일 |
| H80 검사 도구 | 기존 정지 RAW 1개와 9월 26일 ULog로 실행 성공 |
| diff 공백 검사 | CSV의 CRLF를 허용하는 `cr-at-eol` 옵션으로 통과 |
| 전체 ROS 빌드·platform 재시험 | 미실시 |
| Gazebo·실물 비행·정확도 시험 | 미실시 |

첫 pytest는 PYTHONPATH에서 ROS 경로가 빠져 수집에 실패했다.
Humble을 소싱하고 기존 PYTHONPATH를 보존해 재실행했다.
통과 명령은 다음과 같다.

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH="src/drone_uwb:src/drone_demo:${PYTHONPATH}" \
  python3 -m pytest -q src/drone_uwb/test src/drone_demo/test
```

이번 임시 실행 산출물은 `/tmp/uwb-precommit-lrkk2b1m/`에 있다.
임시 경로이므로 영구 증빙으로 취급하지 않는다.
과거 검증 기록은 당시 결과를 보존했다.
CSV는 Python csv 기본 출력인 CRLF를 보존했다.
기본 공백 검사는 CR을 경고하여 CRLF 허용 옵션으로 재검사했다.

## 남은 작업

동일 세션의 RAW·Z·자세와 독립 기준 위치를 확보해야 한다.
실측 A/H80 비교와 전체 시험 실행기 구현은 남아 있다.
자세한 범위는 [H80 준비 기록](uwb_h80_readiness_20260926.md)을 따른다.
