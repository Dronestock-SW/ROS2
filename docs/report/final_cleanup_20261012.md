# FINAL 브랜치 대조·실행 경계 정리
모든 현재 브랜치를 대조한 코드 정리 기록이다.
새 FINAL을 설치하거나 재검증할 때 읽는다.

브랜치 누락과 실행 계약 오류를 수정했다.
실행 8의 물리 밀림 원인은 아직 미확정이다.
코드 정리와 현장 비행 성공을 구분한다.

## 대조 범위

원격 11개와 로컬 보존 5개를 확인했다.
원격 전체 heads를 fetch한 뒤 조상을 대조했다.
기준 FINAL은 `767a35c`다.
작업 중 올라온 `76d64b3`도 포함했다.
B4 옵션·진단 도구·기록 수정을 보존했다.
[브랜치 SHA·소스 차이](evidence/final_cleanup_20261012/branch_inventory.json)를 보존했다.

| 원격 브랜치 | 검토 시 HEAD | 처리 |
|---|---|---|
| FINAL | 767a35c | 실행 8 분석·LocalXY 기준 유지 |
| codex/flight-uwb-ai-integration-20261007 | 1cc7e3b | FINAL에 포함된 조상 |
| codex/mission-flight-flow-20261008 | b1364ee | FINAL에 포함된 조상 |
| codex/qr-inventory-pipeline | 0b29560 | QR 통합이 FINAL에 포함됨 |
| codex/uwb-web-handoff-20261007 | 7cfc482 | FINAL에 포함된 조상 |
| codex/web-flight-handoff-20261008 | 8c2aec3 | FINAL에 포함된 조상 |
| develop | 09582b9 | FINAL에 포함된 조상 |
| feature | deb940f | FINAL에 포함된 조상 |
| feature_uwb | 3e7f3ef | 누락 guard·기록 수정 대조 |
| main | 45fffc9 | 비교만 수행. 수정·push 없음 |
| uwb-f | 0257c32 | FINAL에 포함된 조상 |

로컬 보존 브랜치도 모두 FINAL의 조상이었다.
local-jetson-integration·mission-chain-faults·native-hover-test를 포함한다.
로컬 mission-flight-flow·web-flight-handoff도 확인했다.
feature_uwb의 비병합 커밋 12개를 대조했다.
수평 ODOMETRY 본체는 FINAL에 이미 복사돼 있었다.
guard 함수와 송신 상태 기록은 빠져 있었다.
다른 브랜치의 전체 파일로 FINAL을 덮지 않았다.
이유: 최신 LocalXY·웹 ARM·호버를 되돌리게 된다.

## 수정한 실행 계약

실제 송신 경로의 TypeError를 없앴다.
ROS와 오프라인 시험이 같은 관측 경계를 사용한다.

| 경계 | 수정·유지 |
|---|---|
| ObservationGuard.check | reject_jumps bool 인자 복구 |
| 수정 근거 | feature_uwb `97d0b5a`·Jetson 복사본 |
| 실제 송신 콜백 | guarded_horizontal_odometry 사용 |
| 기본 점프 보호 | true. 한계값·raw guard 유지 |
| 새 보정 예제 | jump_guard_enabled=true |
| 새 송신기 시작 | 보호 ON/OFF 모두 최신 착륙·DISARM 필요 |
| 실행 8 false 설정 | 과거 비공개 기록 보존. 현장 값 변경 없음 |
| 기록 | local_odometry_status·flight_state·FC target_local 추가 |
| 기록 근거 | feature_uwb `8b5be3b` 보완 |
| 구독 수 | raw sink·outbound setpoint에는 추가 구독 없음 |
| 사건 시각 | wall·monotonic·ROS 시각을 함께 기록 |

우회는 false를 명시한 설정에만 적용된다.
원본 시각·값·신선도·B_TF raw 검사는 유지한다.
오래된 좌표를 새 관측으로 재발행하지 않는다.
시작 때 확인한 관측 세션은 ARM 뒤에도 유지한다.
비행 중 새 송신기를 시작해 추정 위치를 바꾸지 않는다.

## B4 높이 연구와 현장 경로

기본 진단과 연구의 출력 거절 적용을 분리했다.
임계값을 낮추거나 실패를 성공으로 바꾸지 않았다.

FINAL의 B4 높이 검사는 기존에 기본 적용됐다.
11일 기록은 운영 미배포를 명시했다.
Jetson 실제 B_TF 코드에도 이 검사가 없었다.
기존 호버 재생은 검사 적용 시 2,961→0개였다.
거리·배치 보정 전 운영 적용을 보류한 실험이다.

`enforce_b4_height_integrity`는 명시적 bool이다.
기본 false는 현장 배포 B_TF 경로를 유지한다.
false에서도 검사값과 enforced=false를 기록한다.
true는 기존 연구의 차단·복구 상태를 유지한다.
연구 시험은 true를 명시한다.
현장에 적용했다고 소급하지 않는다.

파일 재생은 다음처럼 비교한다.
출력은 새 분석 파일이다. CAPTURE 원본은 보존한다.

```bash
python3 src/drone_uwb/tools/replay_manual_btf.py "$CAPTURE" --output "$BEFORE_OUT"
python3 src/drone_uwb/tools/replay_manual_btf.py "$CAPTURE" --output "$AFTER_OUT" --enforce-b4-height-integrity
python3 src/drone_uwb/tools/compare_b4_integrity.py "$BEFORE_OUT" "$AFTER_OUT" --output "$COMPARE_OUT"
```

FC 고도 제어·ToF 입력은 바꾸지 않는다.
센서 bias·기하 보정은 여전히 필요하다.
기본 경로의 가용성이 정확도를 보증하지 않는다.

## 새 체크아웃의 실행 준비

다른 브랜치의 임시 Python 소스를 요구하지 않는다.
ROS와 제3자 라이브러리는 따로 설치한다.

requirements-field.txt에 두 의존성을 고정했다.
pymavlink 2.4.49·websockets 15.0.1이다.
ROS Humble 메시지·MAVROS·NumPy도 필요하다.
의존 패키지 drone_demo도 함께 빌드한다.
세 패키지만 선택하던 이전 절차의 누락을 수정했다.
--check-only도 의존성·부팅·태그를 검사한다.
좌표·레버암·지연 설정의 불일치도 거부한다.
이 검사는 노드나 비행 프로세스를 시작하지 않는다.
기체 ID·domain은 검증한 설정에서 만든다.
기존 Tag 6은 domain 2로 유지한다.

정지 보정은 두 Y축 부호를 모두 처리한다.
±180도 경계에서도 방위 평균을 올바르게 구한다.
정상 자세·50개 이상 표본·1도 범위를 요구한다.
연결·착륙·DISARM·정지·신선도도 확인한다.
동적 정확도나 센서 지연 보정을 뜻하지 않는다.

별도 자동 UWB 서비스를 함께 추가하지 않았다.
FINAL 시작 도구가 관측 송신기 하나를 관리한다.
이유: FC 입력의 중복 소유를 막아야 한다.
같은 계정용 DDS 선택도 FINAL에서 유지했다.

## 검증

Jetson의 별도 소스 폴더에서 검증했다.
운영 소스·서비스·FC 설정은 수정하지 않았다.
Python ROS 시험은 domain 99를 사용한다.
C++ ROS 시험은 합성 domain 173·181·182·183을 쓴다.

| 항목 | 결과 |
|---|---|
| 수정 전 Linux Python 전체 | 1,071 통과·4 실패 |
| 기존 실패 2개 | 웹 ARM 추가 뒤 시험 어댑터 필드 누락 |
| 기존 실패 2개 | bridge 0.35초·직접 UWB 0.25초 혼동 |
| 수정 후 전체 Python | 1,105 통과·0 실패·0 skip |
| ROS 패키지 빌드 | drone_uwb·drone_demo·drone_mission·drone_platform_link 성공 |
| C++ native AI 빌드 | 전체 빌드 성공 |
| CTest 마지막 전체 묶음 | 31/32 통과. native_hover_ros 실패 |
| 실패 항목 단독 재확인 | native_hover_ros 1/1 통과 |
| CTest 개별 항목 통과 범위 | 전체 32개. 묶음 반복 안정성 미확인 |
| 새 실제 비행·SITL | 미실시 |

[검증 영수증·원본 시험 출력](evidence/final_cleanup_20261012/README.md)을 보존했다.
Jetson에서 시험한 변경 소스 42개를 해시로 대조했다.
Windows 개행은 Git의 LF 바이트로 정규화했다.
승인된 합성 JSON의 원본 해시는 그대로다.

CTest의 간헐 실패를 미해결로 남긴다.
이전 묶음은 scan_sensor_transport·웹 IPC가 실패했다.
마지막 묶음은 합성 입력 나이가 약 0.657초였다.
native_hover_ros는 TELEMETRY_NOT_FRESH로 시작을 막았다.
이때 명령 호출은 없었다. 한계값은 바꾸지 않았다.
동일 설정의 단독 재실행은 통과했다.
마지막 묶음의 합성 임시 파일은 /dev/shm에 뒀다.
운영 파일 저장 위치는 변경하지 않았다.
이 결과만으로 저장장치·스케줄링 원인을 확정하지 않는다.
단독 통과를 전체 묶음 32/32 성공으로 합산하지 않는다.

native_hover_ros는 기존 C++ 2초 호버 시험이다.
FINAL LocalXY의 10초 호버는 Python 시험에 포함된다.
두 경로의 실기 성공은 모두 미검증이다.

UWB 0.25초·bridge 0.35초를 유지했다.
두 검사의 독립성을 회귀 시험으로 확인한다.
END 발행 전 완료 기록 fsync 시험도 유지한다.
Windows 초기 전체 수집은 환경 오류로 중단됐다.
Git symlink가 링크 문자열로 체크아웃됐다.
ROS·fcntl·websockets도 없는 환경이었다.
이를 센서나 비행 로직 실패로 집계하지 않았다.
Linux의 정상 symlink·ROS 환경에서 전체를 검증한다.

Windows의 archive 변환도 별도로 확인했다.
Git 원본 LF가 CRLF로 바뀌면 쉘 실행이 실패했다.
승인된 합성 JSON의 바이트 해시도 달라졌다.
스크립트·해당 JSON에 LF 속성을 추가했다.
기존 원본 기록의 -text 보호 규칙을 보존했다.
합성 JSON 승인 해시 목록은 변경하지 않았다.
새 회귀 시험은 실제 Git archive를 검사한다.
원본 JSONL은 CRLF 그대로, 스크립트는 LF다.

C++ 웹 시험의 자식 환경도 수정했다.
시험 Rig가 PYTHONPATH를 덮어쓰던 문제다.
상속한 별도 의존성 경로를 유지하도록 바꿨다.
시스템 Python에 라이브러리가 없어도 검증할 수 있다.

## 유지한 목표와 남은 조사

수직 이륙·연속 10초 호버·착륙을 유지한다.
높이는 1.7m다. 상승 제한은 0.5m/s다.
호버 XY 경로 길이는 0m다.
밀려간 위치로 출발 목표를 바꾸지 않는다.
추력·게인·회전·공백 한계값도 변경하지 않았다.

실행 8 추정 변화는 최종 공백보다 먼저였다.
영상에는 실제 측면 이동도 보인다.
관측 오류와 이동의 정확한 분리는 남았다.
당시 원시 UWB·B_TF decision 기록은 미확보다.
광류 활성만으로 직접 UWB 만료를 무시하지 않는다.
현재 LocalXY는 관측 만료 시 LAND를 요청한다.

다음 현장 검증은 축·크기·시각 대조가 먼저다.
원시·decision·bridge·FC 목표를 함께 기록한다.
영상의 절대 시각 동기화도 미확정이다.
코드 시험 통과를 실비행 성공으로 표시하지 않는다.

[실행 8 분석](hover_run8_analysis_20261012.md)과
[새 FINAL 설치·재개](../runbooks/final_resume.md)를 함께 읽는다.
