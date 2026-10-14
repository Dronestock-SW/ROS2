# 13일 저녁 지상 웹 대기
현장 설정 전 수행한 지상 점검 기록이다.
현장 설정 완료 뒤 비행 세션으로 전환할 때 읽는다.

웹과 원본 기록은 실행 중이다. 비행 준비는 미완료다.
사용자는 현재 지상 점검만 요청했다.
확인 시각은 2026-10-13 18:11~18:20 KST다.

| 항목 | 확인 결과 |
|---|---|
| 로컬 브랜치 | 이번에-상원-한거-통합-브랜치, 5db9c0d |
| 실행 소스 | position-origin-release.json의 b5ea7a4 |
| 실행기·AI 파일 | 현재 SHA256이 release와 일치 |
| FC | DISARM·지상·AUTO.LOITER |
| RC·배터리 | RC lost, 배터리 connected=false |
| UWB | 10초간 heartbeat 2개. 거리·XY·decision 없음 |
| TFmini Plus | 거리 0m·품질 0. 유효 거리 아님 |
| FC 융합 | cs_ev_pos=false, cs_opt_flow=false |
| 추정 상태 | fake position=true, heading_good_for_control=false |
| 유지 설정 | EV delay 0ms, 이륙 1.7m, 상승 제한 0.5m/s |
| 웹 | HTTP 200, sangwon_cpp_executor |
| 비행 권한 | execute=false, output_enabled=false, 요청 전달 비활성 |
| 시작점 | mission_start_xy_m과 home_xy_m 모두 미확정 |
| 출력 검사 | 10초 목표 송신 0회, C++ 명령 원장 0바이트 |
| 원본 수집 | collecting. 3초간 3,371,283바이트 증가 |
| ULog | 617.94 → 632.18MiB, 마지막 logger dropout 0 |
| Jetson 여유 공간 | 마지막 확인 약 5.1GiB |

FC ULog는 `/fs/microsd/log/2026-10-11/22_42_14.ulg`다.
파일명 날짜와 현재 날짜는 다르다.
부팅 ID는 `92410646-dd90-47bc-b737-b9547cac388e`다.
ULog 다운로드와 영상 촬영은 아직 하지 않았다.
실제 비행·밀림 원인 확인도 미실시다.

## 실행과 기록 위치

웹 주소는 `http://127.0.0.1:8001`이다.
웹 systemd unit은 `dronestock-ground-web-20261013.service`다.
현재 노트북 터널 PID는 45024다. 재사용 전 확인한다.
지상 세션은 다음 slot의 `sessions/ground-web-20261013-1820`에 있다.
폴더명은 식별자이며 실제 시각은 JSON 기록을 따른다.

```text
/home/arialhanho/ROS2-review-20261008-codex/.review/fc-owned-runtime-20261013
```

PC 사본은 `Drone5/outputs/ground_web_20261013`에 있다.
FC 조회 JSON·웹 상태·10초 관측 결과를 보존했다.
원본 수집은 `manual-captures-20261013/status.json`을 따른다.
센서 서비스와 MAVROS PID 2721은 유지했다.
LiDAR 원본 수집은 기존 상태다. SLAM 보완은 켜지 않았다.
main과 배포 소스는 수정하지 않았다.

후속 FC 조회 한 번은 첫 응답이 없어 중단됐다.
다음 조회 때 SSH 연결도 끊겼다. 원인은 미확정이다.
기존 센서 서비스를 재시작하지 않고 터널을 복구했다.
복구 뒤 FC 고정 조회 15개와 웹 HTTP가 통과했다.
실패 JSON도 PC에 보존했다.

## 현장 설정 완료 뒤

[저녁 절차](../runbooks/evening_web_flight_20261013.md)를 따른다.
먼저 RC·앵커·배터리·현장 조종자와 기수를 확인한다.
실제 UWB·ToF와 지상 상태를 다시 읽는다.
기수가 확인되기 전에는 지상 정렬을 수행하지 않는다.
현재 웹은 관측 전용이므로 비행 세션 전환이 필요하다.

DISARM·지상·활성 임무 없음과 unit 신원을 확인한다.
그 뒤 지상 웹 unit만 중지한다.
`systemctl --user stop dronestock-ground-web-20261013.service`
기존 MAVROS·센서·원본 수집기는 유지한다.
새 실행 폴더에서 현재 부팅·기수로 정렬한다.
실제 원장·미션 저장소를 쓰는 runbook 실행기를 시작한다.
관측 송신 뒤 FC EV·광류 상태를 다시 확인한다.
원본·C++ 기록·ULog 증가와 영상 준비를 확인한다.
첫 시험은 TAKEOFF → POSITION 5초 → 별도 START다.
제자리 호버 10초 뒤 원래 이륙점으로 복귀·착륙한다.
그 결과를 확인하기 전 좌표 이동 시험은 하지 않는다.
