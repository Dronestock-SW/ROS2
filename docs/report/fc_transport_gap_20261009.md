# FC 관측 공백 조사 — 2026-10-09

실물 수신 공백과 시계 복구 수정을 기록한다.
실제 발행·START 적용 전에 이 결과를 확인한다.
현재 결과만으로 하드웨어 고장을 단정하지 않는다.

## 실물 수신 기록

90초 관측에서는 긴 공백이 재현되지 않았다.
FC 원시 메시지41,584개를 수신했다.
원시 수신 최대 간격은59.7ms였다.
IMU는61.7ms, B_TF XY는86.2ms였다.
첫 기록은ToF 토픽 경로가 틀렸다.
이 기록을ToF 수신 증거로 사용하지 않는다.

경로를 수정한180초 기록은모든 토픽을 받았다.
원시 메시지80,995개·IMU17,961개를 받았다.
ToF1,772개·XY7,028개를 받았다.
원시 수신 최대 간격은331.8ms였다.
같은 구간의IMU·ToF·XY도지연됐다.
진단 타이머 최대 간격은66.6ms였다.
UWB 원시 수신의최대 간격은89.1ms였다.
FC 응답 RTT는그 구간에도약1~4ms였다.
FC 응답 계산 후ROS 토픽 전달 지연이 관측됐다.
이 구간을FC USB 고장으로 단정하지 않는다.
이전1.8초 RTT 증가와동일 원인인지는 미확정이다.

기록은받는 동안 메모리에 한정해 보관했다.
진단 도구는종료 후 파일로 저장했다.
추가 기록IO가센서 콜백을 막지 않게 했다.
진단 도구의FC 명령·발행자는0개다.

## 시계 복구 수정

유효한 시계의복구 대기만 줄였다.
처음에는좋은 응답30개로 시계를 획득한다.
RTT20ms 초과·중복·역순 응답은수락하지 않는다.
지연 응답으로마지막 유효 시각을 갱신하지 않는다.
유효 응답이0.5초 이상 없으면발행 조건을 닫는다.
같은 시계의새 저지연 응답을 받으면재개한다.
오프셋5ms 초과 변경·FC 연결 해제는재획득한다.
이때는다시30개가 필요하다.
시계 변경 시B_TF 센서 버퍼도 초기화한다.

측정 지연보정 완료 플래그는바꾸지 않았다.
오래된 관측·공분산·출력 나이 검사를유지했다.
원래 표본 시각을갱신해 반복 발행하지 않는다.

이전 공백의시간 응답21개는RTT20ms를 넘었다.
최대 RTT는1,794.18ms였다.
기록 재생에서시계 조건 복구가2.907초 빨라졌다.
이는시계 처리 재생 결과다.
실제 통신 공백 자체가해결됐다는 뜻은 아니다.

## 통신 경로 비교

실행 중MAVROS와 FC 연결을유지했다.
별도 수신기에서기본 DDS와로컬 UDP를 비교했다.
UDP 프로필은진단 프로세스에만 적용한다.
시스템 기본값·핫스팟·FC 파라미터는바꾸지 않는다.
XML은[Fast DDS 2.6.11 공식 UDP 설정](https://fast-dds.docs.eprosima.com/en/v2.6.11/fastdds/transport/udp/udp.html)을 따른다.

```bash
python3 src/drone_uwb/tools/observe_fc_transport.py \
  --seconds 90 --mavros-pid CURRENT_PID --output NEW_DIRECTORY
# 다른 터미널의 별도 수신 프로세스에만 적용한다.
FASTRTPS_DEFAULT_PROFILES_FILE=src/drone_uwb/config/runtime/fastdds_loopback_udp.xml \
  python3 src/drone_uwb/tools/observe_fc_transport.py \
  --seconds 90 --mavros-pid CURRENT_PID --output OTHER_NEW_DIRECTORY
```

MAVROS 발견 통신 소켓에누적 drop11,745개가 있었다.
데이터 UDP 소켓은0개였다.
누적값만으로이번 공백의 원인을 확정하지 않는다.
CPU cgroup의throttled 시간은0이었다.

동시90초 비교는두 수신기 모두 긴 공백이 없었다.
기본 경로의원시 최대 간격은73.7ms였다.
UDP 경로는63.4ms였다.
ToF는각각167.3ms와114.9ms였다.
긴 실패가재현되지 않아UDP 수정 효과는미확정이다.
운영 통신 설정은바꾸지 않았다.

## 저장 실패와 관측 분리

저장 실패가관측 발행을 멈추는 결합을 제거했다.
기존UWB 노드는기록 오류 후파싱도 중단했다.
B_TF 노드도기록 오류 시즉시 반환했다.
실물 디스크 여유는281MiB까지 줄었다.
기록기의256MiB 예약 한계에 가까웠다.
이 조건은통신과 무관하게관측을 중단시킬 수 있다.
앞선 모든 공백의원인으로 확정한 것은 아니다.

수정 후기록 오류는상태에 별도로 남는다.
기록기는기존 용량 제한을 계속 적용한다.
새 관측은기존 유효성·시각 검사를 통과해야 한다.
오래된 표본을반복하거나새 시각을 붙이지 않는다.
기록 종료 오류도보고하고노드 종료를 계속한다.
미션의기록 준비 조건이나명령 권한은바꾸지 않는다.

`test_recording_independence.py`는저장 실패를 주입한다.
정상 입력·거부 입력·만료 입력을 구분한다.
미관측 고도를채우지 않는지도 확인한다.
Jetson에서저장·시계 관련23개 시험을 통과했다.
이 중7개는관측 노드의저장 오류 주입 시험이다.
`drone_uwb`의colcon 빌드도 통과했다.

090427 관측 세션을정상 종료했다.
닫힌 로그1,821,200,354bytes를백업 대상으로 묶었다.
압축본은247,756,074bytes다.
SHA256은`2fb66f9e63a450c2ed37b1798cec7622cca292e1b0ea662aca55031c75c28c1f`다.
PC 복사본 해시가일치했다.
열린 파일이없고원본 해시가그대로임을 확인했다.
그 후닫힌 원본만 정리했다.
직후 디스크 여유는1,995MiB였다.
FC 재부팅·파라미터 쓰기는하지 않았다.

```bash
python3 -m pytest -q \
  src/drone_uwb/test/integration/test_recording_independence.py \
  src/drone_uwb/test/integration/test_async_recording.py \
  src/drone_uwb/test/integration/test_btf_clock_adapter.py \
  src/drone_uwb/test/integration/test_clock_readiness.py
colcon build --symlink-install --packages-select drone_uwb
```

## 수정 후 지상 수신 — 19:15 KST

웹·MAVROS·UWB 관측을 복구했다.
8350·8351 HTTP 연결을 확인했다.
FC는 DISARM·지상·POSCTL 상태였다.
웹에는 최신 FC 상태가 도착했다.
새 세션은 `field-ground-101412-281553c0`이다.
적용 코드는 `9f6ecf9`다.
다른 현장 체크아웃·서비스는 보존했다.

30초 읽기 전용 기록은 모든 토픽을 수신했다.
FC 원시 메시지 13,756개를 받았다.
IMU 2,916개·ToF 290개를 받았다.
B_TF XY 1,102개를 받았다.
새 기록 오류는 없었다.
XY는 약 (5.266, 2.108)m였다.
ToF의 유효 측정 수는 0이었다.
표시 높이 0.15m는 지상 안테나 기준이다.
실측 ToF 고도로 분류하지 않는다.

약 0.34초 수신 공백은 다시 나타났다.
FC 원시 최대 간격은 337.2ms였다.
XY는 339.8ms, ToF는 390.5ms였다.
동시에 UWB ROS 수신도 358.3ms 벌어졌다.
관측 도구 타이머 최대 간격은 76.1ms였다.
저장 결합 수정만으로 통신 문제는 해결되지 않았다.
근거는 `transport-after-recording-fix-1014`다.

정상 bridge는 비활성 상태로 복구했다.
실제 설정의 좌표·시간 보정은 미확정이다.
FC 위치 추정 유효 상태도 false였다.
지상 임시 발행기를 비행용으로 재시작하지 않았다.
START와 명령 출력은 계속 잠겨 있다.
RC 존재를 이유로 미확정 값을 true로 쓰지 않았다.
SITL·실비행 검증은 이번 수정에서 미실시다.

## 검증 범위

Windows 관련 시험74개를통과했다.
시계 관련 시험은Jetson에서79개 통과했다.
실제 FC 상시 발행과START 활성은아직 미완료다.
지상 후보의좌표·고정 지연 보정도미확정이다.
ARM·이륙 명령은보내지 않았다.
