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
별도 수신기에서기본 DDS와로컬 UDP를 비교한다.
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

## 검증 범위

Windows 관련 시험74개를통과했다.
ROS 전용4개는실행 대상Jetson에서 확인한다.
실제 FC 상시 발행과START 활성은아직 미완료다.
지상 후보의좌표·고정 지연 보정도미확정이다.
ARM·이륙 명령은보내지 않았다.
