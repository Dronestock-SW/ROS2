# 수동 호버링 로그 데이터 확보

2026-10-14 수동비행의 원본 확보 기록이다.
20:36~20:37 비행을 재분석할 때 읽는다.

로그 확보를 완료했다. 호버링 성능 판정은 미실시다.
코드와 FC 파라미터는 변경하지 않았다.

| 항목 | 확인 결과 |
|---|---|
| 센서 기록 범위(KST) | 20:35:53.282~20:37:51.333 |
| ARM / DISARM | 20:36:13 / 20:37:31 |
| 수집 종료 | 착륙 후 자동 종료, writer_drained=true |
| 필수 토픽 누락 / 기록 거부 | 0 / 0 |
| LiDAR scan / point_cloud | 각각 1,174개 |
| IMU / ToF | 11,444 / 2,730개 |
| 광류 | MAVLink 106 패킷 758개 |
| FC ULog | ID 49, 6,812,803 bytes |

## 원본 위치

[전체 로그 ZIP](evidence/manual_hover_20261014_2036/all-logs-20261014-2036.zip)에 원본과 검증 자료를 담았다.
[파일 설명](evidence/manual_hover_20261014_2036/READ_ME.txt)과
[토픽 목록](evidence/manual_hover_20261014_2036/inventory.json)을 함께 읽는다.
[ZIP SHA256](evidence/manual_hover_20261014_2036/SHA256SUMS.txt)으로 전송을 검증한다.

## ULog 전송 검증

최초 두 수신본은 780바이트가 달랐다.
차이가 있는 38개 블록을 FC에서 재조회했다.
각 블록은 연속 2회 일치한 값을 사용했다.
별도 파일 `log-49-confirmed-blocks.ulg`로 보존했다.
최초 수신본 두 개도 ZIP 안에 남겼다.

[재조회 증거](evidence/manual_hover_20261014_2036/block-read-verification.json)를 남겼다.
[파서 검사](evidence/manual_hover_20261014_2036/ulog-validation.json)는
file_corruption=false, dropout=0이다.
분석에는 confirmed-blocks 파일을 사용한다.
최초 수신본은 손상 표시가 있어 분석에서 제외한다.

## 범위와 남은 작업

사용자는 용량 때문에 통합 기록을 꺼두었다.
이번 비행은 승인된 임시 기록기로 수집했다.
영구 부팅 설정은 변경하지 않았다.
기록기는 최대 10분/512MiB로 제한했다.

FC 목록의 UTC 표기는 실제 비행 시각과 달랐다.
ARM·모드·DISARM 전환을 대조해 파일을 식별했다.
LiDAR 원본은 젯슨 events.jsonl에 있다.
FC ULog에 별도 LiDAR 진단 토픽은 없다.
광류 전용 ROS 토픽 대신 MAVLink 원본이 있다.

첫 번째 20:15 비행의 전체 ULog는 포함하지 않았다.
원격 원본은 삭제하지 않았다.
시간 정합·센서 공백·융합 상세 분석은 남았다.
