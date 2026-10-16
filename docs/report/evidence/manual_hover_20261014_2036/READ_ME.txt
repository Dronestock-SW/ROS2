2026-10-14 20:36~20:37 KST 수동비행 로그 묶음

센서 기록: 20:35:53.282~20:37:51.333 KST. ARM 20:36:13, DISARM 20:37:31.
manual-flight-20261014-after-battery/events.jsonl: UWB 원본/판정, LiDAR scan/point_cloud, IMU, ToF, RC, FC 상태, MAVLink 원본 포함.
manifest.json / summary.json: 수집 설정과 종료 상태. 정상 종료, writer_drained=true, required_topics_missing=[], rejected_events=0.
광류는 /uas1/mavlink_source의 MAVLink 106 패킷에 포함. 전용 ROS 광류 토픽은 미수신.
manual-flight-20261014-2036-traces.tar.gz: 보조 UWB 순환 추적 로그 스냅샷. 비행 전후 다른 시각도 포함 가능.

분석에 사용할 FC ULog: log-49-confirmed-blocks.ulg
첫 두 수신본은 780바이트가 달랐으며 파서 손상 플래그가 발생함.
차이가 있는 38개 90바이트 블록을 FC에서 재조회하여 각 블록이 연속 2회 일치한 값으로 별도 파일 구성.
이 파일은 pyulog file_corruption=false, dropout=0, FC 시간 21.169~191.224초.
block-read-verification.json: 블록 재조회 증거. ulog-validation.json: 파서 검사 및 전체 토픽 목록.
flight-20261014-2036-ulog/log-49.ulg 및 log-49-repeat.ulg는 손상된 최초 수신본 보존용. 분석에는 사용하지 말 것.
LOG_ENTRY의 UTC 표기 대신 ARM/모드/DISARM 전환을 젯슨 기록과 대조해 이번 비행 파일로 확인.
LiDAR 원본은 젯슨 events.jsonl에 있으며 FC ULog에는 별도 LiDAR 진단 토픽이 없음.

코드/FC 파라미터는 변경하지 않음. 원격 원본 삭제하지 않음. 기록 서비스는 착륙 후 자동 종료됨.
이 묶음은 두 번째 비행(20:36~20:37) 자료이며, 첫 번째 비행 ULog 전체는 포함하지 않음.
