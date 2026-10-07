# ROS2 비행·UWB·AI 통합 기준
기존 모듈의 책임과 연결 조건을 정한다.
통합 코드나 실물 시험을 변경할 때 읽는다.

통합해도 실물 비행 준비 완료를 뜻하지 않는다.
부팅은 관측만 시작한다. 새 START 없이 실행하지 않는다.

```text
앵커 A1..A4 -> Tag A(5/domain1) 또는 B(6/domain2)
                 | RAW 거리 + 각 거리 시각 + TDMA
                 v
             원본 유효성 검사
                 |                  ToF / IMU
                 |                      |
                 |                Pixhawk -> MAVROS
                 |                      | 거리 / 자세
                 v                      v
               B_TF <--------- 장착값으로 구한 안테나 높이
                 |               (XY 계산에만 사용)
                 v
          안테나 XY(uwb_map), z 미관측
                 | 지도→ENU 회전/이동 + 공분산 회전
                 v
             출력 조건 검사 -------- 현재 차단
                 | 향후 실측·검증 후
                 v
        MAVROS vision_pose -> PX4 EKF2 (수평만)
                                  | FC 내부 센서 융합
                                  v
                          PX4 local_position XYZ
                                  |
                       C++ 관측기 / 목표 도착 판정

카메라 / 바코드 -> 검출·정렬 제안 -> 미완료: 스캔 창·명령 중재
웹 임무 -> AI C++ BT -> 미완료: 실제 PX4 명령 writer
UWB 관측 -> Wi-Fi 웹 표시 / 별도 LoRa 상태 -> 로컬 웹 PC
```

B_TF는 네 거리의 원래 계산이 일치하면 유지한다.
불일치할 때 ToF·자세로 후보를 검사한다.
안테나 높이는 XY 계산의 보조 입력이다.
새 고도 추정기나 Z 목표를 만들지 않는다.
웹 표시·LoRa 수신도 비행 명령의 증거가 아니다.

| 경계 | 이번 기준 |
|---|---|
| UWB 입력 선택 | 일반 설정 `uwb_xy`, A/B 설정 `btf_xy` |
| 지도→PX4 | XY와 2×2 공분산을 같은 회전으로 변환 |
| 안테나 기준점 | bridge에서 안테나 위치 유지. FC 위치·자세로 위치를 다시 추정하지 않음 |
| 레버암 | PX4 body 원점→안테나 FRD 실측값. `EKF2_EV_POS_X/Y/Z` 읽기값과 대조 |
| ToF→안테나 | B_TF 전처리용 FLU 벡터. 위 레버암과 다른 값 |
| Z·자세 | XY 메시지에서는 미관측. 큰 공분산과 `EKF2_EV_CTRL=1` 필요 |
| 시각 | 원본 stamp 유지. 미래·노후·중복·역순 거절. TIMESYNC 연속성 검사 |
| 전달 조건 | 앵커 배치·좌표 정렬·시간 지연·장착 실측 확인 및 FC 설정 일치 |
| 기본 출력 | 비활성. `ground_only=true`에서는 ARM 상태도 차단 |
| PX4 XYZ | 같은 FC 메시지의 XYZ만 관측. UWB XY+ToF를 합쳐 만들지 않음 |
| 웹·LoRa | 웹 관측과 별도 PC의 LoRa 생존 상태. 비행 권한 없음 |

PX4 레버암 정의는 [공식 외부 위치 문서](https://docs.px4.io/main/en/ros/external_position_estimation)를 따른다.
FRD는 전방·우측·하방이다. ROS ENU→NED 변환은 MAVROS가 담당한다.
PX4 파라미터를 이 코드에서 바꾸지 않는다.
장치의 실제 펌웨어와 파라미터 지원 여부도 확인한다.

## 기존 비행 명령 경로

| 구현 | 현재 범위 | 후속 연결 |
|---|---|---|
| `drone_demo.sitl_navigation` | 공중 Hold에서 수평 DO_REPOSITION. 시동·이륙 없음 | 실제 SITL 시행과 융합 검증 |
| `PX4MissionMonitor` | PX4 위치·속도·추정 상태로 도착 판정 | 실제 FC/시계/원점 검증 |
| `sangwon_AI` C++ BT | REPLAY 임무·모드 전환·원장·RC 반환 | 실물 writer·독립 failsafe 검증 |
| `aruco_servo_node` | 0.15m/s 이하 속도 제안 | 카메라→body 부호 실측·단일 명령 중재 |
| 웹 planner | PLANNING_ONLY 수신·receipt | 실행 snapshot 변환과 새 START |

ACK만으로 도착을 판정하지 않는다.
위치·속력·연속 유지와 FC 목표 반영을 함께 본다.
기존 SITL 경로와 C++ 모의 경로를 보존한다.
실물용으로 조용히 전환하지 않는다.
이유: 두 경로의 권한·시계·좌표 검증 범위가 다르다.

## 미완료 모듈

| 모듈 | 있는 것 | 남은 것 |
|---|---|---|
| 카메라 | IMX219, gscam, camera_info 설정 | 현재 초점/교정의 현물 일치 |
| ArUco | 검출·정렬·거리별 허용오차 | 원본 시각·스캔 창·기체/마커 문맥 연결 |
| QR/바코드 | DE2110 reader, 카메라 decoder, parser, recorder | 기대 라벨 검증·원본 시각·내구성 최종 결과 연결 |
| LiDAR | 고정 드라이버·launch·Tmini Pro 약 10Hz 실물 수신 | 시작 checksum 진단·장착 TF·스캔매칭·동적 장애물 반영 |
| AI | C++ BT·경로·스캔·모드·가드·관측 | 실물 센서 producer, 실제 PX4 출력, 공동 E2E |
| 부팅 | 기존 관측 서비스 5개 | 통합판 전환 후 cold boot 검증 |

미측정 값을 0 또는 확인 완료로 대체하지 않는다.
이유: 임의 배치의 연결 시험과 위치 정확도는 다르다.
