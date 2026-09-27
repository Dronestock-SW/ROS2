# SW 아키텍처 (현재 구현 기준)

> 이 문서는 **2026-08-16 시점에 실제로 도는 것**을 그린 구조도다.
> 계획도가 아니다. 미구현은 점선 + "예정"으로 구분해 표시한다.
> 기준 커밋 `7438c17`. 실행 확인 근거: [evidence/10_node_graph.txt](evidence/10_node_graph.txt)

---

## 결론 3줄

```text
1. 구현된 데이터 경로 2개 — LiDAR → /scan, CSI 카메라 → /camera/image_raw.
2. 두 경로는 서로 연결되지 않는다. 소비하는 노드가 아직 없기 때문이다.
3. 자체 작성 노드 0개. 실행 중인 노드 3개는 전부 외부 패키지다.
```

**하드웨어 표기 범위**: SW가 접하는 인터페이스 경계(디바이스 노드 / 시리얼 파라미터 / MAVLink)까지만.
센서 내부 동작·배선·기구부는 HW팀 작성분이라 그리지 않는다.

---

## 1. 전체 구조

```mermaid
graph LR
  %% ===== 하드웨어 인터페이스 경계 =====
  subgraph HW["하드웨어 인터페이스 경계 (SW가 접하는 지점까지)"]
    direction TB
    DEV_LIDAR["/dev/lidar<br/>CP2102 · 230400 8N1"]
    DEV_CAM["/dev/video0<br/>MIPI CSI-2 · nvarguscamerasrc"]
    DEV_UWB["/dev/uwb<br/>CP2102 · 921600 8N1 · JSON"]
    DEV_PX["/dev/pixhawk<br/>MAVLink · 115200"]
  end

  %% ===== 구현 완료 =====
  subgraph IMPL["companion (Jetson) — 구현 완료"]
    direction TB
    N_LIDAR["ydlidar_ros2_driver_node<br/><i>제조사 패키지</i>"]
    N_TF["static_tf_pub_laser<br/><i>tf2_ros</i>"]
    N_CAM["gscam_node<br/><i>apt gscam 2.0.2</i>"]
    T_SCAN(["/scan<br/>LaserScan · 10Hz"])
    T_PC(["/point_cloud<br/>PointCloud · 미사용"])
    T_IMG(["/camera/image_raw<br/>Image rgb8 1640x1232"])
    T_CI(["/camera/camera_info<br/>CameraInfo · 값 비어 있음"])
    T_TFS(["/tf_static<br/>base_link→laser_frame"])
  end

  %% ===== 미구현 =====
  subgraph PLAN["companion — 미구현 (예정)"]
    direction TB
    N_UWB["uwb_node<br/>예정 · Phase 0"]
    N_VP["vision_pose 변환<br/>예정 · Phase 1"]
    N_MAV["mavros_node<br/>예정 · Phase 1"]
    N_SM["스캔매칭 노드<br/>예정 · Phase 2"]
    N_QR["QR/ArUco 인식 노드<br/>예정 · Phase 3"]
    N_FSM["임무 FSM<br/>예정 · Phase 1"]
    N_CTRL["위치제어 PID<br/>예정 · Phase 1"]
    N_LORA["LoRa 텔레메트리<br/>예정 · Phase 1"]
    T_UWBP(["/uwb_pose<br/>예정"])
    T_ODOM(["/mavros/local_position/odom<br/>예정"])
    T_CMD(["/cmd_vel<br/>예정"])
  end

  %% ===== 외부 =====
  subgraph EXT["기체 외부"]
    direction TB
    PX4["PX4 EKF2<br/>HW팀 작성분"]
    GS["지상국 → 웹<br/>예정 · Phase 4"]
  end

  %% --- 구현된 흐름 (실선) ---
  DEV_LIDAR ==> N_LIDAR
  N_LIDAR ==> T_SCAN
  N_LIDAR ==> T_PC
  N_TF ==> T_TFS
  DEV_CAM ==> N_CAM
  N_CAM ==> T_IMG
  N_CAM ==> T_CI

  %% --- 미구현 흐름 (점선) ---
  DEV_UWB -.-> N_UWB
  N_UWB -.-> T_UWBP
  T_UWBP -.-> N_VP
  N_VP -.-> N_MAV
  N_MAV -.-> DEV_PX
  DEV_PX -.-> N_MAV
  N_MAV -.-> T_ODOM
  DEV_PX -.- PX4
  T_SCAN -.-> N_SM
  N_SM -.-> N_MAV
  T_IMG -.-> N_QR
  T_CI -.-> N_QR
  T_UWBP -.-> N_FSM
  T_SCAN -.-> N_FSM
  N_QR -.-> N_FSM
  T_ODOM -.-> N_FSM
  N_FSM -.-> N_CTRL
  N_CTRL -.-> T_CMD
  T_CMD -.-> N_MAV
  N_FSM -.-> N_LORA
  N_LORA -.-> GS

  classDef done fill:#dff0d8,stroke:#3c763d,stroke-width:2px,color:#1b3a1e
  classDef topic fill:#e8f0fe,stroke:#3367d6,stroke-width:1.5px,color:#12294f
  classDef todo fill:#f5f5f5,stroke:#999,stroke-width:1.5px,stroke-dasharray:5 4,color:#444
  classDef hw fill:#fff3cd,stroke:#8a6d3b,stroke-width:1.5px,color:#4a3a12
  classDef ext fill:#fde7e9,stroke:#a94442,stroke-width:1.5px,color:#4a1416

  class N_LIDAR,N_TF,N_CAM done
  class T_SCAN,T_PC,T_IMG,T_CI,T_TFS topic
  class N_UWB,N_VP,N_MAV,N_SM,N_QR,N_FSM,N_CTRL,N_LORA,T_UWBP,T_ODOM,T_CMD,GS todo
  class DEV_LIDAR,DEV_CAM,DEV_UWB,DEV_PX hw
  class PX4 ext
```

범례:

| 표기 | 뜻 |
|---|---|
| 굵은 실선 (`==>`) | 실행 확인된 데이터 흐름 |
| 점선 (`-.->`) | 미구현. "예정" 표기 |
| 초록 상자 | 동작 중인 노드 |
| 파란 캡슐 | 발행 중인 토픽 |
| 회색 점선 상자 | 미구현 노드·토픽 |
| 노란 상자 | 하드웨어 인터페이스 경계 |
| 붉은 상자 | 기체 외부 / 타 팀 소관 |

---

## 2. 구현된 부분만 확대

현재 실제로 도는 것은 이것뿐이다.

```mermaid
graph LR
  DEV_L["/dev/lidar<br/>230400 8N1"] ==> ND["ydlidar_ros2_driver_node"]
  ND ==> S(["/scan<br/>LaserScan"])
  ND ==> PC(["/point_cloud"])
  ND -.-> SRV1{{"/start_scan<br/>/stop_scan<br/>std_srvs/Empty"}}
  TFP["static_tf_pub_laser"] ==> TFS(["/tf_static<br/>base_link→laser_frame<br/>(0, 0, 0.02)"])

  DEV_C["/dev/video0<br/>nvarguscamerasrc"] ==> NC["gscam_node"]
  NC ==> IMG(["/camera/image_raw<br/>rgb8 1640x1232"])
  NC ==> CI(["/camera/camera_info<br/>빈 값"])
  NC ==> TR(["/camera/image_raw/compressed<br/>/compressedDepth · /theora<br/>image_transport 자동 생성"])
  NC -.-> SRV2{{"/set_camera_info<br/>sensor_msgs/SetCameraInfo"}}

  S -.-> NONE1["구독자 없음"]
  IMG -.-> NONE2["구독자 없음"]

  classDef done fill:#dff0d8,stroke:#3c763d,stroke-width:2px,color:#1b3a1e
  classDef topic fill:#e8f0fe,stroke:#3367d6,stroke-width:1.5px,color:#12294f
  classDef hw fill:#fff3cd,stroke:#8a6d3b,stroke-width:1.5px,color:#4a3a12
  classDef none fill:#f5f5f5,stroke:#999,stroke-dasharray:5 4,color:#444
  classDef srv fill:#ede7f6,stroke:#5e35b1,color:#2a1a52

  class ND,NC,TFP done
  class S,PC,IMG,CI,TFS,TR topic
  class DEV_L,DEV_C hw
  class NONE1,NONE2 none
  class SRV1,SRV2 srv
```

**두 경로가 만나지 않는 것이 현재 구조의 핵심 특징이다.** 센서 데이터를 받는 노드가 없어
`/scan`과 `/camera/image_raw`의 구독자 수는 0이다
(`ros2 topic info /scan --verbose` → `Subscription count: 0`, [evidence/04_lidar_launch.txt](evidence/04_lidar_launch.txt)).

---

## 3. 기동 구성 (launch 관점)

```mermaid
graph TB
  L1["lidar.launch.py"] --> INC["ydlidar_launch.py<br/><i>제조사 launch 포함</i>"]
  L1 --> P1[/"lidar_tmini.yaml<br/>port: /dev/lidar<br/>frequency: 10.0"/]
  INC --> ND["ydlidar_ros2_driver_node"]
  INC --> TFP["static_tf_pub_laser"]
  P1 --> ND

  L2["camera.launch.py"] --> NC["gscam_node<br/><i>노드 직접 실행</i>"]
  L2 --> P2[/"camera_imx219.yaml<br/>gscam_config: nvarguscamerasrc...<br/>image_encoding: rgb8"/]
  P2 --> NC

  L3["통합 bringup launch<br/>예정"] -.-> L1
  L3 -.-> L2

  classDef launch fill:#dff0d8,stroke:#3c763d,stroke-width:2px,color:#1b3a1e
  classDef node fill:#e8f0fe,stroke:#3367d6,stroke-width:1.5px,color:#12294f
  classDef param fill:#fff8e1,stroke:#c49000,color:#4a3a12
  classDef todo fill:#f5f5f5,stroke:#999,stroke-dasharray:5 4,color:#444

  class L1,L2,INC launch
  class ND,TFP,NC node
  class P1,P2 param
  class L3 todo
```

| launch | 파일 | 커밋 |
|---|---|---|
| lidar.launch.py | [src/drone_bringup/launch/lidar.launch.py](../../src/drone_bringup/launch/lidar.launch.py) | `bf3c96d` |
| camera.launch.py | [src/drone_bringup/launch/camera.launch.py](../../src/drone_bringup/launch/camera.launch.py) | `3f7f136` |

두 launch를 한 번에 띄우는 통합 launch는 **미작성**이다.

---

## 4. 목표 구조 (Phase 4 완료 시) — 참고용

현재와의 거리를 보이기 위한 계획도다. **전 요소가 미구현**이므로 전부 점선으로 그린다.

```mermaid
graph LR
  subgraph D1["드론 1 (ROS_DOMAIN_ID=1)"]
    direction TB
    S1["센서<br/>UWB · LiDAR · 카메라"] -.-> C1["companion<br/>관측치 공급 + 임무 FSM"]
    C1 -.-> P1["PX4 EKF2<br/>단일 상태추정기"]
    P1 -.-> C1
    C1 -.-> L1["LoRa 송신"]
  end
  subgraph D2["드론 2 (ROS_DOMAIN_ID=2)"]
    direction TB
    S2["센서 동일"] -.-> C2["companion<br/>동일 코드"]
    C2 -.-> P2["PX4 EKF2"]
    P2 -.-> C2
    C2 -.-> L2["LoRa 송신"]
  end
  L1 -.-> GS["지상국<br/>LoRa 수신 → 웹 중계"]
  L2 -.-> GS
  GS -.-> WEB["웹<br/>개발 보류"]
  C1 x--x|"드론 간 통신 없음 (결정 7·9)"| C2

  classDef todo fill:#f5f5f5,stroke:#999,stroke-width:1.5px,stroke-dasharray:5 4,color:#444
  class S1,C1,P1,L1,S2,C2,P2,L2,GS,WEB todo
```

두 드론 사이의 `x-.-x`는 **통신 없음**을 뜻한다 (roadmap 결정 7, 9).
격리는 `ROS_DOMAIN_ID`로만 하며 토픽 네임스페이스 접두어는 도입하지 않는다.

---

## 5. 구조상 확정된 제약

코드를 쓸 때 지켜야 하는 경계다. 각 항목에 이유를 붙인다.

| 제약 | 이유 | 근거 |
|---|---|---|
| companion에 비행용 위치 추정기 금지 | 공분산 이중 계상(과신) / 정보 순환 / 고장 책임 분리 불가 | [roadmap.md:44-51](../roadmap.md#L44-L51) |
| `odom→base_link` TF는 형식 변환만 | 재추정하면 위와 같은 문제가 생김 | roadmap.md:49-50 |
| `map→odom` 발행자는 항상 1개 | Phase 2에서 SLAM 패키지와 충돌 | roadmap.md:51 |
| companion에서 z 제어 금지 | 고도는 TFmini Plus + PX4 EKF 담당 | [altitude_policy.md:3](../altitude_policy.md#L3) |
| 좌표축 변환을 노드에서 하지 않음 | MAVROS가 ENU↔NED를 이미 변환. 두 번 뒤집으면 축이 반대로 감 | [uwb_node_design.md:229](../uwb_node_design.md#L229) |
| 제조사 코드(`ydlidar_ros2_driver`) 수정 금지 | 새 버전과 충돌 + 별도 저장소라 우리 수정이 기록되지 않음 | [README.md:25-27](../../README.md#L25-L27) |
| `/scan` 등 대용량 토픽의 LoRa 송신 금지 | LoRa는 수 kbps 저대역폭 | [roadmap.md:71-72](../roadmap.md#L71-L72) |
| 토픽 네임스페이스 접두어 도입 금지 | 격리는 `ROS_DOMAIN_ID`로 충분. 두 드론의 ROS2는 서로 불가시 | roadmap.md:23, 결정 7 |
| USB 잭 위치 고정 | LiDAR·UWB가 같은 CP2102에 같은 serial `0001`. udev가 포트 위치로만 구분 | [config/udev/99-dronestock.rules:15-32](../../config/udev/99-dronestock.rules#L15-L32) |

`camera/` 접두어에 대한 주의: 이것은 **센서 묶음 이름이지 드론 구분 접두어가 아니다.**
gscam이 스스로 `camera/` 아래에 발행하며, 군집 격리 규칙과 무관하다
([camera.launch.py:10-15](../../src/drone_bringup/launch/camera.launch.py#L10-L15)).

---

## 6. 다이어그램의 근거

| 다이어그램 요소 | 확인 방법 | 증빙 |
|---|---|---|
| 노드 3개 | `ros2 node list` | [evidence/10_node_graph.txt](evidence/10_node_graph.txt) |
| 토픽 및 타입 | `ros2 topic list -t` | 동일 |
| 노드별 발행/구독 | `ros2 node info` | 동일 |
| 서비스 | `ros2 service list` | 동일 |
| 구독자 0 | `ros2 topic info /scan --verbose` | [evidence/04_lidar_launch.txt](evidence/04_lidar_launch.txt) |
| TF 값 (0, 0, 0.02) | launch 기동 로그 | 동일 |
| 미구현 노드 목록 | `ls src/`, grep 전수 | [evidence/06_uwb_fail.txt](evidence/06_uwb_fail.txt), [evidence/08_qr_marker_fail.txt](evidence/08_qr_marker_fail.txt), [evidence/09_comm.txt](evidence/09_comm.txt) |
| 시리얼 파라미터 | udev 규칙 파일 | [config/udev/99-dronestock.rules](../../config/udev/99-dronestock.rules) |
