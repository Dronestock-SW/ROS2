# UWB 가상 시험장을 만든 방법

설정 파일·모델·센서·PX4의 연결을 설명한다. 가상 시험의 구현 범위를 이해할 때 읽는다.

## 구성 원리

기존 PX4 모델을 조합하고 별도 UWB 계산 도구를 붙인다.
새로운 비행제어기를 작성한 것은 아니다.
Gazebo는 기체 운동과 센서 측정을 계산한다.
PX4 SITL은 PC에서 비행제어 소프트웨어를 실행한다.
상태 추정과 자세·위치 제어는 PX4가 담당한다.
사용자 WSL의 현재 확인 결과는 [시험 기록](report/uwb_gazebo_equipment_20260927.md)에 있다.

```text
앵커 좌표 JSON + 장비 배치 JSON + 기본 시험 설정
                   │
                   ▼
           gazebo_rig.py 생성기
             │            │
             ▼            ▼
      드론 model.sdf    시험장 .sdf
             └─────┬──────┘
                   ▼
          Gazebo 물리·센서 계산
                   │ 센서 메시지
                   ▼
                gz_bridge
                   │ PX4 내부 메시지
                   ▼
          PX4 상태 추정·비행제어
                   │ 모터 명령
                   └──────────→ Gazebo 기체 운동
```

이 그림은 설계와 코드의 연결 구조다.
기동 성공만으로 모든 센서 수신을 확인한 것은 아니다.

## 만든 파일과 재사용한 기능

JSON은 설정표이고 SDF는 가상 장비의 설계도다.
생성기는 두 설정표를 읽어 SDF를 작성한다.

| 파일 | 역할 |
|---|---|
| `src/drone_uwb/config/anchors_20260906.json` | A1~A4의 같은 좌표를 화면과 계산에 제공 |
| `src/drone_uwb/config/gazebo_equipment.json` | 센서 장착·질량·측정 범위·시작 위치 설정 |
| `src/drone_uwb/drone_uwb/integration/gazebo_rig.py` | PX4 원본 모델을 읽고 별도 모델·시험장 생성 |
| `runs/equipment_02/models/dronestock_x500/model.sdf` | 기체·센서·모터 구성을 저장 |
| `runs/equipment_02/worlds/dronestock_uwb.sdf` | 앵커·무늬 바닥·시험 벽을 저장 |
| `runs/equipment_02/trial.json` | 가상 UWB·계산 비교에 같은 배치 전달 |
| `runs/equipment_02/manifest.json` | 입력·출력 해시와 생성 당시 검증 상태 기록 |

모델 생성 과정에서는 PX4의 x500 기체를 재사용했다.
하방 거리계·광학흐름·2D LiDAR·카메라도 원본을 조합했다.
생성 소스의 링크·센서 이름은 `gz_bridge`와 맞췄다.

| 구성 | 구현 방식 | 재현 한계 |
|---|---|---|
| FC | PC의 PX4 SITL | 실물 보드의 전기 특성을 재현하지 않음 |
| IMU·기압·자력계·GNSS | x500의 센서와 Gazebo 시스템 플러그인 | 현재 수신·융합 상태는 별도 확인 필요 |
| TFmini Plus 역할 | 아래를 향하는 Gazebo 거리 센서 | 제조사 회로·광학 특성 전체 모사 아님 |
| 광학흐름 | 원본 flow 모델과 `OpticalFlowSystem` | 실제 표면·센서별 오차 교정은 별도 |
| 수평 LiDAR·카메라 | Gazebo 거리·영상 센서 | 시험 측정 범위·해상도 사용 |
| UWB 앵커 | 저장 좌표의 표시물과 계산 기준점 | 표시물 자체는 UWB 거리를 발행하지 않음 |
| UWB 태그 | 기체의 표시물과 Python 가상 거리 생성기 | 전파·CIR 모델 미포함 |

사용자 보고에 따라 FC와 LiDAR의 수평 위치는 앞 0.14m다.
UWB와 ToF의 수평 위치는 기체 중심이다.
센서 장착 높이·카메라·광학흐름 배치는 시험값이다.
모델 질량 2kg도 대략적인 사용자 보고에 따른 시험값이다.
기체의 실제 질량중심·관성은 아직 식별하지 않았다.

## 가상 UWB 거리를 만드는 방법

기체 위치에서 안테나 위치를 구한 뒤 앵커까지의 거리를 계산한다.
[gazebo_geometry.py](../src/drone_uwb/drone_uwb/processing/gazebo_geometry.py)의 구현은 다음과 같다.

```text
안테나 위치 = 기체 기준점 위치 + 회전을 반영한 안테나 장착 위치
기하 거리   = 안테나와 각 앵커 사이의 3차원 거리
가상 RAW    = 기하 거리 + 설정 편향 + 정규분포 잡음
보정 거리   = RAW - 설정 편향
```

단위는 m이다. 앵커 i의 기하 거리는 다음과 같다.

```text
d_i = sqrt((x_tag-x_i)^2 + (y_tag-y_i)^2 + (z_tag-z_i)^2)
```

[gazebo_ranges.py](../src/drone_uwb/drone_uwb/integration/gazebo_ranges.py)는
Gazebo 위치·자세를 받아 이 RAW를 발행하고 기록한다.
시뮬레이션 측정시각과 앵커 순서를 함께 보관한다.
정답 위치는 평가용 파일에 별도로 저장한다.
현재 실시간 생성기의 오차는 설정 편향과 정규분포 잡음이다.
벽을 배치해도 UWB 차폐 편향이 자동 생성되지는 않는다.

```text
Gazebo 기체 위치·자세
  → 안테나 위치 계산
  → 앵커별 가상 RAW 생성·기록
  → 같은 입력 조건으로 A/B/C/D/WLS 계산 비교
  → 선정한 위치 관측의 PX4 전달·융합 시험
```

이 UWB 생성기는 별도 실행하는 프로그램이다.
PX4 시작 명령은 이 프로그램을 자동 실행하지 않는다.
현재 파일 비교 도구의 높이는 시뮬레이터 안테나 위치에서 가져온다.
Gazebo ToF 측정을 비교기의 높이 입력으로 연결하는 작업은 남았다.
UWB 위치 관측을 PX4에 전달하는 가상 비행 시험도 남았다.

## 통신 계층

Gazebo 내부 통신과 지상국 통신은 역할이 다르다.

| 연결 | 방식 | 역할 |
|---|---|---|
| Gazebo ↔ PX4 | Gazebo Transport와 `gz_bridge` | 센서 수신·모터 명령 전달 |
| PX4 내부 모듈 | uORB | 센서·상태 추정·제어 메시지 전달 |
| Gazebo ↔ 가상 UWB Python | Gazebo Transport | 기체 위치 수신·가상 RAW 발행 |
| PX4 ↔ QGroundControl | MAVLink | 상태 확인·조종·설정 통신 |
| 프로젝트의 ROS 2 코드 | 이번 WSL 명령에서는 노드 실행 없음 | 이후 시스템 통합과 구분 |

연결 근거는 사용자 버전의
[GZBridge.cpp](https://github.com/PX4/PX4-Autopilot/blob/c4e4ef98e9d75063bf3d53ebb2716221ee7505ae/src/modules/simulation/gz_bridge/GZBridge.cpp)와
[PX4 실행 문서](https://docs.px4.io/main/en/sim_gazebo_gz/)다.
이번 코드가 ROS 2 저장소에 있어도 WSL에서는 Python으로 직접 실행한다.

## 현재 확인 범위

사용자 로그는 새 시험장 준비와 PX4 시작 완료를 보여준다.

| 항목 | 현재 근거 |
|---|---|
| 새 시험장·드론 파일 | 생성·설치·SDF 검사 확인 |
| Gazebo 시험장 시작 | `Gazebo world is ready` |
| 새 모델의 PX4 연결 초기화 | `gz_bridge`의 월드·모델 이름과 시작 완료 로그 |
| Gazebo 화면 | GUI 시작 로그만 확인. 화면 자체는 미확인 |
| 센서별 실제 값 | PX4 `listener` 등으로 후속 확인 필요 |
| 가상 UWB RAW | 코드 준비. 이번 시작 로그에는 실행 근거 없음 |
| UWB를 사용한 가상 호버링 | 미실시 |

반복된 `No connection to the GCS`는 지상국 미연결을 나타낸다.
초기 기압·전원 경고도 있었으며 지속 여부는 미확인이다.
기본 GNSS 센서도 포함돼 있으므로 UWB 융합 검증과 구분한다.
다음 실행 절차는 [장비 적용 문서](uwb_gazebo_equipment_runbook.md)를 따른다.
