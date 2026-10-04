# 구매 장비와 Gazebo 구성 대조

2026-09-27 장비 반영 범위를 확인한 기록이다.
구매 목록과 가상 기체의 차이를 확인할 때 읽는다.

## 판정과 자료 범위

**현재 구성은 모든 장비의 제품별 플러그인을 갖추지 않았다.**
센서 일부는 범용 가상 센서로 구현했다.
구매 엑셀 원본은 이번 검색에서 찾지 못했다.
따라서 구매 목록 전체와의 대조는 미완료다.
아래 표는 [장비 문서](../equipment_inventory.md)를 기준으로 한다.
이 문서의 확보 상태를 구매 증빙으로 간주하지 않는다.

검색한 범위는 다음과 같다.

- `/home`, `/tmp`, `/mnt`, `/media`, `/opt`의 Excel·ODS 파일명.
- `/home/pgyxn`의 ZIP 내부 파일명. 도구 캐시는 제외했다.
- 저장소의 모든 로컬 Git 참조에 있는 Excel·ODS 파일 이력.
- 저장소 문서의 엑셀·구매·견적 언급.

도구용 엑셀 템플릿 외에 구매 엑셀을 찾지 못했다.
Windows PC의 파일시스템은 이번 검색에 포함되지 않았다.
사용자에게 파일명 또는 저장 위치를 요청했다.
사용자는 현재 디렉토리 또는 `docs`를 지목했다.
저장소 전체를 무시 파일까지 포함해 재검색했다.
Excel·ODS 확장자 파일은 0개였다.
ZIP·Office 컨테이너 후보의 내부 목록도 확인했다.
추가 구매 목록은 찾지 못했다.
`docs`의 CSV 검색 결과는 시험·센서 기록이었다.

## 장비별 대조 결과

아래의 반영은 파일 정의 기준이다.
센서 값의 연속 수신이나 제품 동등성을 뜻하지 않는다.

| 장비 문서의 항목 | 현재 가상 구성 | 판정·남은 차이 |
|---|---|---|
| Jetson Orin Nano Super | companion 계산을 외부 Python 프로세스로 수행하는 구조 | 장치 플러그인 없음. 처리 지연·전력 미모사 |
| Pixhawk 6C Mini | PX4 SITL과 IMU·기압·자력계 센서 | 기능 대체. 보드 자체의 제품 모델 아님 |
| DWM3000EVB | 태그 표시와 별도 `gazebo_ranges.py` | SDF UWB 플러그인 없음. 거리 발행기 별도 실행 필요 |
| Holybro PMW3901 | `flow_camera`, `optical_flow`, `libOpticalFlowSystem.so` | 광학흐름 기능 반영. 칩 특성·실측 오차 미반영 |
| YDLIDAR T-mini Pro | `lidar_2d_v2` GPU LiDAR | 360도, 720광선, 0.02~12m, 10Hz 시험 설정 |
| Benewake TFmini Plus | `lidar` GPU LiDAR | 하향 단일 광선, 0.1~12m, 100Hz 근사 |
| ArduCAM B0191 / IMX219 | `camera` | 640×480, 30Hz, 수평 시야각 1.2rad 시험 설정 |
| DYSCAN DE2110 | 해당 센서·플러그인 없음 | QR 판독 기능 미반영 |
| HELTEC WIFI LoRa 32 | 해당 센서·플러그인 없음 | 무선 링크·지연·손실 미반영 |
| Hobbywing XRotor 2807 1300KV ×4 | `MulticopterMotorModel` ×4 | x500 기본 상수. 구매 모터의 추력 곡선 미반영 |
| MicoAir AM32 4in1 70A | 별도 ESC 모델 없음 | 모터 구동 경로는 있으나 제품 특성 미반영 |
| Vega 6S 22.2V 2900mAh 70C | 해당 배터리 플러그인 없음 | 제품별 용량·방전·전압강하 모델 미반영 |
| MATEK PM12S-4A | 해당 파워 모듈 플러그인 없음 | 전압·전류 측정 회로 미반영 |
| XT60 14AWG 100mm | 개별 모델 없음 | 전원 배선 미반영 |
| DC 5.5×2.5mm 18AWG | 개별 모델 없음 | 보조 전원 배선 미반영 |
| 외주 설계 프레임 | x500 형상·관성 기반 | 총질량만 2kg으로 비례 조정. 실물 형상·질량 분포 미반영 |
| 공용 UWB 앵커 ×4 | 월드 표시와 거리 계산 좌표 | RF·CIR·자동 장애물 NLOS 모사 없음 |
| 지상국 LoRa 수신기·컴퓨터 | 해당 장비 모델 없음 | 지상국 통신 기능 검증과 별도 |
| 충전기·tether | 해당 장비 모델 없음 | 비행 중 충전·줄 구속 모델 없음 |

케이블·컴퓨터까지 개별 플러그인이 필요한지는 시험 목적에 따른다.
다만 이들을 제품별로 구현했다는 근거는 없다.
가상 센서가 있다는 사실과 구매 제품 재현은 구분한다.

## 직접 확인한 정의와 실행 근거

수정 생성본을 XML로 읽어 센서와 플러그인을 열거했다.

- [기체 SDF](../../data/processed/uwb/gazebo_equipment_20260927/rig_wall_name_fix/models/dronestock_x500/model.sdf): 센서 정의 9개, 플러그인 5개.
- [월드 SDF](../../data/processed/uwb/gazebo_equipment_20260927/rig_wall_name_fix/worlds/dronestock_uwb.sdf): 플러그인 12개.
- [생성 코드](../../src/drone_uwb/drone_uwb/integration/gazebo_rig.py): 원본 x500·센서 복사와 질량 조정 확인.
- [UWB 발행기](../../src/drone_uwb/drone_uwb/integration/gazebo_ranges.py): Gazebo 위치 수신과 별도 RAW 발행 확인.
- [설정](../../src/drone_uwb/config/gazebo_equipment.json): 시험값과 `external_output_allowed=false` 확인.

기체 플러그인은 모터 4개와 `MotorFailurePlugin`이다.
월드에는 공통 센서 시스템과 `OpticalFlowSystem`이 있다.
플러그인 수는 구현한 구매 장비 수와 같지 않다.
기압·IMU·자력계는 각 시스템을 통해 처리한다.
LiDAR·카메라는 공통 `Sensors` 시스템을 쓴다.

장비 문서에 없는 `navsat_sensor`도 남아 있다.
월드에는 `NavSat` 시스템이 선언되어 있다.
이는 x500 기반에서 복사된 GPS 구성이다.
현재 PX4가 GPS를 융합하는지는 별도 확인이 필요하다.

[보관된 WSL 시작 로그](../../data/processed/uwb/gazebo_equipment_20260927/wsl_startup_after_wall_fix.txt)는
`dronestock_uwb`와 `dronestock_x500_0`의 시작을 보여준다.
그러나 센서별 값 수신을 입증하지는 않는다.
이 로그에는 UWB 발행기 실행 결과도 없다.
현재 WSL 프로세스에 직접 접속해 검사하지 않았다.

## 이번 변경·검증과 남은 작업

이번 작업은 대조 문서와 색인만 추가했다.
모델·설정·비행 파라미터는 변경하지 않았다.
XML 재파싱과 코드 대조를 실시했다.
Gazebo 실행·비행·센서 수신 검증은 미실시다.
현재 작업공간에서 `gz` 실행기를 찾지 못했다.
코드 변경이 없어 빌드·회귀 테스트는 미실시다.

남은 작업은 구매 엑셀 원본의 행별 대조다.
WSL에서는 센서별 수신과 UWB 발행을 확인해야 한다.
GPS 융합 상태도 실내 시험 조건과 대조해야 한다.
미반영 장비의 구현 범위는 그 뒤 결정한다.
