# 상단 LiDAR 실측·지상 확인
FC에 명령을 보내지 않고 입력을 확인하는 절차다.
장착 측정과 LiDAR 공백 보완 활성화 전에 읽는다.

현재는 이동 시험과 yaw 실측이 남았다.
손으로 옮길 수 없으면 정지 수집까지만 한다.
정지 정합만으로 활성화 확인값을 만들지 않는다.
이유: 이동 방향과 상승 중 구조물을 검증하지 못한다.

## X·Y·Z 측정

기체 앞 +X, 왼쪽 +Y, 위 +Z를 사용한다.
이는 companion의 body FLU 좌표다.
PX4 파라미터의 FRD는 오른쪽·아래가 양수다.
두 좌표의 Y·Z 부호를 혼용하지 않는다.

```text
                   LiDAR 회전축/광선 평면 중심
                              *
                              | +Z
                              |
        +Y (기체 왼쪽) <------ FC 기준점 ------> +X (기체 앞)
```

1. 기존 FC-to-tag 실측의 FC 기준점을 확인한다.
   케이스 중심을 썼다면 그 사실과 오차를 적는다.
   IMU 기준점과 케이스 중심은 자동으로 같지 않다.
2. LiDAR 회전축을 FC 높이 평면에 수직 투영한다.
   추나 직각자·수직 자를 사용한다.
3. FC 기준점부터 투영점까지 전후 X를 잰다.
   앞이면 +, 뒤면 -다.
4. 같은 투영점의 좌우 Y를 잰다.
   왼쪽이면 +, 오른쪽이면 -다.
5. FC 기준점부터 광선 평면까지 수직 Z를 잰다.
   케이스 윗면까지의 높이가 아니다.
6. mm 단위 측정값·측정 오차·사진을 보존한다.
   기체 평면과 LiDAR 평면의 기울기도 확인한다.

사용자 설명은 바로 위 약 15cm다.
예상값은 X≈0, Y≈0, Z≈+0.15m다.
아직 자로 잰 값으로 등록하지 않았다.
예제의 X·Y·yaw는 null로 유지한다.

## yaw 측정

기체와 LiDAR 케이스 방향만으로 yaw를 정하지 않는다.
실제 출력 점의 정면·왼쪽 각도를 비교한다.

1. LiDAR 투영점에서 정면과 평행한 선을 잡는다.
   그 선에 세로 표적을 둔다.
   광선 높이에 맞춘 봉이나 작은 상자를 쓴다.
   1–2m 정도 떨어뜨리고 정면임을 자로 확인한다.
2. 원본 /point_cloud에서 표적의 점군을 찾는다.
   각 점의 각도는 atan2(y,x)다.
   표적 중심 각도를 phi_front로 기록한다.
3. sensor_to_body_yaw_rad는 -phi_front다.
   결과를 -pi부터 +pi로 정규화한다.
   예: 표적이 센서 +20도면 장착 yaw는 -20도다.
4. 같은 투영점에서 왼쪽과 평행한 선을 잡는다.
   그 선에 표적을 둔다.
   phi_left + 장착 yaw가 +90도인지 확인한다.
5. 왼쪽이 -90도로 보이면 yaw만 조정하지 않는다.
   드라이버 반전과 좌표 부호를 확인한다.
6. 표적 위치·각도 오차와 캡처 원본을 보존한다.

FC 중심선에 표적을 놓으면 장착 Y를 고려한다.
표적 body 좌표를 T, LiDAR 위치를 L로 놓는다.
방향 beta=atan2(T.y-L.y,T.x-L.x)를 계산한다.
장착 yaw는 beta-센서에서 읽은 각도다.
위 평행선 배치에서는 beta가 0도 또는 90도다.

scan_plane_parallel_to_body_xy도 실측한다.
기체 평면과 광선 평면이 평행해야 한다.
기체를 못 옮겨도 표적 배치는 가능할 수 있다.
할 수 없는 검증은 미실시로 적는다.

## 읽기 전용 정지 수집

센서와 같은 domain·localhost 환경을 사용한다.
소스와 C++ 빌드는 final_resume 절차를 따른다.
다음 명령은 ARM·모드 변경·목표를 보내지 않는다.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=2 ROS_LOCALHOST_ONLY=1
CAPTURE="$PWD/.review/lidar-ground-NEW"
python3 -m drone_uwb.integration.ros.bench_probe \
  --output "$CAPTURE" --seconds 25
python3 src/sangwon_AI/ops/lidar_timing_audit.py \
  --capture "$CAPTURE" --output "$CAPTURE/timing-audit.json"
python3 src/sangwon_AI/ops/lidar_gap_capture_replay.py \
  --capture "$CAPTURE" \
  --binary "$PWD/.review/final-ai-build/sangwon_lidar_relative_replay" \
  --input-source sdk_pointcloud --output "$CAPTURE-shadow"
```

CAPTURE는 매번 새 이름을 쓴다.
기존 로그를 덮어쓰지 않기 위해서다.
SDK stamps가 없는 입력은 거부한다.
0인 시각을 수신 주기로 채우지 않는다.
0개 수집을 합격으로 처리하지 않는다.

## 동적 대조와 활성화

실측값이 모인 뒤 기존 설정을 활성화한다.
먼저 모터를 움직일 수 없는 지상에서 대조한다.

| 확인 | 필요한 기록 |
|---|---|
| X·Y 이동 | 알려진 이동 방향·거리와 LiDAR 상대 이동 |
| yaw 회전 | 알려진 회전과 회전 보정. 가짜 병진이 없는지 |
| 시각 | 원본 stamps, 전후 FC 자세, 고정 지연 평가 |
| 상하 위치 | 상승해도 같은 수직 구조가 보이는지 |
| 악조건 | 통로·유리·사람·차폐에서 승인하지 않는지 |
| UWB 기준점 | 실제 새 UWB와 스캔 시각 차이 30ms 이내 |
| 복구 | UWB 복귀 시 차이 0.2m 검사와 실패 시 LAND |

정지 점군은 높이 변경 후 같은 벽을 보장하지 않는다.
상단 LiDAR는 한 장의 수평면을 관측하기 때문이다.
이 LiDAR는 FC 위치 추정에 보내지 않는다.
짧은 UWB 공백의 미션 판단만 보완한다.
위치 추정이 장시간 사라져도 나는 기능이 아니다.

| lidar_gap 설정 | 등록할 근거 |
|---|---|
| input_source | sdk_pointcloud |
| sensor_to_body_flu_m | 실측 X·Y·Z (m) |
| sensor_to_body_yaw_rad | 실측 yaw (rad) |
| mount_confirmed / mount_ref | 위 실측과 평행면 기록 |
| timing_confirmed / timing_ref | 설치 SDK와 동적 시각 대조 |
| vertical_structure_confirmed / scene_ref | 높이 변경과 구조물 대조 |
| enabled | 위 항목과 UWB 기준점 확인 뒤 true |

false나 null을 가짜 참조 문자열로 바꾸지 않는다.
실기에서 잘못된 관측을 승인하지 않기 위해서다.
웹 START는 실제 UWB와 FC 준비도 필요하다.
켜져 있기만 한 센서로는 시작할 수 없다.
RC·배터리·현장 조종자 조건도 기존 절차를 따른다.

## 기록한 범위

12일 정지·시각 확인을 지상 기록에 저장했다.
이동 대조·yaw·상승 구조는 미확인이다.
[지상 확인 기록](../report/lidar_ground_audit_20261012.md)을 읽는다.
웹부터 END까지의 모사 결과도 그 기록에서 찾는다.
