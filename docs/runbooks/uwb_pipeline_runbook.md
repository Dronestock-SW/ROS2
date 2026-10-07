# UWB 파이프라인 실행 절차

합성 입력과 기록 파일을 처리하는 절차다.
게이트를 닫은 채 데이터 흐름을 시험할 때 읽는다.

## 준비

companion 저장소 루트에서 실행한다.
설계 범위는 [개발 기준](../architecture/uwb_pipeline_design.md)을 본다.

```bash
cd /home/pgyxn/github/ROS2
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 합성 자료 생성과 처리

매번 새 출력 폴더를 지정한다.
기존 폴더는 로그 혼합을 막기 위해 거부한다.

```bash
ros2 run drone_uwb uwb_pipeline \
  --simulate --duration 10 --seed 7 \
  --config src/drone_uwb/config/pipeline/preimu_pipeline.json \
  --anchors src/drone_uwb/config/anchors/anchors_20261004.json \
  --output /tmp/uwb_pipeline_run_01
```

이 명령은 ROS topic이나 장치 포트를 열지 않는다.
`ros2 run`은 설치된 실행 파일을 찾는 데만 쓴다.

| 결과 파일 | 내용 |
|---|---|
| `input.jsonl` | 파싱 전 입력 원문. 오류 줄도 보존 |
| `diagnostics.jsonl` | 입력 줄별 처리 결과·중간값·닫힌 게이트 |
| `summary.json` | 적용 설정·출처·건수·입력 SHA256 |

## 같은 입력 재생

원본 입력과 같은 설정으로 다시 실행한다.

```bash
ros2 run drone_uwb uwb_pipeline \
  --input /tmp/uwb_pipeline_run_01/input.jsonl \
  --config src/drone_uwb/config/pipeline/preimu_pipeline.json \
  --output /tmp/uwb_pipeline_replay_01
cmp /tmp/uwb_pipeline_run_01/diagnostics.jsonl /tmp/uwb_pipeline_replay_01/diagnostics.jsonl
```

`cmp`가 출력 없이 종료하면 진단 로그가 동일하다.
실행 출처가 달라 summary metadata는 다를 수 있다.

## 후속 ToF·자세 자료 추가

한 줄에 입력 봉투 하나를 저장한다.
아래 값은 형식을 설명하는 예다.

```json
{"source":"simulation","host_received_monotonic_ns":11001000000,"message":{"type":"tof_sample","clock_domain":"host_monotonic_us","measurement_time_us":11000000,"distance_m":1.13,"valid":true}}
```

자세는 `type=attitude_sample`을 쓴다.
`quaternion_wxyz`와 `rotation_convention=R_WB`를 추가한다.
위 ToF 예시의 `distance_m`는 자세 메시지에 넣지 않는다.

실측 자료는 source를 `measured`로 기록한다.
수신 순서와 공통 시간축을 함께 보존한다.
실제 시간 매핑 없이 clock_domain만 바꾸지 않는다.
이유: 시각 이름을 바꿔도 동기화되지는 않는다.

맨 처음에는 UWB 부팅 상태를 포함한다.
UWB message는 원래 태그 JSON을 그대로 넣는다.
기존 `received.jsonl`을 쓸 때는 출처를 명시한다.
변환 전 원본은 별도 보존한다.

## 기대 결과

현재 성공 조건은 입력부터 로그까지 연결되는 것이다.
좌표 정확도는 이번 시험에서 판정하지 않는다.

| 확인 | 기대값 |
|---|---|
| 정상 UWB 입력 | `stages.input=accepted` |
| 거리 처리 | `cal_slant_m=raw_slant_m-range_bias_m` |
| 정상 ToF 입력 | `stages.sensor_buffer=recorded` |
| 계산·전달 게이트 | 모두 closed |
| 위치·사용 가능 여부 | x/y/z=null, valid=false |
| 동일 입력 재생 | diagnostics 바이트 동일 |
