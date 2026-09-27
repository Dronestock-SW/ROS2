# 성능 지표 측정 스크립트 사용법

> 이 문서는 SW 성능 지표를 실제로 재는 절차다. 측정할 때 순서대로 따라 한다.
> 무엇을 왜 재는지는 [../03_metrics_plan.md](../03_metrics_plan.md)를 본다. 여기는 절차서다.
> 최종 갱신: 2026-08-16

---

## 결론 3줄

```text
1. 두 단계다 — record_metrics.sh 로 기록, analyze_metrics.py 로 산출.
2. 지금 잴 수 있는 것은 갱신 주기와 발행 지연 2종. 나머지는 대상 노드가 없다.
3. /uwb_pose 는 이미 대상 목록에 넣어 뒀다. uwb_node 완성 즉시 같은 명령으로 측정된다.
```

---

## 1. 준비

```bash
source /opt/ros/humble/setup.bash
source ~/drone_ws/install/setup.bash
```

한글 폰트가 없으면 그래프의 한글이 네모로 나온다. 확인·설치:

```bash
fc-list :lang=ko | head            # 결과가 나오면 설치되어 있다
sudo apt install -y fonts-noto-cjk # 없을 때만
rm -rf ~/.cache/matplotlib         # 설치 후 캐시 삭제 (안 지우면 계속 못 찾는다)
```

1호기(Jetson Orin Nano)는 2026-08-16 기준 Noto Sans CJK 설치 확인됨.

---

## 2. 측정할 대상을 먼저 띄운다

| 재려는 것 | 먼저 띄울 것 |
|---|---|
| `/scan` | `ros2 launch drone_bringup lidar.launch.py` |
| `/camera/image_raw`, `/camera/camera_info` | `ros2 launch drone_bringup camera.launch.py` |
| `/uwb_pose` | uwb_node — **미구현** |

기록 스크립트는 **발행 중인 토픽만 자동으로 골라 담는다.** 없는 토픽은 건너뛰고 목록에 남긴다.

---

## 3. 기록 — `record_metrics.sh`

```bash
cd ~/drone_ws
./docs/report/scripts/record_metrics.sh -d 120 -l solo
```

| 옵션 | 뜻 | 기본값 |
|---|---|---|
| `-d, --duration` | 기록 길이(초) | 120 (full 프로파일은 20) |
| `-p, --profile` | `light` 또는 `full` | light |
| `-o, --outdir` | bag 저장 상위 폴더 | `~/drone_ws/metrics_bags` |
| `-l, --label` | bag 이름 꼬리표 | 없음 |
| `--no-cpulog` | CPU 동시 기록 끄기 | 기록함 |

### 프로파일 선택

| 프로파일 | 담는 토픽 | 용량 | 언제 쓰나 |
|---|---|---|---|
| `light` | `/scan`, `/camera/camera_info`, `/uwb_pose` | 작음 (20초에 약 1.6 MB) | **기본.** 주기·지연 측정은 이걸로 충분 |
| `full` | light + `/camera/image_raw` | 초당 약 182 MB | 영상 원본이 꼭 필요할 때만 |

> `/camera/image_raw`는 프레임당 6,061,440 byte다. 30Hz면 초당 182 MB.
> 120초 기록하면 약 21 GB가 된다. **`full`은 20초 이내로만 쓴다.**
>
> 카메라 발행 주기만 필요하면 `light`로 충분하다. gscam이 프레임마다 이미지와 `camera_info`를
> 함께 발행하므로, `camera_info`의 주기가 곧 카메라 파이프라인 속도다
> (2026-08-16 자체검증: `camera_info` 29.9985 Hz, GST_ARGUS 센서 모드 29.999999 fps와 일치).

### 함께 생기는 파일

| 파일 | 내용 |
|---|---|
| `<이름>/` | rosbag2 디렉터리 |
| `<이름>_condition.txt` | 기록 조건 메모. **`[확인필요]` 항목을 손으로 채운다** |
| `<이름>_cpu.txt` | 1초 간격 CPU/메모리 (지연 스파이크 원인 확인용) |

조건 메모를 안 채우면 나중에 그 수치가 어떤 상황에서 나온 값인지 알 수 없다.
**측정 직후에 바로 채운다.**

### 측정은 2회 한다

[03_metrics_plan.md](../03_metrics_plan.md) 2.3의 조건이다.

```bash
./docs/report/scripts/record_metrics.sh -d 120 -l solo       # 대상 노드만 단독 기동
./docs/report/scripts/record_metrics.sh -d 120 -l allnodes   # 전체 노드 동시 기동
```

부하가 있을 때만 나타나는 지연이 있어서, 단독 측정만으로는 실비행 상황을 대표하지 못한다.

---

## 4. 산출 — `analyze_metrics.py`

```bash
python3 docs/report/scripts/analyze_metrics.py ~/drone_ws/metrics_bags/metrics_light_solo_20260816_170000
```

| 옵션 | 뜻 |
|---|---|
| `-o, --outdir` | 출력 폴더. 기본은 `<bag경로>_analysis` |
| `--topics` | 특정 토픽만 분석 (예: `--topics /scan /uwb_pose`) |

### 출력 파일

| 파일 | 내용 |
|---|---|
| `summary.txt` | 사람이 읽는 요약 |
| `summary.csv` | 토픽별 통계 1행 — **보고서 표에 붙이는 것은 이것** |
| `raw_<토픽>.csv` | 메시지 1건 = 1행. 경과시간·간격·지연 |
| `plot_<토픽>.png` | 간격 시계열 / 간격 분포 / 지연 시계열 |
| `plot_compare_hz.png` | 토픽별 평균 발행률 비교 |
| `condition.txt` | 기록 조건 사본 (결과만 떼어 봐도 조건을 알 수 있게) |

CSV는 **UTF-8 BOM**으로 쓴다. 엑셀에서 바로 열어도 한글이 깨지지 않는다.

### 산출 항목

| 항목 | 뜻 |
|---|---|
| `평균_Hz` | (메시지수 − 1) / 기록길이 |
| `중앙값간격_s` | 간격의 중앙값. 평균보다 스파이크에 덜 흔들린다 |
| `간격_p95` / `p99` / `최대` | 꼬리 지연. **평균만 보면 안 되는 이유가 여기 있다** |
| `결손횟수` | 중앙값 간격의 2배를 넘은 구간 수 |
| `지연_*` | `header.stamp` → bag 기록 시각의 차 (ms) |

---

## 5. 결과 해석 — 틀리기 쉬운 곳 2개

### 5.1 "발행 지연"은 센서 지연이 아니다

이 스크립트가 재는 지연 구간은 **`header.stamp`부터 bag 기록 시각까지**다.
`header.stamp`가 센서 노출 시각이어야 "센서 지연"이 되는데, 현 설정은 그렇지 않다.

| 노드 | `header.stamp`의 정체 | 근거 |
|---|---|---|
| gscam | 노드가 프레임을 받은 시각 | `use_gst_timestamps: false` ([camera_imx219.yaml:51](../../../src/drone_bringup/params/camera_imx219.yaml#L51)) |
| uwb_node (예정) | 노드가 시리얼 줄을 받은 ROS 시각 | [uwb_node_design.md:180](../../uwb_node_design.md#L180) |

보고서에 쓸 때 **"발행 → 기록 구간"이라고 구간을 밝힌다.** 그냥 "지연 Xms"로 적으면 과소 보고가 된다.

### 5.2 `/scan`의 지연 100ms는 정상이다

2026-08-16 자체검증에서 `/scan`의 발행 지연 중앙값이 101.07ms로 나왔다. 큰 값처럼 보이나
**LiDAR 1회전이 100ms(10Hz)**이고 `header.stamp`가 스캔 시작 시각이다. 즉 대부분이 스캔 누적 시간이다.

- 회전 주기가 바뀌면 이 값도 같이 바뀐다. 절대값이 아니라 **주기 대비**로 본다
- 순수 전송 지연을 보려면 1회전분을 뺀 나머지를 본다 (약 1ms 수준으로 보이나 단정하지 않는다)

---

## 6. 자체검증 기록

스크립트가 실제로 도는 것을 확인한 결과다.

| 항목 | 값 |
|---|---|
| 실행일 | 2026-08-16 |
| 조건 | LiDAR + 카메라 동시 기동, light 프로파일, 20초 |
| `/scan` | 192메시지, 평균 9.9743 Hz |
| `/camera/camera_info` | 578메시지, 평균 29.9985 Hz |
| 한글 그래프 | 정상 렌더링 확인 |

출력물: [../evidence/metrics_selftest/](../evidence/metrics_selftest/)

---

## 7. 알려진 제약

| 제약 | 내용 | 회피 |
|---|---|---|
| 총 기록 길이 | `ros2 bag record -d`는 **파일 1개당 최대 길이**지 총 길이가 아니다 (Humble/rosbag2 0.15.16). 총 길이로 쓰면 기록이 안 끝난다 | 스크립트가 `timeout -s INT`로 끊는다 |
| Image 역직렬화 | `full` 프로파일 분석은 프레임마다 6MB를 푸느라 느리다 | 주기만 필요하면 `light` |
| 측위 RMSE | 이 스크립트는 산출하지 않는다. 기준 위치(ground truth) 수단이 미정 | [03_metrics_plan.md](../03_metrics_plan.md) 1.3 확정 후 추가 |
| QR 인식률 | 미포함. 인식 노드 자체가 없다 | Phase 3 |

---

## 8. uwb_node 완성 후 할 일

코드 수정 없이 그대로 쓸 수 있다. `/uwb_pose`는 이미 대상 목록에 있다.

```bash
ros2 run drone_uwb uwb_node          # 노드 기동
./docs/report/scripts/record_metrics.sh -d 120 -l uwb
python3 docs/report/scripts/analyze_metrics.py ~/drone_ws/metrics_bags/metrics_light_uwb_<시각>
```

판정 기준: EKF2 권장 30~50Hz 충족 여부 ([roadmap.md:97](../../roadmap.md#L97)).
센서 단 실측은 약 43Hz다 (HW팀 작성분, 2026-07-30 230샘플) — **ROS2 토픽 주기와 별도로 본다.**
