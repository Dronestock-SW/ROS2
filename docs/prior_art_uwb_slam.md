# 선행 사례 조사 — UWB 기반 실내 드론 / UWB+LiDAR 융합 SLAM

> 목적: 로드맵 결정 1(UWB + LiDAR 스캔매칭 융합)이 검증된 방향인지, 그리고 우리가
> 아직 결정하지 않은 구멍이 무엇인지 외부 사례로 확인한다.
> 조사일: 2026-07-27

## 요약 (결론 먼저)

1. **연구 영역에서 UWB+LiDAR+IMU 융합은 확립된 방향이다.** 결정 1은 유지해도 좋다.
2. **상용 창고 재고 드론은 전부 UWB를 쓰지 않는다.** 전원 카메라/LiDAR 기반 무인프라 SLAM이다.
   다만 그 이유는 기술적 우월성보다 **사업적 제약**(고객사 창고에 인프라 설치 협상 불가)으로 보인다.
   자체 창고에서 운용하는 우리에게는 해당하지 않는 제약이다.
3. **★ 우리 설계에 빠진 것이 하나 발견됐다 — yaw(방위) 소스.**
   UWB는 heading을 주지 않고, 철제 랙 창고에서 지자기 센서는 신뢰할 수 없다.
   이 구멍을 메우지 않으면 Phase 1 이륙 단계에서 막힌다. 상세는 §4 R1.

## 1. 상용 창고 재고 드론 — 아무도 UWB를 쓰지 않는다

| 회사 | 위치추정 방식 | 인프라 |
|---|---|---|
| **Verity** (스위스) | LiDAR + 카메라 + 관성센서 SLAM. 실시간 지도 작성 후 그 안에서 cm급 자기위치추정 | 마커·비콘 불필요. 무조명 창고에서 동작, 랙 간격 20~30cm 통과 |
| **Corvus Robotics** | 카메라 입력만 사용하는 학습 기반(신경망) 자율주행 | "GPS 없음, 위치추정 비콘 없음, 랙에 스티커 없음" 명시 |
| **Gather AI** | 기성 드론 + 자체 자율주행 소프트웨어 | 창고 레이아웃/인프라 변경 불요를 세일즈 포인트로 내세움 |

**주목할 발언** — Corvus는 "전통적 컴퓨터비전 기법으로 무인프라 자율주행을 만드는 것은 정말 어렵다"고
밝히고 그래서 학습 기반으로 갔다고 설명한다. 즉 **무인프라는 쉬운 길이어서 택한 게 아니라,
고객 창고에 장비를 설치할 수 없어서 감수한 어려운 길이다.**

**우리에게 주는 시사점**

- 상용이 UWB를 안 쓴다는 사실이 "UWB가 열등하다"는 뜻은 아니다. 판매 모델의 차이다
- 우리는 앵커 4기를 이미 제작·설치·좌표계 검증까지 마쳤다. 이 자산을 버릴 이유가 없다
- 오히려 UWB가 있으면 Corvus가 신경망까지 동원해 푼 문제(절대 위치의 드리프트 없는 기준)를
  훨씬 단순하게 해결할 수 있다. **우리 구조의 강점으로 봐야 한다**
- 단 Verity가 "랙 간격 20~30cm 통과"를 하는 건 LiDAR 기반 국소 인식 덕이다.
  UWB의 ~30cm 오차로는 그 정밀도가 안 나온다 → **정밀 작업은 마커(Phase 3)가 담당**한다는
  우리 로드맵 구조가 옳다

## 2. 연구 — UWB+LiDAR+IMU 융합은 확립된 분야

| 연구 | 융합 대상 | 요지 |
|---|---|---|
| **VIRAL SLAM** (NTU, Nguyen 외) | IMU + 카메라 + 다중 LiDAR + **UWB 레인징** | 타이트 커플링 다중모달 SLAM. UWB 좌표계와 SLAM 좌표계 불일치를 별도 문제로 다뤄 bundle adjustment로 변환 추정 |
| **Lidar/UWB Fusion Based SLAM with Anti-degeneration** | LiDAR + UWB | 특징 없는 공간에서 LiDAR가 퇴화(degeneration)할 때 UWB가 구제 |
| **UWB/LiDAR Fusion for Cooperative Range-Only SLAM** | LiDAR + UWB 거리만 | 거리 측정만으로 협조 SLAM |
| **TC-ESKF UWB/IMU** | UWB + IMU 타이트 커플링 | UWB의 저주파·자세 미제공 한계를 IMU로 보완, 다중 에포크 이상치 제거 |

**핵심 상보성 (연구계 공통 진술)**

```text
UWB         : 절대 위치, 드리프트 없음    ↔  NLOS에 취약, 저주파, 방위 없음
LiDAR SLAM  : 국소 정밀, 고주파           ↔  누적 드리프트, 특징 없는 곳에서 퇴화
```

> "LiDAR SLAM은 NLOS 상황의 UWB 정확도 저하를 보완할 수 있고, UWB는 특징 정보가 없을 때
> LiDAR 위치추정을 보조할 수 있다"

**결론: 결정 1(UWB + LiDAR 스캔매칭 융합)은 학계에서 잘 다져진 조합이다. 방향 유지.**

**단 VIRAL SLAM이 지적한 좌표계 정합 문제는 우리에게도 온다.** UWB 앵커 좌표계와
SLAM이 만든 map 좌표계는 서로 다른 원점·방향을 갖는다. Phase 2에서 이 변환을 어떻게
추정·고정할지 정해야 한다.

## 3. PX4 통합 — 검증된 경로가 존재한다

```text
UWB 태그 → companion 노드 → /mavros/vision_pose/pose
                              → VISION_POSITION_ESTIMATE (MAVLink)
                              → vehicle_visual_odometry (uORB)
                              → EKF2
```

**필수 설정**

| 항목 | 내용 |
|---|---|
| `EKF2_EV_CTRL` | UWB는 x,y만 주므로 **수평 위치만** 융합하도록 설정 |
| `EKF2_HGT_REF` | Range(TFmini) — UWB는 z를 주지 않음 (altitude_policy와 일치) |
| `EKF2_EV_DELAY` | 파이프라인 전체 지연. 경험적 실측 필요 |
| `EKF2_EV_POS_X/Y/Z` | 태그의 기체 중심 대비 장착 오프셋 |
| 전송률 | **30~50Hz 이상**. 너무 낮으면 EKF2가 아예 융합하지 않음 |
| 좌표계 | ROS는 ENU/FLU, PX4는 NED/FRD — 변환 필수 |

**PX4 네이티브 UWB 드라이버(`uwb_sr150`)는 우리 경우에 해당 없음.** NXP SR150 칩 전용이고
용도도 정밀착륙(Landing Target Estimator)이다. DWM3000EVB는 companion 경유가 맞다.
**결정 10의 구조와 일치한다.**

## 4. 우리 설계의 리스크 4가지 (조사에서 도출)

### R1 ★ yaw 소스가 정해져 있지 않다 — 최우선

**문제의 구조**

- UWB는 **위치만 주고 방위(heading)를 주지 않는다**
- 실내 철제 구조물은 지자기를 크게 왜곡한다. 연구 문헌은 "GPS 거부 환경에서 지자기계는
  신뢰할 수 없게 되는 경향"이라고 명시하며, 강철문 근처를 실내 소형 UAV의 **항법상 위험 요소**로 규정한다
- **창고는 철제 랙으로 가득한 공간이다.** 지자기 yaw는 사실상 못 쓴다고 봐야 한다
- yaw가 틀리면 위치 보정이 **엉뚱한 방향으로 적용**되어 기체가 한쪽으로 흘러간다

**실제 보고 사례** — PX4 공식 포럼에 우리와 거의 동일한 구성의 실패 사례가 있다.

```text
UWB x/y (속도·heading·z 없음) + 거리계 + 광학흐름 + vision_pose
EKF2를 수평 위치만 융합하도록 설정
PX4 1.15
→ 이륙 직후 한 방향으로 드리프트
```

이 스레드는 문제 제기와 로그 링크만 확인됐고 해결 답변은 회수하지 못했다.
그러나 **구성이 우리 Phase 1 계획과 사실상 같다**는 점만으로도 경고로 충분하다.

**대안 (연구 문헌이 제시하는 것)**

| 방법 | 내용 | 우리 적용성 |
|---|---|---|
| **듀얼 태그** | 태그 2개로 기체에 고정된 국소 기준 프레임 생성 → heading 산출 | 태그 추가 구매 필요. 가장 직접적 |
| **에고모션 결합** | 단일 태그 + IMU/오도메트리 이동 추정 결합 | 추가 하드웨어 불요. 정지 시 heading 미확정 |
| **스캔매칭 yaw** | LiDAR 스캔매칭이 산출한 yaw를 EV yaw로 공급 | Phase 2 이후에나 가능 |
| **UWB RSS 기반** | 수신신호강도로 heading 추정 (연구 단계) | 실용성 낮음 |

**→ Phase 1 착수 전에 결정해야 하는 항목이다. 결정 11 후보.**

### R2 금속 랙 환경의 NLOS 열화

- 청정 조건 ~10cm, **실제 배치는 통상 ~30cm에 수렴**한다
- 금속 장애물·설비가 UWB 거리 측정 정확도를 크게 떨어뜨린다
- **선반 사이 통로 비행은 최악 조건**이다 — 랙이 앵커와의 직선경로를 가린다.
  그런데 그게 정확히 우리 임무 구간이다
- 대응: 정밀 정렬은 UWB가 아니라 마커(Phase 3)에 맡기는 현 구조가 옳다.
  추가로 **실제 통로 안에서의 UWB 오차 실측**이 Phase 0/1에 필요하다

### R3 멀티 태그 시 태그당 업데이트율 하락

- 우리는 TWR(양방향 레인징) 방식이다. **태그마다 앵커 4기와 순차적으로 메시지를 교환**한다
- 문헌: "각 태그가 최소 4개 앵커와 여러 메시지를 교환해야 하므로 다수 태그 동시 측위 시
  확장성이 크게 제한된다." TWR은 태그 1개 측위에 9회 메시지 교환이 필요하다
- **`equipment_inventory.md`의 "멀티 태그 지원 확인됨"은 "동작한다"는 뜻이지
  "속도가 유지된다"는 뜻이 아니다.** 2호기 투입 시 1호기의 UWB 갱신율이 떨어질 수 있다
- §3에서 본 대로 EKF2는 **전송률이 낮으면 융합 자체를 거부**한다. 직접 연결되는 리스크다
- 기준선 확보(2026-07-30): 태그 1개 **43Hz** 실측. HW 파트가 초기 12Hz에서 상향한 결과이며
  §3의 권장 전송률 30~50Hz를 단독 운용에서는 충족한다
- 그래도 R3은 살아 있다: 단순 절반 가정 시 태그 2개는 약 21Hz로 권장 하한 30Hz에 미달한다
- 대응: 2호기 조립 전에 **태그 2개 동시 운용 시 태그당 Hz 실측**. 부족하면 TDoA 전환 검토

### R4 앵커 볼록껍질(convex hull) 바깥의 정확도 급락

- TIERS의 UWB 드론 데이터셋(IROS 2020)은 앵커가 이루는 다각형 **내부와 외부의 성능을
  구분해 측정**한 것을 기여점으로 내세운다. 바깥에서 정확도가 나빠지기 때문이다
- 참고로 이 데이터셋 구성은 우리와 매우 유사하다: DWM1001 + PX4 Pixhawk + companion + **TFmini**
- 대응: 앵커 4기가 이루는 사각형이 **임무 구역 전체를 감싸는지** 확인. 안 감싸면 앵커 증설

## 5. 권고 사항

**로드맵 반영**

1. **결정 11 신설 — yaw 소스 확정.** R1은 Phase 1 이륙 시퀀스의 선행 조건이다
2. Phase 0에 항목 추가:
   - UWB 정지 상태 노이즈 실측 (→ `EKF2_EVP_NOISE` 값의 근거)
   - 앵커 볼록껍질 대비 임무 구역 커버리지 확인
   - ~~태그당 업데이트율 측정 (2호기 대비 기준선)~~ → 완료: 태그 1개 43Hz (2026-07-30).
     남은 것은 태그 2개 동시 측정
3. Phase 2에 항목 추가: UWB 앵커 좌표계 ↔ SLAM map 좌표계 변환 추정 방법 확정

**유지할 것**

- 결정 1(UWB + 스캔매칭 융합) — 학계 근거 충분
- 결정 10(PX4 EKF2 단일 추정기) — PX4 공식 통합 경로와 일치
- 정밀 작업을 마커에 맡기는 Phase 3 구조 — UWB 정확도 한계상 필연

## 출처

- [VIRAL SLAM: Tightly Coupled Camera-IMU-UWB-Lidar SLAM](https://arxiv.org/abs/2105.03296)
- [Lidar/UWB Fusion Based SLAM with Anti-degeneration Capability](https://www.researchgate.net/publication/347844159_LidarUWB_Fusion_Based_SLAM_with_Anti-degeneration_Capability)
- [UWB/LiDAR Fusion For Cooperative Range-Only SLAM](https://arxiv.org/pdf/1811.02854)
- [An Indoor UAV Localization Framework with ESKF Tightly-Coupled Fusion and Multi-Epoch UWB Outlier Rejection](https://pmc.ncbi.nlm.nih.gov/articles/PMC12737027/)
- [TIERS UWB Drone Dataset (IROS 2020)](https://github.com/TIERS/uwb-drone-dataset)
- [Heading Estimation Using Ultra-Wideband Received Signal Strength](https://arxiv.org/pdf/2109.04868)
- [A UWB-Ego-Motion Particle Filter for Indoor Pose Estimation](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC11014049/)
- [Warning for indoor Micro UAVs with magnetometers: stay away from steel doors!](https://diydrones.com/profiles/blogs/warning-for-indoor-micro-uavs)
- [Analysis of the Scalability of UWB Indoor Localization Solutions for High User Densities](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC6022048/)
- [UWB Localization Techniques — TDoA vs Two-Way Ranging (Inpixon)](https://www.inpixon.com/blog/uwb-localization-tdoa-vs-twr)
- [UWB Indoor Positioning Accuracy Guide (Lansitec)](https://www.lansitec.com/blogs/uwb-accuracy-in-2026-essential-guide-to-10-cm-indoor-positioning/)
- [Using Vision or Motion Capture Systems for Position Estimation — PX4](https://docs.px4.io/main/en/ros/external_position_estimation)
- [Using PX4's Navigation Filter (EKF2) — PX4](https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf)
- [Using the ECL EKF (v1.13) — PX4](https://docs.px4.io/v1.13/en/advanced_config/tuning_the_ecl_ekf)
- [Drifting indoor drone using UWB data on mavros/vision — PX4 Forum](https://discuss.px4.io/t/drifting-indoor-drone-using-uwb-data-on-mavros-vision/39872)
- [PX4 uwb_sr150 driver](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/uwb/uwb_sr150)
- [Corvus Robotics](https://www.corvus-robotics.com/) / [Corvus 무인프라 자율주행 보도](https://dronexl.co/2024/12/21/corvus-robotics-warehouse-ai-powered-autonomous-drones/)
- [Verity IERA Award 2026 — 실내 드론 재고 시스템](https://roboticsbusinessnews.com/news/79/3116/verity-wins-iera-award-2026-for-autonomous-indoor-drone-inventory-system.html)
- [Gather AI vs Verity 비교](https://www.gather.ai/gather-ai-vs-verity)
