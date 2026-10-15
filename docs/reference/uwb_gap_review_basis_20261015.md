# 드론 설계 검토의 상세 근거: 하드웨어·운영 판본·검토 이력

본문은 [개선안](../architecture/uwb_gap_recovery_20261015.md)에서 읽는다. 이 문서는 본문의 판단에 사용한 정확한 사양과 기록을 보존한다. 명목 사양, 과거 설정, 실제 배포, 시험 결과를 구분했으며 이번 재구성에서 장치를 다시 측정하거나 설정하지 않았다.

**운용·배포 기준**

| 항목 | 현재 기준 | 해석 |
|---|---|---|
| 수평 경로속도 | 0.25m/s | 앞서 제시한 2~3m/s를 대체한 운용 조건 |
| 상승·하강 | 상승 최대 0.5m/s, 하강 0.25m/s | 수평 감속도와 단위·의미가 다름 |
| 원하는 정확도 | 수평 10cm 이내 | 우선 실제 목표 도착·호버 오차로 정의. 이동 중 오차는 별도 평가 |
| 장애물 여유 | 기체 외곽에서 30cm 이상 | 진행방향과 기체 전체 이동 외곽을 기준으로 판단 |
| 운영 B_TF | 0110da8 + A-cache, acache-20261015 | 10/15 16:04 배포 기록 기준 |
| 최신 통합 소스 | a029476 위에 A-cache | 운영 전체가 이 소스로 교체됐다는 뜻은 아님 |
| A/B 시험 기준 | 563960e + 계측 | 운영본·최신 통합본과 구분 |
| 센서 수신 | 유효 ToF 약 50Hz, companion IMU 약 99.8Hz | FC 내부 IMU 주기나 최악 지연의 보증이 아님 |

운영 A-cache는 설치·기동·수치 계약·관련 회귀 검사를 통과했다. 다만 당시 UWB RAW가 없어 실제 XY 계산과 발행은 확인하지 못했다. 이후 다른 작업의 실제 FC 지상 주입은 기록 기반 가상 ODOMETRY 시험이므로, 이 미완료 항목을 대신하지 않는다. [운영 배포 기록](../report/btf_acache_deployment_20261015.md)

**전체 하드웨어와 장착 기준**

| 장비 | 확인된 모델·조건 | 설계에 반영할 사항 |
|---|---|---|
| FC | Holybro Pixhawk 6C Mini. 제조사 명목 STM32H743 Cortex-M7 최대 480MHz, flash 2MB·RAM 1MB | 자세·속도·위치 제어와 EKF 책임 유지. 현재 CPU 과부하가 측정됐다는 뜻은 아님 |
| Companion | Jetson Orin Nano Super. 개발키트 명목 6코어 Cortex-A78AE 최대 1.7GHz, 8GB LPDDR5, 최대 67 INT8 sparse TOPS, 7~25W | GPU 성능은 Python GC 정지의 해결책이 아님. 실제 메모리 구성·전력 모드·클록은 제품 사양과 구분 |
| UWB | Qorvo DWM3000EVB 태그, 자체 앵커 4개 | Jetson에서 거리→XY·품질·공분산 계산. 앵커 좌표·거리 교정·세션 식별을 함께 관리 |
| Optical flow | BOM은 Holybro PMW3901. 실제 사용은 확인됐지만 BOM 미입고 표기와 장치 신원이 불일치 | 탑재 모델·축·품질·높이 범위를 확인한 뒤 설정. 특정 칩의 사양을 추정 적용하지 않음 |
| 수평 LiDAR | YDLIDAR T-mini Pro, 360도 수평. 과거 연결값 230400baud·약 10Hz | 별도 2D 속도 관측으로 활용. 동일 EV 위치 출처로 위장하거나 수직 속도를 0으로 채우지 않음 |
| 하방 ToF | Benewake TFmini Plus. 문서상 I²C bus 2·0x10·400kHz. 내부 100Hz 설정 ACK, 유효 companion 수신 약 50Hz | flow 축척·높이 보정에 필요한 거리와 시각을 유지. 50Hz가 물리 측정 지연 20ms를 뜻하지 않음 |
| 카메라 | ArduCAM B0191 IMX219, CAM0 | 마커 관측 후보. 검증된 VIO나 상시 위치 보조로 계산하지 않음 |
| QR | DYSCAN DE2110 | 식별 문자열 처리. 항법 실행을 막지 않게 분리 |
| 통신 | HELTEC WIFI LoRa 32, 별도 지상 수신기 | 요약 상태 전달. 제어·복구 판단이 지상국 왕복에 의존하지 않게 유지 |
| 추진계 | Hobbywing XRotor 2807 1300KV ×4, MicoAir AM32 4in1 70A | 명목 전류·KV로 실제 감속도를 계산하지 않음 |
| 전원 | Vega 6S 22.2V 2900mAh 70C, MATEK PM12S-4A | 전압 저하·센서/Jetson 전원 안정성·추력 여유를 고려 |
| 배선 | XT60 14AWG 100mm, DC 5.5×2.5mm 18AWG | 전원 경로의 실제 연결·접촉·진동 영향과 함께 검토 |
| 기체 | 외주 프레임, 질량 약 2kg 이하라는 사용자 추정 | 실측 중량·프로펠러 직경/피치·무게중심·전체 외곽·제동 성능이 아직 수치로 확정되지 않음 |

명목 보드 사양: [Holybro](https://docs.holybro.com/autopilot/pixhawk-6c-mini/overview), [NVIDIA](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/nano-super-developer-kit/). 부품·장착 이력: [장비 문서](../equipment_inventory.md).

앵커는 6.3×4.6m 배치가 기록돼 있다. 높이는 과거 BOM의 2.2m와 10/14 수동 비행의 사용자 확인 0.15m가 다르므로 세션별 실제 배치로 고정해야 한다. 과거 높이를 현재 모든 분석에 적용하지 않는다.

장착 벡터도 기준점을 구분한다. 10/12 최신 추정은 FC 기준 FLU에서 태그 `(-0.10,0,+0.07)m`, LiDAR `(0,0,+0.10)m`, LiDAR yaw 약 −91.5도다. 별도의 ToF→태그 운영값은 `(0,0,+0.15)m`이며 사용자 근사 측정은 +0.17m다. 이 둘을 같은 높이로 비교하면 안 된다. FC `EKF2_EV_POS_Z=-0.10m` 기록은 FC→태그 FRD 추정 −0.07m와 대조할 대상이며, 실측 없이 어느 값을 정답으로 채택하지 않는다.

태그 위치와 FC 제어 기준점을 혼동하면 수평 10cm 레버암 자체가 목표 오차와 같은 크기다. 위치 보정을 Jetson과 PX4에서 중복 적용하지 않도록 책임을 한 곳으로 정한다. 회전 중 센서 속도 차이도 같은 장착 벡터로 처리한다.

**본문에서 다룬 설정의 나머지 조건**

고도 기준은 기압, 거리계는 조건부 보조라는 기록이다. 외부 XY를 사용한다는 이유로 EV 높이 융합을 추가하지 않는다. `EKF2_EV_CTRL=1`, `EKF2_EV_NOISE_MD=0`, `EKF2_EVP_GATE=5`, `EKF2_EVP_NOISE=0.10m`, `EKF2_EV_DELAY=0ms`는 10/12 문서 기준값이며 현재 기체에서 새로 읽은 값이 아니다. 공분산의 분산 단위 m²와 표준편차 단위 m를 구별한다.

작은 지연 수정 후보도 기존 150/100/20ms 검사 의미를 보존해야 한다. 입력·출력 유효성, ToF 시각, IMU 시각을 검사하는 기준이 서로 다르므로 이를 누적 대기시간으로 더하지 않는다. 단순 입력 만료 때의 ClockMap 보존과 실제 재부팅·시계 변경 때의 초기화도 구별한다.

현재 독립 송신은 50ms, 즉 20Hz이며 `PositionTarget`의 mask는 2552다. 위치·yaw만 사용한다. 경로의 가속도 파라미터를 바꾸는 것만으로 이 입력의 궤적 생성이 바뀌지는 않는다. 목표 생성 및 복구 코드는 본문에 연결한 최신 통합 사본을 기준으로 읽었다.

C++ 이전을 선택하더라도 OS 스케줄, 잠금, 메모리, 전송 지연은 남는다. 기존 수치·측정 시각·거부 판정이 보존되는지와 실제 발행 나이를 함께 비교한다. 현재 릴리스는 이전 슬롯의 native 라이브러리를 참조하므로 자체 포함 패키지로 바꾸기 전에는 이전 슬롯을 삭제하지 않는다.

**기존 Claude Pro 3차 검토 기록**

Claude Code 로그인 방식 `claude.ai / firstParty / pro`와 별도 API 키 환경변수 부재를 확인한 뒤, 사용자 지정에 따라 구독 연결을 사용했다. 각 차수에는 앞선 답변과 반론을 함께 전달했다. 세 답변은 모두 정상 반환됐고, Claude의 도구 실행은 비활성화했다. 연결 실패와 응답 형식 오류는 검토 횟수에 포함하지 않았다.

| 차수 | 주고받은 내용 | 최종 반영 |
|---|---|---|
| [1차](../report/evidence/uwb_architecture_review_20261015/review-1.md) | 하드웨어·기존 증거·구조 초안의 반론 요청 | 거리 모순, 제어속도와 경로속도의 차이, 도착 반경 불일치를 우선 과제로 채택 |
| [2차](../report/evidence/uwb_architecture_review_20261015/review-2.md) | 1차의 EV 지연 해석·비공면 앵커·속도 제안·단순 reset 허용을 반박 | 통신 지연 중복 보정, B4 수학 계약을 깨는 배치 변경, 근거 없는 0.5~0.75m/s 상한 제안을 철회 |
| [3차](../report/evidence/uwb_architecture_review_20261015/review-3.md) | 지도 변환이 drift를 흡수하는 오류, FC와 companion의 reset 보상 경합, 고착 상태 진단을 검토 | 고정 목적지·입력 좌표 변환·임시 제어 참조 분리, 새 관측·새 융합·세대 일관성, heading·fallback·이륙 전 정합 조건 추가 |

위 세 차례는 문서 재구성 전에 완료한 기술 검토다. 이번에는 같은 내용을 다시 Claude에 보내지 않았다. 기존 비행제어·Physical AI·임베디드/센서 AI 검토도 실제 전문가의 실기 인증을 뜻하지 않는다. 재구성에서는 한 모델이 논리, 비전공 독자, 전공자 관점으로 원문을 다시 읽었다. 기존 검토에 사용한 MD 75개 경로와 판본 대조는 [자료 목록](../report/evidence/uwb_architecture_review_20261015/source_inventory.md)에 보존했다.

Claude 제안 중 전송 지연을 EV_DELAY에 그대로 넣는 안, 비공면 앵커 배치를 가벼운 변경으로 다루는 안, 근거 없는 0.5~0.75m/s 상한, companion에서 reset delta를 더하면 해결된다는 설명은 채택하지 않았다. 최종 제안은 본문을 따른다.

원본 문서는 [수정 전 스냅샷](../report/evidence/uwb_architecture_review_20261015/report_before_restructure.md), 문장별 판단과 이동 내역은 [재구성 검토](../report/evidence/uwb_architecture_review_20261015/restructure_notes.md), 사실 보존의 수동 점검은 [검토 메모](../report/evidence/uwb_architecture_review_20261015/restructure_notes.md)에 남겼다. 스냅샷의 상대 링크는 원래 REPORT.md 위치를 기준으로 하므로 현재 근거 링크는 이 문서와 본문을 이용한다.
