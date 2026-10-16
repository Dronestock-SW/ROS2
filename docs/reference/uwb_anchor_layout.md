# UWB 앵커 좌표와 적용 상태

앵커 좌표와 운영·준비본의 차이를 정리한 사전이다.
배치를 바꾸거나 입력 설정을 선택할 때 읽는다.

2026-10-16 요청한 다음 배치는 높이 2.1m다.
사용자는 네 안테나 중심과 공통 바닥 기준을 확인했다.
XY 6.3×4.6m는 유지한다. 실제 이동은 아직 전이다.
현재 운영 candidate-v4는 여전히 0.15m 설정이다.
책상 위 시험 이후 위치 발행 차단도 유지한다.
[준비·전환 상태](../runbooks/anchor_height_2p1_20261016.md)를 따른다.

## 다음 배치

**네 앵커의 z만 2.1m로 준비한다.**
XY 원점은 A1의 바닥 투영점이며 Z=0은 공통 바닥이다.
A2 방향은 +x, A3 방향은 +y다.
앵커 안테나 높이와 드론 높이는 구분한다.
[고도 원칙](../altitude_policy.md)을 따른다.

| 앵커 | x (m) | y (m) | z (m) |
|---|---:|---:|---:|
| A1 | 0.0 | 0.0 | 2.1 |
| A2 | 6.3 | 0.0 | 2.1 |
| A3 | 0.0 | 4.6 | 2.1 |
| A4 | 6.3 | 4.6 | 2.1 |

```text
        +y
         ^
 A3 (0,4.6) -------- 6.3m -------- A4 (6.3,4.6)
         |                              |
       4.6m                           4.6m
         |                              |
 A1 (0,0) ---------- 6.3m -------- A2 (6.3,0) --> +x

 각 안테나 중심 z = 2.1m, 공통 바닥 z = 0m
```

배열 순서는 `[A1,A2,A3,A4]`를 유지한다.
두 대각선은 계산상 약 7.80064m다.
사용자 지정과 독립 측량 완료를 구분한다.
`installation_confirmed`와 `independently_verified`는 false다.

## 적용 위치

**준비 파일과 실행 중인 파일을 구분한다.**

| 구분 | 경로·상태 |
|---|---|
| 새 좌표 | [anchors_20261016_z2p1.json](../../src/drone_uwb/config/anchors/anchors_20261016_z2p1.json) |
| 새 배치 ID | `warehouse-rectangle-6p3x4p6-z2p1-20261016` |
| 저장소 수신 예제 | [uwb_tag_b.yaml](../../src/drone_uwb/config/runtime/uwb_tag_b.yaml)의 `anchor_file` |
| 저장소 B_TF 예제 | [uwb_btf_tag_b.json](../../src/drone_uwb/config/runtime/uwb_btf_tag_b.json)의 네 좌표 |
| Jetson 준비본 | `/home/arialhanho/anchor-height-2p1-20261016/prepared/` |
| 현재 운영본 | `/home/arialhanho/esp32-single-tag-20261015/candidate-v4/`, z=0.15m·발행 차단 유지 |
| 일반 데모·Gazebo·Tag A·z2p2 예제 | 기존 목적과 좌표 유지 |

앵커 R2 응답기는 높이 좌표를 저장하지 않는다.
태그는 RAW 거리를 보내고 배치 ID를 heartbeat에 표시한다.
실제 거리→XY 계산은 companion의 좌표 설정을 쓴다.
거리 bias는 이번 변경으로 재교정하지 않는다.
태그 지상 높이·ToF 장착 간격의 0.15m도 유지한다.

## 과거 자료

**과거 재생은 당시 좌표와 설정을 사용한다.**

| 자료 | 사용할 배치 |
|---|---|
| 다음 2.1m 설치 | `anchors_20261016_z2p1.json`, 실제 확인 후 전환 |
| 0.15m 시험·기존 candidate-v4 | `anchors_20261007_z015.json` |
| 과거 높이 2.2m 시험 | `anchors_20261004.json` |
| 2026-09-06 정지 기록 | `anchors_20260906.json`과 고정된 입력 해시 |
| 기존 원본·처리 기록 | 기록 당시 좌표·설정·manifest |

0.15m·2.2m의 날짜별 좌표와 기록 해시를 보존한다.
당시 결과의 재현 근거를 유지하기 위해서다.
[9월 실측 이력](uwb_anchor_survey.md)은 이전 배치의 근거다.
