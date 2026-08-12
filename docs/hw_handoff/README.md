# HW 파트 수신 자료 (docs/hw_handoff/)

이 폴더는 **HW 파트에서 받은 자료의 보관소**다. 우리가 쓴 문서가 아니다.
기술 판단이 필요할 때는 여기가 아니라 docs/roadmap.md 를 본다.

## 파일

| 파일 | 종류 | 언제 읽나 |
|---|---|---|
| [easy_guide.md](easy_guide.md) | 설명 | **여기부터 읽는다.** 두 자료를 배경지식 없이 읽히게 풀어 쓴 글 + 우리 쪽 판단 |
| [uwb_serial_log_operator_guide.md](uwb_serial_log_operator_guide.md) | 사전 | UWB 로그 필드 하나하나의 뜻을 찾을 때 |
| [px4_offboard_mavlink_params.md](px4_offboard_mavlink_params.md) | 사전 | PX4 파라미터·MAVLink 경로를 찾을 때 |

## 수신 이력

| 항목 | 내용 |
|---|---|
| 수신일 | 2026-08-12 |
| 원본 1 | `uwb_serial_log_operator_guide.md` (HW 파트 작성, 검증 기준 2026-08-09 실측) |
| 원본 2 | `PX4_ROS2_MAVLink_OFFBOARD_정리.pdf` (12쪽, 작성일 2026-08-09) |

## 원본 상태 — 주의

- **두 파일 모두 전사본이다.** 원본 바이너리(PDF)와 원본 .md 파일은 이 저장소에 없다.
- 원본 .md는 **인코딩이 깨진 상태로 도착**했다. UTF-8 파일을 CP1252로 읽은 mojibake다.
  내용을 복원해 UTF-8로 다시 적었으므로 의미는 같지만 바이트 단위 원본은 아니다.
  → HW 파트에 UTF-8 원본 재요청이 필요하다.
- 원본 PDF가 필요하면 이 폴더에 그대로 넣는다. 파일명은 `px4_offboard_mavlink_params.pdf`로 통일한다.

## 이 자료가 우리 작업에 주는 결론 3줄

```text
1. UWB 현재 오차 약 0.66 m. 원인은 통신이 아니라 앵커 bias(교정 미실시).
   → 교정 전까지 vision_pose를 실비행에 쓰지 않는다.
2. UWB 현재 위치와 목표 setpoint는 서로 다른 MAVLink 경로로 넣는다. 섞으면 드론이 안 움직인다.
3. EKF2_EV_CTRL=1 (X/Y만). 높이는 TFmini Plus. altitude_policy.md 와 일치한다.
```

## 미해결 항목

| 항목 | 내용 | 다음 행동 |
|---|---|---|
| 위치 제어 주체 | HW 문서는 PX4 Position Controller가 위치 루프를 돈다고 본다. roadmap 결정 4는 companion이 위치제어 + 게인 스케줄링을 맡는다고 본다 | 설계 논의에서 확정 |
| 태그 펌웨어 버전 | 실측 로그에 `kf_*` 필드가 없고 `layout_id`가 소스와 다르다. 저장소와 다른 빌드일 가능성 | HW에 firmware version + Git commit 요청 |
| 원본 인코딩 | 수신 .md가 mojibake | UTF-8 원본 재요청 |
