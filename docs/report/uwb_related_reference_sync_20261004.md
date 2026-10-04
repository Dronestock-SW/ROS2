# UWB 관련 문서의 현재 기준 동기화

관련 문서의 배치·처리 설명을 정리한 기록이다. 현재 기준과 과거 자료를 구분할 때 읽는다.

## 변경 내용

현재 XY를 6.3×4.6m 직사각형으로 일치시켰다.
A1=(0,0), A2=(6.3,0), A3=(0,4.6), A4=(6.3,4.6)이다.

| 문서 | 반영 내용 |
|---|---|
| `reference/uwb_h80_qs10_reference.md` | ZIP의 과거 배치와 현재 설정 구분 |
| `architecture/uwb_jetson_port_plan_20260920.md` | 과거 이식 계획에 현재 좌표·구현 위치 연결 |
| `reference/uwb_preimu_field_mapping.md` | 당시 매핑과 현재 API 구분 |
| `reference/companion_platform_api_v1_review.md` | 과거 비직교 축 설명의 적용 시점 표시 |
| `reference/companion_platform_alignment_request.md` | 현재 원점·직교 축 기준 추가 |
| `architecture/uwb_node_design.md` | RAW 수신 후 companion XY 계산 역할 반영 |

설정 기준은 [현재 앵커 좌표](../reference/uwb_anchor_layout.md)다.
과거 실측 수치·ZIP 좌표·시험 로그는 변경하지 않았다.

## 확인 결과와 남은 작업

이번 문서 갱신에서는 실행 설정·계산 코드를 변경하지 않았다.
갱신 문서와 문서 색인의 로컬 링크 153개를 확인했고 누락은 없었다.
`git diff --check`도 통과했다.
코드 테스트·빌드·장치 검증은 미실시다.

후속 회신에서 사용자는 드론의 가상 고도 제거를 요청했다.
ToF 실측이 가능하다고 확인했다.
앵커 설치 높이와 구분해 반영했다.
[가상 고도 제거 결과](demo_measured_tof_20261004.md)에서 변경·검증 범위를 확인한다.
