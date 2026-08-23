# 증빙: 측정 스크립트 자체검증 실행 (20초)
# 수집일시: 2026-08-16T16:52:36+09:00
# 라벨: 03_metrics_plan.md 7장 측정 도구 / 01_sw_status.md 2.3 카메라 발행률 / 04_evidence_index.md D-10

실행 명령:
  ./docs/report/scripts/record_metrics.sh -d 20 -l selftest -o <경로>
  python3 docs/report/scripts/analyze_metrics.py <bag경로>

이 폴더의 파일은 스크립트가 실제로 낸 출력물이다.
  summary.txt / summary.csv : 산출 통계
  condition.txt             : 기록 조건 (스크립트가 자동 생성)
  plot_*.png                : 그래프 (한글 렌더링 확인용 포함)

## 핵심 결과
  /scan             192메시지 / 19.149s / 평균 9.9743 Hz
  /camera/camera_info 578메시지 / 19.234s / 평균 29.9985 Hz

## 해석 시 주의 2가지
1. /scan 의 발행지연 중앙값 101.07ms 는 파이프라인 지연이 아니다.
   header.stamp 가 스캔 시작 시각이고 1회전이 100ms 이므로, 대부분이 스캔 누적 시간이다.
   순수 전송 지연은 이 값에서 1회전분을 뺀 약 1ms 수준으로 보이나 단정하지 않는다.
2. /camera/camera_info 는 gscam 이 프레임마다 이미지와 함께 발행하므로 카메라 파이프라인
   속도의 대리 지표다. 29.9985 Hz 는 카메라가 30fps 로 돌고 있음을 뜻한다.
