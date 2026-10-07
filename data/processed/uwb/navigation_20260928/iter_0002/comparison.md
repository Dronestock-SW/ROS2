# UWB 시행 비교
이 문서는 한 번의 파일 재생 결과다. 시행을 검토할 때 읽는다.

실행 ID: `iter_0002`. 자료: `synthetic_pose_fixture`.
높이 출처: `simulator_tag_pose_with_mount_offset`.
변경 가설: 기존 합성 경로에서 A/B/C/D의 7cm 이상 표본과 누락을 함께 기록한다.
이전 시행과의 RAW 관계: `same_raw`.

| 모델 | 출력/전체 | 최대 오차(cm) | RMS(cm) | p95(cm) | 7cm 이상 | 판정 |
|---|---:|---:|---:|---:|---:|---|
| A | 320/320 | 7.868 | 3.085 | 5.263 | 1 | 7cm_max_failed |
| B | 318/320 | 4.540 | 2.242 | 3.227 | 0 | exploratory_or_tuning_only |
| C | 320/320 | 8.503 | 3.142 | 5.355 | 2 | 7cm_max_failed |
| D | 225/320 | 12.311 | 3.664 | 6.685 | 10 | coverage_failed |
| WLS | 320/320 | 7.868 | 3.085 | 5.263 | 1 | 7cm_max_failed |

유효 출력의 최대값만으로 최종 달성을 선언하지 않는다.
센서 지연·ToF 입력과 독립 Gazebo 비행은 검증 전이다.
