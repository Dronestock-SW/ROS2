# UWB 시행 비교
이 문서는 한 번의 파일 재생 결과다. 시행을 검토할 때 읽는다.

실행 ID: `iter_0012_dynamic_w040`. 자료: `synthetic_pose_fixture`.
거리 출처: `regenerated_from_pose`.
높이 출처: `simulator_tag_pose_with_mount_offset`.
변경 가설: Shorter H80 window may reduce reversal lag at the cost of static noise.
이전 시행과의 RAW 관계: `same_raw`.

| 모델 | 출력/전체 | 최대 오차(cm) | RMS(cm) | p95(cm) | 7cm 이상 | 판정 |
|---|---:|---:|---:|---:|---:|---|
| A | 480/480 | 8.051 | 3.000 | 4.918 | 4 | 7cm_max_failed |
| B | 478/480 | 4.777 | 1.612 | 2.671 | 0 | exploratory_or_tuning_only |
| C | 480/480 | 8.089 | 3.004 | 4.926 | 4 | 7cm_max_failed |
| D | 266/480 | 11.602 | 4.897 | 8.380 | 34 | coverage_failed |
| WLS | 480/480 | 8.051 | 3.000 | 4.918 | 4 | 7cm_max_failed |

유효 출력의 최대값만으로 최종 달성을 선언하지 않는다.
센서 지연·ToF 입력과 독립 Gazebo 비행은 검증 전이다.
