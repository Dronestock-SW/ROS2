# UWB 시행 비교
이 문서는 한 번의 파일 재생 결과다. 시행을 검토할 때 읽는다.

실행 ID: `iter_0014_static_raw_w040`. 자료: `gazebo`.
거리 출처: `recorded_capture`.
높이 출처: `simulator_tag_pose_with_mount_offset`.
변경 가설: A shorter H80 window should reduce dynamic lag; check its static noise cost on the recorded RAW.
이전 시행과의 RAW 관계: `same_raw`.

| 모델 | 출력/전체 | 최대 오차(cm) | RMS(cm) | p95(cm) | 7cm 이상 | 판정 |
|---|---:|---:|---:|---:|---:|---|
| A | 696/696 | 8.538 | 3.265 | 5.490 | 7 | 7cm_max_failed |
| B | 694/696 | 5.580 | 1.874 | 3.284 | 0 | exploratory_or_tuning_only |
| C | 696/696 | 8.595 | 3.268 | 5.503 | 10 | 7cm_max_failed |
| D | 370/696 | 12.161 | 5.039 | 8.917 | 60 | coverage_failed |
| WLS | 696/696 | 8.538 | 3.265 | 5.490 | 7 | 7cm_max_failed |

유효 출력의 최대값만으로 최종 달성을 선언하지 않는다.
센서 지연·ToF 입력과 독립 Gazebo 비행은 검증 전이다.
