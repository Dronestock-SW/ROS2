# UWB 시행 비교
이 문서는 한 번의 파일 재생 결과다. 시행을 검토할 때 읽는다.

실행 ID: `iter_0009_b_m03_a2_bias_unmodelled`. 자료: `synthetic_pose_fixture`.
거리 출처: `regenerated_from_pose`.
높이 출처: `simulator_tag_pose_with_mount_offset`.
변경 가설: A2_bias_unmodelled 조건에서 M03 게이트 추가가 B의 오차를 줄이는지, 출력은 유지하는지 확인한다.
이전 시행과의 RAW 관계: `same_raw`.

| 모델 | 출력/전체 | 최대 오차(cm) | RMS(cm) | p95(cm) | 7cm 이상 | 판정 |
|---|---:|---:|---:|---:|---:|---|
| A | 320/320 | 28.654 | 15.954 | 25.687 | 161 | 7cm_max_failed |
| B | 318/320 | 27.049 | 16.713 | 26.311 | 162 | 7cm_max_failed |
| C | 320/320 | 26.771 | 15.415 | 24.809 | 162 | 7cm_max_failed |
| D | 225/320 | 28.729 | 15.199 | 25.548 | 110 | coverage_failed |
| WLS | 320/320 | 28.654 | 15.954 | 25.687 | 161 | 7cm_max_failed |
| B_M03 | 318/320 | 27.049 | 16.718 | 26.311 | 161 | 7cm_max_failed |

유효 출력의 최대값만으로 최종 달성을 선언하지 않는다.
센서 지연·ToF 입력과 독립 Gazebo 비행은 검증 전이다.
