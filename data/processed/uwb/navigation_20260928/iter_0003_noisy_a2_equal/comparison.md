# UWB 시행 비교
이 문서는 한 번의 파일 재생 결과다. 시행을 검토할 때 읽는다.

실행 ID: `iter_0003_noisy_a2_equal`. 자료: `synthetic_pose_fixture`.
높이 출처: `simulator_tag_pose_with_mount_offset`.
변경 가설: noisy_A2_equal 조건에서 A/B/C/D의 오차·출력률·7cm 이상 표본을 비교한다.
이전 시행과의 RAW 관계: `none`.

| 모델 | 출력/전체 | 최대 오차(cm) | RMS(cm) | p95(cm) | 7cm 이상 | 판정 |
|---|---:|---:|---:|---:|---:|---|
| A | 320/320 | 47.866 | 16.741 | 33.473 | 227 | 7cm_max_failed |
| B | 318/320 | 19.643 | 6.665 | 13.004 | 95 | 7cm_max_failed |
| C | 320/320 | 45.768 | 17.318 | 34.581 | 228 | 7cm_max_failed |
| D | 167/320 | 47.294 | 17.206 | 37.003 | 102 | coverage_failed |
| WLS | 320/320 | 47.866 | 16.741 | 33.473 | 227 | 7cm_max_failed |

유효 출력의 최대값만으로 최종 달성을 선언하지 않는다.
센서 지연·ToF 입력과 독립 Gazebo 비행은 검증 전이다.
