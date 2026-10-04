# UWB 시행 비교
이 문서는 한 번의 파일 재생 결과다. 시행을 검토할 때 읽는다.

실행 ID: `iter_0005_dropout`. 자료: `synthetic_pose_fixture`.
높이 출처: `simulator_tag_pose_with_mount_offset`.
변경 가설: dropout 조건에서 A/B/C/D의 오차·출력률·7cm 이상 표본을 비교한다.
이전 시행과의 RAW 관계: `none`.

| 모델 | 출력/전체 | 최대 오차(cm) | RMS(cm) | p95(cm) | 7cm 이상 | 판정 |
|---|---:|---:|---:|---:|---:|---|
| A | 310/320 | 7.868 | 3.080 | 5.251 | 1 | coverage_failed |
| B | 306/320 | 4.540 | 2.234 | 3.217 | 0 | coverage_failed |
| C | 310/320 | 8.503 | 3.138 | 5.327 | 2 | coverage_failed |
| D | 220/320 | 12.311 | 3.634 | 6.670 | 9 | coverage_failed |
| WLS | 310/320 | 7.868 | 3.080 | 5.251 | 1 | coverage_failed |

유효 출력의 최대값만으로 최종 달성을 선언하지 않는다.
센서 지연·ToF 입력과 독립 Gazebo 비행은 검증 전이다.
