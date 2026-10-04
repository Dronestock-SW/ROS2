# UWB 시행 비교
이 문서는 한 번의 파일 재생 결과다. 시행을 검토할 때 읽는다.

실행 ID: `iter_0016_eval_w040`. 자료: `synthetic_pose_fixture`.
거리 출처: `regenerated_from_pose`.
높이 출처: `simulator_tag_pose_with_mount_offset`.
변경 가설: B 0.4s should reduce reversal maximum without violating 7cm or 99% coverage on held-out synthetic path.
이전 시행과의 RAW 관계: `same_raw`.

| 모델 | 출력/전체 | 최대 오차(cm) | RMS(cm) | p95(cm) | 7cm 이상 | 판정 |
|---|---:|---:|---:|---:|---:|---|
| A | 560/560 | 8.290 | 3.190 | 5.467 | 6 | 7cm_max_failed |
| B | 558/560 | 3.763 | 1.599 | 2.745 | 0 | simulator_runtime_unverified |
| C | 560/560 | 8.269 | 3.194 | 5.431 | 5 | 7cm_max_failed |
| D | 335/560 | 10.289 | 4.573 | 8.140 | 39 | coverage_failed |
| WLS | 560/560 | 8.290 | 3.190 | 5.467 | 6 | 7cm_max_failed |

유효 출력의 최대값만으로 최종 달성을 선언하지 않는다.
센서 지연·ToF 입력과 독립 Gazebo 비행은 검증 전이다.
