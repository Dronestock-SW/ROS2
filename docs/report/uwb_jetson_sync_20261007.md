# Jetson 4앵커 저장소 적용 기록 — 2026-10-07
이 문서는 Jetson 적용과 검사 결과 기록이다.
현재 설치 상태와 다음 실물 시험을 확인할 때 읽는다.

## 적용 결론

Jetson 저장소와 `drone_uwb` 설치를 갱신했다.
기본 수신과 B_TF는 네 앵커를 요구한다.
FC 위치 전달은 비활성 설정을 유지했다.
시동·비행·목표 명령은 보내지 않았다.
적용 시각은 `2026-10-07T10:43:42+09:00`다.

| 항목 | 확인값 |
|---|---|
| 장치 | `pgyxn@100.110.163.94`, `user-desktop` |
| 저장소 | `/home/pgyxn/github/ROS2` |
| 현재 브랜치 | `feature_uwb` |
| 적용 전 커밋 | `7065fcde62a6cb4171e85ed7525364b392706ae7` |
| 적용한 기능 커밋 | `e1461a6d4468a28a84b3ce70431eeb5341cd388a` |
| 로컬 main | 같은 기능 커밋으로 순방향 갱신 |
| origin 두 브랜치 | fetch 시 같은 기능 커밋 확인 |

적용한 기능은 다음 세 커밋이다.
이 기록을 담는 문서 커밋은 그 뒤에 이어진다.

| 작업 ID | 커밋 |
|---|---|
| UWB-REAL-BTF | `032e115b04c6f7662c5db1f066fd219bf07b11bb` |
| UWB-A123-DIAG | `c8c179ed12f54a3ba3ad8360189a3a9a4004aafb` |
| UWB-FOUR-RESTORE | `e1461a6d4468a28a84b3ce70431eeb5341cd388a` |

## 기존 변경과 기록 보존

수정·미추적 파일 121개를 적용 전에 백업했다.
추적 파일 수정은 7개다.
미추적 파일은 114개다.
압축 안의 파일 해시를 원본과 대조했다.
적용 전에 원본이 바뀌지 않았음도 확인했다.

백업 경로는 다음과 같다.

```text
/home/pgyxn/github/ROS2_sync_backups/20261007T013912980764Z
```

`manifest.json`에 파일별 SHA-256을 기록했다.
`tracked.patch`와 `staged.patch`도 보존했다.
`refs.txt`와 `reflog.txt`에 Git 상태를 보존했다.
압축본은 `working_files.tar.gz`다.
압축 크기는 11,722,309바이트다.
압축본 SHA-256은 다음과 같다.

```text
48f610aa8032fd45a6e1613e023ba75d20c8b35f29d13b1a4ba2a63d3d228567
```

Git stash도 별도로 보존했다.
stash 커밋은 `890b7711e9e77674f20568ce994364c545df32eb`다.
강제 reset·clean·기록 삭제를 사용하지 않았다.

| 비교 대상 | 처리 결과 |
|---|---|
| GitHub와 같은 파일 76개 | 원본 바이트 일치 확인 |
| 그중 원시 기록 58개 | 커밋 반영 뒤에도 원본 해시 일치 |
| 내용이 다른 문서 5개 | 후속 정리 내용을 대조해 반영 |
| Jetson에만 있는 기록 40개 | 원래 경로로 복원하고 해시 확인 |

다른 문서는 README·roadmap·A123 절차·두 결과 문서다.
후속 설명과 줄바꿈 차이를 확인했다.
이전 문서 원본도 백업·stash에 보존했다.
충돌 해결용 소스 변경은 필요하지 않았다.

Jetson 전용 원본은 다음 다섯 기록 묶음이다.
평가 결과나 커밋 대상으로 새로 확정하지 않았다.
원래 위치에 미추적 상태로 남겼다.

| `data/raw/` 아래 폴더 | 보존 범위 |
|---|---|
| `a123_P1_r1_20261005_185944` | 준비 중 제외 기록 |
| `axis_x_test_20261005_172411` | 기존 원본 |
| `square_center_20261005_173744` | 기존 원본 |
| `static_retest_20261005_171658` | 기존 원본 |
| `static_retest_20261005_171802` | 기존 원본 |

## 빌드와 설정 검사

Jetson에서 패키지 빌드와 관련 검사 134개를 통과했다.
빌드 명령은 다음과 같다.

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select drone_uwb
source install/setup.bash
PYTHONPATH=src/drone_uwb:src/drone_demo \
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONIOENCODING=utf-8 \
python3 -m pytest src/drone_uwb/test/processing src/drone_uwb/test/experiments -q
```

| 검사 | 결과 |
|---|---|
| colcon 패키지 빌드 | 1개 패키지 통과, 6.62초 |
| 처리·알고리즘 pytest | 134개 통과, 15.11초 |
| ROS 모듈·B_TF 모듈 import | 3개 통과 |
| ROS 연결부·launch·도구 구문 | 20개 파일 통과 |
| ROS 실행 파일 설치 | `uwb_node`, `uwb_btf_node` 등 확인 |
| 소스·설치된 runtime 설정 | 아래 값 모두 일치 |

| 적용값 | 소스 | 설치본 |
|---|---|---|
| 기본 `min_anchors` | 4 | 4 |
| 기본 `active_anchor_mask` | 15 | 15 |
| B_TF `required_anchor_count` | 4 | 4 |
| FC bridge `enabled` | false | false |
| B_TF `external_output_allowed` | false | false |
| 별도 A123 진단 설정 | 3개·마스크 7 | 3개·마스크 7 |

앵커 지도는 6.3×4.6m, 높이 2.2m다.
사용자 확인 장착 간격 12cm를 유지했다.
거리 편향 보정은 아직 미교정 상태다.
A123는 별도 진단 설정으로만 남겼다.
실행하지 않은 수신을 검사 통과로 계산하지 않았다.

빌드·검사 로그와 판정 JSON은 백업 폴더에 있다.
`build.log`, `pytest.log`, `validation.json`을 따른다.
복원·대조 결과는 `application.json`에 있다.

## 실행 상태와 남은 실물 확인

이 저장소의 UWB 수신·기록 실행은 없었다.
기존 실행을 종료하지 않았다.
다른 작업 경로의 PX4 observer를 유지했다.
확인 시 `/dev/pixhawk`는 존재했다.
`/dev/uwb`는 존재하지 않았다.
직렬 장치에 새 수신기를 실행하지 않았다.

| 남은 확인 | 상태 |
|---|---|
| UWB 장치 연결·`/dev/uwb` 복구 | 필요 |
| 실제 A1~A4 응답 | 미실시 |
| 새 네 앵커·ToF·자세 관측 기록 | 미실시 |
| 다점 거리 교정·독립 위치 정확도 | 미실시 |
| FC 지상 융합·시각 및 좌표 정렬 | 미실시 |
| 실물 위치 유지·목표 이동·비행 | 미실시 |

다음 수신 시험은 [복원 절차](../runbooks/uwb_four_anchor_restore_20261007.md)를 따른다.
저장소 적용 완료와 A4 복구를 구분한다.
이번 작업에서 FC 시동 상태를 실시간 측정하지 않았다.
과거 P1 오차와 관측 공백은 계속 미해결 근거로 남긴다.
