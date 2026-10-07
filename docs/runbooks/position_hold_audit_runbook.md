# Position 위치 유지 로그 분석 절차
이 문서는 ULog의 수평 위치 유지 분석 절차다.
새 제어 업데이트의 결과를 비교할 때 읽는다.

## 준비

원본 ULog와 별도의 출력 경로를 준비한다.
도구는 ROS 2·MAVROS·기체에 연결하지 않는다.

| 의존성 | 용도 |
|---|---|
| Python 3·NumPy | 파일 처리·수치 계산 |
| [PX4 pyulog](https://github.com/PX4/pyulog) | ULog 해석 |
| Matplotlib | `--plot` 사용 시 그림 저장 |

pyulog 설치 환경이 있으면 그대로 사용한다.
소스로 사용할 때는 해당 경로를 `PYTHONPATH`에 넣는다.
9월 27일 분석은 아래 커밋을 사용했다.

```bash
git clone https://github.com/PX4/pyulog.git /tmp/dronestock-pyulog
git -C /tmp/dronestock-pyulog checkout ffbe3755d903e93797a89fb4fce26e0c0420a42f
```

## 실행

workspace 루트에서 독립 도구를 실행한다.
출력 폴더는 새 경로를 사용한다.
이유: 기존 시험 결과를 덮어쓰지 않기 위해서다.

```bash
PYTHONPATH=/tmp/dronestock-pyulog \
MPLCONFIGDIR=/tmp/dronestock-matplotlib \
python3 src/drone_bringup/tools/analyze_position_hold.py \
  log_23_2026-9-27-13-34-50.ulg \
  --output-dir /tmp/position_hold_review \
  --plot
```

## 결과 확인

먼저 모드와 계산 표본 수를 확인한다.
Position 구간이 없으면 빈 결과가 정상이다.

| 출력 | 읽을 내용 |
|---|---|
| `summary.json` | 모드 구간·오차·속도·융합·파라미터·사건 |
| `samples.csv` | 과거 표본으로 연결한 현재 위치·목표·RC |
| `position_hold.png` | 위치 궤적·목표 편차·속도·스틱 |

CSV의 `qualified`는 기본 표본 조건이다.
구간 통계는 모드 진입 전 목표도 추가 제외한다.
`position_hold_candidate`는 분석용 분류다.
비행 허용이나 안전 판정을 내리는 값이 아니다.
새 펌웨어에서는 모드 번호 정의도 대조한다.
기록 주기가 달라지면 분석용 최대 나이를 검토한다.
수치를 바꾸면 별도 실행으로 기록한다.

오차는 PX4 EKF와 목표 사이 차이다.
실측 정확도는 외부 기준 위치로 따로 평가한다.
센서의 기록 Hz를 실제 측정 Hz로 단정하지 않는다.

[9월 27일 결과](../report/position_hold_20260927.md)에 최초 기준을 남겼다.
