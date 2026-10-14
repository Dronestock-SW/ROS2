# C++ B_TF 기록 비교 절차
Jetson에서 내부 기록 비용을 비교하는 절차다.
운영 서비스·FC를 바꾸지 않는 재생 시험에 사용한다.

**C++와 입력을 고정하고 ON/OFF만 바꾼다.**
출력 폴더는 새 경로를 사용한다.
원본·현재 실행 소스는 수정하지 않는다.

1. 별도 소스 폴더와 C++ 라이브러리를 준비한다.
   빌드·업로드가 겹치면 먼저 상태를 확인한다.
2. 같은 설정 파일에 C++ 라이브러리 절대 경로를 지정한다.
   `numerical_backend=cpp`를 요구한다.
3. ROS 환경을 로드하고 아래 명령을 실행한다.
4. 네 실행의 입력·설정·라이브러리 해시를 대조한다.
5. 기록 누락·원본 지연·추가 만료를 함께 확인한다.

`SOURCE`는 격리 소스 루트다.
`CAPTURE`는 manifest와 입력이 있는 원본 폴더다.
`CONFIG`는 네 실행에 공통으로 사용할 JSON이다.
`RESULT`는 아직 없는 출력 폴더다.
실제 경로를 지정한 뒤 실행한다.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1
export PYTHONPATH="$SOURCE/src/drone_uwb:$PYTHONPATH"
unset BTF_REPLAY_PROFILE
python3 "$SOURCE/src/drone_uwb/tools/benchmark_btf_recording.py" "$CAPTURE" \
  --events-file events.recovered-uwb-imu-excerpt.jsonl \
  --config-file "$CONFIG" --output "$RESULT"
python3 "$SOURCE/src/drone_uwb/tools/analyze_btf_recording.py" \
  "$CAPTURE/events.recovered-uwb-imu-excerpt.jsonl" \
  "$RESULT/1-on" "$RESULT/2-off" "$RESULT/3-off" "$RESULT/4-on" \
  --output "$RESULT/age-analysis.json"
```

전체 원본이면 `--events-file events.jsonl`을 명시한다.
부분 파일을 전체 원본 이름으로 바꾸지 않는다.
재생 토픽 접두사는 출력 경로별로 분리한다.
DOMAIN_ID 2나 실제 FC 발행자로 연결하지 않는다.
이유: 기록 재생은 실제 비행 입력이 아니다.

OFF에서도 콜백·잠금·계산 시간은 메모리에 쌓는다.
종료 뒤 `probe.json`에 저장한다.
노드·센서·C++ 알고리즘은 각 비교 안에서 동일하다.
기록 큐 오류나 계측 누락이 있으면 별도로 보고한다.
발행 도구 지연도 `summary.json`에서 확인한다.

원본 시각 나이와 새 콜백 대기를 분리해 읽는다.
두 번씩의 비교를 장기 보장으로 표현하지 않는다.
I/O wait는 호스트 전체 값이다.
다른 프로세스의 부하까지 기록 탓으로 계산하지 않는다.
