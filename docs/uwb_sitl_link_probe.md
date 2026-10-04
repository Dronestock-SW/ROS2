# PX4 SITL MAVLink 읽기 점검

이 문서는 UWB 관측을 보내기 전 PX4 링크와 설정을 읽는 절차다.
사용자 WSL의 `pymavlink` 연결을 확인할 때 읽는다.

## 현재 결론

`sitl_link_probe`는 PX4 SITL의 온보드 UDP 수신 포트 `14540`을 연다.
PX4 heartbeat를 받은 뒤 필요한 파라미터만 요청한다.
파라미터 변경, `ODOMETRY` 송신, 시동·비행 명령은 하지 않는다.

PX4의 사용 중인 [MAVLink 포트 안내](https://docs.px4.io/main/en/simulation/px4_sitl_prebuilt_packages)는
온보드 API의 원격 포트를 `14540`, PX4 로컬 포트를 `14580`으로 설명한다.
사용자 `mavlink status`의 instance #1 출력도 이 값과 일치했다.
[pymavlink 공식 사용 안내](https://mavlink.io/en/mavgen_python/)는
`udpin:localhost:14540`으로 SITL 메시지를 받는 예를 제공한다.

## WSL 실행

PX4·Gazebo를 실행 중인 WSL Ubuntu의 새 창에서 입력한다.
다른 프로그램이 `14540` 포트를 쓰고 있다면 먼저 해당 프로그램을 확인한다.
시험 결과마다 새 출력 파일명을 사용한다.
[현재 누적본 반영 절차](uwb_gazebo_navigation_runbook.md#갱신본-반영)를 먼저 따른다.
아래 명령은 누적본의 프로젝트 루트에서 실행한다.

```bash
export PYTHONPATH="$PWD/src/drone_uwb${PYTHONPATH:+:$PYTHONPATH}"
~/.venvs/px4/bin/python -m drone_uwb.integration.sitl_link_probe \
  --output runs/sitl_link_probe_20260928_01.json \
  --duration-s 15
```

먼저 `pymavlink import completed`가 나오는지 확인한다.
그다음 `status`가 `complete`인지 본다.
`no_heartbeat`이면 PX4 온보드 링크나 포트 점유를 확인한다.
`partial`이면 `missing`에 응답받지 못한 파라미터가 남는다.
`probe_error`와 `user_stopped`도 같은 JSON 파일에 사유를 기록한다.

조회 항목은 `EKF2_EV_CTRL`, `EKF2_EV_NOISE_MD`, `EKF2_EV_DELAY`,
`EKF2_EV_POS_X/Y/Z`, `EKF2_EVP_NOISE`, `EKF2_GPS_CTRL`,
`EKF2_OF_CTRL`, `EKF2_RNG_CTRL`, `EKF2_HGT_REF`다.
SITL 외부 위치 관측 설정과 다른 위치원을 한 번에 대조한다.

PX4는 정수형 `PARAM_VALUE`의 값을 float 필드에 비트 그대로 복사한다.
따라서 `raw_float` 숫자를 정수로 반올림하지 않는다.
도구는 `mav_param_type`에 따라 비트를 복원해 `value`를 기록한다.
원본 비트는 `raw_bits_hex`에 남긴다.
[사용자 PX4 커밋의 구현](https://github.com/PX4/PX4-Autopilot/blob/c4e4ef98e9d75063bf3d53ebb2716221ee7505ae/src/modules/mavlink/mavlink_parameters.cpp)을
기준으로 했다.

## 확인 범위와 다음 단계

단위 시험 3개에서 정수 비트 복원과 읽기 요청만 발생하는 경로를 확인했다.
2026-10-02 사용자 WSL에서 `pymavlink` 가져오기가 성공했다.
가상환경 `~/.venvs/px4/bin/python`의 종료 코드는 0이다.
같은 시점의 PX4 프로세스·Gazebo 토픽 조회는 무출력이었다.
UDP 수신·파라미터 응답은 아직 미실시다.
이 도구가 `complete`를 반환해도 PX4의 비행 준비나 UWB 융합은 입증되지 않는다.
QGroundControl의 Arming Check Report와 이후 외부 관측 수신·EKF 사용 상태를 별도 확인한다.
