# Jetson 개발 실행 환경
이 문서는 적용한 Python·ROS·C++ 실행 환경을 설명한다.
개발 명령 실행·환경 재설치·미완 항목 확인 때 읽는다.

상태: 2026-10-04 SSH 적용·개발 검사 통과.
대상: arialhanho@100.110.163.94, Ubuntu 22.04 aarch64.
경로: `/home/arialhanho/Desktop/ROS2/src/sangwon_AI`.
환경 검사는 실기체 preflight 승인을 발급하지 않는다.

## 1. 적용 결과

기존 시스템 라이브러리를 유지하고 보조 환경을 추가했다.

| 항목 | 결과 |
|---|---|
| 시스템 | ROS Humble, MAVROS 2.14, OpenCV 4.5.4, GCC 11.4, CMake 3.22 |
| `.venv` | 시스템 Python 3.10 기반. ROS·OpenCV·cv_bridge·NumPy 1.21.5 사용 |
| 추가 패키지 | pip 25.2, websockets 15.0.1, cffi 1.17.1, pycparser 2.22 |
| `.venv-log` | 로그 분석 전용. pyulog 1.2.2, NumPy 1.26.4, pip 25.2 |
| 공통 환경 | ROS setup·자체 C++ 설치 overlay·ROS_DOMAIN_ID=1·Python 환경 |
| 설치 반복 | 기존 환경을 지우지 않고 재실행 성공 |
| 검사 | ROS·영상·스캐너 라이브러리 import, 영상 메시지 왕복, 로그 환경 import 통과 |
| 기존 시험 | CTest 5/5 통과. 모의 비행·감시기·호스트 검사 |
| 자동 진단 | 기존 사용자 서비스 active 확인. 실행 인터프리터는 시스템 Python 유지 |

pyulog 1.2.2는 Python 3.10에서 NumPy >=1.25를 요구한다.
ROS 영상 환경의 NumPy와 분리해 설치했다.
ULog 파싱은 별도 프로세스의 `.venv-log`로 실행한다.
주 환경에 pyulog와 새 NumPy를 추가 설치하지 않는다.
이유는 ROS 영상 바이너리와의 의존성 충돌을 피하기 위해서다.
시스템 PyNaCl의 누락 cffi 의존성은 `.venv` 안에서 보완했다.
두 환경 모두 `pip check`가 통과했다.

`.venv`는 시스템 패키지를 읽으므로 완전한 독립 환경은 아니다.
apt/ROS 업데이트 뒤 같은 검사를 다시 실행한다.
추가 패키지 버전·wheel SHA256은 requirements에 고정했다.
비밀키·실제 웹 주소·UWB 통신값은 아직 넣지 않았다.

## 2. 사용 방법

터미널에서는 환경을 불러온 뒤 작업한다.

```bash
cd /home/arialhanho/Desktop/ROS2/src/sangwon_AI
source deployment/env.sh
python ops/env_doctor.py
ctest --test-dir .build/colcon-build/sangwon_ai_replay --output-on-failure
```

비대화형 명령은 공통 실행기를 쓴다.

```bash
bash deployment/run_env.sh python ops/env_doctor.py
bash deployment/run_env.sh sangwon_replay --scenario nominal
```

이 스크립트는 명령의 실행 환경만 준비한다.
센서·MAVROS·모터·자율 임무를 자동 시작하지 않는다.
기존 ROS_DOMAIN_ID가 1이 아니면 덮어쓰지 않고 거부한다.
별도 기체에 배포할 때는 도메인 정책을 개정한다.

로그 분석 예시:

```bash
env -u PYTHONPATH -u PYTHONHOME .venv-log/bin/python -c 'from pyulog import ULog; print("ULog parser available")'
```

## 3. 재현과 근거

설치는 관리자 권한 없이 우리 디렉터리에서 수행한다.

```bash
/usr/bin/python3 ops/setup_python_env.py
```

시스템에 ensurepip가 없어 `venv --without-pip` 방식으로 만들었다.
고정 pip wheel의 SHA256을 검사하고 각 환경에만 설치한다.
외부 Python 패키지는 PyPI wheel·고정 해시로 검증한다.
원격 설치 셸 스크립트를 다운로드해 실행하지 않는다.
두 venv에 COLCON_IGNORE를 두어 패키지 탐색에서 제외한다.
생성 환경·wheel 캐시는 Git에서 제외한다.

| 파일 | 역할 |
|---|---|
| [설치기](ops/setup_python_env.py) | venv·고정 pip·패키지 설치 |
| [ROS 환경](deployment/env.sh) | source 진입점 |
| [실행기](deployment/run_env.sh) | 명령·향후 서비스용 공통 환경 |
| [개발 의존성](deployment/requirements-jetson.txt) | ROS 보조 패키지·해시 |
| [로그 의존성](deployment/requirements-log.txt) | ULog 분석 패키지·해시 |
| [환경 검사](ops/env_doctor.py) | 읽기 전용 검사, 비행 권한 없음 |
| [검사 결과](reports/environment_2026-10-04.json) | 당시 버전·import·장치·권한 증거 |

공식 근거: [Python venv](https://docs.python.org/3.10/library/venv.html).
패키지 근거: [pyulog 1.2.2](https://pypi.org/pypi/pyulog/1.2.2/json).

## 4. 아직 남은 준비

개발 환경과 전체 기체 운영 환경을 구분한다.

| 항목 | 상태·다음 조치 |
|---|---|
| 직렬/HID 권한 | dialout 미가입. 기존 udev는 해당 그룹을 사용 |
| 관리자 명령 | `sudo -n` 실패. 비밀번호 필요, 자동 적용 못 함 |
| 장치 | Pixhawk/LiDAR/UWB/QR 장치 별칭 미관측 |
| 실제 웹 | 시험 서버·인증·계약 회신 필요 |
| UWB | 전체 사양 수령 뒤 어댑터·연결 방식 결정 |
| ArUco·QR | 설치 라이브러리와 실제 센서 데이터·미세 조정 연동은 별도 |
| 전체 자동 기동 | 현재 호스트 진단만 적용. 운용용 노드·서비스와 콜드 부팅 시험 필요 |

관리자는 Jetson 터미널에서 다음을 적용할 수 있다.

```bash
sudo usermod -aG dialout arialhanho
```

새 세션과 서비스에 그룹이 반영됐는지 재확인한다.
기존 SSH·측정 작업을 임의 종료하거나 재부팅하지 않았다.
실기체 포트·PX4 파라미터·RC 설정은 변경하지 않았다.
