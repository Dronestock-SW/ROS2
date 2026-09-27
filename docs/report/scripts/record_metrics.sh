#!/usr/bin/env bash
# 성능 지표 측정용 rosbag 기록 스크립트 — Dronestock SW
#
# 목적: 03_metrics_plan.md의 "지금 측정 가능한 지표" 2종을 기록한다.
#         - 위치 추정 갱신 주기
#         - 노드 처리 지연 (발행 -> 기록 구간)
#
# 이 스크립트는 기록만 한다. 값 산출은 analyze_metrics.py 가 한다.
# 사용법은 같은 폴더의 README.md 를 본다.

set -euo pipefail
export LC_ALL=C.UTF-8
export LANG=C.UTF-8

# ---- 기본값 --------------------------------------------------------------
DURATION=120
PROFILE="light"
OUTDIR="$HOME/drone_ws/metrics_bags"
LABEL=""
CPULOG=1

usage() {
  cat <<'USAGE'
사용법: ./record_metrics.sh [옵션]

옵션:
  -d, --duration <초>    기록 길이. 기본 120 (full 프로파일은 기본 20)
  -p, --profile <이름>   light | full. 기본 light
                           light : /scan, /camera/camera_info, /uwb_pose
                                   메시지가 작아 장시간 기록에 적합
                           full  : light + /camera/image_raw
                                   ★ 초당 약 182 MB. 20초면 약 3.6 GB
  -o, --outdir <경로>    bag 저장 상위 폴더. 기본 ~/drone_ws/metrics_bags
  -l, --label <문자열>   bag 이름에 붙일 꼬리표 (예: solo, allnodes)
      --no-cpulog        CPU 사용률 동시 기록 안 함
  -h, --help             이 도움말

예:
  ./record_metrics.sh -d 120 -l solo
  ./record_metrics.sh -p full -d 20 -l cam_raw
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -d|--duration) DURATION="$2"; shift 2 ;;
    -p|--profile)  PROFILE="$2";  shift 2 ;;
    -o|--outdir)   OUTDIR="$2";   shift 2 ;;
    -l|--label)    LABEL="$2";    shift 2 ;;
    --no-cpulog)   CPULOG=0;      shift 1 ;;
    -h|--help)     usage; exit 0 ;;
    *) echo "알 수 없는 옵션: $1" >&2; usage; exit 1 ;;
  esac
done

# ---- ROS 환경 확인 -------------------------------------------------------
if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "[오류] ROS 환경이 소싱되지 않았다." >&2
  echo "       source /opt/ros/humble/setup.bash" >&2
  echo "       source ~/drone_ws/install/setup.bash" >&2
  exit 1
fi
echo "[정보] ROS_DISTRO=${ROS_DISTRO}  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-미설정}"

# ---- 프로파일별 토픽 -----------------------------------------------------
CANDIDATES_LIGHT=(/scan /camera/camera_info /uwb_pose)
CANDIDATES_FULL=(/scan /camera/camera_info /uwb_pose /camera/image_raw)

case "$PROFILE" in
  light) CANDIDATES=("${CANDIDATES_LIGHT[@]}") ;;
  full)
    CANDIDATES=("${CANDIDATES_FULL[@]}")
    # full 프로파일에서 -d 를 안 줬으면 20초로 낮춘다 (용량 폭주 방지)
    if [[ "$DURATION" == "120" ]]; then DURATION=20; fi
    echo "[경고] full 프로파일은 /camera/image_raw 원본을 기록한다."
    echo "       1640x1232 rgb8 = 프레임당 6,061,440 byte."
    echo "       30Hz 기준 초당 약 182 MB → ${DURATION}초면 약 $((DURATION * 182 / 1024)) GB."
    echo "       디스크 여유를 확인할 것. 계속하려면 5초 안에 Ctrl+C 를 누르지 말 것."
    sleep 5
    ;;
  *) echo "[오류] 프로파일은 light 또는 full 이어야 한다: $PROFILE" >&2; exit 1 ;;
esac

# ---- 실제 존재하는 토픽만 추린다 ------------------------------------------
echo "[정보] 발행 중인 토픽 확인..."
AVAILABLE="$(ros2 topic list 2>/dev/null || true)"
TOPICS=()
MISSING=()
for t in "${CANDIDATES[@]}"; do
  if grep -qx -- "$t" <<<"$AVAILABLE"; then TOPICS+=("$t"); else MISSING+=("$t"); fi
done

if [[ ${#MISSING[@]} -gt 0 ]]; then
  echo "[정보] 미발행이라 기록에서 제외한 토픽: ${MISSING[*]}"
  echo "       (미구현 노드는 03_metrics_plan.md 참조. 예: /uwb_pose 는 uwb_node 미작성)"
fi
if [[ ${#TOPICS[@]} -eq 0 ]]; then
  echo "[오류] 기록할 토픽이 하나도 없다. launch 를 먼저 띄울 것." >&2
  echo "       ros2 launch drone_bringup lidar.launch.py" >&2
  echo "       ros2 launch drone_bringup camera.launch.py" >&2
  exit 1
fi
echo "[정보] 기록 대상: ${TOPICS[*]}"

# ---- 출력 경로 -----------------------------------------------------------
STAMP="$(date +%Y%m%d_%H%M%S)"
NAME="metrics_${PROFILE}_${STAMP}"
[[ -n "$LABEL" ]] && NAME="metrics_${PROFILE}_${LABEL}_${STAMP}"
mkdir -p "$OUTDIR"
BAGDIR="${OUTDIR}/${NAME}"

# ---- 기록 조건 메모 (후처리에서 함께 보고서에 싣는다) ---------------------
META="${OUTDIR}/${NAME}_condition.txt"
{
  echo "# 측정 조건 기록"
  echo "기록일시    : $(date -Iseconds)"
  echo "프로파일    : ${PROFILE}"
  echo "기록길이(초): ${DURATION}"
  echo "꼬리표      : ${LABEL:-없음}"
  echo "대상토픽    : ${TOPICS[*]}"
  echo "제외토픽    : ${MISSING[*]:-없음}"
  echo "ROS_DISTRO  : ${ROS_DISTRO}"
  echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-미설정}"
  echo "호스트      : $(uname -n) / $(uname -r)"
  echo
  echo "## 기록 시점에 떠 있던 노드"
  ros2 node list 2>/dev/null || echo "(조회 실패)"
  echo
  echo "## 아래는 사람이 손으로 채운다 (03_metrics_plan.md 8장 참조)"
  echo "앵커 설치 높이 : [확인필요]"
  echo "태그 장착 높이 : [확인필요]"
  echo "측정 위치      : [확인필요]"
  echo "조도           : [확인필요]"
  echo "비고           : "
} > "$META"
echo "[정보] 조건 메모: ${META}"

# ---- CPU 동시 기록 -------------------------------------------------------
CPUFILE="${OUTDIR}/${NAME}_cpu.txt"
CPUPID=""
if [[ "$CPULOG" -eq 1 ]]; then
  ( echo "# 1초 간격 CPU/메모리 (지연 스파이크와의 상관 확인용)"
    top -b -d 1 -n "$((DURATION + 5))" | grep -E "^%Cpu|^MiB Mem" ) > "$CPUFILE" 2>&1 &
  CPUPID=$!
  echo "[정보] CPU 기록: ${CPUFILE}"
fi

# ---- 기록 ----------------------------------------------------------------
# ★ ros2 bag record 의 -d/--max-bag-duration 은 "총 기록 길이"가 아니라
#   "파일 1개당 최대 길이"다 (Humble/rosbag2 0.15.16). 이걸 총 길이로 쓰면
#   기록이 끝나지 않고 파일만 계속 쪼개진다 (2026-08-16 실측 확인).
#   따라서 총 길이는 timeout 으로 끊는다. SIGINT 를 보내야 bag 이 정상 닫힌다.
echo "[정보] ${DURATION}초 기록 시작 → ${BAGDIR}"
set +e
timeout -s INT "${DURATION}s" ros2 bag record -o "$BAGDIR" "${TOPICS[@]}"
RC=$?
# timeout 이 SIGINT 로 끊으면 124(타임아웃) 또는 130(SIGINT)이 나온다. 둘 다 정상 종료다.
if [[ $RC -eq 124 || $RC -eq 130 ]]; then RC=0; fi
set -e
[[ -n "$CPUPID" ]] && { wait "$CPUPID" 2>/dev/null || true; }

if [[ $RC -ne 0 ]]; then
  echo "[오류] ros2 bag record 종료코드 ${RC}" >&2
  exit $RC
fi

echo
echo "[완료] 기록 끝났다."
echo "  bag      : ${BAGDIR}"
echo "  조건메모 : ${META}   ← [확인필요] 항목을 손으로 채울 것"
echo "  CPU      : ${CPUFILE}"
echo
echo "다음 단계:"
echo "  python3 $(dirname "$0")/analyze_metrics.py ${BAGDIR}"
