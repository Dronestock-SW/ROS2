"""ArUco 정렬 오차 계산 노드 — Dronestock 1호기 (Phase 3, 비주얼 서보잉 1단계).

/aruco_detections를 구독해 "마커에 정렬하려면 얼마나 움직여야 하는지" 오차값을
계산해서 /aruco_alignment/error 로 발행한다.

아직 PX4/MAVROS로는 아무것도 안 보낸다 — 계산 로직이 맞는지 로그·토픽으로
먼저 검증하는 단계다(2026-09-13 실측: ArUco 검출+pose는 확인됐지만 그 값을
실제 이동 명령으로 바꾸는 코드는 없었음). 마커를 손으로 움직여보면서 오차값이
말이 되는지 확인한 뒤에야 다음 단계(MAVROS setpoint 연결)로 넘어간다.

좌표계:
    카메라가 드론 정면에 장착돼 있다(equipment_inventory.md, position: front).
    ArUco pose의 x(좌우)·z(거리)만 쓴다. y(상하)는 다루지 않는다 — companion은
    z(고도) 직접 제어 금지(altitude_policy.md)다. 고도·자세는 PX4가 담당한다.

    lateral_error 부호는 정상이다(2026-10-04 확인). 그 전까지 711표본 연속
    음수만 나와 카메라 장착 오프셋을 의심했으나, 라벨을 화면 오른쪽으로 크게
    옮기자 +1.27~+3.50cm가 32표본 나왔다. 사람이 늘 광축 왼쪽에서 라벨을 들어
    생긴 편향이었다 — 코드 문제가 아니다.

목표값(임시, 튜닝 가능 — params_file로 덮어쓸 수 있다):
    target_distance_m = 0.25  (2026-10-04 변경. 근거는 아래 "목표거리 재검토" 참조)
    lateral_tolerance_m = 0.03
    distance_tolerance_m = 0.02

목표거리 재검토(2026-10-04, 0.20 → 0.25):
    제약이 바뀌었다. 예전에는 ArUco 검출 하한이 목표거리를 막았지만, 지금은
    QR 프레이밍 하한이 막는다. 마커가 159mm에서 100mm로 작아지면서 ArUco 하한이
    충분히 내려갔기 때문이다.

    기준값 (camera_imx219_calib.yaml fx=1098.86 fy=1095.37, 1640x1232):

    | 제약             | 거리     | 산출 근거                                   |
    |------------------|----------|---------------------------------------------|
    | ArUco 검출 하한  | 11.9cm   | 마커+여백 1모듈(133.3mm)이 화면 높이 1232px에|
    | QR 프레이밍 하한 | 21.1cm   | QR 바깥 끝 157.5mm가 화면 반폭 820px에       |
    | QR 판독 상한     | 53.6cm   | 실측(2026-10-04). 계산값 아님 — 아래 참조    |

    QR 프레이밍 하한 157.5mm = ArUco↔QR 중심간격 123mm(예시 PDF 레이아웃 실측)
    + QR 본체 절반 30mm + 정숙구역 4모듈 4.5mm. 즉 ArUco를 화면 중앙에 정렬한
    상태에서 20cm는 QR 우측이 화면 밖으로 잘린다 — 20cm가 성립 불가능한 이유다.

    0.25(25cm)는 21.1~53.6cm 구간의 아래쪽이다. 하한 21.1cm가 유일한 실질
    제약이고, distance_tolerance_m=0.02를 더한 허용 하한 23cm가 거기서 1.9cm
    떨어져 있다. 상한은 25cm에서 2배 넘게 떨어져 있어 신경 쓸 필요가 없다.

    QR 판독 상한 실측(2026-10-04, 실제 데이터 라벨 156B, 손에 들고 3회 스윕
    2579표본):
        53.6cm까지 판독됐고 54cm 이상에서 0%였다. 44~46cm 정지 구간(198표본)
        판독률 95%.
        계산 기준이던 "모듈당 4px"는 실제보다 보수적이다 — 53.6cm를 역산하면
        모듈당 2.0~2.3px다. 4px로 잡으면 상한이 31cm로 나와 실측의 60%밖에
        안 된다. 이 기준으로 설계 판단을 하지 말 것.

        단, 이 스윕에서 **거리보다 라벨 기울기가 판독을 더 좌우했다**.
        37cm 정지 구간(190표본)이 9%인데 45cm 정지 구간(198표본)이 95%였다 —
        거리 역전이다. 손으로 든 라벨의 기울기·반사 탓이다. 그래서 53.6cm는
        "이 거리까지 읽힌 적이 있다"는 하한 보장이지 거리 한계 자체가 아니다.
        거리만의 한계를 재려면 라벨을 고정 거치대에 세워야 한다.

    모듈 수는 payload 길이와 ECC 레벨이 함께 정한다. 현재 라벨은 156B이고
    실측 모듈 수가 53(ECC M)과 61(ECC Q) 사이 56으로 나와 확정하지 못했다.
    상한을 실측으로 잡은 이상 이 값은 설계에 쓰지 않는다.

    예비책(지금은 불필요, 기록만):
        정렬 목표를 ArUco 중심이 아니라 라벨 중심으로 옮기면 하한이 약 17.2cm로
        내려간다(lateral_error에서 123mm의 절반인 61.5mm를 빼면 된다). 이때는
        제약이 QR이 아니라 ArUco 쪽으로 넘어간다 — 라벨 중심 기준으로 ArUco
        바깥 끝이 128.2mm라 그쪽이 먼저 화면을 벗어난다.

    미확인: 위 값은 전부 기하 계산이고, 사람이 든 라벨 기준 실측도 아직 없다.
    드론 호버링 안정성(프롭워시 포함)은 비행 실측 전이다 — 기대만큼 안정적이지
    않으면 중첩 마커(큰 마커 안에 작은 마커) 방식으로 전환 검토 필요.

실행:
    ros2 run drone_bringup aruco_alignment_node
"""

import json
import math

import rclpy
from aruco_opencv_msgs.msg import ArucoDetection
from rclpy.node import Node
from std_msgs.msg import String


class ArucoAlignmentNode(Node):

    def __init__(self):
        super().__init__('aruco_alignment_node')

        self.declare_parameter('target_distance_m', 0.25)
        self.declare_parameter('lateral_tolerance_m', 0.03)
        self.declare_parameter('distance_tolerance_m', 0.02)
        self.declare_parameter('marker_id', -1)  # -1이면 처음 보이는 마커 아무거나

        self._target_distance = self.get_parameter('target_distance_m').value
        self._lateral_tol = self.get_parameter('lateral_tolerance_m').value
        self._distance_tol = self.get_parameter('distance_tolerance_m').value
        self._marker_id = self.get_parameter('marker_id').value

        self._sub = self.create_subscription(
            ArucoDetection, '/aruco_detections', self._on_detection, 10)
        self._pub = self.create_publisher(String, '/aruco_alignment/error', 10)

        self._miss_streak = 0

        self.get_logger().info(
            f'aruco_alignment_node 시작 — target_distance={self._target_distance}m, '
            f'lateral_tol=±{self._lateral_tol}m, distance_tol=±{self._distance_tol}m')

    def _on_detection(self, msg):
        marker = self._pick_marker(msg.markers)
        if marker is None:
            self._miss_streak += 1
            if self._miss_streak % 30 == 0:
                self.get_logger().warn(f'마커 {self._miss_streak}프레임 연속 미검출')
            self._publish_invalid('marker_missing')
            return
        self._miss_streak = 0

        lateral_error = marker.pose.position.x
        distance_error = marker.pose.position.z - self._target_distance
        if not (math.isfinite(lateral_error) and math.isfinite(distance_error)
                and marker.pose.position.z > 0):
            self._publish_invalid('invalid_pose')
            return
        aligned = (abs(lateral_error) <= self._lateral_tol
                   and abs(distance_error) <= self._distance_tol)

        payload = {
            'valid': True,
            'marker_id': marker.marker_id,
            'lateral_error_m': round(lateral_error, 4),
            'distance_error_m': round(distance_error, 4),
            'aligned': aligned,
        }
        self._pub.publish(String(data=json.dumps(payload)))
        self.get_logger().info(
            f'id={marker.marker_id} 좌우오차={lateral_error:+.3f}m '
            f'거리오차={distance_error:+.3f}m aligned={aligned}')

    def _publish_invalid(self, reason):
        self._pub.publish(String(data=json.dumps({
            'valid': False, 'aligned': False, 'reason': reason,
            'marker_id': self._marker_id,
        })))

    def _pick_marker(self, markers):
        if not markers:
            return None
        if self._marker_id < 0:
            return markers[0]
        for m in markers:
            if m.marker_id == self._marker_id:
                return m
        return None


def main(args=None):
    rclpy.init(args=args)
    node = ArucoAlignmentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # ros2 launch 의 SIGINT 는 rclpy 가 먼저 shutdown 해서, 무조건 부르면
        # "rcl_shutdown already called" 로 exit code 1 이 된다.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
