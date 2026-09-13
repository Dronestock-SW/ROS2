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
    z(고도) 직접 제어 금지(altitude_policy.md)라서, 상하 정렬은 이 노드 밖에서
    별도로(실측 길이 ÷ 시간 = 하드코딩된 값) 처리하기로 함.

목표값(임시, 튜닝 가능 — params_file로 덮어쓸 수 있다):
    target_distance_m = 0.20  (2026-09-13 변경. 근거는 아래 "목표거리 재검토" 참조)
    lateral_tolerance_m = 0.03
    distance_tolerance_m = 0.02

목표거리 재검토(2026-09-13, 0.175 → 0.20):
    159mm 마커 기준 d_min(검출에 필요한 최소거리, 여백 1모듈 가정) ≈ 18.8cm임을
    calib fy로 계산 후 실측 확인함(카메라 앞에서 마커를 천천히 접근시켜 검출
    끊기는 지점 관찰). 손 떨림 있는 상태로는 18~18.6cm 부근에서 검출이
    간헐적으로 끊겼고, 손 떨림 최소화하니 18.6cm까지는 끊김 없이 검출됨.
    즉 17.5cm 목표는 d_min보다 가까워 애초에 성립 불가능했다.
    0.20(20cm)은 QR 안정 상한(15~20cm의 위쪽 끝)과 ArUco 안정 하한(18.6~18.8cm)이
    겹치는 지점이라 골랐다 — distance_tolerance_m=0.02를 감안해도 허용 하한이
    18cm라 여전히 타이트하다. 이 값은 사람 손으로 든 마커 기준이고, 실제 드론
    호버링 안정성(프롭워시 포함)은 아직 비행 실측 전이라 미확인 — 이게 기대만큼
    안정적이지 않으면 중첩 마커(큰 마커 안에 작은 마커) 방식으로 전환 검토 필요.

실행:
    ros2 run drone_bringup aruco_alignment_node
"""

import json

import rclpy
from aruco_opencv_msgs.msg import ArucoDetection
from rclpy.node import Node
from std_msgs.msg import String


class ArucoAlignmentNode(Node):

    def __init__(self):
        super().__init__('aruco_alignment_node')

        self.declare_parameter('target_distance_m', 0.20)
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
            return
        self._miss_streak = 0

        lateral_error = marker.pose.position.x
        distance_error = marker.pose.position.z - self._target_distance
        aligned = (abs(lateral_error) <= self._lateral_tol
                   and abs(distance_error) <= self._distance_tol)

        payload = {
            'marker_id': marker.marker_id,
            'lateral_error_m': round(lateral_error, 4),
            'distance_error_m': round(distance_error, 4),
            'aligned': aligned,
        }
        self._pub.publish(String(data=json.dumps(payload)))
        self.get_logger().info(
            f'id={marker.marker_id} 좌우오차={lateral_error:+.3f}m '
            f'거리오차={distance_error:+.3f}m aligned={aligned}')

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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
