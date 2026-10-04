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

목표값(임시, 튜닝 가능 — params_file로 덮어쓸 수 있다):
    target_distance_m = 0.25  (2026-10-04 변경. 근거는 아래 "목표거리 재검토" 참조)
    lateral_tolerance_m = 0.03
    distance_tolerance_m = 0.02

목표거리 재검토(2026-10-04, 0.20 → 0.25):
    제약이 바뀌었다. 예전에는 ArUco 검출 하한이 목표거리를 막았지만, 지금은
    QR 프레이밍 하한이 막는다. 마커가 159mm에서 100mm로 작아지면서 ArUco 하한이
    충분히 내려갔기 때문이다.

    기준값 (camera_imx219_calib.yaml fx=1098.86 fy=1095.37, 1640x1232):

    | 제약             | 거리   | 산출 근거                                     |
    |------------------|--------|-----------------------------------------------|
    | ArUco 검출 하한  | 11.9cm | 마커+여백 1모듈(133.3mm)이 화면 높이 1232px에 |
    | QR 프레이밍 하한 | 21.1cm | QR 바깥 끝 157.5mm가 화면 반폭 820px에        |
    | QR 판독 상한     | 31.1cm | 현재 라벨 53모듈, 모듈당 4px                  |

    QR 프레이밍 하한 157.5mm = ArUco↔QR 중심간격 123mm(예시 PDF 레이아웃 실측)
    + QR 본체 절반 30mm + 정숙구역 4모듈 4.5mm. 즉 ArUco를 화면 중앙에 정렬한
    상태에서 20cm는 QR 우측이 화면 밖으로 잘린다 — 20cm가 성립 불가능한 이유다.

    0.25(25cm)는 21.1~31.1cm 구간의 가운데다. distance_tolerance_m=0.02를 더해도
    23~27cm라 양끝에 여유가 남는다. 20cm 때 허용 하한이 18cm로 하한선에 붙어
    있던 것과 다르다.

    모듈 수는 payload 길이와 ECC 레벨이 함께 정한다 — 둘 다 상한을 움직인다.
    현재 라벨(125B)은 53모듈로 봤다: 실측 QR 폭 379px ÷ 모듈 7.2px = 52.6이고,
    사양표상 125B가 정확히 들어가는 조합이 version 9(53모듈, ECC Q)다.
    같은 125B라도 ECC M이면 49모듈이 되어 상한이 33.6cm로 올라간다.
    길어지는 쪽이 위험하다 — 251B(61모듈)면 27.0cm까지 내려와 25cm의 위쪽
    여유가 사라진다. 재고 스키마에 필드를 더하거나 라벨을 재인쇄할 때는
    모듈 수를 먼저 세고 이 표를 다시 계산할 것.

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
