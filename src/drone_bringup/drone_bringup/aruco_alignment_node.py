"""
ArUco 정렬 오차 계산 노드 — Dronestock 1호기 (Phase 3, 비주얼 서보잉 1단계).

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
    | QR 프레이밍 하한 | 21.1cm   | 계산. 실측 21.2cm로 검증됨 — 아래 참조       |
    | QR 판독 상한     | 47.4cm   | 실측(2026-10-04). 계산값 아님 — 아래 참조    |

    QR 프레이밍 하한 157.5mm = ArUco↔QR 중심간격 123mm(예시 PDF 레이아웃 실측)
    + QR 본체 절반 30mm + 정숙구역 4모듈 4.5mm. 즉 ArUco를 화면 중앙에 정렬한
    상태에서 20cm는 QR 우측이 화면 밖으로 잘린다 — 20cm가 성립 불가능한 이유다.

    0.25(25cm)는 21.1~47.4cm 구간의 아래쪽이다. 하한 21.1cm가 유일한 실질
    제약이고, distance_tolerance_m=0.02를 더한 허용 하한 23cm가 거기서 1.9cm
    떨어져 있다. 상한은 25cm에서 2배 넘게 떨어져 있어 신경 쓸 필요가 없다.

    QR 판독 상한 실측(2026-10-04, 실제 데이터 라벨 156B):
        라벨을 거치해 기울기를 9~10도로 고정하고 2회 스윕 3250표본을 쟀다.

        | 거리      | 표본  | 판독률 |
        |-----------|-------|--------|
        | 27~47cm   | 1800+ | 85~100%|
        | 47.4cm    | —     | 판독 성공 최대 |
        | 53~65cm   | 1342  | 0%     |

        47.4~53cm는 측정하지 않았다 — 상한이 목표 25cm에서 2배 멀어 설계
        판단에 영향이 없다고 보고 중단했다.

        계산 기준이던 "모듈당 4px"는 보수적이다 — 47.4cm를 역산하면 모듈당
        2.3~2.6px다. 4px로 잡으면 상한이 31cm로 나와 실측의 65%밖에 안 된다.
        이 기준으로 설계 판단을 하지 말 것.

    기울기가 거리보다 판독을 더 좌우한다 (위 측정의 가장 중요한 결과):
        손에 들고 쟀던 1차 측정에서는 37cm 정지 구간(190표본) 판독률이 9%,
        45cm 정지 구간(198표본)이 95%로 거리 역전이 났다. 거치해서 기울기를
        9~10도로 고정하자 27~47cm가 85~100%로 평탄해졌다 — 역전은 전적으로
        기울기 탓이었다.
        기울기 13도만 돼도 39~41cm 판독률이 55%로 떨어진다(29표본).
        비행 중 라벨을 비스듬히 보면 거리가 맞아도 못 읽는다는 뜻이다.
        서보잉은 거리보다 **정면성**을 먼저 맞춰야 한다.

    모듈 수는 payload 길이와 ECC 레벨이 함께 정한다. 현재 라벨은 156B이고
    실측 모듈 수가 53(ECC M)과 61(ECC Q) 사이 56으로 나와 확정하지 못했다.
    상한을 실측으로 잡은 이상 이 값은 설계에 쓰지 않는다.

    예비책(지금은 불필요, 기록만):
        정렬 목표를 ArUco 중심이 아니라 라벨 중심으로 옮기면 하한이 약 17.2cm로
        내려간다(lateral_error에서 123mm의 절반인 61.5mm를 빼면 된다). 이때는
        제약이 QR이 아니라 ArUco 쪽으로 넘어간다 — 라벨 중심 기준으로 ArUco
        바깥 끝이 128.2mm라 그쪽이 먼저 화면을 벗어난다.

    QR 프레이밍 하한 실측(2026-10-04, 817표본):
        계산값 21.1cm가 실측 21.2cm로 확인됐다. 오차 0.1cm다.

        거리만 보지 않고 "QR 우측 끝이 화면 안에 얼마나 여유 있게 들어오는가"로
        분석했다. 좌우가 틀어진 표본도 쓸 수 있어 표본이 네 배로 늘었다.
            여유 = 화면 반폭 - (좌우오차 + 123mm + QR절반 30mm + 정숙구역 4.5mm)

        | 여유     | 표본 | 판독률 | 좌우오차 0일 때 거리 |
        |----------|------|--------|----------------------|
        | -2.45cm  |  113 |     0% | 17.8cm               |
        | -1.45cm  |   94 |    32% | 19.2cm               |
        | -0.45cm  |   38 |    71% | 20.5cm               |
        | +0.05cm  |   51 |    90% | 21.2cm               |
        | +1.55cm  |   74 |    93% | 23.2cm               |

        여유가 0이 되는 지점이 안정 판독 하한과 겹친다 — 기하 모델이 맞다는 뜻이다.
        17.8cm 아래로는 어떤 조건에서도 0%다. 과접근 시 안전 한계로 쓸 수 있다.

        목표 25cm는 허용 하한 23cm에서도 여유가 +1.41cm 남는다.

    미확인:
        드론 호버링 안정성(프롭워시 포함)은 비행 실측 전이다 — 기대만큼
        안정적이지 않으면 중첩 마커(큰 마커 안에 작은 마커) 방식으로 전환
        검토 필요.

정면성(orientation) 미사용(2026-10-04 판단):
    이 노드는 ArUco pose의 orientation을 쓰지 않는다. 위치 오차(x·z)만 낸다.

    정면성은 측정이 아니라 임무 계획으로 맞춘다. 선반 방위는 이미 아는 값이기
    때문이다 — 창고 좌표계는 UWB 앵커로 측량돼 있고 구역 분할은 정적이다
    (roadmap 결정 9). 드론이 라벨 기울기를 볼 필요 없이 그 선반의 방위로
    접근하면 된다.

    허용 예산은 10도 안팎이다. 위 측정에서 9~10도는 판독률 85~100%로 평탄했고
    13도에서 55%로 떨어졌다(표본 29개라 정확한 문턱값은 아니다).

    다시 볼 조건:
        임무 계획대로 접근했는데 실제 기울기가 10도를 넘어 QR이 안 읽힐 때.
        선반이 삐뚤게 설치됐거나 PX4 yaw 추정이 흘렀을 때 그렇게 된다.

    그때 orientation을 쓰기 전에 먼저 잴 것:
        정면 부근에서 yaw 추정이 얼마나 떠는지. 마커가 정면에 가까울수록 어느
        쪽으로 기울었는지 구분이 어려워진다 — 정렬 목표가 바로 그 정면 부근이라
        가장 필요한 곳에서 값이 가장 안 믿음직할 수 있다. 라벨을 거치해 0·5·10·
        15도로 바꿔가며 추정값이 실제 각도를 따라가는지와 떨림 폭을 본다.

    지금 재지 않는 이유:
        쓸 주체가 없다. 이 값을 받아 판단할 BT는 roadmap 결정 5에서 Phase 3~4
        전환 대상이고, 지금은 aruco_servo_node의 제안값조차 구독자가 없다.
        쓰는 코드 없이 재두면 조건이 바뀌어 다시 재게 된다.

    제어 경로 주의:
        정면성을 쓰게 되더라도 이 노드나 서보 노드가 PX4로 직접 보내지 않는다.
        비전은 오차·제안까지만 내고 판단과 지령은 companion 내부 BT가 한다.
        PX4는 그 위치 지령을 자세로 구현하는 쪽이다(roadmap 결정 4).

좌우 허용오차는 거리에 따라 달라진다(2026-10-07 수정):
    고정 ±3cm만 쓰면 aligned=True 인데 QR 이 안 읽히는 조합이 생긴다.
    QR 이 ArUco 오른쪽에만 있어서, 좌우 오차의 두 방향이 전혀 다르기 때문이다.

        +방향(ArUco 가 오른쪽) → QR 이 더 오른쪽으로 → 화면 밖으로 먼저 잘림
        -방향(ArUco 가 왼쪽)   → QR 이 중앙 쪽으로   → 오히려 여유가 늘어남

    고정값이 틀리는 지점(수정 전 기준):

        | 거리  | 좌우  | QR 여유  | 실측 판독률 |
        |-------|-------|----------|-------------|
        | 23cm  | +3cm  | -1.59cm  |     0~32%   |
        | 25cm  | +3cm  | -0.09cm  |       71%   |
        | 27cm  | +3cm  | +1.40cm  |    90~93%   |

    그래서 +방향 상한을 기하로 계산해 lateral_tolerance_m 과 함께 쓴다.

        lateral_max = min(lateral_tolerance_m,
                          화면 반폭 - qr_outer_offset_m)
        화면 반폭 = (image_width / 2) * z / fx

    -방향은 lateral_tolerance_m 을 그대로 쓴다. ArUco 자신이 화면을 벗어나는
    한계가 23cm 에서 -10.2cm 라 기하 제약이 걸리지 않는다. 그 쪽 ±3cm 는
    "선반 앞에 제대로 섰는가"라는 임무 요구이지 QR 프레이밍 제약이 아니다.

    거리별 +방향 상한(현재 라벨·캘리브레이션 기준):

        | 거리  | 수정 전 | 수정 후 |
        |-------|---------|---------|
        | 23cm  |  +3.0cm |  +1.4cm |
        | 25cm  |  +3.0cm |  +2.9cm |
        | 27cm  |  +3.0cm |  +3.0cm | (기하 상한 +4.4cm 를 ±3cm 가 먼저 막음)

    fx·image_width 는 /camera/camera_info 에서 받는다. 초점을 조정하면
    재캘리브레이션이 따라오고 이 값도 같이 바뀌어야 하는데, 파라미터로 박아두면
    조용히 낡는다(roadmap Phase 0 의 렌즈 초점 항목이 아직 미완이다).
    camera_info 가 아직 안 왔으면 frame_half_width_per_m 기본값을 쓰고 경고한다.

실행:
    ros2 run drone_bringup aruco_alignment_node
"""

import json
import math

import rclpy
from aruco_opencv_msgs.msg import ArucoDetection
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String


class ArucoAlignmentNode(Node):

    def __init__(self):
        super().__init__('aruco_alignment_node')

        self.declare_parameter('target_distance_m', 0.25)
        self.declare_parameter('lateral_tolerance_m', 0.03)
        self.declare_parameter('distance_tolerance_m', 0.02)
        self.declare_parameter('marker_id', -1)  # -1이면 가장 가까운 마커

        # ArUco 중심에서 QR 바깥 끝까지. 123mm(중심간격) + 30mm(QR 절반)
        # + 4.5mm(정숙구역 4모듈). 라벨 레이아웃이 바뀌면 같이 바꾼다.
        self.declare_parameter('qr_outer_offset_m', 0.1575)
        # camera_info 가 오기 전까지 쓰는 폴백. (1640/2)/1098.86 = 0.7462
        self.declare_parameter('frame_half_width_per_m', 0.7462)

        self._target_distance = self.get_parameter('target_distance_m').value
        self._lateral_tol = self.get_parameter('lateral_tolerance_m').value
        self._distance_tol = self.get_parameter('distance_tolerance_m').value
        self._marker_id = self.get_parameter('marker_id').value
        self._qr_outer_offset = self.get_parameter('qr_outer_offset_m').value
        self._half_width_per_m = self.get_parameter('frame_half_width_per_m').value
        self._have_camera_info = False

        self._sub = self.create_subscription(
            ArucoDetection, '/aruco_detections', self._on_detection, 10)
        self._info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self._on_camera_info, 10)
        self._pub = self.create_publisher(String, '/aruco_alignment/error', 10)

        self._miss_streak = 0

        self.get_logger().info(
            f'aruco_alignment_node 시작 — target_distance={self._target_distance}m, '
            f'lateral_tol=±{self._lateral_tol}m, distance_tol=±{self._distance_tol}m, '
            f'qr_outer_offset={self._qr_outer_offset}m')

    def _on_camera_info(self, msg):
        """
        화면 반폭 비율을 실제 캘리브레이션에서 받는다.

        fx 는 projection_matrix(p[0]) 가 아니라 camera_matrix(k[0]) 를 쓴다.
        aruco_opencv 가 pose 를 뽑을 때 쓰는 값이 k 라서, 여기서 다른 값을
        쓰면 좌우오차와 화면 반폭의 기준이 어긋난다.
        """
        fx = msg.k[0]
        if not (math.isfinite(fx) and fx > 0 and msg.width > 0):
            return
        ratio = (msg.width / 2.0) / fx
        if self._have_camera_info and abs(ratio - self._half_width_per_m) < 1e-9:
            return
        self._half_width_per_m = ratio
        self._have_camera_info = True
        self.get_logger().info(
            f'camera_info 반영 — width={msg.width} fx={fx:.2f} '
            f'→ 화면 반폭 {ratio:.4f} m/m')

    def _lateral_bounds(self, z):
        """좌우 허용 구간 (하한, 상한). 상한만 거리에 따라 좁아진다."""
        framing_max = self._half_width_per_m * z - self._qr_outer_offset
        return -self._lateral_tol, min(self._lateral_tol, framing_max)

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
        lateral_min, lateral_max = self._lateral_bounds(marker.pose.position.z)
        aligned = (lateral_min <= lateral_error <= lateral_max
                   and abs(distance_error) <= self._distance_tol)

        if not self._have_camera_info and self._miss_streak == 0:
            self.get_logger().warn(
                'camera_info 미수신 — frame_half_width_per_m 기본값으로 판정 중',
                once=True)

        payload = {
            'valid': True,
            'marker_id': marker.marker_id,
            'lateral_error_m': round(lateral_error, 4),
            'distance_error_m': round(distance_error, 4),
            # 소비자가 "왜 aligned 가 아닌지" 알 수 있게 상한을 같이 싣는다.
            'lateral_max_m': round(lateral_max, 4),
            'aligned': aligned,
        }
        self._pub.publish(String(data=json.dumps(payload)))
        self.get_logger().info(
            f'id={marker.marker_id} 좌우오차={lateral_error:+.3f}m '
            f'(상한 {lateral_max:+.3f}m) 거리오차={distance_error:+.3f}m '
            f'aligned={aligned}')

    def _publish_invalid(self, reason):
        self._pub.publish(String(data=json.dumps({
            'valid': False, 'aligned': False, 'reason': reason,
            'marker_id': self._marker_id,
        })))

    def _pick_marker(self, markers):
        """
        marker_id 가 지정되면 그 마커만, -1이면 가장 가까운 마커를 쓴다.

        예전에는 -1일 때 markers[0] 을 집었다. aruco_opencv 가 넣는 순서는
        검출 순서라 프레임마다 바뀔 수 있어서, 선반 둘이 동시에 보이면 정렬
        대상이 왔다 갔다 한다. 가장 가까운 것을 고르면 결과가 프레임 순서에
        의존하지 않고, "지금 작업 중인 선반"이라는 뜻과도 맞는다.

        선반마다 ArUco 번호를 다르게 붙이기로 하면 임무가 marker_id 를 지정하는
        쪽이 맞다. 그 번호 체계는 아직 미정이라(docs/vision_print 참조) 그때까지
        거리 기준을 기본으로 둔다.
        """
        if not markers:
            return None
        if self._marker_id >= 0:
            for m in markers:
                if m.marker_id == self._marker_id:
                    return m
            return None
        valid = [m for m in markers
                 if math.isfinite(m.pose.position.z) and m.pose.position.z > 0]
        if not valid:
            return None
        return min(valid, key=lambda m: m.pose.position.z)


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
