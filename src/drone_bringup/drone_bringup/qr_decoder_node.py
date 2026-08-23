"""QR 코드 디코더 노드 — Dronestock 1호기.

/camera/image_raw 를 구독해 pyzbar로 QR 코드를 디코딩하고,
디코딩된 문자열을 /qr_code/data 로 발행한다.

/qr_code/data 는 roadmap의 /mission_result 가 아니다:
    /mission_result 는 Phase 4에서 스키마를 정의하는 topic이다(roadmap.md).
    이 노드는 그 전 단계 — 디코딩 자체가 되는지, 목표 Hz가 나오는지 확인하는
    실험용 노드이므로 별도 topic을 쓴다.

ROI(관심 영역) 최적화:
    QR을 한 번 찾으면, 다음 프레임부터는 그 위치 주변만 잘라 디코딩한다.
    전체 프레임 디코딩보다 빠르다 — 비주얼 서보잉 루프에 필요한 최소 Hz
    (드론 속도 10~15cm/s ÷ 허용오차 5~10cm = 1.5~3Hz)를 맞추기 위함.
    ROI에서 못 찾으면 같은 프레임 안에서 즉시 전체 프레임으로 재시도한다.
    QR이 ROI 밖으로 빠져나갔을 때 여러 프레임을 그냥 흘려보내지 않기 위해서다.

watchdog:
    N프레임(기본 15) 연속으로 못 찾으면 경고 로그를 남긴다.
    (docs/glossary.md의 watchdog 항목과 같은 설계 — 조용히 못 찾는 상태가
    가장 늦게 발견되는 고장이다)

실행:
    ros2 run drone_bringup qr_decoder_node
    ros2 run drone_bringup qr_decoder_node --ros-args --params-file \
        src/drone_bringup/params/qr_decoder.yaml
"""

import time

import rclpy
from cv_bridge import CvBridge
from pyzbar.pyzbar import decode as zbar_decode
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


class QrDecoderNode(Node):

    def __init__(self):
        super().__init__('qr_decoder_node')

        self.declare_parameter('roi_margin_px', 60)
        self.declare_parameter('miss_warn_threshold', 15)
        self.declare_parameter('hz_log_interval_sec', 1.0)

        self._roi_margin = self.get_parameter('roi_margin_px').value
        self._miss_warn_threshold = self.get_parameter('miss_warn_threshold').value
        self._hz_log_interval = self.get_parameter('hz_log_interval_sec').value

        self._bridge = CvBridge()
        self._last_bbox = None  # (x, y, w, h) 직전 검출 위치. None이면 전체 프레임 탐색
        self._miss_streak = 0

        self._proc_times = []  # 최근 처리 시간(초) — Hz 로그용
        self._last_hz_log_time = time.monotonic()

        self._sub = self.create_subscription(
            Image, '/camera/image_raw', self._on_image, qos_profile_sensor_data)
        self._pub = self.create_publisher(String, '/qr_code/data', 10)

        self.get_logger().info(
            f'qr_decoder_node 시작 — roi_margin={self._roi_margin}px, '
            f'miss_warn_threshold={self._miss_warn_threshold}프레임')

    def _on_image(self, msg):
        t0 = time.monotonic()

        # pyzbar는 3채널 numpy 배열을 받으면 첫 채널만 떼어 그레이스케일로
        # 오인식한다(파란 채널만 보는 셈). mono8로 직접 받아 이 함정을 피한다.
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        found_bbox, payload = self._decode(frame)

        if found_bbox is not None:
            self._last_bbox = found_bbox
            self._miss_streak = 0
            self._pub.publish(String(data=payload))
        else:
            self._last_bbox = None
            self._miss_streak += 1
            if self._miss_streak % self._miss_warn_threshold == 0:
                self.get_logger().warn(f'QR {self._miss_streak}프레임 연속 미검출')

        self._record_proc_time(time.monotonic() - t0)

    def _decode(self, frame):
        """ROI 우선 → 실패 시 전체 프레임 순서로 디코딩한다.

        반환값: ((x, y, w, h), 디코딩 문자열) 또는 (None, None)
        """
        h_img, w_img = frame.shape[:2]

        if self._last_bbox is not None:
            x0, y0, x1, y1 = self._expand_roi(self._last_bbox, w_img, h_img)
            hit = zbar_decode(frame[y0:y1, x0:x1])
            if hit:
                x, y, w, h = hit[0].rect
                return (x0 + x, y0 + y, w, h), hit[0].data.decode('utf-8')

        hit = zbar_decode(frame)
        if hit:
            return hit[0].rect, hit[0].data.decode('utf-8')

        return None, None

    def _expand_roi(self, bbox, w_img, h_img):
        x, y, w, h = bbox
        m = self._roi_margin
        x0 = max(0, x - m)
        y0 = max(0, y - m)
        x1 = min(w_img, x + w + m)
        y1 = min(h_img, y + h + m)
        return x0, y0, x1, y1

    def _record_proc_time(self, dt):
        self._proc_times.append(dt)

        now = time.monotonic()
        if now - self._last_hz_log_time < self._hz_log_interval:
            return

        avg_dt = sum(self._proc_times) / len(self._proc_times)
        self.get_logger().info(
            f'QR 루프 {1.0 / avg_dt:.2f}Hz (프레임당 {avg_dt * 1000:.1f}ms, '
            f'{len(self._proc_times)}프레임 평균)')
        self._proc_times.clear()
        self._last_hz_log_time = now


def main(args=None):
    rclpy.init(args=args)
    node = QrDecoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
