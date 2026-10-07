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

Otsu 이진화 폴백:
    zbar는 내부 이진화로 흑백을 가르는데, 라벨이 충분히 밝지 않으면 임계값을
    못 잡는다. ROI·전체 프레임 각 단계에서 원본으로 먼저 시도하고, 실패할
    때만 Otsu 이진화로 재시도한다. 조명이 좋으면 원본이 먼저 걸려 Otsu 비용이
    아예 안 든다 (docs/glossary.md의 Otsu 이진화 항목 참조).

    근거 (2026-10-04 실측, 정면 거치 라벨 1프레임):
        원본 실패 / CLAHE 실패 / 2배확대 실패 / 샤픈 실패
        Otsu 성공 / CLAHE+Otsu 성공 (payload 125B)
        QR 영역 밝기가 35~180에 몰려 있다. 같은 프레임의 다른 곳은 255까지
        오르므로 노출이 아니라 라벨 반사율 문제다. Otsu는 임계값을 히스토그램
        에서 직접 찾아(이 프레임에서 101) 통과했다.
        이 폴백이 없을 때 244프레임 연속 미검출 (ArUco는 같은 구간 414샘플
        100% 검출).

    한계: 전역 Otsu는 임계값 하나를 영역 전체에 쓴다. 조명 기울기가 심해 한쪽은
    다 희고 한쪽은 다 검게 나오는 상황에서는 깨진다. 그때는 adaptiveThreshold를
    3단계로 추가해야 한다 — 이번 범위가 아니다. (위 프레임의 기울기는 흰색 수준
    좌 162 / 우 161, 상 164 / 하 155로 약했다)

watchdog:
    N프레임(기본 15) 연속으로 못 찾으면 경고 로그를 남긴다.
    (docs/glossary.md의 watchdog 항목과 같은 설계 — 조용히 못 찾는 상태가
    가장 늦게 발견되는 고장이다)

on/off:
    ~/set_enabled(std_srvs/SetBool)로 켜고 끈다. off면 cv_bridge 변환·
    pyzbar 디코딩 자체를 건너뛴다(qr_reader_node와 달리 이 노드는 처리
    비용이 프레임당 100ms대라 CPU를 실제로 아껴야 한다). qr_fallback_node가
    DE2110 리더기 연속 실패 시에만 켜는 폴백 경로로 쓴다
    (docs/glossary.md의 폴백 항목 참조).

실행:
    ros2 run drone_bringup qr_decoder_node
    ros2 run drone_bringup qr_decoder_node --ros-args --params-file \
        src/drone_bringup/params/qr_decoder.yaml
"""

import time

import cv2
import rclpy
from cv_bridge import CvBridge
from pyzbar.pyzbar import decode as zbar_decode
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import SetBool


class QrDecoderNode(Node):

    def __init__(self):
        super().__init__('qr_decoder_node')

        self.declare_parameter('roi_margin_px', 60)
        self.declare_parameter('miss_warn_threshold', 15)
        self.declare_parameter('hz_log_interval_sec', 1.0)

        self._roi_margin = self.get_parameter('roi_margin_px').value
        self._miss_warn_threshold = self.get_parameter('miss_warn_threshold').value
        self._hz_log_interval = self.get_parameter('hz_log_interval_sec').value

        if self._miss_warn_threshold <= 0:
            self.get_logger().warn(
                f'miss_warn_threshold={self._miss_warn_threshold}는 0 이하라 무효 — '
                '미검출 경고를 비활성화한다')
            self._miss_warn_threshold = None

        self._enabled = True
        self._bridge = CvBridge()
        self._last_bbox = None  # (x, y, w, h) 직전 검출 위치. None이면 전체 프레임 탐색
        self._miss_streak = 0

        self._proc_times = []  # 최근 처리 시간(초) — Hz 로그용
        self._last_hz_log_time = time.monotonic()

        self._sub = None
        self._pub = self.create_publisher(String, '/qr_code/data', 10)
        self._srv = self.create_service(SetBool, '~/set_enabled', self._on_set_enabled)
        self._set_enabled(True)

        self.get_logger().info(
            f'qr_decoder_node 시작 — roi_margin={self._roi_margin}px, '
            f'miss_warn_threshold={self._miss_warn_threshold}프레임')

    def _set_enabled(self, enabled):
        """끌 때 구독 자체를 끊는다.

        콜백 안에서 일찍 return 하는 것만으로는 부족하다. 구독이 살아 있으면
        1640x1232 프레임이 계속 전달돼 역직렬화와 복사가 일어난다. 실측으로
        꺼진 상태에서도 CPU 23%를 쓰고 있었다(2026-10-04). 이 노드를 평소
        꺼두는 이유가 CPU 절약인데(qr_fallback_node 폴백 경로), 그 효과가
        거의 없던 셈이다.
        """
        if enabled and self._sub is None:
            self._sub = self.create_subscription(
                Image, '/camera/image_raw', self._on_image, qos_profile_sensor_data)
        elif not enabled and self._sub is not None:
            self.destroy_subscription(self._sub)
            self._sub = None
            self._last_bbox = None  # 꺼진 동안 프레임이 바뀐다 — ROI는 버린다
            self._miss_streak = 0   # 꺼둔 시간이 미검출로 집계되면 안 된다
        self._enabled = enabled

    def _on_set_enabled(self, request, response):
        self._set_enabled(request.data)
        response.success = True
        response.message = f'enabled={request.data}'
        self.get_logger().info(f'qr_decoder_node enabled={request.data}')
        return response

    def _on_image(self, msg):
        if not self._enabled:
            return

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
            if self._miss_warn_threshold and self._miss_streak % self._miss_warn_threshold == 0:
                self.get_logger().warn(f'QR {self._miss_streak}프레임 연속 미검출')

        self._record_proc_time(time.monotonic() - t0)

    def _decode(self, frame):
        """ROI 우선 → 실패 시 전체 프레임 순서로 디코딩한다.

        반환값: ((x, y, w, h), 디코딩 문자열) 또는 (None, None)
        """
        h_img, w_img = frame.shape[:2]

        if self._last_bbox is not None:
            x0, y0, x1, y1 = self._expand_roi(self._last_bbox, w_img, h_img)
            hit = self._zbar(frame[y0:y1, x0:x1])
            if hit:
                x, y, w, h = hit[0].rect
                return (x0 + x, y0 + y, w, h), hit[0].data.decode('utf-8')

        hit = self._zbar(frame)
        if hit:
            return hit[0].rect, hit[0].data.decode('utf-8')

        return None, None

    def _zbar(self, img):
        """원본으로 먼저 디코딩하고, 실패할 때만 Otsu 이진화로 재시도한다.

        ROI와 전체 프레임 양쪽에서 같은 순서로 쓴다 — 조명이 좋으면 원본이
        먼저 걸려 Otsu 비용이 안 든다. 자세한 근거는 모듈 docstring 참조.
        """
        hit = zbar_decode(img)
        if hit:
            return hit

        _, binary = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        return zbar_decode(binary)

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
        # ros2 launch 의 SIGINT 는 rclpy 가 먼저 shutdown 해서, 무조건 부르면
        # "rcl_shutdown already called" 로 exit code 1 이 된다.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
