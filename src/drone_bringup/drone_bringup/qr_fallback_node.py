"""QR 폴백 코디네이터 노드 — Dronestock 1호기.

DYSCAN DE2110 리더기(/qr_reader/data)를 1순위로 쓰다가, 일정 시간
연속으로 스캔이 안 들어오면 카메라+pyzbar(/qr_code/data, qr_decoder_node)
로 전환한다. 둘 중 어느 쪽에서 오든 결과를 /qr/data 로 통일해 발행한다
— 하위 소비자는 어느 경로로 읽혔는지 신경 쓸 필요가 없다.

실패 판정 기준 (시간 창 방식):
    attempt_timeout_sec(기본 3.0초)마다 한 번씩, 그 창 안에 리더기
    스캔이 하나라도 왔는지 확인한다. 없었으면 실패 1회로 센다.
    리더기는 "지금 읽어봐" 같은 트리거 개념이 없어서(Sense Mode로
    항상 감지 중) 트리거 기반 카운트는 불가능하다 — 시간 창이 유일한
    측정 방법이다.

fallback_threshold(기본 3)회 연속 실패하면 qr_decoder_node를
~/set_enabled(True)로 켠다. 리더기가 다시 성공하면 즉시 qr_decoder_node를
꺼서(False) CPU를 아낀다 — qr_decoder_node는 프레임당 100ms대라 상시
켜두는 비용이 크다(docs/glossary.md 폴백 항목 참조).

실행:
    ros2 run drone_bringup qr_fallback_node
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool

_DECODER_ENABLE_SERVICE = '/qr_decoder_node/set_enabled'


class QrFallbackNode(Node):

    def __init__(self):
        super().__init__('qr_fallback_node')

        self.declare_parameter('attempt_timeout_sec', 3.0)
        self.declare_parameter('fallback_threshold', 3)

        self._attempt_timeout = self.get_parameter('attempt_timeout_sec').value
        self._fallback_threshold = self.get_parameter('fallback_threshold').value

        self._fail_streak = 0
        self._reader_hit_since_tick = False
        self._camera_fallback_active = False
        self._decoder_synced_target = None  # 마지막으로 실제 전송 성공한 목표값

        self._pub = self.create_publisher(String, '/qr/data', 10)
        self._reader_sub = self.create_subscription(
            String, '/qr_reader/data', self._on_reader_data, 10)
        self._camera_sub = self.create_subscription(
            String, '/qr_code/data', self._on_camera_data, 10)

        self._decoder_client = self.create_client(SetBool, _DECODER_ENABLE_SERVICE)
        self._set_decoder_enabled(False)  # 기본은 리더기 우선 — 카메라는 꺼서 시작

        self._timer = self.create_timer(self._attempt_timeout, self._on_attempt_tick)

        self.get_logger().info(
            f'qr_fallback_node 시작 — attempt_timeout={self._attempt_timeout}s, '
            f'fallback_threshold={self._fallback_threshold}회')

    def _on_reader_data(self, msg):
        self._reader_hit_since_tick = True
        self._fail_streak = 0
        self._pub.publish(msg)
        if self._camera_fallback_active:
            self._set_decoder_enabled(False)
            self._camera_fallback_active = False
            self.get_logger().info('리더기 복귀 — 카메라 폴백 끔')

    def _on_camera_data(self, msg):
        self._pub.publish(msg)

    def _on_attempt_tick(self):
        if self._reader_hit_since_tick:
            self._fail_streak = 0
        else:
            self._fail_streak += 1
            self.get_logger().debug(f'리더기 미스 {self._fail_streak}/{self._fallback_threshold}')
            if self._fail_streak >= self._fallback_threshold and not self._camera_fallback_active:
                self._camera_fallback_active = True
                self.get_logger().warn(
                    f'리더기 {self._fail_streak}회 연속 미스 — 카메라 폴백 켬')

        # 목표 상태(_camera_fallback_active)가 마지막으로 성공 전송한 값과 다르면
        # 재전송한다. 기동 시점에 qr_decoder_node 서비스가 아직 안 떠 있어서
        # __init__의 초기 끄기 시도가 실패했을 경우, _decoder_synced_target이
        # None으로 남아있으니 다음 tick에서 자동으로 재시도된다.
        if self._decoder_synced_target != self._camera_fallback_active:
            self._set_decoder_enabled(self._camera_fallback_active)
        self._reader_hit_since_tick = False

    def _set_decoder_enabled(self, enabled):
        if not self._decoder_client.service_is_ready():
            self.get_logger().warn(
                f'{_DECODER_ENABLE_SERVICE} 서비스 없음 — qr_decoder_node가 '
                '같이 떠 있는지 확인하라')
            return
        request = SetBool.Request()
        request.data = enabled
        self._decoder_client.call_async(request)
        self._decoder_synced_target = enabled


def main(args=None):
    rclpy.init(args=args)
    node = QrFallbackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
