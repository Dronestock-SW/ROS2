"""QR 폴백 코디네이터 노드 — Dronestock 1호기.

~/set_enabled(True)로 스캔 세션을 시작한다. 세션 중에는 DYSCAN DE2110
리더기(/qr_reader/data)를 1순위로 쓴다. 일정 시간 연속으로 스캔이
안 들어오면 카메라+pyzbar(/qr_code/data, qr_decoder_node)로 전환한다.
둘 중 어느 쪽에서 오든 첫 결과 한 건을 /qr/data로 발행하고 세션을 끝낸다.

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

세션 경계:
    노드가 실행됐다는 이유만으로 실패 시간을 세지 않는다. FSM/BT가 선반
    정렬을 끝낸 뒤 /qr_fallback_node/set_enabled를 True로 호출해야 한다.
    결과 한 건을 발행하면 세션은 자동 종료된다. 다음 선반에서는 다시
    True를 호출한다.

실행:
    ros2 run drone_bringup qr_fallback_node
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool

from drone_bringup.qr_fallback import QrFallbackState

_DECODER_ENABLE_SERVICE = '/qr_decoder_node/set_enabled'


class QrFallbackNode(Node):

    def __init__(self):
        super().__init__('qr_fallback_node')

        self.declare_parameter('attempt_timeout_sec', 3.0)
        self.declare_parameter('fallback_threshold', 3)
        self.declare_parameter('scan_active_on_start', False)

        self._attempt_timeout = self.get_parameter('attempt_timeout_sec').value
        self._fallback_threshold = self.get_parameter('fallback_threshold').value
        self._state = QrFallbackState(
            self._fallback_threshold,
            active=self.get_parameter('scan_active_on_start').value,
        )
        self._decoder_synced_target = None  # 마지막으로 실제 전송 성공한 목표값
        self._decoder_pending_target = None

        self._pub = self.create_publisher(String, '/qr/data', 10)
        self._reader_sub = self.create_subscription(
            String, '/qr_reader/data', self._on_reader_data, 10)
        self._camera_sub = self.create_subscription(
            String, '/qr_code/data', self._on_camera_data, 10)

        self._decoder_client = self.create_client(SetBool, _DECODER_ENABLE_SERVICE)
        # 기본은 리더기 우선이다. 카메라는 폴백 전까지 끈다.
        self._set_decoder_enabled(False)
        self._srv = self.create_service(SetBool, '~/set_enabled', self._on_set_enabled)

        self._timer = self.create_timer(self._attempt_timeout, self._on_attempt_tick)

        self.get_logger().info(
            f'qr_fallback_node 시작 — attempt_timeout={self._attempt_timeout}s, '
            f'fallback_threshold={self._fallback_threshold}회, '
            f'scan_active={self._state.active}')

    def _on_set_enabled(self, request, response):
        self._state.set_active(request.data)
        self._sync_decoder()
        response.success = True
        response.message = f'scan_active={self._state.active}'
        self.get_logger().info(response.message)
        return response

    def _on_reader_data(self, msg):
        if not self._state.accept_reader():
            self.get_logger().debug('비활성 스캔 세션 — 리더기 결과 무시')
            return
        self._pub.publish(msg)
        self._sync_decoder()
        self.get_logger().info('리더기 판독 성공 — 스캔 세션 종료')

    def _on_camera_data(self, msg):
        if not self._state.accept_camera():
            self.get_logger().debug('카메라 폴백 비활성 — 카메라 결과 무시')
            return
        self._pub.publish(msg)
        self._sync_decoder()
        self.get_logger().info('카메라 판독 성공 — 스캔 세션 종료')

    def _on_attempt_tick(self):
        was_camera_enabled = self._state.camera_enabled
        self._state.tick()
        if self._state.active and not self._state.camera_enabled:
            self.get_logger().debug(
                f'리더기 미스 {self._state.fail_streak}/{self._fallback_threshold}')
        if self._state.camera_enabled and not was_camera_enabled:
            self.get_logger().warn(
                f'리더기 {self._state.fail_streak}회 연속 미스 — 카메라 폴백 켬')
        self._sync_decoder()

    def _sync_decoder(self):
        self._set_decoder_enabled(self._state.camera_enabled)

    def _set_decoder_enabled(self, enabled):
        if self._decoder_pending_target is not None:
            return
        if self._decoder_synced_target == enabled:
            return
        if not self._decoder_client.service_is_ready():
            self.get_logger().warn(
                f'{_DECODER_ENABLE_SERVICE} 서비스 없음 — qr_decoder_node가 '
                '같이 떠 있는지 확인하라')
            return
        request = SetBool.Request()
        request.data = enabled
        self._decoder_pending_target = enabled
        future = self._decoder_client.call_async(request)
        future.add_done_callback(
            lambda completed, target=enabled: self._on_decoder_response(completed, target))

    def _on_decoder_response(self, future, target):
        self._decoder_pending_target = None
        success = False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f'qr_decoder 상태 변경 실패: {exc}')
        else:
            if response.success:
                self._decoder_synced_target = target
                success = True
            else:
                self.get_logger().error(
                    f'qr_decoder 상태 변경 거부: {response.message}')
        # 성공 응답을 기다리는 동안 목표가 바뀌었으면 즉시 다시 맞춘다.
        # 실패는 타이머의 다음 tick에서 재시도해 빠른 재귀 호출을 막는다.
        if success:
            self._sync_decoder()


def main(args=None):
    rclpy.init(args=args)
    node = QrFallbackNode()
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
