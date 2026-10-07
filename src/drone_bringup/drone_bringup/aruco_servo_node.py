"""ArUco 비주얼 서보잉 속도 제안 노드 — Dronestock 1호기 (Phase 3, 2단계).

/aruco_alignment/error(좌우오차·거리오차)를 받아 "이 정도 속도로 움직이면
정렬된다"는 속도 제안값을 계산해서 /aruco_alignment/velocity_suggestion 으로
발행한다.

★ MAVROS/PX4에는 아무것도 안 보낸다 ★
    UWB팀이 위치제어 공용 인프라(FSM + MAVROS setpoint 발행 노드)를 별도로
    만들고 있어서(2026-09-13 확인), 이 노드가 직접 /mavros/setpoint_velocity
    같은 곳에 값을 쏘면 그 인프라와 명령권이 겹치거나 충돌할 수 있다. 그래서
    이 노드는 "제안값"만 만들어 내놓고, 실제로 PX4에 전달하는 건 그 공용
    인프라(FSM)가 이 토픽을 구독해서 하도록 미룬다. 인터페이스(토픽명·단위·
    좌표계)는 그쪽 설계가 확정되면 다시 맞출 예정 — 지금 이름/형식은 잠정이다.

계산 방식: 단순 비례(P) 제어.
    vy = -Kp_lateral * lateral_error   (오차가 +면 마커가 오른쪽 → 왼쪽 명분? 아래 부호 설명 참조)
    vx = -Kp_distance * distance_error (오차가 +면 너무 멂 → 앞으로)
    실제 좌우/전후 어느 축에 매핑되는지는 드론 body frame 정의가 확정된 뒤
    부호를 재확인해야 한다 — 지금은 "오차를 줄이는 방향"이라는 부호 관계만
    맞춰뒀다.
    속도는 roadmap이 제시한 서보잉 속도(10~15cm/s)를 넘지 않게 clamp한다.

정렬 완료(aligned=True) 시에는 속도를 0으로 낸다 — 계속 미세하게 흔들리며
움직이는 걸 막기 위해서다.

실행(검증용, PX4 미연결):
    ros2 run drone_bringup aruco_servo_node
"""

import json
import math
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import String


class ArucoServoNode(Node):

    def __init__(self):
        super().__init__('aruco_servo_node')

        self.declare_parameter('error_timeout_sec', 0.5)
        self.declare_parameter('kp_lateral', 0.6)
        self.declare_parameter('kp_distance', 0.6)
        self.declare_parameter('max_speed_mps', 0.15)  # roadmap 서보잉 속도 상한(10~15cm/s)

        self._kp_lateral = self.get_parameter('kp_lateral').value
        self._kp_distance = self.get_parameter('kp_distance').value
        self._max_speed = self.get_parameter('max_speed_mps').value

        self._timeout = self.get_parameter('error_timeout_sec').value
        if not math.isfinite(self._timeout) or self._timeout <= 0:
            raise ValueError('error_timeout_sec must be positive and finite')
        self._last_valid = None

        self._sub = self.create_subscription(
            String, '/aruco_alignment/error', self._on_error, 10)
        self._pub = self.create_publisher(
            TwistStamped, '/aruco_alignment/velocity_suggestion', 10)

        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._timer = self.create_timer(
            0.05, self._check_timeout, clock=self._steady_clock)

        self.get_logger().info(
            f'aruco_servo_node 시작 (PX4 미연결, 제안값만 발행) — '
            f'kp_lateral={self._kp_lateral}, kp_distance={self._kp_distance}, '
            f'max_speed={self._max_speed}m/s')

    def _on_error(self, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict) or data.get('valid') is not True:
                raise ValueError('invalid observation')
            lateral = data['lateral_error_m']
            distance = data['distance_error_m']
            aligned = data['aligned']
            if not isinstance(aligned, bool):
                raise ValueError('invalid aligned flag')
            for value in (lateral, distance):
                if (isinstance(value, bool) or not isinstance(value, (int, float))
                        or not math.isfinite(value)):
                    raise ValueError('invalid error value')
        except (ValueError, KeyError, TypeError):
            self._last_valid = None
            self._publish_velocity()
            return

        self._last_valid = time.monotonic()
        if aligned:
            self._publish_velocity()
        else:
            self._publish_velocity(
                self._clamp(-self._kp_distance * distance),
                self._clamp(-self._kp_lateral * lateral))

    def _publish_velocity(self, vx=0.0, vy=0.0):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = vx
        cmd.twist.linear.y = vy
        self._pub.publish(cmd)

    def _check_timeout(self):
        # Monotonic time also expires when ROS simulation time is paused.
        if (self._last_valid is None
                or time.monotonic() - self._last_valid >= self._timeout):
            self._publish_velocity()

    def _clamp(self, v):
        return max(-self._max_speed, min(self._max_speed, v))


def main(args=None):
    rclpy.init(args=args)
    node = ArucoServoNode()
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
