"""QR JSON 파서 노드 — Dronestock 1호기.

qr_fallback_node가 리더기(DE2110)/카메라 어느 쪽에서 읽었든 통일해 내보내는
/qr/data 는 raw 문자열이다. 실제 내용은 웹 플랫폼(dronestock-platform)이
정의한 JSON 스키마 drone-stock-item/v1:

    {"schema":"drone-stock-item/v1","code":"...","name":"...",
     "zone":"...","shelf":"...","slot":"...","location":"...","date":"..."}

이 노드는 그 JSON을 파싱·검증해 /qr/item 으로 재발행한다. 하위 소비자
(미션 로직 등)는 raw 문자열을 다시 json.loads() 할 필요 없이, 이미
스키마 검증을 통과한 JSON만 받는다.

zone/shelf/slot 값은 전부 문자열이다(웹 팀 확인, 2026-08-30) — 숫자로
캐스팅하지 않는다.

스키마 불일치 처리:
    "schema" 필드가 없거나 EXPECTED_SCHEMA와 다르면 발행하지 않고 경고만
    남긴다. 웹 쪽이 스키마를 버전업하면 여기서 바로 드러난다 — 조용히
    잘못된 필드를 흘려보내는 것보다, 드론 쪽이 못 알아본 새 스키마라는
    사실을 로그로 남기는 편이 원인 추적에 낫다.

    code/name 필드가 없는 JSON도 같은 이유로 발행하지 않는다 — 웹 폼에서
    둘 다 필수(*) 항목이라, 없다면 정상 스캔이 아니라 데이터 손상으로
    본다. 그 외 필드(zone/shelf/slot/location/date)는 없으면 빈 문자열로
    채워 통과시킨다 — 스캔 당시 폼에 비어 있던 값일 뿐, 파싱 실패로 볼
    이유가 없다.

    JSON이 아닌 문자열(레거시 QR, 스캔 오류 등)도 같은 이유로 무시한다.

실행:
    ros2 run drone_bringup qr_parser_node
"""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

EXPECTED_SCHEMA = 'drone-stock-item/v1'
_OPTIONAL_FIELDS = ('zone', 'shelf', 'slot', 'location', 'date')


class QrParserNode(Node):

    def __init__(self):
        super().__init__('qr_parser_node')

        self._pub = self.create_publisher(String, '/qr/item', 10)
        self._sub = self.create_subscription(String, '/qr/data', self._on_qr_data, 10)

        self.get_logger().info(f'qr_parser_node 시작 — expected schema={EXPECTED_SCHEMA}')

    def _on_qr_data(self, msg):
        try:
            item = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f'JSON 아님 — 무시: {msg.data!r}')
            return

        if not isinstance(item, dict):
            self.get_logger().warn(f'JSON이 객체가 아님 — 무시: {msg.data!r}')
            return

        schema = item.get('schema')
        if schema != EXPECTED_SCHEMA:
            self.get_logger().warn(
                f'스키마 불일치(schema={schema!r}, expected={EXPECTED_SCHEMA!r}) — 무시: '
                f'{msg.data!r}')
            return

        code = item.get('code')
        name = item.get('name')
        if not code or not name:
            self.get_logger().warn(f'code 또는 name 필드 없음 — 무시: {msg.data!r}')
            return

        parsed = {'schema': schema, 'code': code, 'name': name}
        for field in _OPTIONAL_FIELDS:
            parsed[field] = item.get(field, '') or ''

        self._pub.publish(String(data=json.dumps(parsed, ensure_ascii=False)))
        self.get_logger().info(f'QR 파싱: code={code} zone={parsed["zone"]!r}')


def main(args=None):
    rclpy.init(args=args)
    node = QrParserNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
