"""QR 리더기(DYSCAN DE2110) 입력 노드 — Dronestock 1호기.

DE2110은 Sense Mode(2026-08-23 설정 완료)로 동작한다 — 버튼 없이 QR이
렌즈 앞에 나타나면 자동으로 디코딩해 USB HID 키보드 입력처럼 문자를 쏜다.
udev가 만든 /dev/input/qr_reader(config/udev/99-dronestock.rules)를 evdev로
읽어 문자로 재조립하고 /qr_reader/data 로 발행한다.

USB 재연결 자동 복구:
    스캐너가 evdev 장치를 물고 있는 도중 USB가 뽑히면 read_loop()이
    OSError를 던진다. 예전엔 여기서 스레드가 조용히 끝나버려서 노드는
    살아있는데 死장치를 붙든 채 아무 것도 못 읽는 상태가 됐다(2026-08-23
    실측 — 그때는 "Sense Mode가 재연결 시 풀린다"고 오판했는데, 실은
    이 死 파일디스크립터 버그였다. 재연결 후 노드만 새로 띄우면 버튼 없이도
    바로 인식됐다). 이제 OSError를 잡으면 DEVICE_PATH가 다시 열릴 때까지
    재시도한다.

/qr_reader/data 는 qr_decoder_node의 /qr_code/data 와 다른 topic이다:
    qr_decoder_node = 카메라(IMX219)+pyzbar, 비주얼 서보잉 위치 실험용.
    이 노드          = DYSCAN 전용 스캐너, 재고 데이터 판독용(glossary QR 코드 항목).
    목적이 달라 topic도 분리한다.

exclusive grab:
    evdev로 열 때 device.grab()을 건다. 안 걸면 이 키 입력이 포커스 잡은
    다른 프로그램(터미널 등)에도 그대로 들어간다 — QR 데이터가 실수로
    셸 명령처럼 입력될 위험이 있다.

on/off:
    ~/set_enabled(std_srvs/SetBool) 서비스로 켜고 끈다. off여도 장치는
    계속 읽지만(grab 유지), 발행만 하지 않는다 — 하드웨어 버튼을 코드로
    대신 누르는 게 아니라, 소프트웨어 게이트로 대체한 것이다.

실행:
    ros2 run drone_bringup qr_reader_node
    ros2 service call /qr_reader_node/set_enabled std_srvs/srv/SetBool "{data: false}"
"""

import threading
import time

import evdev
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool

DEVICE_PATH = '/dev/input/qr_reader'
_RECONNECT_INTERVAL_SEC = 1.0

# US 배열 evdev keycode -> (평상시 문자, Shift 문자).
# 스캐너 Data Format이 GBK/Unicode든 재고 태그는 영숫자 위주라 이 정도만 다룬다.
_KEYMAP = {
    evdev.ecodes.KEY_1: ('1', '!'), evdev.ecodes.KEY_2: ('2', '@'),
    evdev.ecodes.KEY_3: ('3', '#'), evdev.ecodes.KEY_4: ('4', '$'),
    evdev.ecodes.KEY_5: ('5', '%'), evdev.ecodes.KEY_6: ('6', '^'),
    evdev.ecodes.KEY_7: ('7', '&'), evdev.ecodes.KEY_8: ('8', '*'),
    evdev.ecodes.KEY_9: ('9', '('), evdev.ecodes.KEY_0: ('0', ')'),
    evdev.ecodes.KEY_MINUS: ('-', '_'), evdev.ecodes.KEY_EQUAL: ('=', '+'),
    evdev.ecodes.KEY_SPACE: (' ', ' '), evdev.ecodes.KEY_DOT: ('.', '>'),
    evdev.ecodes.KEY_COMMA: (',', '<'), evdev.ecodes.KEY_SLASH: ('/', '?'),
    evdev.ecodes.KEY_SEMICOLON: (';', ':'), evdev.ecodes.KEY_APOSTROPHE: ("'", '"'),
    evdev.ecodes.KEY_LEFTBRACE: ('[', '{'), evdev.ecodes.KEY_RIGHTBRACE: (']', '}'),
    evdev.ecodes.KEY_BACKSLASH: ('\\', '|'), evdev.ecodes.KEY_GRAVE: ('`', '~'),
}
for _c in 'abcdefghijklmnopqrstuvwxyz':
    _KEYMAP[getattr(evdev.ecodes, f'KEY_{_c.upper()}')] = (_c, _c.upper())

_SHIFT_KEYS = {evdev.ecodes.KEY_LEFTSHIFT, evdev.ecodes.KEY_RIGHTSHIFT}
_ENTER_KEYS = {evdev.ecodes.KEY_ENTER, evdev.ecodes.KEY_KPENTER}


class QrReaderNode(Node):

    def __init__(self):
        super().__init__('qr_reader_node')

        self._enabled = True
        self._lock = threading.Lock()
        self._buf = []
        self._shift = False

        self._pub = self.create_publisher(String, '/qr_reader/data', 10)
        self._srv = self.create_service(SetBool, '~/set_enabled', self._on_set_enabled)

        self._closed = False
        self._device = evdev.InputDevice(DEVICE_PATH)
        self._device.grab()

        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(f'qr_reader_node 시작 — {DEVICE_PATH} 그랩, enabled=True')

    def _on_set_enabled(self, request, response):
        with self._lock:
            self._enabled = request.data
        response.success = True
        response.message = f'enabled={request.data}'
        self.get_logger().info(f'qr_reader_node enabled={request.data}')
        return response

    def _read_loop(self):
        while not self._closed:
            try:
                for event in self._device.read_loop():
                    if event.type == evdev.ecodes.EV_KEY:
                        self._handle_key(event.code, event.value)
            except OSError:
                if self._closed:
                    return  # destroy_node()가 device.close()를 불러 깨어난 정상 종료
                self.get_logger().warn(f'{DEVICE_PATH} 연결 끊김 — 재연결 대기 중')
                self._reconnect()

    def _reconnect(self):
        self._buf.clear()
        self._shift = False
        while not self._closed:
            try:
                self._device = evdev.InputDevice(DEVICE_PATH)
                self._device.grab()
                self.get_logger().info(f'{DEVICE_PATH} 재연결 성공')
                return
            except OSError:
                time.sleep(_RECONNECT_INTERVAL_SEC)

    def _handle_key(self, code, value):
        if code in _SHIFT_KEYS:
            if value in (0, 1):
                self._shift = (value == 1)
            return

        if value != 1:  # 0=release, 2=autorepeat — 눌림(1)만 처리
            return

        if code in _ENTER_KEYS:
            self._emit_line()
            return

        pair = _KEYMAP.get(code)
        if pair is not None:
            self._buf.append(pair[1] if self._shift else pair[0])

    def _emit_line(self):
        payload = ''.join(self._buf)
        self._buf.clear()
        if not payload:
            return

        with self._lock:
            enabled = self._enabled

        if not enabled:
            self.get_logger().debug(f'qr_reader off 상태 — 무시: {payload}')
            return

        self._pub.publish(String(data=payload))
        self.get_logger().info(f'QR 읽음: {payload}')

    def destroy_node(self):
        self._closed = True
        try:
            self._device.ungrab()
        except OSError:
            pass
        try:
            self._device.close()  # 블로킹 중인 read_loop()을 OSError로 깨운다
        except OSError:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = QrReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
