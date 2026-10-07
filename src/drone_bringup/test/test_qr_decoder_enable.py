"""Check that disabling the decoder actually drops the image subscription."""
from types import SimpleNamespace
from unittest.mock import Mock

from drone_bringup.qr_decoder_node import QrDecoderNode


def node():
    """구독 생성·해제만 흉내 낸다 — rclpy 없이 전환 로직을 본다."""
    n = SimpleNamespace(
        _sub=None, _enabled=False, _last_bbox=(1, 2, 3, 4), _miss_streak=7,
        _on_image=Mock(),
        create_subscription=Mock(return_value='SUB'),
        destroy_subscription=Mock(), get_logger=lambda: Mock())
    n._set_enabled = lambda e: QrDecoderNode._set_enabled(n, e)
    return n


def test_enabling_creates_the_subscription():
    n = node()
    n._set_enabled(True)
    assert n._sub == 'SUB'
    assert n._enabled is True
    assert n.create_subscription.call_count == 1


def test_disabling_destroys_the_subscription():
    """콜백에서 return 만 하면 프레임이 계속 전달돼 CPU 를 쓴다."""
    n = node()
    n._set_enabled(True)
    n._set_enabled(False)
    n.destroy_subscription.assert_called_once_with('SUB')
    assert n._sub is None
    assert n._enabled is False


def test_disabling_clears_stale_state():
    n = node()
    n._set_enabled(True)
    n._last_bbox, n._miss_streak = (5, 6, 7, 8), 42
    n._set_enabled(False)
    assert n._last_bbox is None      # 꺼둔 사이 장면이 바뀐다
    assert n._miss_streak == 0       # 꺼둔 시간은 미검출이 아니다


def test_repeated_calls_do_not_stack_subscriptions():
    n = node()
    n._set_enabled(True)
    n._set_enabled(True)
    assert n.create_subscription.call_count == 1
    n._set_enabled(False)
    n._set_enabled(False)
    assert n.destroy_subscription.call_count == 1


def test_toggling_back_on_resubscribes():
    n = node()
    n._set_enabled(True)
    n._set_enabled(False)
    n._set_enabled(True)
    assert n._sub == 'SUB'
    assert n.create_subscription.call_count == 2
