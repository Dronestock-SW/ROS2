"""Exercise loss, malformed input and recovery without camera or flight hardware."""
import json
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest
from drone_bringup.aruco_alignment_node import ArucoAlignmentNode
from drone_bringup.aruco_servo_node import ArucoServoNode


def servo():
    node = SimpleNamespace(
        _last_valid=None, _timeout=0.5, _kp_distance=0.6,
        _kp_lateral=0.6, _max_speed=0.15, _publish_velocity=Mock())
    node._clamp = lambda v: ArucoServoNode._clamp(node, v)
    return node


def message(**changes):
    data = dict(valid=True, aligned=False,
                lateral_error_m=0.1, distance_error_m=0.5)
    data.update(changes)
    return SimpleNamespace(data=json.dumps(data))


def test_timeout_and_recovery():
    node = servo()
    with patch('drone_bringup.aruco_servo_node.time.monotonic', return_value=10):
        ArucoServoNode._on_error(node, message())
    node._publish_velocity.assert_called_with(-0.15, -0.06)
    node._publish_velocity.reset_mock()
    with patch('drone_bringup.aruco_servo_node.time.monotonic', return_value=10.49):
        ArucoServoNode._check_timeout(node)
    node._publish_velocity.assert_not_called()
    with patch('drone_bringup.aruco_servo_node.time.monotonic', return_value=10.5):
        ArucoServoNode._check_timeout(node)
    node._publish_velocity.assert_called_with()
    ArucoServoNode._on_error(node, message())
    node._publish_velocity.assert_called_with(-0.15, -0.06)


@pytest.mark.parametrize('raw', [
    '{', '[]', '{}', '{"valid":false}',
    message(lateral_error_m=float('nan')).data,
    message(distance_error_m=float('inf')).data,
    message(aligned='false').data,
    message(lateral_error_m=True).data,
])
def test_invalid_input_stops(raw):
    node = servo()
    node._last_valid = 1
    ArucoServoNode._on_error(node, SimpleNamespace(data=raw))
    node._publish_velocity.assert_called_once_with()
    assert node._last_valid is None


def test_startup_and_aligned_stop():
    node = servo()
    ArucoServoNode._check_timeout(node)
    node._publish_velocity.assert_called_with()
    ArucoServoNode._on_error(node, message(aligned=True))
    node._publish_velocity.assert_called_with()


def test_missing_target_publishes_invalid():
    node = SimpleNamespace(_marker_id=0, _miss_streak=0, _pub=Mock())
    node._pick_marker = lambda markers: ArucoAlignmentNode._pick_marker(node, markers)
    node._publish_invalid = lambda reason: ArucoAlignmentNode._publish_invalid(node, reason)
    for markers in ([], [SimpleNamespace(marker_id=1)]):
        ArucoAlignmentNode._on_detection(node, SimpleNamespace(markers=markers))
        data = json.loads(node._pub.publish.call_args.args[0].data)
        assert data['valid'] is False
        assert data['aligned'] is False
        assert data['reason'] == 'marker_missing'
