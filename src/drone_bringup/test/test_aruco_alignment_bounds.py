"""Check that alignment only passes when the QR still fits in frame."""
import json
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from drone_bringup.aruco_alignment_node import ArucoAlignmentNode

# camera_imx219_calib.yaml 기준: (1640/2)/1098.86
HALF_PER_M = 0.7462
QR_OUTER = 0.1575


def node(**changes):
    n = SimpleNamespace(
        _target_distance=0.25, _lateral_tol=0.03, _distance_tol=0.02,
        _marker_id=-1, _qr_outer_offset=QR_OUTER,
        _half_width_per_m=HALF_PER_M, _have_camera_info=True,
        _miss_streak=0, _pub=Mock(), get_logger=lambda: Mock())
    n.__dict__.update(changes)
    n._lateral_bounds = lambda z: ArucoAlignmentNode._lateral_bounds(n, z)
    n._pick_marker = lambda ms: ArucoAlignmentNode._pick_marker(n, ms)
    n._publish_invalid = lambda r: ArucoAlignmentNode._publish_invalid(n, r)
    return n


def marker(x, z, marker_id=0):
    return SimpleNamespace(
        marker_id=marker_id,
        pose=SimpleNamespace(position=SimpleNamespace(x=x, y=0.0, z=z)))


def run(n, markers):
    ArucoAlignmentNode._on_detection(n, SimpleNamespace(markers=markers))
    return json.loads(n._pub.publish.call_args[0][0].data)


# 기하 상한 = 0.7462*z - 0.1575 → 23cm:1.41cm, 25cm:2.91cm, 26.5cm:4.02cm
# 26.5cm 를 쓰는 이유: 27cm 는 거리 허용(±2cm)의 정확한 경계라 부동소수점에 걸린다.
@pytest.mark.parametrize('z_cm, lateral_cm, expected', [
    # 멀면 기하 상한이 ±3cm 보다 넓어서 ±3cm 가 그대로 상한이다.
    (26.5, +2.9, True),
    (26.5, -2.9, True),
    # 가까우면 +방향만 좁아진다. 수정 전에는 +2.9cm 도 aligned 였다.
    (23, +2.9, False),
    (23, +1.0, True),
    (23, -2.9, True),
    # 목표 거리의 상한은 2.91cm — 그 양쪽
    (25, +2.95, False),
    (25, +2.5, True),
])
def test_lateral_bound_follows_distance(z_cm, lateral_cm, expected):
    out = run(node(), [marker(lateral_cm / 100, z_cm / 100)])
    assert out['aligned'] is expected


def test_minus_side_is_not_narrowed_by_framing():
    """QR 은 ArUco 오른쪽에만 있다 — 왼쪽으로 치우쳐도 QR 은 안 잘린다."""
    low, high = node()._lateral_bounds(0.23)
    assert low == pytest.approx(-0.03)
    assert high < 0.03


def test_bound_is_published_for_the_consumer():
    out = run(node(), [marker(0.0, 0.23)])
    assert out['lateral_max_m'] == pytest.approx(0.0141, abs=1e-3)


def test_camera_info_overrides_the_fallback_ratio():
    n = node(_half_width_per_m=0.0, _have_camera_info=False)
    ArucoAlignmentNode._on_camera_info(
        n, SimpleNamespace(k=[1098.86] + [0.0] * 8, width=1640))
    assert n._have_camera_info is True
    assert n._half_width_per_m == pytest.approx(HALF_PER_M, abs=1e-4)


@pytest.mark.parametrize('info', [
    SimpleNamespace(k=[0.0] + [0.0] * 8, width=1640),      # fx = 0
    SimpleNamespace(k=[float('nan')] + [0.0] * 8, width=1640),
    SimpleNamespace(k=[1098.86] + [0.0] * 8, width=0),     # width = 0
])
def test_broken_camera_info_is_ignored(info):
    n = node(_half_width_per_m=HALF_PER_M, _have_camera_info=False)
    ArucoAlignmentNode._on_camera_info(n, info)
    assert n._have_camera_info is False
    assert n._half_width_per_m == pytest.approx(HALF_PER_M)


def test_nearest_marker_wins_when_id_is_unset():
    """선반 둘이 보이면 검출 순서가 아니라 거리로 고른다."""
    out = run(node(), [marker(0.0, 0.40, marker_id=7),
                       marker(0.0, 0.25, marker_id=3)])
    assert out['marker_id'] == 3


def test_requested_id_wins_over_distance():
    out = run(node(_marker_id=7), [marker(0.0, 0.40, marker_id=7),
                                   marker(0.0, 0.25, marker_id=3)])
    assert out['marker_id'] == 7


def test_requested_id_missing_is_reported_invalid():
    out = run(node(_marker_id=9), [marker(0.0, 0.25, marker_id=3)])
    assert out['valid'] is False
    assert out['reason'] == 'marker_missing'


def test_markers_with_broken_depth_are_skipped():
    out = run(node(), [marker(0.0, float('nan'), marker_id=7),
                       marker(0.0, 0.25, marker_id=3)])
    assert out['marker_id'] == 3
