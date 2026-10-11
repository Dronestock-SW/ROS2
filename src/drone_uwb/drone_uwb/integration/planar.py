"""Warehouse XY chart to right-handed ENU. Never a 3D attitude transform.

map_y_axis_sign=-1 describes A3 on the right when looking A1 -> A2.
The chart's positive yaw turns +X toward +Y. PX4 body FRD is unchanged.
"""
import math
import numpy as np


def y_axis_sign(settings):
    value = getattr(settings, 'map_y_axis_sign', 1)
    if type(value) is not int or value not in (-1, 1):
        raise ValueError('map_y_axis_sign_must_be_plus_or_minus_one')
    return value


def map_to_enu_matrix(settings):
    angle = math.radians(settings.enu_yaw_deg)
    if not math.isfinite(angle):
        raise ValueError('finite_alignment_yaw_required')
    c, s, sign = math.cos(angle), math.sin(angle), y_axis_sign(settings)
    return np.array([[c, -sign*s], [s, sign*c]])


def point(value):
    result = np.asarray(value, dtype=float)
    if result.shape != (2,) or not np.isfinite(result).all():
        raise ValueError('finite_planar_xy_required')
    return result


def offset(settings):
    return point((settings.enu_offset_x_m, settings.enu_offset_y_m))


def map_xy_to_enu(value, settings):
    return (map_to_enu_matrix(settings) @ point(value) + offset(settings)).tolist()


def enu_xy_to_map(value, settings):
    return (map_to_enu_matrix(settings).T @ (point(value)-offset(settings))).tolist()


def enu_vector_to_map(value, settings):
    return (map_to_enu_matrix(settings).T @ point(value)).tolist()


def map_yaw_to_enu(yaw_deg, settings):
    if not math.isfinite(yaw_deg):
        raise ValueError('finite_yaw_required')
    return math.remainder(settings.enu_yaw_deg+y_axis_sign(settings)*yaw_deg, 360)


def enu_yaw_to_map(yaw_deg, settings):
    if not math.isfinite(yaw_deg):
        raise ValueError('finite_yaw_required')
    return math.remainder(y_axis_sign(settings)*(yaw_deg-settings.enu_yaw_deg), 360)
