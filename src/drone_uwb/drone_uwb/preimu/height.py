"""ToF geometry and filter equations for later use. The pipeline does not call them.

These functions produce observations, never height control commands. Rotation R_WB maps
body vectors into warehouse coordinates; ToF axis and lever arms must be supplied explicitly.
"""
import math

import numpy as np

from .transform import validate_rotation


def _vector(value):
    vector = np.asarray(value, dtype=float)
    if vector.shape != (3,) or not np.isfinite(vector).all():
        raise ValueError('expected a finite 3-vector')
    return vector


def tof_to_fc_height(raw_distance_m, bias_m, rotation_wb, tof_axis_body,
                     tof_lever_arm_body_m, ground_z_m):
    """z_obs = z_ground - (R_WB l_ToF)_z - (d_raw - b_ToF)(R_WB e_ToF)_z."""
    if not all(math.isfinite(v) for v in (raw_distance_m, bias_m, ground_z_m)):
        raise ValueError('nonfinite ToF geometry')
    distance = raw_distance_m - bias_m
    rotation = validate_rotation(rotation_wb)
    axis = _vector(tof_axis_body)
    if not np.isclose(np.linalg.norm(axis), 1.0, rtol=0, atol=1e-9):
        raise ValueError('ToF axis must be a unit vector')
    beam_z = float((rotation @ axis)[2])
    if distance <= 0 or beam_z >= -1e-9:
        raise ValueError('ToF ray must point toward the ground')
    return float(ground_z_m - (rotation @ _vector(tof_lever_arm_body_m))[2] - distance * beam_z)


def antenna_to_fc_position(antenna_position_m, rotation_wb, uwb_lever_arm_body_m):
    """p_FC = p_ant - R_WB l_UWB (warehouse frame)."""
    return _vector(antenna_position_m) - validate_rotation(rotation_wb) @ _vector(uwb_lever_arm_body_m)


def alpha_beta_step(previous_z_m, previous_vz_m_s, observation_m, dt_s):
    """Independent AB state: alpha=.40, beta=.20, |vz|<=.8 m/s (PDF p.9)."""
    if not all(math.isfinite(v) for v in (previous_z_m, previous_vz_m_s, observation_m, dt_s)) or dt_s <= 0:
        raise ValueError('finite state and positive dt required')
    predicted = previous_z_m + previous_vz_m_s * dt_s
    residual = observation_m - predicted
    return predicted + 0.40 * residual, float(np.clip(previous_vz_m_s + 0.20 * residual / dt_s, -0.8, 0.8))


def median3(values):
    if len(values) != 3 or not all(math.isfinite(v) for v in values):
        raise ValueError('three finite observations required')
    return float(sorted(values)[1])
