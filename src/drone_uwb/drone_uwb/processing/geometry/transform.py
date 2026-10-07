"""warehouse -> PX4 local transform. A proper rotation only; flipping Z alone is rejected.

PX4 LOCAL_FRD conversion happens once, at the consumer boundary. Nothing in this package calls it.
"""
import numpy as np


def validate_rotation(matrix, tolerance=1e-9):
    """Return R as a float array, or raise if it is not orthonormal with det=+1."""
    rotation = np.asarray(matrix, dtype=float)
    if rotation.shape != (3, 3) or not np.isfinite(rotation).all():
        raise ValueError('rotation must be a finite 3x3 matrix')
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=tolerance):
        raise ValueError('rotation is not orthonormal')
    if abs(np.linalg.det(rotation) - 1.0) > tolerance:
        raise ValueError('rotation determinant must be +1 (reflection rejected)')
    return rotation


def warehouse_to_px4(point_w, rotation_pw, origin_w):
    """p_PX4 = R_PW (p_W - o_W)."""
    rotation = validate_rotation(rotation_pw)
    return rotation @ (np.asarray(point_w, dtype=float) - np.asarray(origin_w, dtype=float))


def body_to_px4(rotation_pw, rotation_wb):
    """R_PB = R_PW R_WB."""
    return validate_rotation(rotation_pw) @ validate_rotation(rotation_wb)
