"""Gazebo body-FLU to world geometry, independent of ROS and flight control."""
import numpy as np


def rotation_world_body(quaternion_wxyz):
    q = np.asarray(quaternion_wxyz, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1) > 1e-3:
        raise ValueError('unit_wxyz_quaternion_required')
    w, x, y, z = q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def tag_position(pose, offset_body_flu_m):
    body, offset = (np.asarray(v, dtype=float) for v in (pose['position_xyz_m'], offset_body_flu_m))
    if any(v.shape != (3,) or not np.isfinite(v).all() for v in (body, offset)):
        raise ValueError('finite_body_position_and_tag_offset_required')
    return body+rotation_world_body(pose['quaternion_wxyz'])@offset


class VirtualRanges:
    """Declared Gaussian range errors; never an RF / CIR propagation model."""

    def __init__(self, config):
        self.anchors = np.asarray(config['anchors_xyz_m'], dtype=float)
        self.bias = np.asarray(config['bias_m'], dtype=float)
        self.sigma = np.asarray(config['noise_sigma_m'], dtype=float)
        self.offset = config['tag_offset_body_flu_m']
        if (self.anchors.shape != (4, 3) or self.bias.shape != (4,) or self.sigma.shape != (4,)
                or not all(np.isfinite(v).all() for v in (self.anchors, self.bias, self.sigma))
                or np.any(self.sigma < 0)):
            raise ValueError('invalid_virtual_range_config')
        self.rng = np.random.default_rng(config['seed'])
        self.previous = None
        self.sequence = 0

    def sample(self, pose):
        stamp = pose['time_us']
        if (pose['clock_domain'] != 'gazebo_sim_us' or not isinstance(stamp, int)
                or isinstance(stamp, bool) or stamp < 0 or
                (self.previous is not None and stamp <= self.previous)):
            raise ValueError('increasing_gazebo_sim_time_required')
        tag = tag_position(pose, self.offset)
        distances = np.linalg.norm(self.anchors-tag, axis=1)
        raw = distances+self.bias+self.rng.normal(0, self.sigma, 4)
        self.previous = stamp
        row = dict(schema=1, source='simulation', type='sim_uwb_cycle', clock_domain='gazebo_sim_us',
                   seq=self.sequence, time_us=stamp, sample_time_us=[stamp]*4,
                   anchor_order=['A1', 'A2', 'A3', 'A4'], raw_slant_m=raw.tolist(),
                   valid_mask=sum(1 << i for i, v in enumerate(raw) if 0 < v <= 80),
                   range_bias_applied=False, external_output_allowed=False)
        self.sequence += 1
        truth = dict(time_us=stamp, body_xyz_m=pose['position_xyz_m'], tag_xyz_m=tag.tolist(),
                     geometric_ranges_m=distances.tolist())
        return row, truth
