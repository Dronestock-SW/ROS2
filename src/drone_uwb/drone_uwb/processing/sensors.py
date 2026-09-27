"""Bounded ToF/attitude buffers. Input availability is not a trusted Z observation."""
from collections import deque
from copy import deepcopy

import numpy as np

from drone_uwb.contracts.protocol import finite, integer


class SensorInputs:
    def __init__(self, max_age_s=0.10, capacity=512):
        self.max_age_us = round(max_age_s * 1e6)
        self.rows = {name: deque(maxlen=capacity) for name in ('tof_sample', 'attitude_sample')}

    def add(self, message, source, host_ns):
        kind = message['type']
        stamp = message.get('measurement_time_us')
        # Future live adapters must supply an explicit clock mapping before entering this boundary.
        if message.get('clock_domain') != 'host_monotonic_us':
            raise ValueError('sensor_clock_unmapped')
        if not integer(stamp) or not 0 <= stamp <= host_ns // 1000:
            raise ValueError('invalid_sensor_time')
        if not isinstance(message.get('valid'), bool):
            raise ValueError('invalid_sensor_status')
        if message['valid']:
            if kind == 'tof_sample':
                if not finite(message.get('distance_m')) or message['distance_m'] <= 0:
                    raise ValueError('invalid_tof_distance')
            else:
                q = message.get('quaternion_wxyz')
                if (not isinstance(q, list) or len(q) != 4 or not all(finite(v) for v in q)
                        or abs(np.linalg.norm(q) - 1.0) > 1e-6):
                    raise ValueError('invalid_attitude')
                if message.get('rotation_convention') != 'R_WB':
                    raise ValueError('unsupported_attitude_convention')
        self.rows[kind].append({'source': source, 'received_ns': host_ns, 'message': deepcopy(message)})

    def snapshot(self, measurement_time_us):
        result = {}
        for kind, rows in self.rows.items():
            # Only already-arrived, nonfuture samples. Preserve invalid/latest status explicitly.
            candidates = [r for r in rows if r['message']['measurement_time_us'] <= measurement_time_us]
            row = max(candidates, key=lambda r: r['message']['measurement_time_us']) if candidates else None
            age_us = measurement_time_us - row['message']['measurement_time_us'] if row else None
            result[kind] = {
                'sample': deepcopy(row),
                'age_ms': age_us / 1000 if row else None,
                'available': bool(row and row['message']['valid'] and age_us <= self.max_age_us),
            }
        result['z_valid'] = False
        result['z_m'] = None
        result['reason'] = 'height_calculation_gate_closed'
        return result
