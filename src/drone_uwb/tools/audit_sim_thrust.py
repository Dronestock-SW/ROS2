#!/usr/bin/env python3
"""Read normalized thrust during POSCTL; do not infer a motor force curve."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np


def audit(path):
    from pyulog import ULog
    log = ULog(str(path))
    data = {d.name: d.data for d in log.data_list if d.multi_id == 0}
    status = data['vehicle_status']

    def position_mask(times):
        idx = np.searchsorted(status['timestamp'], times, side='right')-1
        clipped = np.maximum(idx, 0)
        return ((idx >= 0) & (status['nav_state'][clipped] == 2)
                & ((times-status['timestamp'][clipped]) <= 1_000_000))

    def stats(values):
        values = np.asarray(values)
        values = values[np.isfinite(values)]
        return dict(count=len(values), median=float(np.median(values)), mean=float(np.mean(values)),
                    p05=float(np.quantile(values, .05)), p95=float(np.quantile(values, .95))) if len(values) else dict(count=0)

    thrust = data['vehicle_thrust_setpoint']
    vector = np.column_stack([thrust[f'xyz[{i}]'] for i in range(3)])
    selected = position_mask(thrust['timestamp'])
    hover = data.get('hover_thrust_estimate')
    motors = data.get('actuator_motors')
    return dict(source_file=Path(path).name, source_sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest(),
                source_firmware=log.msg_info_dict.get('ver_sw'), mode='POSCTL', max_status_age_s=1,
                units='normalized_command_not_newtons',
                thrust_vector_norm=stats(np.linalg.norm(vector[selected], axis=1)),
                valid_hover_estimate=(stats(hover['hover_thrust'][position_mask(hover['timestamp']) & hover['valid'].astype(bool)]) if hover is not None else None),
                motor_controls=({str(i):stats(motors[f'control[{i}]'][position_mask(motors['timestamp'])]) for i in range(4)} if motors is not None else None),
                esc_topics_present=[name for name in data if name.startswith('esc_')],
                initial_parameters={name:log.initial_parameters.get(name) for name in ('MPC_THR_HOVER','MPC_USE_HTE','THR_MDL_FAC','EKF2_IMU_POS_X','EKF2_RNG_POS_X')},
                motor_force_curve_identified=False, parameters_applied=False)


def main():
    sys.stdout.reconfigure(encoding='utf-8')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    opts = parser.parse_args()
    result = audit(opts.input)
    opts.output.parent.mkdir(parents=True, exist_ok=True)
    with opts.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
        stream.write('\n')
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
