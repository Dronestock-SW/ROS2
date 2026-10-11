#!/usr/bin/env python3
"""Bounded B_TF observation on separate diagnostic topics; no FC output.

Uses existing RAW, ToF, IMU and TIMESYNC publishers without opening a port or
restarting MAVROS. The supplied configuration must contain actual measurements.
This capture does not certify physical timing, alignment or EKF fusion.
"""
import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--domain', type=int, choices=(1, 2), required=True)
    parser.add_argument('--seconds', type=float, default=30.)
    args = parser.parse_args()
    if not 5 <= args.seconds <= 60:
        parser.error('--seconds must be between 5 and 60')
    if int(os.environ.get('ROS_DOMAIN_ID', '0')) != args.domain:
        parser.error('ROS_DOMAIN_ID must match --domain')
    config = json.loads(args.config.read_text(encoding='utf-8'))
    if (config.get('tag_id') != {1: '5', 2: '6'}[args.domain]
            or config.get('external_output_allowed') is not False
            or config.get('tdma_mode') != 'required'):
        parser.error('Measured Tag/domain, TDMA and observation-only config required')
    args.output.mkdir(parents=True, exist_ok=False)

    import numpy as np
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from drone_uwb.integration.ros.btf_node import BtfNode, safe_json

    remaps = ['--ros-args', '-r', '__node:=uwb_btf_ground_diagnostic']
    for suffix in ('status', 'decision', 'pose', 'xyz'):
        remaps += ['-r', f'/uwb/btf_{suffix}:=/diagnostic/uwb/btf_{suffix}']
    remaps += ['-p', f'config_file:={args.config.resolve()}',
               '-p', 'require_height_for_pose:=true',
               '-p', f'record_directory:={args.output.resolve() / "btf"}']
    rclpy.init(args=remaps)
    node = None
    started = time.monotonic()
    reason = 'duration_complete'
    positions, ages, selected = [], [], Counter()
    last_seq = None
    try:
        node = BtfNode()
        while rclpy.ok() and time.monotonic() - started < args.seconds:
            rclpy.spin_once(node, timeout_sec=.05)
            if node.fc_state is not None and (
                    not node.fc_state['connected'] or node.fc_state['armed']):
                reason = 'disarmed_connected_ground_capture_required'
                break
            row = node.last
            if row is None or row.get('seq') == last_seq:
                continue
            last_seq = row.get('seq')
            selected[row['reason']] += 1
            if row.get('published') and row.get('xyz_m') is not None:
                positions.append(row['xyz_m'])
                ages.append(row['publish_age_s'])
        if node.fc_state is None:
            reason = 'fc_state_not_received'
    except (KeyboardInterrupt, ExternalShutdownException):
        reason = 'interrupted'
    finally:
        report = dict(utc=datetime.now(timezone.utc).isoformat(),
                      scope='ground_observation_only_not_calibration',
                      reason=reason, duration_s=time.monotonic()-started,
                      fc_output_enabled=False, flight_valid=False,
                      timestamp_calibrated=False,
                      output_prefix='/diagnostic/uwb/btf_',
                      counts=dict(node.counts) if node else {},
                      height_counts=dict(node.height_counts) if node else {},
                      fc_state=node.fc_state if node else None,
                      observed_decisions=dict(selected), xyz_samples=len(positions))
        if positions:
            report.update(xyz_median_m=np.median(positions, axis=0).tolist(),
                          xyz_p05_m=np.percentile(positions, 5, axis=0).tolist(),
                          xyz_p95_m=np.percentile(positions, 95, axis=0).tolist(),
                          publish_age_p95_s=float(np.percentile(ages, 95)))
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        text = json.dumps(safe_json(report), indent=2, ensure_ascii=False, allow_nan=False)
        (args.output/'summary.json').write_text(text+'\n', encoding='utf-8')
        print(text, flush=True)
    return 0 if reason == 'duration_complete' and positions else 1


if __name__ == '__main__':
    raise SystemExit(main())
