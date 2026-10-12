"""Summarize saved replay outputs without reinterpreting them as accuracy."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def summarize(directory):
    summary = json.loads((directory/'summary.json').read_text(encoding='utf-8'))
    rows = [json.loads(line) for line in (directory/'scans.jsonl').read_text(encoding='utf-8').splitlines()]
    segments = {}
    stamps = []
    for row in rows:
        candidate = row['candidate']
        if candidate['pose'] is not None:
            segments.setdefault(candidate['segment'], []).append(candidate)
        if candidate['accepted']:
            stamps.append(row['stamp_ns']/1e9)
    summary['candidate_segments'] = [dict(segment=key, pose_outputs=len(values),
        last_yaw_deg=float(np.rad2deg(values[-1]['pose'][2])),
        max_abs_yaw_deg=float(max(abs(np.rad2deg(v['pose'][2])) for v in values)))
        for key, values in segments.items()]
    summary['accepted_fraction_of_scan_intervals'] = summary['accepted_displacements']/(summary['scans']-1)
    summary['max_accepted_output_gap_s'] = float(max(np.diff(stamps))) if len(stamps)>1 else None
    summary['summary_path'] = str(directory/'summary.json')
    summary['scans_sha256'] = hashlib.sha256((directory/'scans.jsonl').read_bytes()).hexdigest()
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--replays', type=Path, nargs='+', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = dict(schema=1, scope='diagnostic stability and availability; no independent position/rotation truth',
                  results=[summarize(path) for path in args.replays])
    args.output.write_text(json.dumps(result,indent=2,ensure_ascii=False,allow_nan=False)+'\n',encoding='utf-8')


if __name__ == '__main__':
    main()
