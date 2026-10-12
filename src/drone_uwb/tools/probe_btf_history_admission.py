"""File-only comparison: keep incoherent fresh ranges out of both fit histories.

This is an experiment, not a runtime setting or deployment. It tightens input
admission using the existing 0.06m raw pair-residual boundary. Existing clocks,
ToF limits, jump protection and pose publication rules remain unchanged.
No historical positions are repeated and no FC/ROS connection is opened.
"""
import argparse
import hashlib
import json
from pathlib import Path

import drone_uwb.processing.measured_btf as measured
from drone_uwb.acquisition.validation import InvalidInput
import replay_manual_btf as replay
from analyze_btf_gaps import compare_replays, replay_comparison


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    if args.output_dir.exists():
        parser.error('use a new output directory; preserve earlier results')
    args.output_dir.mkdir(parents=True)
    baseline = args.output_dir/'baseline.jsonl'
    candidate = args.output_dir/'quarantined.jsonl'
    original = measured.solve_xy
    baseline_counts = replay.replay(args.capture, baseline)

    def qualified_current_frame(*arguments, **keywords):
        xy, residual = original(*arguments, **keywords)
        if residual >= .06:
            # No reset: keep accepted geometric history, but do not append the
            # rejected cycle to either H80 or ToF subset history. The source
            # validation/TDMA/clock already consumed this actual fresh sample.
            raise InvalidInput('incoherent_raw_frame_quarantined')
        return xy, residual

    try:
        measured.solve_xy = qualified_current_frame
        candidate_counts = replay.replay(args.capture, candidate)
    finally:
        measured.solve_xy = original
    report = {
        'scope': 'offline hypothesis probe; collector order; no independent XY truth or flight approval',
        'runtime_changed': False, 'fc_commands_sent': 0, 'parameter_writes': 0,
        'raw_pair_residual_boundary_m': .06,
        'source_manifest_sha256': hashlib.sha256((args.capture/'manifest.json').read_bytes()).hexdigest(),
        'source_events_sha256': hashlib.sha256((args.capture/'events.jsonl').read_bytes()).hexdigest(),
        'baseline_counts': baseline_counts, 'candidate_counts': candidate_counts,
        'baseline_gaps': replay_comparison(baseline),
        'candidate_gaps': replay_comparison(candidate),
        'comparison': compare_replays(baseline, candidate),
        'limits': 'A lower gap count is not a position-accuracy result. Sequential ranging/motion may also fail this instantaneous check. No runtime promotion without further recorded-case checks.'}
    (args.output_dir/'comparison.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({'output': str(args.output_dir/'comparison.json'),
        'baseline': baseline_counts['counts'], 'candidate': candidate_counts['counts']}))


if __name__ == '__main__':
    main()
