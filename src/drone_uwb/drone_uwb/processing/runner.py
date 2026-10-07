"""Offline pipeline runner. Records input bytes before parsing; no ROS/serial/PX4 connection."""
import argparse
from collections import Counter
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import sys

from drone_uwb.contracts.protocol import InvalidSample, decode_line
from drone_uwb.processing.pipeline import Pipeline, PipelineConfig
from drone_uwb.processing.simulator import simulated_events


def json_line(value):
    return json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(',', ':')) + '\n'


def require_finite_json(value):
    # JSON numeric overflow (1e400) is accepted as inf by Python even without NaN constants.
    if isinstance(value, float) and not math.isfinite(value):
        raise InvalidSample('nonfinite_json')
    if isinstance(value, dict):
        for item in value.values():
            require_finite_json(item)
    elif isinstance(value, list):
        for item in value:
            require_finite_json(item)


def run_lines(lines, output, config, metadata=None):
    """Consume recorded event envelopes. Preserve even malformed lines and never reuse output dirs."""
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    pipeline = Pipeline(config)
    counts = Counter()
    sha = hashlib.sha256()
    with (output / 'input.jsonl').open('xb') as original, \
            (output / 'diagnostics.jsonl').open('x', encoding='utf-8') as diagnostics:
        for index, line in enumerate(lines, 1):
            original.write(line)
            sha.update(line)
            try:
                event = decode_line(line)
                require_finite_json(event)
                result = pipeline.process(event)
            except InvalidSample as exc:
                result = {'type': 'uwb_pipeline_diagnostic', 'schema': 1,
                          'mode': 'pipeline_only', 'valid': False, 'fresh': False,
                          'calculation_gate': 'closed', 'external_output_gate': 'closed',
                          'solver': 'none', 'x_m': None, 'y_m': None, 'z_m': None,
                          'reason': str(exc), 'stages': {'input': 'rejected'}}
            result['input_line'] = index
            counts[result['reason']] += 1
            diagnostics.write(json_line(result))
        original.flush()
        diagnostics.flush()
    summary = {'schema': 1, 'mode': 'pipeline_only', 'input_sha256': sha.hexdigest(),
               'events': sum(counts.values()), 'reasons': dict(counts), 'config': asdict(config),
               'metadata': metadata or {}, 'calculation_gate': 'closed', 'external_output_gate': 'closed',
               'position_calculations': 0, 'external_position_messages': 0,
               'note': 'Pipeline checks only. No height/position accuracy validation or flight output.'}
    (output / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, allow_nan=False, indent=2) + '\n',
                                         encoding='utf-8')
    return summary


def main(args=None):
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8')
    parser = argparse.ArgumentParser(description='UWB pipeline only: simulation/replay, calculation gates closed.')
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--simulate', action='store_true')
    source.add_argument('--input', type=Path, help='Recorded event envelopes, not bare ESP32 JSON lines')
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--anchors', type=Path, help='Required only for generating simulated UWB ranges')
    parser.add_argument('--duration', type=float, default=10.0)
    parser.add_argument('--seed', type=int, default=7)
    parser.add_argument('--output', type=Path, required=True, help='New directory; existing paths are refused')
    options = parser.parse_args(args)
    config = PipelineConfig.from_mapping(json.loads(options.config.read_text(encoding='utf-8')))
    if options.simulate:
        if options.anchors is None:
            parser.error('--simulate requires --anchors')
        if not 0 < options.duration <= 3600:
            parser.error('--duration must be in (0, 3600]')
        layout = json.loads(options.anchors.read_text(encoding='utf-8'))
        events = simulated_events(layout, config.preimu, options.duration, options.seed)
        summary = run_lines((json_line(e).encode('utf-8') for e in events), options.output, config,
                            {'source': 'synthetic_simulator', 'seed': options.seed,
                             'duration_s': options.duration, 'simulation_layout': layout})
    else:
        with options.input.open('rb') as lines:
            summary = run_lines(lines, options.output, config, {'source': 'recorded_events'})
    print(json.dumps({'events': summary['events'], 'reasons': summary['reasons'],
                      'output': str(options.output), 'calculation_gate': 'closed',
                      'external_output_gate': 'closed'}, ensure_ascii=False))


if __name__ == '__main__':
    main()
