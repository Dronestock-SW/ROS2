"""Run reproducible mission assessments using the shared demo inputs."""
import argparse
from dataclasses import asdict, replace
import json
from pathlib import Path
import sys

from .core import DemoRun, load_inputs
from .mission import MissionMonitor, load_mission_config


def run_scenario(config, layout, uwb_settings, mission_config, output):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    demo = DemoRun(config, layout, uwb_settings)
    monitor = MissionMonitor(mission_config)
    monitor.set_target(*demo.target_xy, layout['coordinate_frame'])
    transitions = []
    previous = None
    with (output / 'mission_state.jsonl').open('x', encoding='utf-8') as log:
        for index in range(config.sample_count):
            sample = demo.sample(index)
            t = sample['time_s']
            monitor.update('pose', *sample['truth_xy_m'],
                           1_000_000_000+round(t*1e9), t, 0.0, sample['frame_id'])
            if sample['observation']:
                obs = sample['observation']
                # XY fixtures have no transport delay or hardware clock mapping.
                monitor.update('uwb', obs['x'], obs['y'], obs['stamp_ns'], t,
                               0.0, sample['frame_id'])
            state = monitor.evaluate(t)
            state['demo_time_s'] = t
            log.write(json.dumps(state, allow_nan=False) + '\n')
            if state['state'] != previous:
                transitions.append({'time_s': t, 'state': state['state'], 'reason': state['reason']})
                previous = state['state']
    summary = {'demo': True, 'scenario': config.scenario, 'config': asdict(config),
               'mission_config': asdict(mission_config), 'layout_id': layout['layout_id'],
               'sample_count': config.sample_count, 'transitions': transitions,
               'final_state': state['state'], 'final_arrival_valid': state['arrival_valid'],
               'limits': 'Demo state assessment only; no flight actions or EKF validation.'}
    (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n', encoding='utf-8')
    return summary


def main():
    sys.stdout.reconfigure(encoding='utf-8')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--config', default='')
    parser.add_argument('--mission-config', default='')
    args = parser.parse_args()
    try:
        config, layout, settings = load_inputs(args.config)
        mission_config = load_mission_config(args.mission_config)
        args.output.mkdir(parents=True, exist_ok=False)
        summaries = [run_scenario(replace(config, scenario=scenario), layout, settings,
                                  mission_config, args.output/scenario)
                     for scenario in ('stationary', 'move', 'gap')]
    except (ValueError, OSError) as exc:
        parser.exit(2, str(exc)+'\n')
    print(json.dumps([{k: s[k] for k in ('scenario', 'final_state', 'transitions')}
                      for s in summaries], indent=2))


if __name__ == '__main__':
    main()
