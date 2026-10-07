"""Run declared UWB noise/weight/fault variants on one captured trajectory."""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import sys

from drone_uwb.processing.experiments.gazebo_trial import run
from drone_uwb.processing.experiments.h80_b import write_json


def scenarios(base):
    """Noise changes the sensor; weights change only WLS's assumed quality."""
    variants = {}
    for name in ('equal_quality', 'noisy_A2_equal', 'noisy_A2_weighted',
                 'noisy_A2_wrong_weight', 'A2_bias_unmodelled', 'dropout'):
        config = deepcopy(base)
        config['noise_sigma_m'] = [.03]*4
        config['wls_sigma_m'] = [.03]*4
        config['faults'] = []
        if name.startswith('noisy_A2'):
            config['noise_sigma_m'][1] = .3
        if name == 'noisy_A2_weighted':
            config['wls_sigma_m'][1] = .3
        if name == 'noisy_A2_wrong_weight':
            config['wls_sigma_m'][0] = .3
        if name == 'A2_bias_unmodelled':
            config['faults'] = [dict(start_s=2., end_s=6., anchor_id='A2', bias_m=.35)]
        if name == 'dropout':
            config['faults'] = [dict(start_s=3., end_s=3.25, drop_all=True)]
        variants[name] = config
    return variants


def run_scenarios(input_path, config_path, output):
    """Output directories are new; the source file is never changed."""
    output = Path(output)
    variants = scenarios(json.loads(Path(config_path).read_text(encoding='utf-8')))
    output.mkdir(parents=True, exist_ok=False)
    (output/'profiles').mkdir()
    summaries = {}
    for name, config in variants.items():
        profile = output/'profiles'/f'{name}.json'
        write_json(profile, config)
        summaries[name] = run(input_path, profile, output/name)
    write_json(output/'comparison.json', summaries)
    return summaries


def main(args=None):
    sys.stdout.reconfigure(encoding='utf-8')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    options = parser.parse_args(args)
    result = run_scenarios(options.input, options.config, options.output)
    for name, summary in result.items():
        print(name, json.dumps(summary['evaluation']['pairwise']['A_WLS'], ensure_ascii=False))


if __name__ == '__main__':
    main()
