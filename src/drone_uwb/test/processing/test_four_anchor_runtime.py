"""The normal runtime profile requires A1-A4 and never falls back to A123."""
from dataclasses import fields
from pathlib import Path

import yaml
from test_observations import LAYOUT, cycle, status, process
from drone_uwb.processing.solvers.observations import Processor, Settings


def test_normal_runtime_blocks_missing_a4_and_recovers_with_four():
    path = Path(__file__).parents[2] / 'config/runtime/uwb.yaml'
    params = yaml.safe_load(path.read_text(encoding='utf-8'))['uwb_node']['ros__parameters']
    names = {item.name for item in fields(Settings)}
    settings = Settings(**{name: value for name, value in params.items() if name in names})
    assert settings.min_anchors == 4
    assert settings.active_anchor_mask == 15
    settings.clock_warmup_samples = 2
    settings.recovery_samples = 2
    processor = Processor(LAYOUT, settings)
    process(processor, status())
    missing = cycle(1)
    missing['valid_mask'] = 7
    missing['raw_slant_m'][3] = None
    missing['failure'][3] = 'response_timeout'
    result = process(processor, missing)
    assert result.reason == 'insufficient_anchors'
    assert result.observation is None
    assert process(processor, cycle(2)).observation is None
    recovered = process(processor, cycle(3))
    assert recovered.reason == 'accepted'
    assert recovered.observation.anchor_mask == 15
    assert recovered.details['consistency_redundancy'] is True
