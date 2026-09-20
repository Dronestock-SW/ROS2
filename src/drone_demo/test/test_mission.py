"""Arrival and outage assessments must follow input evidence, not timer ticks."""
from dataclasses import replace
import json
from pathlib import Path

import pytest

from drone_demo.core import DemoConfig, DemoRun
from drone_demo.mission import MissionConfig, MissionMonitor


ROOT = Path(__file__).resolve().parents[2]
CONFIG = MissionConfig(**json.loads((ROOT / 'drone_demo/config/mission.json').read_text(encoding='utf-8')))
DEMO = DemoConfig(**json.loads((ROOT / 'drone_demo/config/demo.json').read_text(encoding='utf-8')))
LAYOUT = json.loads((ROOT / 'drone_uwb/config/anchors_20260906.json').read_text(encoding='utf-8'))


def feed(m, t, x=0.0, y=0.0, uwb=True):
    stamp = 1_000_000_000 + round(t*1e9)
    m.update('pose', x, y, stamp, t, 0.0, 'uwb_map')
    if uwb:
        m.update('uwb', x, y, stamp, t, 0.0, 'uwb_map')
    return m.evaluate(t)


def settled_monitor():
    m = MissionMonitor(CONFIG)
    m.set_target(0, 0, 'uwb_map')
    for i in range(81):
        result = feed(m, i/40)
    assert result['arrival_valid']
    return m


@pytest.mark.parametrize('scenario', ['stationary', 'move', 'gap'])
def test_full_generated_scenarios(scenario):
    config = replace(DEMO, scenario=scenario)
    run = DemoRun(config, LAYOUT)
    m = MissionMonitor(CONFIG)
    m.set_target(*run.target_xyz[:2], 'uwb_map')
    states = []
    for i in range(config.sample_count):
        sample = run.sample(i)
        t = sample['time_s']
        m.update('pose', *sample['truth_xyz_m'][:2], 1_000_000_000+round(t*1e9), t, 0, 'uwb_map')
        if sample['observation']:
            obs = sample['observation']
            m.update('uwb', obs['x'], obs['y'], obs['stamp_ns'], t, 0.01, 'uwb_map')
        states.append(m.evaluate(t))
    assert states[-1]['arrival_valid']
    assert states[-1]['distance_m'] == 0
    assert all(s['flight_output'] is False for s in states)
    assert any(s['state'] == 'RECOVERING' for s in states)
    if scenario != 'stationary':
        assert any(s['state'] == 'MOVING' for s in states)
        assert any(s['state'] == 'APPROACHING' for s in states)
    if scenario == 'gap':
        assert all(not s['arrival_valid'] for s in states[170:220])
        assert any(s['state'] == 'DEGRADED' for s in states[160:200])
        assert any(s['state'] == 'RECOVERING' for s in states[200:225])


def test_passing_near_target_at_speed_is_not_arrival():
    m = MissionMonitor(CONFIG)
    m.set_target(0, 0, 'uwb_map')
    # Stay inside the arrival radius while moving faster than the speed limit.
    import math
    for i in range(200):
        t = i/40
        result = feed(m, t, 0.1*math.cos(4*t), 0.1*math.sin(4*t))
        assert not result['arrival_valid']
    assert result['speed_m_s'] > CONFIG.arrival_speed_m_s


@pytest.mark.parametrize('lost', ['pose', 'uwb', 'both'])
def test_arrival_is_revoked_on_input_loss_and_requires_full_recovery(lost):
    m = settled_monitor()
    for i in range(81, 121):
        t = i/40
        stamp = 1_000_000_000+round(t*1e9)
        for kind in ('pose', 'uwb'):
            if lost not in (kind, 'both'):
                m.update(kind, 0, 0, stamp, t, 0, 'uwb_map')
        result = m.evaluate(t)
    assert result['state'] == 'DEGRADED'
    assert not result['arrival_valid']
    for i in range(121, 140):
        assert not feed(m, i/40)['arrival_valid']
    for i in range(140, 201):
        result = feed(m, i/40)
    assert result['arrival_valid']


def test_duplicate_timestamps_cannot_renew_freshness():
    m = settled_monitor()
    for i in range(81, 120):
        t = i/40
        for kind in ('pose', 'uwb'):
            assert not m.update(kind, 0, 0, 3_000_000_000, t, 0, 'uwb_map')
        result = m.evaluate(t)
    assert result['state'] == 'DEGRADED'


def test_timer_ticks_do_not_complete_settling_without_new_pose():
    m = MissionMonitor(replace(CONFIG, settle_s=0.05))
    m.set_target(0, 0, 'uwb_map')
    for i in range(22):
        result = feed(m, i/40)
    assert result['state'] == 'SETTLING'
    for i in range(1, 10):
        assert not m.evaluate(0.525+i*0.01)['arrival_valid']


def test_invalid_and_changed_targets_clear_previous_arrival_but_duplicates_do_not():
    m = settled_monitor()
    generation = m.target_generation
    assert m.set_target(0, 0, 'uwb_map')
    assert m.evaluate(2.0)['arrival_valid']
    assert m.target_generation == generation
    assert not m.set_target(0, 0, 'map')
    assert m.evaluate(2.0)['state'] == 'WAITING_TARGET'
    m.set_target(2, 0, 'uwb_map')
    assert not feed(m, 2.025)['arrival_valid']


@pytest.mark.parametrize('kind', ['pose', 'uwb'])
@pytest.mark.parametrize('change', ['frame', 'nan', 'future', 'stale'])
def test_invalid_input_blocks_arrival(kind, change):
    m = settled_monitor()
    x = float('nan') if change == 'nan' else 0.0
    age = -0.01 if change == 'future' else 1.0 if change == 'stale' else 0.0
    frame = 'map' if change == 'frame' else 'uwb_map'
    assert not m.update(kind, x, 0, 3_025_000_000, 2.025, age, frame)
    assert not m.evaluate(2.025)['arrival_valid']


def test_brief_departure_between_timer_ticks_restarts_settling():
    m = settled_monitor()
    m.update('pose', 0.3, 0, 3_025_000_000, 2.025, 0, 'uwb_map')
    m.update('pose', 0, 0, 3_050_000_000, 2.050, 0, 'uwb_map')
    assert not feed(m, 2.075)['arrival_valid']
    assert m.evaluate(2.075)['settle_elapsed_s'] == 0


def test_arrival_hysteresis_and_long_gap_before_next_evaluation():
    m = settled_monitor()
    for i in range(1, 91):
        result = feed(m, 2+i/40, x=i*0.002)
        assert result['arrival_valid']
    assert result['distance_m'] == pytest.approx(0.18)
    assert result['state'] == 'ARRIVED'
    assert not feed(m, 6.0, x=0.0)['arrival_valid']


@pytest.mark.parametrize('changes', [
    {'settle_s': 0.0}, {'recovery_samples': 1}, {'arrival_enter_m': 0.3},
    {'pose_timeout_s': float('nan')}, {'frame_id': ''},
])
def test_invalid_monitor_config(changes):
    with pytest.raises(ValueError):
        replace(CONFIG, **changes)
