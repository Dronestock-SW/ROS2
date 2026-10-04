"""Simulation clock must stop with Gazebo and never extrapolate using host time."""
from types import SimpleNamespace

import pytest

from drone_uwb.integration.gazebo_clock import (
    ClockUnavailable, GazeboSimulationClock, simulation_clock_ns,
)
from drone_uwb.integration.sitl_timesync import SITLTimesyncResponder


def test_exact_simulation_time_is_not_advanced_by_host_elapsed_time():
    host = [10_000_000_000]
    clock = GazeboSimulationClock(host_now_ns=lambda: host[0])
    with pytest.raises(ClockUnavailable, match='unavailable'):
        clock.now_ns()
    clock.update(1_234_567_890)
    host[0] += 200_000_000
    assert clock.now_ns() == 1_234_567_890
    assert not clock.update(1_234_567_890)
    host[0] += 51_000_000
    with pytest.raises(ClockUnavailable, match='stalled'):
        clock.now_ns()
    clock.update(1_234_567_891)
    assert clock.now_ns() == 1_234_567_891


def test_simulation_restart_requires_explicit_epoch_reset():
    clock = GazeboSimulationClock(host_now_ns=lambda: 10_000_000_000)
    clock.update(1_000_000_000)
    with pytest.raises(ClockUnavailable, match='reversed') as error:
        clock.update(1)
    assert error.value.reset_required
    with pytest.raises(ClockUnavailable, match='reversed'):
        clock.now_ns()
    clock.reset_epoch()
    clock.update(2)
    assert clock.epoch == 1 and clock.now_ns() == 2


def test_host_reversal_and_invalid_clock_message_are_rejected():
    clock = GazeboSimulationClock(host_now_ns=lambda: 10_000)
    clock.update(1, received_host_ns=10_001)
    with pytest.raises(ClockUnavailable, match='host_clock_reversed'):
        clock.now_ns()
    message = SimpleNamespace(sim=SimpleNamespace(sec=5, nsec=123))
    assert simulation_clock_ns(message) == 5_000_000_123
    message.sim.nsec = 1_000_000_000
    with pytest.raises(ValueError, match='invalid_gazebo_clock'):
        simulation_clock_ns(message)


def test_responder_skips_paused_clock_and_resumes_only_on_new_simulation_tick():
    host = [10_000_000_000]
    clock = GazeboSimulationClock(host_now_ns=lambda: host[0])
    calls = []
    mav = SimpleNamespace(timesync_send=lambda *args: calls.append(args))
    dialect = SimpleNamespace(MAVLink_timesync_message=SimpleNamespace(fieldnames=['tc1', 'ts1']))
    responder = SITLTimesyncResponder(mav, dialect, now_ns=clock.now_ns,
                                      clock_domain=clock.clock_domain)
    message = SimpleNamespace(get_type=lambda: 'TIMESYNC', get_srcSystem=lambda: 1,
                              get_srcComponent=lambda: 1, tc1=0, ts1=1_000_000_000)
    assert responder.handle(message)['action'] == 'clock_unavailable'
    clock.update(1_000_000_000)
    assert responder.handle(message)['action'] == 'responded'
    message.ts1 += 100_000_000
    assert responder.handle(message)['reason'] == 'simulation_clock_not_advanced'
    host[0] += 300_000_000
    assert responder.handle(message)['reason'] == 'simulation_clock_stalled'
    assert len(calls) == 1
    clock.update(1_100_000_000)
    assert responder.handle(message)['action'] == 'responded'
    assert calls[-1] == (1_100_000_000, 1_100_000_000)
