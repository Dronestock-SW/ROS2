"""Clock optimization must preserve NumPy's exact finite-sample statistics."""
import numpy as np
import pytest

from drone_uwb.processing.timing.clock import _median, _percentile95


@pytest.mark.parametrize('count', [1,2,3,19,20,21,30,31,199,200,399,400,401])
def test_clock_statistics_match_numpy_exactly(count):
    rng = np.random.default_rng(813)
    for values in (rng.normal(0, 1e7, count), np.ones(count),
                   np.arange(count, dtype=float), rng.integers(-9,10,count).astype(float)):
        assert _median(values) == float(np.median(values))
        assert _percentile95(values) == float(np.percentile(values, 95))


def test_nonfinite_clock_statistics_keep_numpy_behavior():
    for fn, reference in ((_median, np.median), (_percentile95, lambda x:np.percentile(x,95))):
        assert np.isnan(fn([1.,float('nan')])) and np.isnan(reference([1.,float('nan')]))


def test_clock_states_mapping_and_limits_match_previous_statistics(monkeypatch):
    from drone_uwb.processing.settings import PreimuSettings
    from drone_uwb.processing.timing import clock
    optimized, previous = clock.ClockMap(PreimuSettings()), clock.ClockMap(PreimuSettings())
    rng = np.random.default_rng(51)
    for i in range(560):
        esp = 1_000_000+i*25_000
        delay = int(rng.integers(0, 1_000_000))+(60_000_000 if 300<=i<320 else 0)
        host = 10_000_000_000+round(esp*1000.1)+delay
        optimized.update(esp, host)
        with monkeypatch.context() as patch:
            patch.setattr(clock, '_median', lambda v:float(np.median(v)))
            patch.setattr(clock, '_percentile95', lambda v:float(np.percentile(v,95)))
            previous.update(esp, host)
        assert optimized.diagnostics() == previous.diagnostics()
        assert optimized.beta == previous.beta
        assert optimized.host_s(esp) == previous.host_s(esp)
