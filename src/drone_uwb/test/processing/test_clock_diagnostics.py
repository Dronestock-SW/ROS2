from drone_uwb.processing.settings import PreimuSettings
from drone_uwb.processing.timing.clock import ClockMap


def test_clock_reasons_distinguish_rate_from_queue_residual():
    c=ClockMap(PreimuSettings())
    assert c.diagnostics()['reason']=='insufficient_samples'
    for n in range(301): c.update(1_000_000+n*25000, int(10e9+n*25e6*.99))
    assert not c.ready and c.diagnostics()['reason']=='scale_out_of_bounds'
    assert c.diagnostics()['residual_p95_s'] < .001
    c.reset()
    for n in range(301):
        c.update(1_000_000+n*25000, int(10e9+n*25e6+(70e6 if n%2 else 0)))
    assert not c.ready and c.diagnostics()['residual_p95_s'] > .05
    assert not c.diagnostics()['fixed_delay_calibrated']
