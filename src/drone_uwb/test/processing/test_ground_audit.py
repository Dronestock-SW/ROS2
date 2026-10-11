"""Transport window acceptance must reject partial windows and burst losses."""
import pytest
from drone_uwb.integration.ground_audit import audit
from test_tdma_stream import event, pair, status


def events(skip=(), tag='6', stop=4):
    yield event(status(tag), 1_000_000_000)
    for seq in range((stop-1)*40+1):
        if seq in skip:
            continue
        raw, diag = pair(seq, tag)
        mono = 1_000_000_000+seq*25_000_000
        yield event(raw, mono)
        yield event(diag, mono+1_000_000)


def test_complete_original_receipt_windows_and_no_flight_claim():
    out = audit(events(), '6', seconds=2, warmup_s=1)
    assert out['complete_window'] and out['transport_pass']
    assert out['count'] == 80 and out['min_1s_hz'] == 40
    assert not out['fusion_verified'] and not out['flight_verified']


@pytest.mark.parametrize('skip,stop,reason', [((50,51),4,'consecutive_loss_pass'),
    (tuple(range(50,60)),4,'rate_pass'), ((),3,'complete_window')])
def test_missing_or_partial_data_cannot_pass(skip,stop,reason):
    out = audit(events(skip=skip,stop=stop), '6', seconds=2, warmup_s=1)
    assert not out[reason] and not out['transport_pass']


def test_wrong_tag_and_empty_data_are_not_a_measurement():
    assert not audit(events(tag='5'), '6', seconds=2, warmup_s=1)['transport_pass']
    assert not audit([], '6', seconds=2)['complete_window']
