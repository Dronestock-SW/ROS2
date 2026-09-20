"""Check durable records and duplicate boundaries with no camera attached."""
import sqlite3

import pytest
from drone_bringup.qr_record_store import QrRecordStore

ITEM = dict(schema='drone-stock-item/v1', code='TEST', name='제품')


def test_continuous_reads_and_reappearance(tmp_path):
    store = QrRecordStore(str(tmp_path / 'qr.db'))
    first = store.record(ITEM, now=0)
    assert store.record(dict(reversed(list(ITEM.items()))), now=2) is None
    assert store.record(ITEM, now=4) is None
    second = store.record(ITEM, now=7)
    assert second['client_scan_id'] != first['client_scan_id']
    assert store.db.execute('SELECT count(*) FROM qr_observations').fetchone()[0] == 2
    store.close()


def test_changed_location_and_restart_preserve_records(tmp_path):
    path = str(tmp_path / 'qr.db')
    store = QrRecordStore(path)
    first = store.record(ITEM, now=0)
    assert store.record(dict(ITEM, location='A-2'), now=1) is not None
    store.close()
    store = QrRecordStore(path)
    row = store.db.execute('SELECT client_scan_id, item_json FROM qr_observations '
                           'WHERE client_scan_id=?', (first['client_scan_id'],)).fetchone()
    assert row[0] == first['client_scan_id']
    assert '제품' in row[1]
    assert store.record(ITEM, now=0)['session_id'] != first['session_id']
    store.close()


def test_write_failure_does_not_suppress_retry(tmp_path):
    store = QrRecordStore(str(tmp_path / 'qr.db'))
    store.db.execute('PRAGMA query_only=ON')
    with pytest.raises(sqlite3.OperationalError):
        store.record(ITEM, now=0)
    store.db.execute('PRAGMA query_only=OFF')
    assert store.record(ITEM, now=1) is not None
    store.close()


@pytest.mark.parametrize('item', [[], {}, dict(ITEM, code=3), dict(ITEM, name='')])
def test_reject_invalid_data(tmp_path, item):
    store = QrRecordStore(str(tmp_path / 'qr.db'))
    with pytest.raises(ValueError):
        store.record(item)
    assert store.db.execute('SELECT count(*) FROM qr_observations').fetchone()[0] == 0
    store.close()


def test_parser_to_recorder_pipeline(tmp_path):
    import json
    from types import SimpleNamespace
    from unittest.mock import Mock

    from drone_bringup.qr_parser_node import QrParserNode
    from drone_bringup.qr_recorder_node import QrRecorderNode
    from std_msgs.msg import String

    store = QrRecordStore(str(tmp_path / 'qr.db'))
    logger = Mock()
    recorder = SimpleNamespace(_store=store, get_logger=lambda: logger)
    parser = SimpleNamespace(_pub=Mock(), get_logger=lambda: logger)
    parser._pub.publish.side_effect = lambda msg: QrRecorderNode._on_item(recorder, msg)
    for _ in range(3):
        QrParserNode._on_qr_data(parser, String(data=json.dumps(ITEM)))
    rows = store.db.execute('SELECT item_json FROM qr_observations').fetchall()
    assert len(rows) == 1
    assert json.loads(rows[0][0])['location'] == ''
    QrRecorderNode._on_item(recorder, String(data='broken JSON'))
    logger.error.assert_called_once()
    store.close()
