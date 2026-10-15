"""Read a bounded tail of an ongoing capture without adding a ROS subscriber.

This is a progress snapshot, never a replacement for the completed summary.
Run on the recording host so its monotonic time has the same origin.
"""
import argparse
import json
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('capture', type=Path)
    args = parser.parse_args()
    meta = json.loads((args.capture/'capture.json').read_text(encoding='utf-8'))
    events = args.capture/'events.jsonl'
    with events.open('rb') as stream:
        stream.seek(0, 2)
        size = stream.tell()
        start = max(0, size-1024*1024)
        stream.seek(start)
        if start:
            stream.readline()  # The first line can be incomplete.
        rows = stream.read().splitlines()
    latest = {}
    latest_ns = None
    for line in rows:
        try:
            row = json.loads(line)
        except (ValueError, UnicodeDecodeError):
            continue  # A concurrent writer can leave the last line incomplete.
        latest_ns = row['capture_monotonic_ns']
        value = row['value']
        latest[(row['topic'], value.get('type'))] = value
    raw = latest.get(('/uwb/raw', 'uwb_raw_cycle'), {})
    heartbeat = latest.get(('/uwb/raw', 'uwb_raw_status'), {})
    now = time.monotonic_ns()
    print(json.dumps(dict(
        progress_only=True,
        elapsed_s=round((now-meta['measurement_start_monotonic_ns'])/1e9, 3),
        planned_s=meta['seconds'], bytes_written=size,
        latest_record_age_ms=round((now-latest_ns)/1e6, 3) if latest_ns else None,
        raw={key: raw.get(key) for key in ('boot_id', 'session', 'seq', 'valid_mask',
             'skipped_cycles', 'tx_dropped', 'clock_sync_valid')},
        heartbeat={key: heartbeat.get(key) for key in ('lora_tx_done', 'lora_tx_timeouts',
                   'lora_watchdog_timeouts', 'lora_recoveries', 'uwb_recoveries',
                   'free_heap', 'reset_reason', 'bench_controls')})))


if __name__ == '__main__':
    main()
