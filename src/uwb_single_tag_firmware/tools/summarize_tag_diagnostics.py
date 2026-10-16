"""Read-only summary of Tag B diagnostic JSONL; no device or network access.

RAW opportunities, actual attempts, and selectively emitted detailed events have
different denominators. This tool does not infer a physical RF failure cause.
"""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path
import sys


CAUSES = (
    'None', 'NotReady', 'TxStartRejected', 'RxTimeout', 'RxError',
    'RadioDeadline', 'McuGuard', 'FrameLength', 'FrameIdentity',
    'ReportRejected', 'TofInvalid', 'RangeInvalid', 'ClockMapInvalid',
    'Budget', 'Success',
)
STAGES = ('NotStarted', 'PollTx', 'ResponseRx', 'FinalTx', 'ReportRx',
          'Distance', 'ClockMap', 'Done')
KINDS = {'uwb_raw_cycle', 'uwb_raw_status', 'uwb_rf_attempt', 'uwb_rf_recovery'}
COUNTERS = ('diag_attempts', 'diag_dropped', 'diag_stale', 'diag_high_water')


def uint(value, maximum=0xffffffff):
    return type(value) is int and 0 <= value <= maximum


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def identity(message):
    require(message.get('schema') == 2 and type(message.get('schema')) is int
            and message.get('tag_id') == '6', 'identity_schema_or_tag')
    startup = message.get('type') == 'uwb_rf_recovery' and message.get('reason') == 'startup'
    require(uint(message.get('boot_id')) and message['boot_id']
            and uint(message.get('session')) and (message['session'] or startup), 'identity_boot_or_session')
    return message['boot_id'], message['session']


def array(message, name, maximum):
    values = message.get(name)
    require(type(values) is list and len(values) == 4
            and all(uint(v, maximum) for v in values), name)
    return values


def unwrap(row):
    """Accept direct UART, capture envelopes, and ROS capture value wrappers."""
    for _ in range(4):
        if not isinstance(row, dict) or row.get('direction') == 'host_to_tag':
            return None
        if row.get('type') in KINDS:
            return row
        row = row.get('message', row.get('value'))
    return None


def ratio(numerator, denominator):
    return dict(numerator=numerator, denominator=denominator,
                fraction=numerator / denominator if denominator else None)


def labelled(counts, labels):
    return {name: counts[index] for index, name in enumerate(labels)}


class Summary:
    def __init__(self):
        self.records = Counter()
        self.invalid = Counter()
        self.seen = {}
        self.raw = Counter()
        self.anchors = {str(i): Counter() for i in range(1, 5)}
        self.first_causes = Counter()
        self.final_causes = Counter()
        self.first_failures = Counter()
        self.final_failures = Counter()
        self.events = Counter()
        self.event_causes = Counter()
        self.event_stages = Counter()
        self.heartbeats = defaultdict(list)
        self.recoveries = []
        self.raw_sources = Counter()

    def unique(self, key, message, kind):
        canonical = json.dumps(message, sort_keys=True, separators=(',', ':'))
        if key in self.seen:
            self.records[kind + '_duplicates'] += 1
            if self.seen[key] != canonical:
                self.records[kind + '_conflicting_duplicates'] += 1
            return False
        self.seen[key] = canonical
        return True

    def ingest(self, row):
        message = unwrap(row)
        if message is None:
            self.records['unrelated_or_host_to_tag'] += 1
            return
        kind = message['type']
        self.records[kind + '_seen'] += 1
        # Legacy rows remain visible; never invent missing first-attempt data.
        if kind != 'uwb_raw_status' and (type(message.get('diag_version')) is not int
                                       or message['diag_version'] != 1):
            self.records[kind + '_without_diag_v1'] += 1
            return
        if kind == 'uwb_raw_status' and 'diag_attempts' not in message:
            self.records['status_without_diagnostic_counters'] += 1
            return
        try:
            source = identity(message)
            if kind == 'uwb_raw_cycle':
                self.add_raw(source, message)
            elif kind == 'uwb_rf_attempt':
                self.add_attempt(source, message)
            elif kind == 'uwb_raw_status':
                self.add_status(source, message)
            else:
                self.add_recovery(source, message)
        except ValueError as exc:
            self.invalid[kind + ':' + str(exc)] += 1

    def add_raw(self, source, message):
        require(uint(message.get('seq')), 'seq')
        failures = array(message, 'failure', 13)
        first = array(message, 'first_failure', 13)
        attempts = array(message, 'attempts', 2)
        causes = array(message, 'cause', 14)
        first_causes = array(message, 'first_cause', 14)
        transactions = array(message, 'transaction', 0xffffffff)
        first_transactions = array(message, 'first_transaction', 0xffffffff)
        mask = message.get('valid_mask')
        require(uint(mask, 15), 'valid_mask')
        for i, count in enumerate(attempts):
            require(bool(mask & (1 << i)) == (failures[i] == 0), 'mask_failure_mismatch')
            require((causes[i] == 14) == (failures[i] == 0)
                    and (first_causes[i] == 14) == (first[i] == 0), 'success_cause_mismatch')
            if count == 0:
                require(transactions[i] == first_transactions[i] == 0
                        and failures[i] == first[i] and failures[i] in (1, 12)
                        and causes[i] == first_causes[i], 'unattempted_contract')
            elif count == 1:
                require(transactions[i] == first_transactions[i]
                        and failures[i] == first[i] and causes[i] == first_causes[i],
                        'single_attempt_contract')
            else:
                require(first[i] != 0 and transactions[i] != first_transactions[i],
                        'retry_without_first_failure_or_new_transaction')
        require(attempts.count(2) <= 1, 'multiple_retries_per_cycle')
        if not self.unique(('raw', *source, message['seq']), message, 'raw'):
            return
        self.raw['cycles'] += 1
        self.raw_sources[source] += 1
        for i, count in enumerate(attempts):
            metrics = dict(opportunities=1, attempted_opportunities=int(count > 0),
                           unattempted_opportunities=int(count == 0), ranging_attempts=count,
                           first_successes=int(first[i] == 0), final_successes=int(failures[i] == 0),
                           retries=int(count == 2), recovered_by_retry=int(count == 2 and failures[i] == 0))
            self.raw.update(metrics)
            self.anchors[str(i + 1)].update(metrics)
            self.first_causes[first_causes[i]] += 1
            self.final_causes[causes[i]] += 1
            self.first_failures[first[i]] += 1
            self.final_failures[failures[i]] += 1

    def add_attempt(self, source, message):
        limits = dict(seq=0xffffffff, anchor=4, attempt=2, transaction=0xffffffff,
                      failure=13, cause=14, stage=7, start_us=2**53-1, end_us=2**53-1,
                      radio_start=0xffffffff, radio_end=0xffffffff, status=0xffffffff,
                      wait_stage=7, wait_start_us=2**53-1, wait_end_us=2**53-1,
                      wait_radio_start=0xffffffff, wait_radio_end=0xffffffff,
                      deadline_radio=0xffffffff)
        require(all(uint(message.get(k), bound) for k, bound in limits.items()), 'attempt_fields')
        require(message['anchor'] >= 1 and message['attempt'] >= 1
                and message['end_us'] >= message['start_us']
                and message['wait_end_us'] >= message['wait_start_us'], 'attempt_order')
        require(all(type(message.get(k)) is bool for k in ('rf_progress', 'poll_tx_confirmed')),
                'attempt_flags')
        require((message['failure'] == 0) == (message['cause'] == 14), 'success_cause_mismatch')
        require(message['failure'] != 0 or message['attempt'] == 2, 'first_success_is_counter_only')
        key = ('attempt', *source, message['seq'], message['anchor'], message['attempt'])
        if not self.unique(key, message, 'attempt'):
            return
        self.events['received_unique'] += 1
        self.events['first_attempt' if message['attempt'] == 1 else 'retry'] += 1
        self.events['failures' if message['failure'] else 'successful_retries'] += 1
        self.events['poll_tx_confirmed'] += int(message['poll_tx_confirmed'])
        self.events['rf_progress_observed'] += int(message['rf_progress'])
        if message['cause'] == 6:
            self.events['mcu_guard_without_observed_rf_progress'] += int(not message['rf_progress'])
        self.event_causes[message['cause']] += 1
        self.event_stages[message['stage']] += 1

    def add_status(self, source, message):
        require(all(uint(message.get(k)) for k in COUNTERS), 'status_counters')
        values = message.get('diag_failures')
        require(type(values) is list and len(values) == len(CAUSES)
                and all(uint(v) for v in values), 'status_failure_array')
        require(values[14] == 0, 'success_is_not_a_failure')
        key = ('status', *source, *(message[k] for k in COUNTERS), *values)
        if not self.unique(key, message, 'status'):
            return
        # Firmware counters live across RF sessions and reset at MCU boot.
        self.heartbeats[source[0]].append({k: message[k] for k in (*COUNTERS, 'diag_failures')})

    def add_recovery(self, source, message):
        require(message.get('event') in ('scheduled', 'init_start', 'init_success', 'init_failed'),
                'recovery_event')
        require(message.get('reason') in ('startup', 'mcu_guard', 'tx_fault_streak', 'bench_request'),
                'recovery_reason')
        require(all(uint(message.get(k), 2**53-1) for k in ('at_us', 'detected_us', 'next_cycle_us'))
                and uint(message.get('seq')) and uint(message.get('recoveries')),
                'recovery_fields')
        require(message['at_us'] >= message['detected_us'], 'recovery_time_order')
        key = ('recovery', *source, message['seq'], message['event'], message['at_us'])
        if self.unique(key, message, 'recovery'):
            self.recoveries.append(message)

    @staticmethod
    def metrics(counts):
        result = dict(counts)
        opportunities = counts['opportunities']
        attempted = counts['attempted_opportunities']
        result.update(first_success_per_opportunity=ratio(counts['first_successes'], opportunities),
                      final_success_per_opportunity=ratio(counts['final_successes'], opportunities),
                      first_success_per_attempted_anchor=ratio(counts['first_successes'], attempted),
                      final_success_per_attempted_anchor=ratio(counts['final_successes'], attempted),
                      retry_recovery=ratio(counts['recovered_by_retry'], counts['retries']))
        return result

    def result(self):
        counters = []
        for boot, rows in sorted(self.heartbeats.items()):
            deltas = Counter()
            failure_deltas = Counter()
            regressions = 0
            for previous, current in zip(rows, rows[1:]):
                if (any(current[k] < previous[k] for k in COUNTERS)
                        or any(b < a for a, b in zip(previous['diag_failures'], current['diag_failures']))):
                    # A wrap/reorder/reset is not an unbounded positive delta.
                    regressions += 1
                    continue
                deltas.update({k: current[k] - previous[k] for k in COUNTERS})
                failure_deltas.update({i: b-a for i, (a, b) in enumerate(
                    zip(previous['diag_failures'], current['diag_failures']))})
            counters.append(dict(boot_id=boot, distinct_snapshots=len(rows), first=rows[0], last=rows[-1],
                                 observed_nonregressing_deltas=dict(deltas),
                                 observed_failure_deltas=labelled(failure_deltas, CAUSES),
                                 counter_regressions=regressions,
                                 maximum_reported={k: max(row[k] for row in rows) for k in COUNTERS}))
        losses = any(row['maximum_reported']['diag_dropped'] or row['maximum_reported']['diag_stale']
                     for row in counters)
        recovery_counts = Counter((row['reason'], row['event']) for row in self.recoveries)
        starts = {}
        timings = []
        for row in sorted(self.recoveries, key=lambda item: (item['boot_id'], item['at_us'])):
            key = row['boot_id'], row['recoveries']
            if row['event'] == 'init_start':
                starts[key] = row
                timings.append(dict(boot_id=row['boot_id'], recoveries=row['recoveries'],
                                    reason=row['reason'], metric='detected_to_init_start_us',
                                    value=row['at_us']-row['detected_us']))
            elif row['event'] in ('init_success', 'init_failed') and key in starts:
                timings.append(dict(boot_id=row['boot_id'], recoveries=row['recoveries'],
                                    reason=row['reason'], metric=row['event']+'_duration_us',
                                    value=row['at_us']-starts.pop(key)['at_us']))
        warnings = [
            'Ranging attempts include rejected TX starts; they are not confirmed on-air transmissions.',
            'Detailed events omit normal first-attempt successes; their counts are not a total-attempt denominator.',
            'RAW rates cover received unique cycles only; absent cycles are not reconstructed.',
            'McuGuard and rf_progress=false do not establish DW3000 failure or a physical RF root cause.',
            'Zero observed diagnostic drops does not prove complete capture; startup/tail/host losses remain possible.',
            'Counter deltas cover heartbeat endpoints, not necessarily the RAW time window; compare only aligned windows.',
            'Bench recovery bypasses the natural 1500 ms wait; compare reasons separately.',
        ]
        if losses:
            warnings.append('Firmware reported cumulative diagnostic queue loss or stale drops; detail is incomplete.')
        if not counters:
            warnings.append('No diagnostic heartbeat counters are available; queue loss is unknown.')
        if self.invalid or self.records['malformed_json_or_utf8']:
            warnings.append('Malformed or contract-invalid records were excluded and counted.')
        return dict(schema=1, diagnostic_version=1, evidence_kind='record_summary_not_hardware_validation',
                    physical_rf_cause_confirmed=False, records=dict(self.records), invalid_records=dict(self.invalid),
                    raw=self.metrics(self.raw), anchors={k: self.metrics(v) for k, v in self.anchors.items()},
                    raw_sources=[dict(boot_id=b, session=s, cycles=n) for (b, s), n in sorted(self.raw_sources.items())],
                    first_causes=labelled(self.first_causes, CAUSES), final_causes=labelled(self.final_causes, CAUSES),
                    first_failure_codes=dict(self.first_failures), final_failure_codes=dict(self.final_failures),
                    detailed_events=dict(self.events), detailed_event_causes=labelled(self.event_causes, CAUSES),
                    detailed_event_stages=labelled(self.event_stages, STAGES), heartbeat_counters=counters,
                    diagnostic_loss_reported_cumulative=losses if counters else None,
                    recovery_counts=[dict(reason=r, event=e, count=n) for (r, e), n in sorted(recovery_counts.items())],
                    recovery_timings=timings, warnings=warnings)


def decode_json(line):
    def reject_constant(value):
        raise ValueError('nonfinite_json:' + value)
    def unique_keys(pairs):
        row = {}
        for key, value in pairs:
            require(key not in row, 'duplicate_json_key')
            row[key] = value
        return row
    return json.loads(line, parse_constant=reject_constant, object_pairs_hook=unique_keys)


def summarize(paths):
    summary = Summary()
    sources = []
    for path in map(Path, paths):
        digest = hashlib.sha256()
        size = lines = 0
        with path.open('rb') as stream:
            for lines, raw in enumerate(stream, 1):
                digest.update(raw)
                size += len(raw)
                if not raw.strip():
                    summary.records['blank_lines'] += 1
                    continue
                try:
                    row = decode_json(raw.decode('utf-8-sig' if lines == 1 else 'utf-8'))
                except (ValueError, UnicodeError):
                    summary.records['malformed_json_or_utf8'] += 1
                    continue
                summary.ingest(row)
        sources.append(dict(path=str(path.resolve()), sha256=digest.hexdigest(), bytes=size, lines=lines))
    return dict(sources=sources, **summary.result())


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('inputs', type=Path, nargs='+', help='Read-only JSONL files, in capture order')
    parser.add_argument('--output', type=Path, help='New UTF-8 JSON summary; existing files are never overwritten')
    args = parser.parse_args(argv)
    if args.output is not None and args.output.exists():
        parser.error('output must be a new file; existing files and input aliases are never overwritten')
    result = summarize(args.inputs)
    payload = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + '\n'
    if args.output is not None:
        with args.output.open('x', encoding='utf-8', newline='\n') as stream:
            stream.write(payload)
    else:
        if hasattr(sys.stdout, 'reconfigure'):
            sys.stdout.reconfigure(encoding='utf-8')
        sys.stdout.write(payload)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
