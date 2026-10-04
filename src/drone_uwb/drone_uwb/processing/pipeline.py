"""One-process pipeline scaffold. All height/position calculations and external outputs are closed.

Only validation, time bookkeeping, range calibration/gating and sensor buffering run.
rawxy.py, h80.py, qs10.py, height.py and transform.py remain independently callable math modules;
they are intentionally not imported here. This is not the final jetson_uwb_preimu output schema.
"""
from copy import deepcopy
from dataclasses import asdict, dataclass, field

from drone_uwb.contracts.protocol import integer
from drone_uwb.processing.timing.clock import ClockMap
from drone_uwb.acquisition.validation import InputValidator, InvalidInput
from drone_uwb.processing.ranges import RangeDecision, RangeGate, RangeHistory, subtract_bias
from drone_uwb.processing.timing.sensors import SensorInputs
from drone_uwb.processing.settings import PreimuSettings, settings_from_mapping


@dataclass
class PipelineConfig:
    preimu: PreimuSettings = field(default_factory=PreimuSettings)
    range_bias_source: str = 'unconfigured'
    calculation_enabled: bool = False
    external_output_enabled: bool = False

    def __post_init__(self):
        if self.calculation_enabled is not False or self.external_output_enabled is not False:
            raise ValueError('pipeline_only: calculation and external output gates must stay closed')
        if not isinstance(self.range_bias_source, str) or not self.range_bias_source:
            raise ValueError('range_bias_source required')

    @classmethod
    def from_mapping(cls, values):
        values = dict(values)
        values['preimu'] = settings_from_mapping(values.get('preimu', {}))
        return cls(**values)


class Pipeline:
    def __init__(self, config):
        self.config = deepcopy(config)
        self.settings = self.config.preimu
        # The runner is a recorded session, not a live UART watchdog.
        self.validator = InputValidator(self.settings, require_recent_status=False)
        self.last_host_ns = None
        self.uwb_source = None
        self.reset_temporal()

    def reset_temporal(self):
        self.clock = ClockMap(self.settings)
        self.gates = [RangeGate(self.settings) for _ in range(4)]
        self.history = RangeHistory(self.settings.window_s)
        self.sensors = SensorInputs()
        self.last_source_us = None
        self.last_cycle_host_ns = None

    def _base(self, event):
        message = event.get('message', {})
        message = message if isinstance(message, dict) else {}
        return {
            'type': 'uwb_pipeline_diagnostic', 'schema': 1, 'mode': 'pipeline_only',
            'event_type': message.get('type'), 'seq': message.get('seq'),
            'source': event.get('source'),
            'host_rx_time_us': event.get('host_received_monotonic_ns', 0) // 1000
                if integer(event.get('host_received_monotonic_ns')) else None,
            'calculation_gate': 'closed', 'external_output_gate': 'closed',
            'valid': False, 'fresh': False, 'solver': 'none',
            'x_m': None, 'y_m': None, 'z_m': None,
            'reason': 'calculation_gate_closed',
            'stages': {'input': 'pending', 'clock': 'pending', 'range': 'pending',
                       'sensor_buffer': 'pending', 'height': 'blocked',
                       'raw_xy_h80_q': 'blocked', 'position_gate': 'blocked',
                       'coordinate_transform': 'blocked', 'external_output': 'blocked'},
        }

    def process(self, event):
        result = self._base(event)
        try:
            self._process(event, result)
        except InvalidInput as exc:
            if exc.reset:
                self.reset_temporal()
            result['reason'] = exc.reason
            result['stages']['input'] = 'rejected'
        except (ValueError, KeyError, TypeError) as exc:
            result['reason'] = str(exc)
            result['stages']['input'] = 'rejected'
        return result

    def _process(self, event, result):
        host_ns, source, message = (event[k] for k in ('host_received_monotonic_ns', 'source', 'message'))
        if not integer(host_ns) or host_ns < 0 or not isinstance(message, dict):
            raise ValueError('invalid_event')
        if source not in ('simulation', 'measured'):
            raise ValueError('input_source_required')
        if self.last_host_ns is not None and host_ns < self.last_host_ns:
            raise ValueError('host_time_reversed')
        self.last_host_ns = host_ns
        kind = message.get('type')
        if kind in ('tof_sample', 'attitude_sample'):
            self.sensors.add(message, source, host_ns)
            result['stages'].update(input='accepted', sensor_buffer='recorded')
            result['reason'] = 'sensor_recorded_calculation_closed'
            return
        if kind == 'pipeline_tick':
            age = (host_ns - self.last_cycle_host_ns) / 1e9 if self.last_cycle_host_ns is not None else None
            result.update(input_age_s=age, input_stale=age is None or age > self.settings.failed_after_s)
            result['reason'] = 'input_stale' if result['input_stale'] else 'calculation_gate_closed'
            result['stages']['input'] = 'tick'
            return
        if kind not in ('uwb_raw_status', 'uwb_raw_cycle'):
            raise ValueError('unsupported_type')
        self.validator.check_common(message)
        if self.uwb_source is not None and source != self.uwb_source:
            self.validator.disconnect()
            self.reset_temporal()
        self.uwb_source = source
        if kind == 'uwb_raw_status':
            if message.get('event') == 'boot':
                self.validator.disconnect()
                self.reset_temporal()
            self.validator.on_status(message, host_ns)
            result['stages']['input'] = 'status_accepted'
            return
        cycle = self.validator.on_cycle(message, host_ns, 0)
        self.last_cycle_host_ns = host_ns
        result['stages']['input'] = 'accepted'
        self.clock.update(cycle.end_us, host_ns)
        result['clock'] = {'state': self.clock.state, 'alpha': self.clock.alpha,
                           'residual_p95_s': self.clock.residual_p95_s,
                           'method': 'receive_lower_envelope', 'hardware_synchronized': False}
        result['stages']['clock'] = self.clock.state
        times = [round(self.clock.host_s(t) * 1e6) if self.clock.ready and t is not None else None
                 for t in cycle.sample_us]
        result['source_time_host_us'] = times
        usable_times = [times[i] for i in cycle.indices if times[i] is not None]
        target_us = max(usable_times) if usable_times else None
        result['measurement_time_us'] = target_us
        result['source_age_ms'] = (host_ns / 1000 - target_us) / 1000 if target_us is not None else None
        result['source_fresh'] = (0 <= result['source_age_ms'] <= self.settings.max_queue_s * 1000
                                  if target_us is not None else False)
        result['sensor_inputs'] = self.sensors.snapshot(target_us if target_us is not None else host_ns // 1000)
        result['sensor_selection_time_us'] = target_us if target_us is not None else host_ns // 1000
        result['stages']['sensor_buffer'] = 'snapshot' if target_us is not None else 'awaiting_uwb_clock'
        result['z_source'] = (result['sensor_inputs']['tof_sample']['sample'] or {}).get('source')
        result['raw_slant_m'] = deepcopy(message['raw_slant_m'])
        calibrated = subtract_bias(message['raw_slant_m'], self.settings.range_bias_m)
        result.update(cal_slant_m=calibrated, range_bias_m=list(self.settings.range_bias_m),
                      range_bias_source=self.config.range_bias_source,
                      range_bias_calibrated=self.settings.range_bias_calibrated,
                      seq_gap=cycle.seq_gap, input_excluded=cycle.excluded)
        decisions = [RangeDecision(False, reason=cycle.excluded.get(i, 'input_not_usable')) for i in range(4)]
        source_gap = False
        for i in sorted(cycle.indices, key=lambda i: cycle.sample_us[i]):
            stamp = cycle.sample_us[i]
            if self.last_source_us is not None and (stamp - self.last_source_us) / 1e6 > self.settings.recovery_gap_s:
                self.history.rows.clear()
                source_gap = True
            self.last_source_us = stamp
            decisions[i] = self.gates[i].update(stamp, calibrated[i])
            if decisions[i].accepted:
                self.history.append(i, stamp, decisions[i].value_m)
        self.history.prune(cycle.end_us)
        result.update(range_decisions=[asdict(d) for d in decisions],
                      range_accepted=[d.accepted for d in decisions],
                      range_pending_count=[d.pending_count for d in decisions],
                      accepted_range_m=[d.value_m for d in decisions],
                      history_counts=self.history.counts(), source_gap_detected=source_gap)
        result['stages']['range'] = 'processed'
        result['blocked_reasons'] = ['height_calculation_gate_closed', 'position_calculation_gate_closed',
                                     'external_output_gate_closed']
        if not self.clock.ready:
            result['blocked_reasons'].append('clock_unsynced')
        for name in ('tof_sample', 'attitude_sample'):
            if not result['sensor_inputs'][name]['available']:
                result['blocked_reasons'].append(name + '_unavailable')
        if any(d.reason == 'range_gate_pending' for d in decisions):
            result['blocked_reasons'].append('range_gate_pending')
