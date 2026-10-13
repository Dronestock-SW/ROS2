"""Bounded asynchronous evidence recording; no control decisions or FC writes."""
import json
import math
import os
import queue
import threading
import time


def clean(value):
    if isinstance(value, dict):
        return {k: clean(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [clean(v) for v in value]
    return None if type(value) is float and not math.isfinite(value) else value


class AsyncEventRecorder:
    def __init__(self, stream, capacity=4096):
        self.stream = stream
        self.queue = queue.Queue(capacity)
        self.error = None
        self.submitted = self.written = 0
        self.worker = threading.Thread(target=self._run, name='flight-evidence-writer', daemon=True)
        self.worker.start()

    def submit(self, row):
        if self.error:
            return False
        value = dict(row, record_sequence=self.submitted + 1)
        try:
            self.queue.put_nowait(value)
        except queue.Full:
            self.error = 'event_record_queue_full'
            return False
        self.submitted += 1
        return True

    def _run(self):
        while True:
            value = self.queue.get()
            try:
                if value is None:
                    return
                if isinstance(value, threading.Event):
                    self.stream.flush()
                    os.fsync(self.stream.fileno())
                    value.set()
                elif not self.error:
                    value['written_monotonic_ns'] = time.monotonic_ns()
                    self.stream.write(json.dumps(clean(value), ensure_ascii=False, allow_nan=False)+'\n')
                    self.stream.flush()
                    self.written += 1
            except (OSError, ValueError) as exc:
                self.error = 'event_record_write_failed:' + str(exc)
                if isinstance(value, threading.Event):
                    value.set()
            finally:
                self.queue.task_done()

    def sync(self, timeout=3.):
        barrier = threading.Event()
        try:
            self.queue.put(barrier, timeout=timeout)
        except queue.Full:
            return False
        return barrier.wait(timeout) and not self.error

    def close(self):
        ok = self.sync()
        try:
            self.queue.put(None, timeout=3.)
        except queue.Full:
            return False
        self.worker.join(3.)
        if self.worker.is_alive():
            return False
        self.stream.close()
        return ok


def sensor_sources(samples):
    out = {}
    for key, (value, received) in samples.items():
        if hasattr(value, 'header'):
            stamp = value.header.stamp
            out[key] = dict(stamp_ns=stamp.sec*1_000_000_000+stamp.nanosec,
                            received_monotonic_ns=round(received*1e9), frame_id=value.header.frame_id)
            if key=='estimator':
                names=('attitude_status_flag','velocity_horiz_status_flag','velocity_vert_status_flag',
                    'pos_horiz_rel_status_flag','pos_vert_abs_status_flag','const_pos_mode_status_flag',
                    'pred_pos_horiz_rel_status_flag','gps_glitch_status_flag','accel_error_status_flag')
                out[key]['flags']={k:getattr(value,k,None) for k in names}
        elif key in ('bridge','timesync'):
            out[key] = dict(received_monotonic_ns=round(received*1e9), report=value)
    return clean(out)


def flight_sample(sample, *, phase, target, feedback, observation, sources, owner):
    names = ('connected', 'mode', 'armed', 'landed', 'xy', 'uwb_xy', 'velocity_xy',
             'altitude_enu_m', 'vertical_speed_m_s', 'yaw_deg', 'battery',
             'pose_stamp_ns', 'uwb_stamp_ns', 'bridge_stamp_ns', 'bridge_published',
             'bridge_gate', 'bridge_ready', 'alignment_matches', 'bridge_contract_error',
             'estimator_valid', 'vertical_estimator_valid', 'state_age_s', 'landed_age_s',
             'estimator_age_s', 'pose_age_s', 'uwb_age_s', 'height_age_s', 'bridge_age_s',
             'bridge_observation_age_s', 'battery_age_s', 'rc_age_s', 'takeoff_param_age_s')
    inputs={k: getattr(sample,k) for k in names}
    return clean(dict(inputs, type='flight_sample', record_schema='flight-evidence/2', state=phase,
                      input_snapshot=inputs, target=target,
                      target_feedback=feedback, observation_status=observation,
                      sensor_sources=sources, executor_owner=owner))
