"""Leased C++ BT decisions; worker I/O never blocks the ROS executor.

The C++ process has no FC transport. There is no Python decision fallback.
"""
from dataclasses import asdict
import hashlib
import json
import math
import os
from pathlib import Path
import select
import subprocess
import tempfile
import threading
import time
import uuid

SCHEMA = 'sangwon-field-ai/1'
BACKEND = 'sangwon_cpp_bt'


def inspect_binary(path):
    p = Path(path).resolve(strict=True)
    if not p.is_file() or not os.access(p, os.X_OK):
        raise ValueError('cpp_ai_executable_required')
    return p, hashlib.sha256(p.read_bytes()).hexdigest()


def state_input(s, feedback_age_s):
    names = ('connected', 'armed', 'landed', 'mode', 'xy', 'uwb_xy', 'velocity_xy',
             'vertical_speed_m_s', 'altitude_enu_m', 'pose_age_s', 'uwb_age_s',
             'state_age_s', 'landed_age_s', 'estimator_age_s', 'estimator_valid',
             'vertical_estimator_valid', 'bridge_ready', 'alignment_matches',
             'bridge_observation_age_s', 'bridge_age_s', 'bridge_contract_error', 'bridge_stamp_ns',
             'battery', 'battery_age_s', 'uwb_stamp_ns', 'lidar_scan', 'lidar_input_error')
    def clean(v):
        if isinstance(v, (list, tuple)): return [clean(x) for x in v]
        if isinstance(v, dict): return {k:clean(x) for k,x in v.items()}
        if type(v) is float and not math.isfinite(v): return None
        return v
    values = {k: clean(getattr(s, k)) for k in names}
    values['feedback_age_s'] = clean(feedback_age_s)
    return values


class CppMissionAI:
    def __init__(self, binary):
        path, self.binary_sha256 = inspect_binary(binary)
        self.session_id = uuid.uuid4().hex
        self.lock = threading.Lock()
        self.condition = threading.Condition(self.lock)
        self.error = None
        self.pending = self.reply = self.preparation = None
        self.preparing = False
        self.sequence = 0
        self.reply_received_s = None
        self.last_roundtrip_s = self.max_roundtrip_s = 0.
        self.last_submit_s = self.max_submit_s = self.max_input_to_reply_s = 0.
        self.plan_ref = None
        self.prepare_home = self.prepare_ground_height = None
        self.stderr = tempfile.TemporaryFile()
        self.process = subprocess.Popen([str(path), '--stdio'], stdin=subprocess.PIPE,
            stdout=subprocess.PIPE, stderr=self.stderr, bufsize=0)
        os.set_blocking(self.process.stdin.fileno(),False)
        try:
            hello = self._exchange('hello', {}, timeout=2.)
            hello.pop('_sequence')
            if hello != {'tree': 'FieldMission', 'mission_scope': 'LOCAL_XY_HOVER_RETURN'}:
                raise ValueError('cpp_ai_capability_mismatch')
        except BaseException:
            self.close()
            raise
        self.stopped = False
        self.worker = threading.Thread(target=self._run, name='cpp-mission-ai', daemon=True)
        self.worker.start()

    def _exchange(self, method, payload, timeout=.1):
        started=time.monotonic();end=started+timeout
        self.sequence += 1
        req = dict(schema=SCHEMA, session_id=self.session_id, sequence=self.sequence,
                   method=method, payload=payload)
        raw = json.dumps(req, allow_nan=False, separators=(',', ':')).encode('utf-8') + b'\n'
        if len(raw) > 262144: raise ValueError('cpp_ai_request_too_large')
        if time.monotonic()>=end:raise ValueError('cpp_ai_request_encoding_timeout')
        offset=0
        while offset<len(raw):
            remaining=end-time.monotonic()
            if remaining<=0 or not select.select([], [self.process.stdin], [], remaining)[1]:
                raise ValueError('cpp_ai_write_timeout')
            try:offset+=os.write(self.process.stdin.fileno(),raw[offset:offset+4096])
            except BlockingIOError:continue
        data = bytearray()
        while not data.endswith(b'\n'):
            if not select.select([self.process.stdout], [], [], max(0., end-time.monotonic()))[0]:
                raise ValueError('cpp_ai_reply_timeout')
            chunk = os.read(self.process.stdout.fileno(), 65536)
            if not chunk: raise ValueError('cpp_ai_process_exited')
            data.extend(chunk)
            if len(data) > 65536: raise ValueError('cpp_ai_reply_too_large')
        response = json.loads(data)
        if (response.get('schema') != SCHEMA or response.get('backend') != BACKEND
                or response.get('session_id') != self.session_id
                or response.get('sequence') != self.sequence
                or response.get('fc_transport') is not False or response.get('ok') is not True):
            raise ValueError('cpp_ai_rejected:' + str(response.get('code', 'reply_contract_mismatch')))
        answer = response['payload']
        self.last_roundtrip_s=time.monotonic()-started
        self.max_roundtrip_s=max(self.max_roundtrip_s,self.last_roundtrip_s)
        answer['_sequence'] = self.sequence
        return answer

    def _run(self):
        while True:
            with self.condition:
                self.condition.wait_for(lambda: self.stopped or self.pending is not None)
                if self.stopped: return
                method, value = self.pending
                self.pending = None
            try:
                result = self._exchange(method, value)
                with self.condition:
                    if method == 'prepare':
                        self.preparation = result
                        self.plan_ref = result['plan_ref']
                    elif method == 'release':
                        if result.get('execution_id') != self.execution_id or result.get('plan_ref') != self.plan_ref:
                            raise ValueError('cpp_ai_release_context_mismatch')
                        if self.reply: self.reply = {**self.reply, **result, 'expires_s': 0.}
                    else:
                        if (result.get('execution_id') != value['execution_id']
                                or result.get('plan_ref') != value['plan_ref']
                                or result.get('sample_time_s') != value['sample_time_s']
                                or result.get('expires_s') != value['sample_time_s'] + .15):
                            raise ValueError('cpp_ai_decision_context_mismatch')
                        self.reply = result
                        self.reply_received_s = time.monotonic()
                        self.max_input_to_reply_s=max(self.max_input_to_reply_s,
                            self.reply_received_s-value['sample_time_s'])
            except Exception as exc:
                with self.condition:
                    self.error = str(exc)
                    self.stopped = True
                self.process.terminate()
                return

    def prepare(self, request, settings, s, ceiling):
        with self.condition:
            if self.error: raise ValueError(self.error)
            if self.preparing and request['request_id'] != self.execution_id:
                raise ValueError('cpp_ai_new_request_requires_new_session')
            if not self.preparing:
                self.execution_id = request['request_id']
                self.prepare_home = tuple(s.xy)
                self.prepare_ground_height = s.altitude_enu_m
                config = asdict(settings)
                self.pending = ('prepare', dict(execution_id=self.execution_id, settings=config,
                    home_xy=self.prepare_home, height_reference_m=s.altitude_enu_m+(0. if settings.start_mode == 'takeoff_hold' else settings.expected_mis_takeoff_alt_m),
                    waypoints=request['waypoints'], ceiling_height_m=ceiling))
                self.preparing = True
                self.condition.notify()
            return self.preparation is not None

    def step(self, s, now, *, executor_phase, feedback_age_s, landing_confirmed=False,
             executor_failure='', command=''):
        submitted_at=time.monotonic()
        with self.condition:
            if self.error: raise ValueError(self.error)
            if self.process.poll() is not None: raise ValueError('cpp_ai_process_exited')
            if self.plan_ref is None: return None
            self.pending = ('step', dict(execution_id=self.execution_id, plan_ref=self.plan_ref,
                sample_time_s=now, executor_phase=executor_phase, executor_failure=executor_failure,
                command=command, landing_confirmed=landing_confirmed, state=state_input(s, feedback_age_s)))
            self.condition.notify()
            self.last_submit_s=time.monotonic()-submitted_at
            self.max_submit_s=max(self.max_submit_s,self.last_submit_s)
            reply = self.reply
            if reply and 0 <= now-reply['sample_time_s'] <= .15:
                return dict(reply)
            return None

    def status(self, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            return dict(backend=BACKEND, session_id=self.session_id,
                process_id=self.process.pid,
                binary_sha256=self.binary_sha256, connected=not self.error and self.process.poll() is None,
                plan_ref=self.plan_ref, phase=self.reply.get('phase') if self.reply else None,
                decision_sequence=self.reply.get('_sequence') if self.reply else None,
                decision_age_s=max(0., now-self.reply['sample_time_s']) if self.reply else None,
                last_roundtrip_s=self.last_roundtrip_s,max_roundtrip_s=self.max_roundtrip_s,
                last_submit_s=self.last_submit_s,max_submit_s=self.max_submit_s,
                max_input_to_reply_s=self.max_input_to_reply_s,
                error=self.error, fc_transport=False)

    def release(self, outcome, reason):
        with self.condition:
            if self.error or self.plan_ref is None or self.process.poll() is not None: return
            self.pending = ('release', dict(execution_id=self.execution_id, plan_ref=self.plan_ref,
                outcome=outcome, reason=reason))
            self.condition.notify()

    def close(self):
        if hasattr(self, 'condition'):
            with self.condition:
                self.stopped = True
                self.condition.notify_all()
        if hasattr(self, 'process'):
            if self.process.poll() is None: self.process.terminate()
            try: self.process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                self.process.kill(); self.process.wait(timeout=1)
            self.process.stdin.close(); self.process.stdout.close()
        if hasattr(self, 'worker'): self.worker.join(timeout=1)
        if hasattr(self, 'stderr'): self.stderr.close()
