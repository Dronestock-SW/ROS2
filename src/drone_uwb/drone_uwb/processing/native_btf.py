"""Optional in-process C++ B_TF. Transport, source clocks and height stay upstream.

An explicitly selected missing/broken backend fails closed; no silent fallback.
Each instance owns one native handle and serializes calls including teardown.
"""
import ctypes
import hashlib
import json
from pathlib import Path
import threading
import time


class NativeBtf:
    def __init__(self, config):
        self._lock = threading.RLock()
        self._handle = None
        path = Path(config['native_btf_library'])
        if not path.is_absolute() or not path.is_file():
            raise ValueError('absolute_native_btf_library_required')
        self.identity = dict(backend='cpp', abi=1, library=str(path.resolve()),
                             fresh_b4_recovery_enabled=bool(config.get('fresh_b4_recovery_enabled', False)),
                             sha256=hashlib.sha256(path.read_bytes()).hexdigest())
        self._lib = ctypes.CDLL(str(path))
        self._lib.btf_abi_version.argtypes = []
        self._lib.btf_abi_version.restype = ctypes.c_int
        if self._lib.btf_abi_version() != 1:
            raise ValueError('native_btf_abi_mismatch')
        self._lib.btf_create.argtypes = []
        self._lib.btf_create.restype = ctypes.c_void_p
        self._lib.btf_destroy.argtypes = [ctypes.c_void_p]
        self._lib.btf_destroy.restype = None
        self._lib.btf_request.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
        self._lib.btf_request.restype = ctypes.c_char_p
        self._handle = self._lib.btf_create()
        if not self._handle:
            raise RuntimeError('native_btf_allocation_failed')
        self._call(action='configure', config=config)

    def _call(self, **request):
        with self._lock:
            if not self._handle:
                raise ValueError('native_btf_closed')
            result = json.loads(self._lib.btf_request(
                self._handle, json.dumps(request, allow_nan=False).encode()))
            if 'error' in result:
                raise RuntimeError('native_btf:' + result['error'])
            return result

    def reset(self, *, preserve_motion=False, preserve_height_conflict=True):
        self._call(action='reset', preserve_motion=preserve_motion,
                   preserve_height_conflict=preserve_height_conflict)

    def process(self, cycle, heights, height_source):
        started = time.perf_counter_ns()
        result = self._call(action='process', frame=dict(
            cycle_start_us=int(cycle.start_us), time_us=int(cycle.end_us),
            valid_mask=sum(1 << i for i in cycle.indices),
            sample_time_us=[int(t) if i in cycle.indices else None for i,t in enumerate(cycle.sample_us)],
            raw_slant_m=[float(r) if i in cycle.indices else None for i,r in enumerate(cycle.raw)], sample_height_m=heights,
            height_source=height_source or 'missing'))
        result['native_call_ms'] = (time.perf_counter_ns()-started)/1e6
        # Keep the old reason for compatibility, but never describe impossible
        # measured ranges as solely a missing-height/transport problem.
        causes = []
        if result.get('range_geometry_reason') == 'ranges_inconsistent_independent_of_height':
            causes.append('range_geometry_inconsistent')
        if result['reason'] in ('measured_height_unavailable', 'partial_measured_height_required'):
            causes.append('measured_height_unavailable')
        if not result['ok'] and not causes:
            causes.append(result['reason'])
        result['observation_issues'] = causes
        b = dict(ok=result['B']['ok'], reason=result['B']['reason'],
                 xy_m=result['B']['xy_m'], velocity_m_s=result['B']['velocity_m_s'],
                 fit=result['B'], fresh_observation_count=result['fresh_observation_count'],
                 newest_sample_us=result['newest_sample_us'],
                 window_count=result['history_samples'], reset_reason=result['reset_reason'])
        result['numerical_backend'] = self.identity
        return b, result

    def close(self):
        with self._lock:
            if self._handle:
                self._lib.btf_destroy(self._handle)
                self._handle = None

    def __del__(self):
        if getattr(self, '_handle', None):
            self.close()
