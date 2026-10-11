import datetime as dt
import json
import math
import os
from pathlib import Path
import tempfile

MAX_BYTES = 2 * 1024 * 1024

def utc_now():
    return dt.datetime.now(dt.timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")

def encode(value):
    return json.dumps(value, ensure_ascii=False, separators=(",", ":"), sort_keys=True, allow_nan=False).encode("utf-8")

def decode(raw):
    if len(raw) > MAX_BYTES:
        raise ValueError("MESSAGE_TOO_LARGE")
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError("DUPLICATE_JSON_KEY")
            result[key] = value
        return result
    def bad_constant(value):
        raise ValueError("NON_FINITE_JSON")
    def finite_float(value):
        number = float(value)
        if not math.isfinite(number):
            raise ValueError("NON_FINITE_JSON")
        return number
    return json.loads(raw, object_pairs_hook=pairs, parse_constant=bad_constant, parse_float=finite_float)

def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    fd, tmp = tempfile.mkstemp(dir=path.parent, prefix=".atomic-")
    try:
        with os.fdopen(fd, "wb") as handle:
            handle.write(encode(value) + b"\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)

def runtime_path(root, relative):
    path = (root / relative).resolve()
    if not path.is_relative_to((root / ".runtime").resolve()):
        raise ValueError("STATE_PATH_OUTSIDE_RUNTIME")
    return path
