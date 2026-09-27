"""Shared JSON and scalar contract; no processing or device dependencies."""
import json
import math

class InvalidSample(ValueError):
    """A protocol or measurement validation failure."""


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def integer(value):
    return isinstance(value, int) and not isinstance(value, bool)


def decode_line(line):
    def invalid_constant(value):
        raise InvalidSample('nonfinite_json')
    try:
        obj = json.loads(line, parse_constant=invalid_constant)
    except (ValueError, UnicodeError) as exc:
        raise InvalidSample('invalid_json') from exc
    if not isinstance(obj, dict):
        raise InvalidSample('not_an_object')
    return obj
