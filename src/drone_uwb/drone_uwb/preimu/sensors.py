"""Compatibility path. Implementation: drone_uwb.processing.sensors."""
import importlib
import sys

sys.modules[__name__] = importlib.import_module("drone_uwb.processing.sensors")
