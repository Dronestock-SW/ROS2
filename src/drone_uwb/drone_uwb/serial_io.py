"""Compatibility path. Implementation: drone_uwb.acquisition.serial_io."""
import importlib
import sys

sys.modules[__name__] = importlib.import_module("drone_uwb.acquisition.serial_io")
