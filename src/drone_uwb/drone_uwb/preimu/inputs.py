"""Compatibility path. Implementation: drone_uwb.acquisition.validation."""
import importlib
import sys

sys.modules[__name__] = importlib.import_module("drone_uwb.acquisition.validation")
