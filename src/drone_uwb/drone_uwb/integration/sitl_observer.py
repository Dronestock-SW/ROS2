"""Compatibility path. Implementation: drone_uwb.integration.sitl.sitl_observer."""
import importlib
import sys

sys.modules[__name__] = importlib.import_module("drone_uwb.integration.sitl.sitl_observer")
