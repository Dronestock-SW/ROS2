"""Compatibility path. Implementation: drone_uwb.integration.sitl.px4_clock_tracker."""
import importlib
import sys

sys.modules[__name__] = importlib.import_module("drone_uwb.integration.sitl.px4_clock_tracker")
