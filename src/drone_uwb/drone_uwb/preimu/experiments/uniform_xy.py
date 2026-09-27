"""Compatibility path. Implementation: drone_uwb.processing.experiments.uniform_xy."""
import importlib
import sys

sys.modules[__name__] = importlib.import_module("drone_uwb.processing.experiments.uniform_xy")
