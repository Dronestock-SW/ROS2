"""Compatibility path. Implementation: drone_uwb.processing.solvers.intersections."""
import importlib
import sys

sys.modules[__name__] = importlib.import_module("drone_uwb.processing.solvers.intersections")
