"""Compatibility path. Implementation: drone_uwb.processing.geometry.gazebo_geometry."""
import importlib
import sys

sys.modules[__name__] = importlib.import_module("drone_uwb.processing.geometry.gazebo_geometry")
