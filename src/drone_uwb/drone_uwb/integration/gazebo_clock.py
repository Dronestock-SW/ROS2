"""Compatibility path. Implementation: drone_uwb.integration.gazebo.gazebo_clock."""
import importlib
import sys

sys.modules[__name__] = importlib.import_module("drone_uwb.integration.gazebo.gazebo_clock")
