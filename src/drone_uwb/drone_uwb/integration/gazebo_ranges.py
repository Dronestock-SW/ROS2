"""Compatibility path. Implementation: drone_uwb.integration.gazebo.gazebo_ranges."""
import importlib
import sys

if __name__ == "__main__":
    importlib.import_module("drone_uwb.integration.gazebo.gazebo_ranges").main()
else:
    sys.modules[__name__] = importlib.import_module("drone_uwb.integration.gazebo.gazebo_ranges")
