"""Compatibility path. Implementation: drone_uwb.integration.gazebo.gazebo_capture."""
import importlib
import sys

if __name__ == "__main__":
    importlib.import_module("drone_uwb.integration.gazebo.gazebo_capture").main()
else:
    sys.modules[__name__] = importlib.import_module("drone_uwb.integration.gazebo.gazebo_capture")
