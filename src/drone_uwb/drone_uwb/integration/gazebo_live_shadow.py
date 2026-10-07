"""Compatibility path. Implementation: drone_uwb.integration.gazebo.gazebo_live_shadow."""
import importlib
import sys

if __name__ == "__main__":
    importlib.import_module("drone_uwb.integration.gazebo.gazebo_live_shadow").main()
else:
    sys.modules[__name__] = importlib.import_module("drone_uwb.integration.gazebo.gazebo_live_shadow")
