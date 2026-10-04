"""Compatibility path. Implementation: drone_uwb.integration.ros.node."""
import importlib
import sys

if __name__ == "__main__":
    importlib.import_module("drone_uwb.integration.ros.node").main()
else:
    sys.modules[__name__] = importlib.import_module("drone_uwb.integration.ros.node")
