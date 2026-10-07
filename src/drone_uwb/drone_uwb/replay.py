"""Compatibility path. Implementation: drone_uwb.integration.replay."""
import importlib
import sys

if __name__ == "__main__":
    importlib.import_module("drone_uwb.integration.replay").main()
else:
    sys.modules[__name__] = importlib.import_module("drone_uwb.integration.replay")
