"""Compatibility path. Implementation: drone_uwb.integration.bridge."""
import importlib
import sys

if __name__ == "__main__":
    importlib.import_module("drone_uwb.integration.bridge").main()
else:
    sys.modules[__name__] = importlib.import_module("drone_uwb.integration.bridge")
