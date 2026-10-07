"""Compatibility path. Implementation: drone_uwb.processing.runner."""
import importlib
import sys

if __name__ == "__main__":
    importlib.import_module("drone_uwb.processing.runner").main()
else:
    sys.modules[__name__] = importlib.import_module("drone_uwb.processing.runner")
