"""Compatibility path. Implementation: drone_uwb.processing.experiments.baseline_a."""
import importlib
import sys

if __name__ == "__main__":
    importlib.import_module("drone_uwb.processing.experiments.baseline_a").main()
else:
    sys.modules[__name__] = importlib.import_module("drone_uwb.processing.experiments.baseline_a")
