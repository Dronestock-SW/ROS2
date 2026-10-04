"""Compatibility path. Implementation: drone_uwb.integration.sitl.sitl_target_contract."""
import importlib
import sys

sys.modules[__name__] = importlib.import_module("drone_uwb.integration.sitl.sitl_target_contract")
