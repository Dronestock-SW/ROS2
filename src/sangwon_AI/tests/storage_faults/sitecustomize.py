"""Enabled only by the isolated storage regression fixture's PYTHONPATH."""
import os
import time

if os.environ.get('ROS_DOMAIN_ID') == '99' and os.environ.get('ROS_LOCALHOST_ONLY') == '1':
    original = os.fsync
    delayed = set()

    def slow(descriptor):
        path = os.readlink('/proc/self/fd/'+str(descriptor))
        if path.endswith(('ledger.tmp', 'observer-events.jsonl')) and path not in delayed:
            delayed.add(path)
            time.sleep(4.)
        return original(descriptor)

    os.fsync = slow
