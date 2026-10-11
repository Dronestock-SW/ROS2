"""Stop executor callbacks before invalidating their ROS context."""
import signal

import rclpy
from rclpy.signals import SignalHandlerOptions
_shutdown_requested = False


def shutdown_requested():
    return _shutdown_requested


def init_for_main(args=None):
    global _shutdown_requested
    _shutdown_requested = False
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    def stop_requested(_signum, _frame):
        global _shutdown_requested
        _shutdown_requested = True
        raise KeyboardInterrupt

    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, stop_requested)
