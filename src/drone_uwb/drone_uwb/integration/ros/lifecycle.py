"""Stop executor callbacks before invalidating their ROS context."""
import signal

import rclpy
from rclpy.signals import SignalHandlerOptions


def init_for_main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    def stop_requested(_signum, _frame):
        raise KeyboardInterrupt

    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, stop_requested)
