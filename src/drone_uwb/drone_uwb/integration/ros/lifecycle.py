"""Stop executor callbacks before invalidating their ROS context."""
import signal

import rclpy
from rclpy.signals import SignalHandlerOptions
_shutdown_requested = False


def shutdown_requested():
    return _shutdown_requested


def init_for_main(args=None, *, interrupt_spin=True):
    global _shutdown_requested
    _shutdown_requested = False
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    def stop_requested(_signum, _frame):
        global _shutdown_requested
        _shutdown_requested = True
        if interrupt_spin:
            raise KeyboardInterrupt

    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, stop_requested)


def spin_until_stopped(node):
    # No Python exception is raised from a signal during a pybind message
    # conversion. Finish the callback, then stop before the next callback.
    executor=rclpy.get_global_executor()
    executor.add_node(node)
    try:
        while rclpy.ok() and not shutdown_requested():
            executor.spin_once(timeout_sec=.05)
    finally:
        executor.remove_node(node)
