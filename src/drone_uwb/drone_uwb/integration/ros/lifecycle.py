"""Stop executor callbacks before invalidating their ROS context."""
import signal
import threading

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


def spin_until_stopped(node, *, executor=None, ingress_node=None):
    # No Python exception is raised from a signal during a pybind message
    # conversion. Finish the callback, then stop before the next callback.
    executor=executor if executor is not None else rclpy.get_global_executor()
    executor.add_node(node)
    ingress_executor = receiver = None
    stop = threading.Event()
    errors = []
    if ingress_node is not None:
        from rclpy.executors import SingleThreadedExecutor
        ingress_executor = SingleThreadedExecutor(context=node.context)
        ingress_executor.add_node(ingress_node)
        def receive():
            try:
                while rclpy.ok() and not stop.is_set():
                    ingress_executor.spin_once(timeout_sec=.05)
            except Exception as exc:
                errors.append(exc)
        receiver = threading.Thread(target=receive, name='btf-sensor-ingress', daemon=True)
        receiver.start()
    try:
        while rclpy.ok() and not shutdown_requested():
            if errors:
                raise errors[0]
            executor.spin_once(timeout_sec=.05)
    finally:
        if receiver:
            stop.set()
            ingress_executor.wake()
            receiver.join()
            ingress_executor.shutdown()
        executor.remove_node(node)
