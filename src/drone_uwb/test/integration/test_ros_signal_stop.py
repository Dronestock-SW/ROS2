"""Signals stop B_TF between callbacks without hiding live callback errors."""
import importlib.util
from pathlib import Path
import signal
import sys
from types import SimpleNamespace
import pytest


@pytest.fixture
def lifecycle(monkeypatch):
    handlers={};activity=[]
    ros=SimpleNamespace(init=lambda **kwargs:activity.append(('init',kwargs)),ok=lambda:True)
    ros.get_global_executor=lambda:SimpleNamespace(add_node=lambda node:None,remove_node=lambda node:None,
        spin_once=lambda timeout_sec:ros.spin_once(None,timeout_sec=timeout_sec))
    monkeypatch.setitem(sys.modules,'rclpy',ros)
    monkeypatch.setitem(sys.modules,'rclpy.signals',SimpleNamespace(SignalHandlerOptions=SimpleNamespace(NO=0)))
    monkeypatch.setattr(signal,'signal',lambda signum,handler:handlers.update({signum:handler}))
    path=Path(__file__).parents[2]/'drone_uwb/integration/ros/lifecycle.py'
    spec=importlib.util.spec_from_file_location('signal_stop_fixture',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module,ros,handlers,activity


def test_BTF_stop_signal_finishes_current_conversion_before_exit(lifecycle):
    module,ros,handlers,activity=lifecycle;module.init_for_main(interrupt_spin=False)
    def take_message(node,timeout_sec):
        activity.append('conversion_start');handlers[signal.SIGTERM](signal.SIGTERM,None)
        activity.append('conversion_finished')
    ros.spin_once=take_message;module.spin_until_stopped(object())
    assert activity[1:]==['conversion_start','conversion_finished']
    assert module.shutdown_requested()


def test_active_callback_errors_remain_failures(lifecycle):
    module,ros,handlers,activity=lifecycle;module.init_for_main(interrupt_spin=False)
    def take_message(*args,**kwargs):raise RuntimeError('real conversion failure')
    ros.spin_once=take_message
    with pytest.raises(RuntimeError,match='real conversion failure'):module.spin_until_stopped(object())
    assert not module.shutdown_requested()


def test_other_nodes_keep_their_existing_interrupt_policy(lifecycle):
    module,ros,handlers,activity=lifecycle;module.init_for_main()
    with pytest.raises(KeyboardInterrupt):handlers[signal.SIGTERM](signal.SIGTERM,None)
