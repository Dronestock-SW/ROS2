"""Exercise command construction with a fake client; never initialize ROS."""
from datetime import datetime, timezone
from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy')
from drone_mission.node import FlightNode
from drone_mission.contracts import Settings


def test_normal_arm_is_persisted_once_without_mode_or_takeoff(tmp_path):
    order=[]; callbacks=[]; requests=[]
    def call(request):
        order.append('send');requests.append(request)
        return SimpleNamespace(add_done_callback=callbacks.append)
    with (tmp_path/'events').open('w') as log:
        obj=SimpleNamespace(settings=Settings(execute=True),operator_session_id='current',
            session=SimpleNamespace(consumed=set()),snapshot=lambda:object(),operator_arm_gate=lambda _: '',
            remember=lambda _:order.append('persist'),record=lambda _:None,record_fault=False,log=log,
            command_clients={'arm':SimpleNamespace(call_async=call)})
        p=dict(ok=True,contract_version='1.0',drone_id='5',anchor_layout_id=obj.settings.layout_id,
            operator_session_id='current',control_action='arm',control_request_id='one-click',
            control_requested_at=datetime.now(timezone.utc).isoformat())
        FlightNode.operator_arm(obj,p)
        FlightNode.operator_arm(obj,p)
        assert order==['persist','send'] and len(requests)==1 and requests[0].value is True
        assert obj.operator_arm_pending
        callbacks[0](SimpleNamespace(result=lambda:SimpleNamespace(success=True,result=0)))
        assert obj.operator_arm_result['accepted'] is True
        assert obj.operator_arm_pending  # ACK alone does not prove actual ARM.


def test_rejection_consumes_request_without_sending(tmp_path):
    with (tmp_path/'events').open('w') as log:
        obj=SimpleNamespace(settings=Settings(execute=True),operator_session_id='current',
            session=SimpleNamespace(consumed=set()),snapshot=lambda:object(),
            operator_arm_gate=lambda _: 'fresh_physical_rc_required',remember=lambda _:None,
            record=lambda _:None,record_fault=False,log=log,command_clients={})
        p=dict(ok=True,contract_version='1.0',drone_id='5',anchor_layout_id=obj.settings.layout_id,
            operator_session_id='current',control_action='arm',control_request_id='one-click',
            control_requested_at=datetime.now(timezone.utc).isoformat())
        FlightNode.operator_arm(obj,p)
        assert obj.operator_arm_result['accepted'] is False
        assert 'one-click' in obj.session.consumed
