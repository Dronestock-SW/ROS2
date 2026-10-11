"""Transport boundary tests; no real server access."""
import hashlib
import hmac
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
from sangwon_web.common import decode, runtime_path
from sangwon_web.transport import Transport, NoRedirect


class BoundaryTests(unittest.TestCase):
    def test_identity_is_data_and_bound_to_one_drone(self):
        import tempfile
        from sangwon_web.identity import load_identity
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'device.env'
            path.write_text('DRONESTOCK_DEVICE_AUTH_ID=test-device\nDRONESTOCK_DEVICE_AUTH_SECRET='+'a'*64+'\nDRONESTOCK_DRONE_ID=5\n')
            if os.name == 'posix': os.chmod(path, 0o600)
            self.assertEqual(load_identity(path, '5')['DRONESTOCK_DRONE_ID'], '5')
            with self.assertRaisesRegex(ValueError, 'BINDING'): load_identity(path, '6')
            path.write_text(path.read_text()+'DRONESTOCK_DRONE_ID=5\n')
            with self.assertRaisesRegex(ValueError, 'INVALID_IDENTITY'): load_identity(path, '5')

    def test_strict_json(self):
        for value in ('{"a":1,"a":2}', '{"a":{"x":1,"x":2}}', '{"x":NaN}', '{"x":1e999}'):
            with self.subTest(value=value), self.assertRaises(ValueError): decode(value)
        self.assertEqual(decode('{"a":{"x":1},"b":{"x":2}}')["b"]["x"], 2)

    def test_observe_cannot_write(self):
        transport = Transport(dict(mode="observe", base_url="http://127.0.0.1:9"))
        with self.assertRaisesRegex(ValueError, "READ_ONLY"):
            transport.request("POST", "/api/test/", {})
        with self.assertRaisesRegex(ValueError, "READ_ONLY"):
            transport.publish([])

    def test_remote_write_requires_registration(self):
        with self.assertRaisesRegex(ValueError, "REGISTRATION_REQUIRED"):
            Transport(dict(mode="contract", base_url="http://192.0.2.1"))

    def test_origin_and_snapshot(self):
        transport = Transport(dict(mode="observe", base_url="http://127.0.0.1:9"))
        for path in ("//example.org/data", "http://example.org/", "/api\\other"):
            with self.assertRaises(ValueError): transport.request("GET", path)
        with self.assertRaisesRegex(ValueError, "LOCATION_REFUSED"):
            transport.snapshot(dict(content_url="http://example.org/api/mission-snapshots/a/content/"))
        with patch.object(transport, "request_bytes", return_value=b"{}"):
            with self.assertRaisesRegex(ValueError, "SNAPSHOT_CONFLICT"):
                transport.snapshot(dict(content_url="/api/mission-snapshots/a/content/",byte_length=2,sha256="incorrect"))
        with self.assertRaisesRegex(ValueError, "REDIRECT_REFUSED"):
            NoRedirect().redirect_request(None,None,302,None,None,"http://example.org/")

    def test_hmac_exact_body_and_path(self):
        transport = Transport(dict(mode="contract",base_url="http://127.0.0.1",auth=dict(mode="hmac_v1")))
        class Nonce: hex = "testnonce"
        with patch.dict(os.environ,DRONESTOCK_DEVICE_AUTH_ID="test-id",DRONESTOCK_DEVICE_AUTH_SECRET="test-secret"), \
             patch("sangwon_web.transport.time.time",return_value=100), \
             patch("sangwon_web.transport.uuid.uuid4",return_value=Nonce()):
            raw=b'{"value":1}\n'
            headers=transport.headers("POST","/api/test/?v=1",raw)
            expected="\n".join(("POST","/api/test/?v=1",hashlib.sha256(raw).hexdigest(),"100","testnonce"))
            self.assertEqual(headers["X-DS-Signature"],hmac.new(b"test-secret",expected.encode(),hashlib.sha256).hexdigest())

    def test_state_cannot_escape(self):
        root=Path(__file__).resolve().parents[1]
        with self.assertRaisesRegex(ValueError,"OUTSIDE_RUNTIME"): runtime_path(root,"../outside")

    def test_session_hmac_and_bootstrap_are_explicit(self):
        transport=Transport(dict(mode="contract",base_url="http://127.0.0.1",auth=dict(mode="hmac_session_v2")))
        class Nonce: hex="testnonce"
        with patch.dict(os.environ,DRONESTOCK_DEVICE_AUTH_ID="test-id",DRONESTOCK_DEVICE_AUTH_SECRET="test-secret"), \
             patch("sangwon_web.transport.time.time",return_value=100), \
             patch("sangwon_web.transport.uuid.uuid4",return_value=Nonce()):
            with self.assertRaisesRegex(ValueError,"CONTROL_SESSION_REQUIRED"):
                transport.headers("GET","/api/test/",b"")
            bootstrap=transport.headers("POST","/api/drones/TEST-1/control-sessions/",b"{}",session_request=True)
            expected_bootstrap="\n".join(("POST","/api/drones/TEST-1/control-sessions/",hashlib.sha256(b"{}").hexdigest(),"100","testnonce"))
            self.assertEqual(bootstrap["X-DS-Signature"],hmac.new(b"test-secret",expected_bootstrap.encode(),hashlib.sha256).hexdigest())
            self.assertNotIn("X-DS-Control-Session",bootstrap)
            transport.set_session("session-1")
            raw=b'{"value":1}\n'
            headers=transport.headers("POST","/api/test/?v=1",raw)
            expected="\n".join(("POST","/api/test/?v=1",hashlib.sha256(raw).hexdigest(),"100","testnonce","1.1-draft.4","session-1"))
            self.assertEqual(headers["X-DS-Signature"],hmac.new(b"test-secret",expected.encode(),hashlib.sha256).hexdigest())
            self.assertEqual(headers["X-DS-Control-Session"],"session-1")
        with self.assertRaisesRegex(ValueError,"INVALID_CONTROL_SESSION"):
            transport.set_session("session\nforged")

    def test_readiness_ack_binds_report(self):
        context=dict(boot_id="b",runtime_session_id="r",control_session_id="c")
        message=dict(drone_id="TEST-1",context=context,readiness_seq=7)
        ack=dict(type="readiness.ack",accepted=True,contract_version="1.1-draft.4",**message)
        Transport.validate_readiness_ack(message,ack)
        rejected=dict(ack,accepted=False,code='INVALID_COMPANION_REPORT',diagnostic_code='REPORT_TIME_INVALID',reason='private arbitrary response')
        with self.assertRaisesRegex(ValueError,'READINESS_REJECTED:INVALID_COMPANION_REPORT:REPORT_TIME_INVALID'):
            Transport.validate_readiness_ack(message,rejected)
        rejected['diagnostic_code']='unsafe response data'
        with self.assertRaisesRegex(ValueError,'READINESS_REJECTED:INVALID_COMPANION_REPORT$'):
            Transport.validate_readiness_ack(message,rejected)
        for key,value in [("accepted",False),("readiness_seq",6),("readiness_seq",True),
                          ("drone_id","other"),("context",dict(context,control_session_id="old"))]:
            with self.subTest(key=key),self.assertRaises(ValueError):
                Transport.validate_readiness_ack(message,{**ack,key:value})

    def test_ack_timeout_closes_connection(self):
        from unittest.mock import Mock
        t=Transport(dict(mode="contract",base_url="http://127.0.0.1",websocket_url="ws://127.0.0.1/ws/"))
        peer=Mock(); peer.recv.side_effect=TimeoutError(); t.ws=peer
        # Existing connection avoids importing websockets on Windows in this unit test.
        import sys, types
        with patch.dict(sys.modules,{"websockets.sync.client":types.SimpleNamespace(connect=Mock())}):
            with self.assertRaises(TimeoutError): t.publish([{"type":"readiness"}])
        peer.close.assert_called_once()
        self.assertIsNone(t.ws)

    def test_both_report_acks_are_consumed_and_bound(self):
        from unittest.mock import Mock
        import types
        context=dict(boot_id="b",runtime_session_id="r",control_session_id="c")
        messages=[dict(type=kind,drone_id="TEST-1",contract_version="1.1-draft.4",context=context,**{kind+"_seq":7})
                  for kind in ("readiness","telemetry")]
        acks=[dict(m,type=m['type']+'.ack',accepted=True) for m in messages]
        t=Transport(dict(mode="contract",base_url="http://127.0.0.1",websocket_url="ws://127.0.0.1/ws/"))
        peer=Mock(); peer.recv.side_effect=[__import__('json').dumps(a) for a in acks]; t.ws=peer
        with patch.dict(sys.modules,{"websockets.sync.client":types.SimpleNamespace(connect=Mock())}):
            t.publish(messages)
        self.assertEqual(peer.recv.call_count,2)
        self.assertEqual(t.last_telemetry_ack['telemetry_seq'],7)
        with self.assertRaisesRegex(ValueError,"TELEMETRY_ACK_REQUIRED"):
            Transport.validate_report_ack(messages[1],acks[0],"telemetry")
        with self.assertRaisesRegex(ValueError,"READINESS_ACK_CONTEXT"):
            Transport.validate_readiness_ack(messages[0],dict(type='readiness.ack',accepted=True))


if __name__ == "__main__": unittest.main()
