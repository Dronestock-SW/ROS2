"""Check redirect/auth classification with an isolated HTTP server."""
import importlib.util
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import threading
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location(
    "web_contract_probe", Path(__file__).resolve().parents[1] / "ops/check_web_contract.py")
monitor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(monitor)


class MonitorTest(unittest.TestCase):
    def test_auth_redirects_and_json(self):
        hits = []
        response = {"status": 302, "location": "/platform/login/?next=private-value"}

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def do_GET(self):
                hits.append(self.path)
                mission = self.path.endswith("companion-mission/")
                self.send_response(200 if mission else response["status"])
                self.send_header("Content-Type", "application/json")
                if not mission:
                    self.send_header("Location", response["location"])
                self.end_headers()
                self.wfile.write(b'{"contract_version":"1.1"}' if mission else b'{}')

        server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        original = monitor.Transport
        def local_transport(config):
            return original({**config, "base_url": "http://127.0.0.1:%d" % server.server_port})
        try:
            with patch.object(monitor, "Transport", local_transport):
                for status, location, expected in (
                    (302, "/platform/login/?next=private-value", "AUTH_REQUIRED"),
                    (302, "http://elsewhere.invalid/platform/login/", "REDIRECT_REFUSED"),
                    (401, "", "AUTH_REQUIRED"),
                    (403, "", "AUTH_REQUIRED"),
                    (404, "", None),
                ):
                    with self.subTest(status=status, location=location):
                        response.update(status=status, location=location)
                        hits.clear()
                        result = monitor.probe()
                        state = result["/api/drones/5/autonomy-state/"]
                        self.assertEqual(state["http_status"], status)
                        self.assertEqual(state.get("response_state"), expected)
                        self.assertEqual(result["/api/drones/5/companion-mission/"]["contract_version"], "1.1")
                        self.assertEqual(len(hits), 2)  # Never visit redirect destination.
                        self.assertNotIn("private-value", json.dumps(result))
                        self.assertNotIn("elsewhere.invalid", json.dumps(result))
        finally:
            server.shutdown()
            server.server_close()
            worker.join(timeout=2)


if __name__ == "__main__":
    unittest.main()
