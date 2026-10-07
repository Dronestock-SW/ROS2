import importlib.util
from pathlib import Path
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('source_handoff', ROOT / 'ops/build_source_handoff.py')
module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


class SourceHandoffTests(unittest.TestCase):
    def test_device_local_config_runtime_and_reports_are_excluded(self):
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            public = ('CMakeLists.txt', '.gitignore', 'README.md', 'config/perception.observe.json', 'src/runtime.cpp')
            private = ('config/web.local.json', 'config/companion.local.json', 'config/perception.local.json',
                       '.runtime/private/device.env', 'reports/WEB_JETSON_SERVICE_STATUS_2026-10-04.json',
                       'python/__pycache__/cache.py', 'config/.private/setup.json')
            for relative in (*public, *private):
                path = root / relative;path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(b'private-local-canary' if relative in private else b'public-source\r\n')
            contents = module.source_contents(root)
            self.assertEqual(set(contents), set(public))
            self.assertNotIn(b'private-local-canary', b''.join(contents.values()))
            self.assertEqual(contents['src/runtime.cpp'], b'public-source\n')


if __name__ == '__main__':
    unittest.main()
