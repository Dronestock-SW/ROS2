"""Export the canonical companion source into its reviewed web Git checkout."""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CHECKOUT = ROOT / '.runtime' / 'web-development'
DESTINATION = CHECKOUT / 'companion' / 'sangwon_AI'
DIRECTORIES = ('src', 'include', 'python', 'tests', 'trees', 'ops', 'deployment', 'config', 'contracts')
EXTENSIONS = {'.py', '.cpp', '.hpp', '.h', '.xml', '.json', '.html', '.sh', '.service', '.target', '.txt', '.md'}
DOCUMENTS = ('README.md', 'BUILD_REPLAY.md', 'DESIGN.md', 'BT_SPEC.md', 'SERVICE_ARCHITECTURE.md',
             'BOOT_PREFLIGHT_SPEC.md', 'RC_EMERGENCY_SPEC.md', 'PX4_INTERFACE.md', 'PX4_PARAMETER_PROFILE.md',
             'QR_SCAN_SPEC.md', 'TELEMETRY_CHANNEL_SPEC.md', 'UWB_INTEGRATION_REQUEST.md',
             'JETSON_ENV.md', 'JETSON_DEPLOYMENT.md', 'JETSON_SETUP.md', 'IMPLEMENTATION_BASELINE.md',
             'IMPLEMENTATION_STATUS.md', 'DEVELOPMENT_HANDOFF.md', 'WEB_FOLLOWUP_2026-10-04.md',
             'WEB_BACKEND_INTEGRATION_2026-10-04.md', 'MEASUREMENT_REGISTER.md', 'INTEGRATION_PLAN.md',
             'VALIDATION_PLAN.md', 'GLOSSARY.md', 'WEB_JETSON_RUNBOOK.md')


def main():
    if not (CHECKOUT / '.git').is_dir():
        raise SystemExit('Expected reviewed web-development Git checkout')
    destination = DESTINATION.resolve()
    if not destination.is_relative_to(CHECKOUT.resolve()):
        raise SystemExit('Export destination is outside the reviewed checkout')
    files = {ROOT / name for name in (*DOCUMENTS, 'CMakeLists.txt', '.gitignore')}
    for directory in DIRECTORIES:
        for path in (ROOT / directory).rglob('*'):
            if (path.is_file() and not path.is_symlink() and path.suffix in EXTENSIONS
                    and not path.name.endswith('.local.json')
                    and not any(part.startswith('.') or part == '__pycache__' for part in path.relative_to(ROOT).parts)):
                files.add(path)
    manifest = []
    for path in sorted(files):
        if not path.is_file() or path.is_symlink():
            raise SystemExit('Missing source or symlink in export')
        relative = path.relative_to(ROOT)
        target = destination / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        # These are UTF-8 text sources. LF is pinned by the receiving .gitattributes.
        body = path.read_bytes().replace(b'\r\n', b'\n')
        body.decode('utf-8')
        target.write_bytes(body)
        manifest.append({'path': relative.as_posix(), 'bytes': len(body), 'sha256': hashlib.sha256(body).hexdigest()})
    (destination / 'SOURCE_MANIFEST.json').write_text(json.dumps({
        'scope': 'SOURCE_ONLY_NO_RUNTIME_NO_PRIVATE_KEYS', 'files': manifest}, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({'exported_files': len(manifest), 'destination': str(destination), 'flight_authority': False}))


if __name__ == '__main__':
    main()
