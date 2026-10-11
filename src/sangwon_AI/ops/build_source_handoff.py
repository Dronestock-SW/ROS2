"""Build a complete source-only handoff; never include runtime, keys or external checkout."""
import datetime as dt
import hashlib
import json
from pathlib import Path
import zipfile

ROOT=Path(__file__).resolve().parents[1]
OUTPUT=ROOT/'WEB_JETSON_SOURCE_HANDOFF_2026-10-04_r2.zip'
DIRS=('src','include','python','tests','trees','ops','deployment','config','contracts','references/web')
EXTENSIONS={'.md','.py','.cpp','.hpp','.h','.xml','.json','.html','.sh','.service','.target','.txt'}
REQUIRED={'CMakeLists.txt','src/service/engine.cpp','src/service/daemon.cpp','src/service/ledger.cpp',
          'python/sangwon_web/adapter.py','python/sangwon_web/transport.py','tests/web_service_integration.py',
          'ops/health_monitor.py','deployment/install_user_stack.sh','deployment/install_user_monitor.sh',
          'deployment/sangwon-core.service','deployment/sangwon-web-adapter.service','deployment/sangwon-health-monitor.service',
          'ops/perception_monitor.py','python/sangwon_sensors/perception.py','deployment/sangwon-perception-monitor.service'}
REQUIRED.update({'src/service/scan_ingress.cpp','ops/scan_replay_bridge.py',
                 'python/sangwon_sensors/scan_window.py','python/sangwon_sensors/delivery.py','tests/scan_sensor_integration.py'})
REQUIRED.update({'src/planner.cpp','include/sangwon_ai/planner.hpp','tests/planner_tests.cpp','tests/web_planner_integration.py'})
REQUIRED.update({'src/px4_observer.cpp','src/px4_health.cpp','include/sangwon_ai/px4_health.hpp',
                 'python/sangwon_sensors/px4.py','config/px4.observe.json','deployment/run_px4_observer.sh',
                 'deployment/sangwon-px4-observer.service','tests/ros_px4_integration.py'})

def source_contents(root):
    root = root.resolve(strict=True)
    files={p for p in root.glob('*.md') if p.is_file() and not p.is_symlink()}|{root/'CMakeLists.txt',root/'.gitignore'}
    for name in DIRS:
        for path in (root/name).rglob('*'):
            if (path.is_file() and not path.is_symlink() and path.suffix in EXTENSIONS
                    and not path.name.endswith('.local.json')
                    and path.resolve().is_relative_to(root)
                    and not any(part.startswith('.') or part=='__pycache__' for part in path.relative_to(root).parts)):
                files.add(path)
    if any(p.is_symlink() or not p.resolve().is_relative_to(root) for p in files):
        raise ValueError('SOURCE_FILE_OUTSIDE_PACKAGE')
    return {p.relative_to(root).as_posix():p.read_bytes().replace(b'\r\n',b'\n') for p in sorted(files)}


def main():
    contents=source_contents(ROOT)
    assert REQUIRED<=contents.keys()
    assert not any(any(part in ('.runtime','.venv','.build','.git','__pycache__') for part in name.split('/')) for name in contents)
    assert not any(name.endswith('.local.json') or name.startswith('reports/') for name in contents)
    manifest={'generated_at':dt.datetime.now(dt.timezone.utc).isoformat(),'scope':'SOURCE_ONLY_NO_SECRETS_NO_RUNTIME',
              'files':[{'path':n,'bytes':len(b),'sha256':hashlib.sha256(b).hexdigest()} for n,b in contents.items()]}
    with zipfile.ZipFile(OUTPUT,'w',zipfile.ZIP_DEFLATED) as archive:
        for name,body in contents.items():archive.writestr('sangwon_AI/'+name,body)
        archive.writestr('sangwon_AI/HANDOFF_MANIFEST.json',json.dumps(manifest,ensure_ascii=False,indent=2).encode())
    with zipfile.ZipFile(OUTPUT) as archive:
        assert archive.testzip() is None
        assert len(archive.namelist())==len(contents)+1
        for item in manifest['files']:
            assert hashlib.sha256(archive.read('sangwon_AI/'+item['path'])).hexdigest()==item['sha256']
    (ROOT/'reports/WEB_SOURCE_HANDOFF_R2_MANIFEST.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({'file':str(OUTPUT),'source_files':len(contents),'bytes':OUTPUT.stat().st_size,
                      'sha256':hashlib.sha256(OUTPUT.read_bytes()).hexdigest(),'verified':True}))

if __name__=='__main__':main()
