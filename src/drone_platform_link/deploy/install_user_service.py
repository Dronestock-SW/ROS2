#!/usr/bin/env python3
"""Install the communication-only user service without changing system Python."""

import hashlib
import io
from pathlib import Path
import shutil
import subprocess
from urllib.request import ProxyHandler, build_opener
from zipfile import ZipFile


WHEEL_URL = (
    'https://files.pythonhosted.org/packages/56/27/'
    '96a5cd2626d11c8280656c6c71d8ab50fe006490ef9971ccd154e0c42cd2/'
    'websockets-13.1-py3-none-any.whl'
)
WHEEL_SHA256 = 'a9a396a6ad26130cdae92ae10c36af09d9bfe6cafe69670fd3b6da9b07b4044f'


def main():
    source = Path(__file__).resolve().parents[1]
    destination = Path.home() / '.local/share/dronestock-companion'
    config = Path.home() / '.config/dronestock-companion'
    units = Path.home() / '.config/systemd/user'
    with build_opener(ProxyHandler({})).open(WHEEL_URL, timeout=20) as response:
        wheel = response.read()
    if hashlib.sha256(wheel).hexdigest() != WHEEL_SHA256:
        raise ValueError('websockets wheel SHA256 mismatch')
    destination.mkdir(parents=True, exist_ok=True)
    with ZipFile(io.BytesIO(wheel)) as archive:
        for item in archive.infolist():
            path = Path(item.filename)
            if path.is_absolute() or '..' in path.parts:
                raise ValueError('invalid wheel path')
        archive.extractall(destination / 'deps')
    shutil.copytree(source / 'drone_platform_link', destination / 'app/drone_platform_link',
                    dirs_exist_ok=True, ignore=shutil.ignore_patterns('__pycache__'))
    config.mkdir(parents=True, exist_ok=True, mode=0o700)
    env = config / 'companion.env'
    if not env.exists():
        shutil.copyfile(source / 'deploy/companion.env.example', env)
    env.chmod(0o600)
    units.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source / 'deploy/dronestock-companion.service',
                    units / 'dronestock-companion.service')
    subprocess.run(['systemd-analyze', '--user', 'verify',
                    str(units / 'dronestock-companion.service')], check=True)
    subprocess.run(['loginctl', 'enable-linger'], check=True)
    subprocess.run(['systemctl', '--user', 'daemon-reload'], check=True)
    subprocess.run(['systemctl', '--user', 'enable', 'dronestock-companion.service'], check=True)
    subprocess.run(['systemctl', '--user', 'restart', 'dronestock-companion.service'], check=True)
    subprocess.run(['systemctl', '--user', 'is-active', 'dronestock-companion.service'], check=True)


if __name__ == '__main__':
    main()
