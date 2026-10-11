"""Load a private identity as data, never shell code."""
import os
from pathlib import Path
import re
import stat


def load_identity(path, drone_id):
    if not re.fullmatch(r'[A-Za-z0-9_-]{1,128}', drone_id):
        raise ValueError('IDENTITY_BINDING_INVALID')
    path = Path(path)
    if path.is_symlink() or not path.is_file():
        raise ValueError('IDENTITY_REGULAR_FILE_REQUIRED')
    if os.name == 'posix' and (stat.S_IMODE(path.stat().st_mode) != 0o600 or path.stat().st_uid != os.getuid()):
        raise ValueError('IDENTITY_OWNER_MODE_REQUIRED')
    rows = path.read_text().splitlines()
    values = {}
    for line in rows:
        key, sep, value = line.partition('=')
        if not sep or key in values:
            raise ValueError('INVALID_IDENTITY')
        values[key] = value
    if (set(values) != {'DRONESTOCK_DEVICE_AUTH_ID', 'DRONESTOCK_DEVICE_AUTH_SECRET', 'DRONESTOCK_DRONE_ID'}
            or values['DRONESTOCK_DRONE_ID'] != drone_id
            or not re.fullmatch(r'[A-Za-z0-9_-]{1,128}', values['DRONESTOCK_DEVICE_AUTH_ID'])
            or not re.fullmatch(r'[a-f0-9]{64}', values['DRONESTOCK_DEVICE_AUTH_SECRET'])):
        raise ValueError('IDENTITY_BINDING_INVALID')
    return values
