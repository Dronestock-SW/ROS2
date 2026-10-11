"""Create pinned Jetson helper environments without changing system packages."""
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import urllib.request
import venv

ROOT = Path(__file__).resolve().parents[1]
PIP_VERSION = "25.2"
PIP_NAME = f"pip-{PIP_VERSION}-py3-none-any.whl"
PIP_URL = ("https://files.pythonhosted.org/packages/b7/3f/"
           "945ef7ab14dc4f9d7f40288d2df998d1837ee0888ec3659c813487572faa/" + PIP_NAME)
PIP_SHA = "6d67a2b4e7f14d8b31b8b52648866fa717f45a1eb70e83002f4331d07e953717"


def run(args, env):
    subprocess.run(args, env=env, check=True, timeout=240)


def create_env(name, system_packages):
    target = ROOT / name
    if target.is_symlink():
        raise RuntimeError(f"Refusing symlink environment: {target}")
    cfg = target / "pyvenv.cfg"
    if target.exists():
        if not cfg.is_file():
            raise RuntimeError(f"Existing directory is not a venv: {target}")
        values = dict(line.split(" = ", 1) for line in cfg.read_text().splitlines() if " = " in line)
        if values.get("include-system-site-packages") != str(system_packages).lower():
            raise RuntimeError(f"Existing environment policy differs: {target}")
        check = subprocess.run([str(target / "bin/python"), "-c",
                                "import json,sys; print(json.dumps([sys.base_prefix,list(sys.version_info[:2])]))"],
                               capture_output=True, text=True, check=True, timeout=10)
        if json.loads(check.stdout) != [sys.base_prefix, [3, 10]]:
            raise RuntimeError(f"Existing interpreter differs: {target}")
    else:
        venv.EnvBuilder(system_site_packages=system_packages, with_pip=False,
                        clear=False, symlinks=True).create(target)
    (target / "COLCON_IGNORE").touch()
    return target / "bin/python"


def install(name, system_packages, requirements):
    python = create_env(name, system_packages)
    env = os.environ.copy()
    env.pop("PYTHONHOME", None)
    env.pop("PYTHONPATH", None)
    env.update(PYTHONNOUSERSITE="1", PYTHONDONTWRITEBYTECODE="1")
    check = subprocess.run([str(python), "-c",
        "import pathlib,pip,sys; assert pip.__version__=='25.2'; "
        "assert pathlib.Path(pip.__file__).is_relative_to(pathlib.Path(sys.prefix))"],
        env=env, capture_output=True, timeout=15)
    if check.returncode:
        cache = ROOT / ".build/env-wheels"
        cache.mkdir(parents=True, exist_ok=True)
        wheel = cache / PIP_NAME
        if not wheel.exists():
            with urllib.request.urlopen(PIP_URL, timeout=30) as response:
                data = response.read(8 * 1024 * 1024 + 1)
            if len(data) > 8 * 1024 * 1024 or hashlib.sha256(data).hexdigest() != PIP_SHA:
                raise RuntimeError("pip wheel failed size/hash verification")
            temp = wheel.with_suffix(".part")
            temp.write_bytes(data)
            temp.replace(wheel)
        if hashlib.sha256(wheel.read_bytes()).hexdigest() != PIP_SHA:
            raise RuntimeError("Cached pip wheel hash mismatch")
        bootstrap = dict(env, PYTHONPATH=str(wheel))
        run([str(python), "-m", "pip", "--isolated", "--disable-pip-version-check",
             "install", "--no-input", "--no-index", "--no-deps", str(wheel)], bootstrap)
    run([str(python), "-m", "pip", "--isolated", "--disable-pip-version-check",
         "install", "--no-input", "--index-url", "https://pypi.org/simple",
         "--require-hashes", "--only-binary=:all:", "--no-deps", "-r",
         str(ROOT / "deployment" / requirements)], env)
    run([str(python), "-m", "pip", "--isolated", "check"], env)


if __name__ == "__main__":
    if sys.version_info[:2] != (3, 10) or platform.system() != "Linux" or platform.machine() != "aarch64":
        raise SystemExit("Use Jetson Ubuntu system Python 3.10 on Linux aarch64")
    if sys.prefix != sys.base_prefix or sys.base_prefix != "/usr":
        raise SystemExit("Run with /usr/bin/python3 outside another virtual environment")
    install(".venv", True, "requirements-jetson.txt")
    install(".venv-log", False, "requirements-log.txt")
    print("Helper environments installed. This does not authorize flight.")
