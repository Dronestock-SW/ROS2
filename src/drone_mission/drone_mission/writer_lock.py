"""Host-local PX4 command ownership shared with the native hover binary."""
import os
from pathlib import Path
import stat


def writer_lock_path(domain):
    # Both physical domains may reach the same USB FC on this host.
    scope = 'flight' if domain in (1, 2) else f'test-{domain}'
    return Path('/tmp') / f'dronestock-px4-writer-{scope}.lock'


class WriterLock:
    def __init__(self, domain):
        import fcntl
        self.fd = None
        fd = os.open(writer_lock_path(domain), os.O_CREAT | os.O_RDWR | os.O_CLOEXEC | os.O_NOFOLLOW, 0o600)
        try:
            info = os.fstat(fd)
            if not stat.S_ISREG(info.st_mode) or info.st_uid != os.geteuid() or info.st_nlink != 1:
                raise RuntimeError('invalid_px4_writer_lock')
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except Exception:
            os.close(fd)
            raise
        self.fd = fd

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
        # Never unlink: another process may already hold this same inode.

    def __del__(self):
        self.close()
