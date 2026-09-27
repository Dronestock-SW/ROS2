"""Receive-only POSIX serial access without DTR/RTS reset commands."""
import copy
import fcntl
import os
import select
import termios


class SerialInput:
    def __init__(self, path, baud=921600):
        self.fd = os.open(path, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
        self.original = None
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.original = termios.tcgetattr(self.fd)
            cfg = copy.deepcopy(self.original)
            cfg[0] = cfg[1] = cfg[3] = 0
            cfg[2] = ((cfg[2] & ~(termios.CSIZE | termios.PARENB | termios.CSTOPB
                                  | getattr(termios, 'CRTSCTS', 0)))
                      | termios.CS8 | termios.CREAD | termios.CLOCAL)
            cfg[4] = cfg[5] = getattr(termios, 'B' + str(baud))
            cfg[6][termios.VMIN] = cfg[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, cfg)
            termios.tcflush(self.fd, termios.TCIFLUSH)
        except BaseException:
            self.close()
            raise

    def read(self):
        if not select.select([self.fd], [], [], 0)[0]:
            return None
        chunk = os.read(self.fd, 65536)
        if not chunk:
            raise OSError('serial device disconnected')
        return chunk

    def close(self):
        if self.fd is not None:
            try:
                if self.original is not None:
                    termios.tcsetattr(self.fd, termios.TCSANOW, self.original)
            finally:
                os.close(self.fd)
                self.fd = None
