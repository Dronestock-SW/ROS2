import socket
import uuid
from .common import encode, decode, MAX_BYTES

class CoreError(RuntimeError):
    pass

class CoreClient:
    def __init__(self, path, timeout=2):
        self.path = str(path)
        self.timeout = timeout

    def call(self, method, payload=None):
        request_id = str(uuid.uuid4())
        raw = encode(dict(ipc_version=1, request_id=request_id, method=method, payload=payload or {})) + b"\n"
        if len(raw) > MAX_BYTES:
            raise ValueError("IPC_MESSAGE_TOO_LARGE")
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
            connection.settimeout(self.timeout)
            connection.connect(self.path)
            connection.sendall(raw)
            data = bytearray()
            while not data.endswith(b"\n"):
                chunk = connection.recv(65536)
                if not chunk:
                    raise CoreError("IPC_INCOMPLETE_RESPONSE")
                data.extend(chunk)
                if len(data) > MAX_BYTES:
                    raise CoreError("IPC_MESSAGE_TOO_LARGE")
        result = decode(data)
        if result.get("ipc_version") != 1:
            raise CoreError("UNSUPPORTED_IPC_VERSION")
        if not result.get("ok"):
            raise CoreError(result.get("code", "CORE_ERROR"))
        if result.get("request_id") != request_id:
            raise CoreError("IPC_RESPONSE_MISMATCH")
        return result["payload"]
