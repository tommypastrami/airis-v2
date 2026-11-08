"""Client of airis_bridge's Unix socket. One request per connection; never queues."""
from __future__ import annotations

import socket
from dataclasses import dataclass
from typing import Optional

from . import config
from .errors import Code, Fault


@dataclass
class ClientResult:
    ok: bool
    code: str
    msg: str = ""
    el_err: float = float("nan")
    az_err: float = float("nan")
    duration_s: float = 0.0


class MountClient:
    def __init__(self, path: str = config.BRIDGE_SOCKET_PATH, timeout_s: float = config.BRIDGE_CLIENT_TIMEOUT_S):
        self.path, self.timeout_s = path, timeout_s
        self.last_error: Optional[str] = None

    def _request(self, line: str, wait_done: bool) -> ClientResult:
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as s:
                s.settimeout(5.0)
                s.connect(self.path)
                s.sendall((line + "\n").encode("ascii"))
                f = s.makefile("r", encoding="ascii")
                s.settimeout(self.timeout_s)
                first = f.readline().strip()
                if not wait_done or not first.startswith("accepted"):
                    return self._parse(first)
                return self._parse(f.readline().strip())
        except socket.timeout:
            self.last_error = f"{Code.LNK_TIMEOUT.value} bridge did not answer"
            return ClientResult(False, Code.LNK_TIMEOUT.value, "bridge did not answer")
        except OSError as e:
            self.last_error = f"{Code.LNK_PORT_LOST.value} bridge unreachable: {e}"
            return ClientResult(False, Code.LNK_PORT_LOST.value, f"bridge unreachable: {e}")

    def _parse(self, text: str) -> ClientResult:
        parts = text.split(maxsplit=4)
        if not parts:
            return ClientResult(False, Code.LNK_PORT_LOST.value, "connection closed without reply")
        if parts[0] == "done":
            try:
                return ClientResult(True, "OK", "", float(parts[1]), float(parts[2]), float(parts[3]))
            except (IndexError, ValueError):
                return ClientResult(True, "OK")
        if parts[0] == "ok":
            return ClientResult(True, "OK")
        if parts[0] == "status":
            return ClientResult(True, "OK", text)
        if parts[0] == "failed":
            code = parts[1] if len(parts) > 1 else Code.MNT_GENERAL.value
            msg = " ".join(parts[2:])
            self.last_error = f"{code} {msg}"
            return ClientResult(False, code, msg)
        return ClientResult(False, Code.LNK_UNEXPECTED.value, text)

    def move(self, el: float, az: float) -> ClientResult:
        if not (config.ANGLE_MIN_DEG <= el <= config.ANGLE_MAX_DEG and config.ANGLE_MIN_DEG <= az <= config.ANGLE_MAX_DEG):
            raise Fault(Code.PTG_RANGE, f"el={el:.2f} az={az:.2f}")
        return self._request(f"move {el:.3f} {az:.3f}", wait_done=True)

    def calibrate(self) -> ClientResult:
        return self._request("cal", wait_done=True)

    def set_sun(self, el: float, az: float) -> ClientResult:
        return self._request(f"sun {el:.3f} {az:.3f}", wait_done=False)

    def status(self) -> ClientResult:
        return self._request("status", wait_done=False)
