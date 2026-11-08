"""A fake ADAPT: connects to AIRIS, sends framed envelopes, and records whatever AIRIS sends back."""
from __future__ import annotations

import socket
import threading
from typing import Optional

from airis.adapt_link import Envelope, JsonCodec, frame, read_frame


class FakeAdapt(threading.Thread):
    def __init__(self, host: str, port: int, codec=None, max_bytes: int = 64 * 1024 * 1024):
        super().__init__(daemon=True, name="fake-adapt")
        self.codec = codec or JsonCodec()
        self.sock = socket.create_connection((host, port), timeout=5.0)
        self.sock.settimeout(0.2)
        self.max_bytes = max_bytes
        self.received: list[Envelope] = []
        self.stop = threading.Event()
        self.start()

    def send(self, env: Envelope) -> None:
        self.sock.sendall(frame(self.codec.encode(env)))

    def send_garbage(self, data: bytes) -> None:
        self.sock.sendall(data)

    def run(self) -> None:
        while not self.stop.is_set():
            try:
                payload = read_frame(self.sock, self.max_bytes)
            except socket.timeout:
                continue
            except OSError:
                break
            if payload is None:
                break
            self.received.append(self.codec.decode(payload))

    def close(self) -> None:
        self.stop.set()
        try:
            self.sock.close()
        except OSError:
            pass

    def wait_for(self, kind: str, n: int = 1, timeout: float = 5.0) -> list[Envelope]:
        import time
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            got = [e for e in self.received if e.kind == kind]
            if len(got) >= n:
                return got
            time.sleep(0.02)
        return [e for e in self.received if e.kind == kind]
