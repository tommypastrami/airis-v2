"""Byte transports for the bridge: a real serial port, or a socket to a simulator."""
from __future__ import annotations

import select
import socket
import time
import logging
from typing import Optional

from . import config

log = logging.getLogger("airis.transport")


class SocketTransport:
    """One end of a ``socket.socketpair()``; the fake Arduino holds the other end."""

    def __init__(self, sock: socket.socket):
        self.sock = sock
        self.sock.setblocking(False)
        self._buf = b""
        self.reopen_count = 0

    def readline(self, timeout: float) -> Optional[bytes]:
        deadline = time.monotonic() + timeout
        while b"\n" not in self._buf:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            r, _, _ = select.select([self.sock], [], [], remaining)
            if not r:
                return None
            try:
                chunk = self.sock.recv(4096)
            except (BlockingIOError, InterruptedError):
                continue
            except OSError:
                return None
            if not chunk:
                return None
            self._buf += chunk
        line, _, self._buf = self._buf.partition(b"\n")
        return line + b"\n"

    def write(self, data: bytes) -> None:
        self.sock.sendall(data)

    def reopen(self) -> bool:
        self.reopen_count += 1
        self._buf = b""
        return True

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass


class SerialTransport:  # pragma: no cover - hardware
    def __init__(self, candidates=config.SERIAL_CANDIDATES, baud=config.SERIAL_BAUD):
        import serial
        self._serial = serial
        self.candidates, self.baud = candidates, baud
        self.ser = None
        self.port = None
        self.reopen_count = 0
        self._buf = b""
        if not self.reopen():
            log.error("no serial port could be opened (tried %s)", ", ".join(candidates))

    def reopen(self) -> bool:
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        self._buf = b""
        for port in self.candidates:
            try:
                ser = self._serial.Serial()
                ser.port, ser.baudrate, ser.timeout = port, self.baud, 0.1
                ser.write_timeout = 1.0  # a stuck port raises instead of blocking the bridge forever
                ser.dtr = False          # set BEFORE open: no auto-reset of the Mega
                ser.rts = False
                ser.exclusive = True
                ser.open()
                time.sleep(0.2)
                ser.reset_input_buffer()
                self.ser, self.port = ser, port
                self.reopen_count += 1
                return True
            except (OSError, ValueError):
                continue
        return False

    def readline(self, timeout: float) -> Optional[bytes]:
        if self.ser is None:
            time.sleep(min(timeout, 0.2))
            return None
        deadline = time.monotonic() + timeout
        while b"\n" not in self._buf:
            if time.monotonic() >= deadline:
                return None                   # the partial line stays in self._buf
            try:
                chunk = self.ser.read(max(1, self.ser.in_waiting))   # blocks at most ser.timeout
            except Exception:
                return None
            self._buf += chunk
            if len(self._buf) > 4096 and b"\n" not in self._buf:
                self._buf = b""               # noise with no terminator: start clean
        line, _, self._buf = self._buf.partition(b"\n")
        return line + b"\n"

    def write(self, data: bytes) -> None:
        if self.ser is None:
            raise OSError("serial port closed")
        self.ser.write(data)

    def close(self) -> None:
        if self.ser is not None:
            self.ser.close()
